/*
 * can-proxy  -  Teensy 4.1 CAN proxy / gateway for BLITZ R-VIT
 *
 *   車両 (CKV36, 500kbps) ──CAN1── Teensy 4.1 ──CAN3── R-VIT
 *
 *   - R-VIT の CAN H/L を車両から切り離し、Teensy が ECM (0x7E0/0x7E8) の代わりに応答する。
 *   - 応答値は「車両ブロードキャストから算出」または「Teensy が自分で ECM に問い合わせた値」をキャッシュしたもの。
 *   - 未知のリクエストは車両へ素通し（パススルー）し、ECU の応答を R-VIT へ返す。
 *
 * ピンアサイン・配線・操作方法は README.md を参照。
 * DID の解析根拠は フレーム解析.md を参照。
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>

// ----------------------------------------------------------------------------
// テストモード
//   CAN_LOOPBACK_TEST を定義してビルドすると、プロキシ動作の代わりに
//   CAN1 <-> CAN3 の双方向通信テスト（ファイル末尾）を実行する。
//   platformio.ini の env:cantest で定義済み（.\monitor_upload.ps1 -Env cantest）。
//   直接有効にする場合は下の行のコメントを外す。
// ----------------------------------------------------------------------------
// #define CAN_LOOPBACK_TEST
#ifdef CAN_LOOPBACK_TEST
#pragma GCC diagnostic ignored "-Wunused-function"  // テスト時はプロキシ側の関数・変数を使わない
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// ============================================================================
// ピンアサイン（詳細は README.md）
// ============================================================================
//   CAN1 (車両側)  : pin 22 = CTX1 -> CAN Pal #1 "TX"
//                    pin 23 = CRX1 <- CAN Pal #1 "RX"
//   CAN3 (R-VIT側) : pin 31 = CTX3 -> CAN Pal #2 "TX"
//                    pin 30 = CRX3 <- CAN Pal #2 "RX"
//   LED            : pin 13 (オンボードLED) R-VIT からの受信で点灯
//   ※ pin 0/1 (CAN2 / Serial1) は将来の VDC-CAN またはラズパイ連携用に空けておく
//   ※ FlexCAN_T4 は CAN1=22/23, CAN3=30/31 固定なのでコード上でのピン指定は不要
static constexpr uint8_t PIN_LED = LED_BUILTIN;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64>  canCar;   // 車両側
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> canRvit;  // R-VIT側（ブリッジモードで車両の全フレームを流すので TX 大きめ）

// ============================================================================
// 設定
// ============================================================================
static constexpr uint32_t CAN_BAUD = 500000;

// 診断 ID
static constexpr uint32_t ID_ECM_REQ      = 0x7E0;  // R-VIT -> ECM
static constexpr uint32_t ID_ECM_RES      = 0x7E8;  // ECM   -> R-VIT
static constexpr uint32_t ID_DIAG_RES_MIN = 0x7E8;
static constexpr uint32_t ID_DIAG_RES_MAX = 0x7EF;
static constexpr uint32_t ID_DIAG_MIN     = 0x700;  // これ以上は診断系とみなしブロードキャスト転送の対象外

// 診断サービス
static constexpr uint8_t SID_READ_DID     = 0x22;
static constexpr uint8_t SID_READ_DID_POS = 0x62;
static constexpr uint8_t SID_NEGATIVE     = 0x7F;
static constexpr uint8_t NRC_PENDING      = 0x78;
static constexpr uint8_t PAD_BYTE         = 0x00;   // R-VIT / ECM ともに 0x00 パディング

// 車両ブロードキャスト ID
static constexpr uint32_t ID_THROTTLE = 0x160;
static constexpr uint32_t ID_RPM      = 0x1F9;
static constexpr uint32_t ID_SPEED    = 0x284;
static constexpr uint32_t ID_COOLANT  = 0x551;

// Teensy -> ECM ポーリング
static constexpr uint32_t POLL_FAST_MS     = 500;     // R-VIT 実測の一巡周期（約25〜30ms）相当
static constexpr uint32_t POLL_SLOW_MS     = 10000;  // 10秒毎
static constexpr uint32_t POLL_RETRY_MS    = 1000;   // まだ一度も取得できていない DID の再試行間隔
static constexpr uint32_t POLL_TIMEOUT_MS  = 50;
static constexpr uint32_t POLL_PENDING_MS  = 500;    // NRC 0x78（応答保留）受信時の待ち延長
static constexpr uint32_t POLL_MIN_GAP_MS  = 5;      // ポーリング送信の最小間隔
static constexpr uint32_t POLL_IDLE_MS     = 2000;   // R-VIT がこの時間要求してこない DID はポーリングしない
                                                     // （R-VIT 側で表示をやめた項目は ECM に問い合わせない）

// パススルー: 未知リクエストを車両へ流した後、この時間は診断応答 (0x7E8-0x7EF) を R-VIT へ返す
static constexpr uint32_t PASSTHROUGH_WINDOW_MS = 300;

static constexpr uint32_t LED_ON_MS            = 20;
static constexpr uint32_t UNKNOWN_LOG_PER_SEC  = 10;
static constexpr uint32_t AUTO_STATUS_MS       = 1000;
static constexpr uint32_t BCAST_DUMP_MS        = 10000;  // 車両ブロードキャスト最新フレームのダンプ周期

// ============================================================================
// 実行時の状態
// ============================================================================
enum class Mode : uint8_t { Proxy, Bridge };

static Mode g_mode         = Mode::Proxy;  // 起動時のモード
static bool g_pollEnabled  = true;         // Teensy -> ECM ポーリング
static bool g_passUnknown  = true;         // 未知リクエストを車両へ素通しする
static bool g_forwardBcast = false;        // Proxy モードで車両ブロードキャストを R-VIT へ転送する
static bool g_verbose      = false;
static bool g_autoStatus   = false;
static bool g_bcastDump    = true;         // 車両ブロードキャスト最新フレームを定期表示

enum class Source : uint8_t { Broadcast, Poll };

struct DidEntry {
  uint16_t    did;
  uint8_t     len;             // 応答データ長 (1 or 2 byte)
  Source      src;
  uint32_t    pollIntervalMs;  // Source::Poll のみ
  const char *name;
  uint8_t     value[2];        // 現在値（初期値 = 実データ受信前に返す値）
  bool        valid;           // 実データで一度でも更新されたか
  uint32_t    updatedMs;
  bool        everPolled;
  uint32_t    lastPollMs;
  uint32_t    proxyCount;
  bool        requested;       // R-VIT から一度でも要求されたか
  uint32_t    lastRequestMs;   // R-VIT から最後に要求された時刻
};

// R-VIT が現在この DID を要求しているか（ポーリング対象の判定）
static inline bool isRequested(const DidEntry &e, uint32_t now) {
  return e.requested && (now - e.lastRequestMs) < POLL_IDLE_MS;
}

// R-VIT がリクエストしてくる DID（フレーム解析.md「フレーム解析」節）
// 初期値はログで観測した値（アイドリング付近）
static DidEntry g_dids[] = {
  // did     len src                interval      name          初期値
  { 0x1101, 1, Source::Broadcast, 0,            "Coolant",    {0x88, 0x00} },  // 0x551 B0 + 10（= 水温+50）
  { 0x1102, 1, Source::Broadcast, 0,            "Speed",      {0x00, 0x00} },  // 0x284 B4-5 / 100 / 2
  { 0x111E, 1, Source::Broadcast, 0,            "Throttle",   {0x00, 0x00} },  // 0x160 B3 + B4上位2bit (10bit) → テーブル換算
  { 0x1201, 2, Source::Broadcast, 0,            "EngineRPM",  {0x00, 0x00} },  // 0x1F9 B2-3 / 100
  { 0x1106, 1, Source::Poll,      POLL_SLOW_MS, "IntakeTemp", {0x5C, 0x00} },
  { 0x110A, 1, Source::Poll,      POLL_FAST_MS, "IgnTiming",  {0x58, 0x00} },
  { 0x111F, 1, Source::Poll,      POLL_SLOW_MS, "OilTemp",    {0x85, 0x00} },
  { 0x1225, 2, Source::Poll,      POLL_FAST_MS, "AF_B1S1",    {0x01, 0x3A} },
};

// 車両ブロードキャストの最新フレーム（全バイト）を保持して定期ダンプする
struct BcastSnap {
  uint32_t      id;
  uint16_t      did;   // 変換先 DID（比較用）
  const char   *name;
  CAN_message_t msg;
  bool          valid;
  uint32_t      rxMs;
  uint32_t      count;
};
static BcastSnap g_bcastSnaps[] = {
  { ID_THROTTLE, 0x111E, "Throttle" },
  { ID_RPM,      0x1201, "EngineRPM" },
  { ID_SPEED,    0x1102, "Speed" },
  { ID_COOLANT,  0x1101, "Coolant" },
};
static uint32_t g_lastBcastDump = 0;

struct Stats {
  uint32_t carRx, carTx, carTxFail;
  uint32_t rvitRx, rvitTx, rvitTxFail;
  uint32_t proxyRes, passToCar, passToRvit, unknownDrop, bcastFwd;
  uint32_t pollReq, pollOk, pollNeg, pollTimeout;
};
static Stats g_stats = {};

struct PollState {
  bool     pending;
  uint16_t did;
  uint32_t sentMs;
  uint32_t timeoutMs;
};
static PollState g_poll = {};
static uint32_t  g_lastPollSentMs   = 0;
static uint32_t  g_passthroughUntil = 0;

static uint32_t g_ledOffAt = 0;
static bool     g_ledOn    = false;

static uint32_t g_rateWindowStart = 0, g_rateWindowCount = 0, g_rvitReqPerSec = 0;
static uint32_t g_unknownWindowStart = 0, g_unknownWindowCount = 0;
static uint32_t g_lastAutoStatus = 0;

// ============================================================================
// ユーティリティ
// ============================================================================
static inline bool timeReached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }
static inline uint8_t clampU8(uint32_t v) { return v > 0xFF ? 0xFF : (uint8_t)v; }

static DidEntry *findDid(uint16_t did) {
  for (auto &e : g_dids) {
    if (e.did == did) return &e;
  }
  return nullptr;
}

static void printFrame(const char *tag, const CAN_message_t &m) {
  Serial.printf("%10lu %-9s %03lX [%u]", (unsigned long)millis(), tag, (unsigned long)m.id, m.len);
  for (uint8_t i = 0; i < m.len && i < 8; i++) Serial.printf(" %02X", m.buf[i]);
  Serial.println();
}

// 受信フレームを送信用にコピー（受信時の付帯情報を持ち込まない）
static CAN_message_t cloneFrame(const CAN_message_t &src) {
  CAN_message_t m;
  m.id             = src.id;
  m.flags.extended = src.flags.extended;
  m.flags.remote   = src.flags.remote;
  m.len            = src.len;
  memcpy(m.buf, src.buf, sizeof(m.buf));
  return m;
}

static void sendCar(const CAN_message_t &m) {
  if (canCar.write(m)) g_stats.carTx++;
  else                 g_stats.carTxFail++;
}

static void sendRvit(const CAN_message_t &m) {
  if (canRvit.write(m)) g_stats.rvitTx++;
  else                  g_stats.rvitTxFail++;
}

static void setDidValue(uint16_t did, uint16_t v, uint32_t now) {
  DidEntry *e = findDid(did);
  if (!e) return;
  if (e->len == 1) {
    e->value[0] = clampU8(v);
  } else {
    e->value[0] = (uint8_t)(v >> 8);
    e->value[1] = (uint8_t)(v & 0xFF);
  }
  e->valid     = true;
  e->updatedMs = now;
}

// ============================================================================
// 車両ブロードキャスト最新フレームの記録 / ダンプ
// ============================================================================
static void recordBroadcast(const CAN_message_t &m, uint32_t now) {
  for (auto &s : g_bcastSnaps) {
    if (s.id != m.id) continue;
    s.msg   = m;
    s.valid = true;
    s.rxMs  = now;
    s.count++;
    return;
  }
}

static void printBcastDump(uint32_t now) {
  Serial.printf("---- bcast latest @%lu ms ----\n", (unsigned long)now);
  for (const auto &s : g_bcastSnaps) {
    Serial.printf(" %03lX %-9s ", (unsigned long)s.id, s.name);
    if (!s.valid) {
      Serial.println("(no frame)");
      continue;
    }
    Serial.printf("[%u]", s.msg.len);
    for (uint8_t i = 0; i < 8; i++) {
      if (i < s.msg.len) Serial.printf(" %02X", s.msg.buf[i]);
      else               Serial.print("   ");
    }
    Serial.printf("  age=%lums cnt=%lu", (unsigned long)(now - s.rxMs), (unsigned long)s.count);
    const DidEntry *e = findDid(s.did);
    if (e) {
      if (e->len == 1) Serial.printf("  -> DID %04X = %02X", e->did, e->value[0]);
      else             Serial.printf("  -> DID %04X = %02X %02X", e->did, e->value[0], e->value[1]);
    }
    Serial.println();
  }
}

// ============================================================================
// アクセルペダル位置 (0x160) -> DID 111E (スロットル開度, 255 = 100%)
// ============================================================================
//   0x160 B3 全bit + B4 bit7-6 = ペダル位置 10bit (0 = 解放, 792 = 踏み込み75%以上で飽和)
//   B4 bit5 = キックダウン域（踏み込み75%以上）, B5-B6 は B3-B4 の反転コピー（誤り検出用）
//   参考: https://github.com/balrog-kun/nissan-qashqai-can-info 「160 D.8-E.7」
//
//   ECM の DID 111E はペダル位置そのものではなくスロットルバルブ開度（ペダル→スロットルの
//   非線形マップを通した値）なので、割り算では合わない。
//   STM32G4 CANLOG/2.log.csv, log.log.csv で R-VIT → ECM の実応答と直前の 0x160 を突き合わせ、
//   安定区間の中央値から作った折れ線テーブルで換算する（相関 r=0.95, 平均誤差 ±3 程度）。
//   ペダル 312 (≈DID 156 = 61%) より上はログに無いため 792 = 255 へ直線外挿。
struct ThrottlePoint { uint16_t pedal; uint8_t did; };
static constexpr ThrottlePoint THROTTLE_MAP[] = {
  {   0,   0 }, {  12,   0 }, {  24,   5 }, {  40,  12 }, {  56,  24 }, {  72,  26 },
  {  88,  30 }, { 104,  31 }, { 120,  34 }, { 136,  36 }, { 152,  40 }, { 168,  43 },
  { 184,  51 }, { 200,  66 }, { 216,  75 }, { 232,  90 }, { 248, 114 }, { 264, 139 },
  { 280, 146 }, { 296, 154 }, { 312, 156 }, { 792, 255 },
};
static constexpr uint16_t PEDAL_MAX = 792;

static uint8_t pedalToThrottleDid(uint16_t pedal) {
  constexpr size_t N = sizeof(THROTTLE_MAP) / sizeof(THROTTLE_MAP[0]);
  if (pedal >= THROTTLE_MAP[N - 1].pedal) return THROTTLE_MAP[N - 1].did;
  for (size_t i = 1; i < N; i++) {
    const ThrottlePoint &a = THROTTLE_MAP[i - 1];
    const ThrottlePoint &b = THROTTLE_MAP[i];
    if (pedal < b.pedal) {
      return (uint8_t)(a.did + (int32_t)(b.did - a.did) * (pedal - a.pedal) / (b.pedal - a.pedal));
    }
  }
  return THROTTLE_MAP[N - 1].did;
}

// ============================================================================
// 車両ブロードキャスト -> DID 値へ変換（フレーム解析.md 参照）
// ============================================================================
static void decodeBroadcast(const CAN_message_t &m, uint32_t now) {
  switch (m.id) {
    case ID_COOLANT:  // B0 = 水温+40 → DID 1101 は 水温+50
      if (m.len >= 1) setDidValue(0x1101, (uint32_t)m.buf[0] + 10, now);
      break;
    case ID_SPEED: {  // B4-5 = 車速 x0.01 km/h → DID 1102 は km/h / 2
      if (m.len >= 6) {
        uint32_t raw = ((uint32_t)m.buf[4] << 8) | m.buf[5];
        setDidValue(0x1102, raw / 100 / 2, now);
      }
      break;
    }
    case ID_THROTTLE: {  // B3 全bit + B4 bit7-6 の 10bit ペダル位置 → テーブル換算
      if (m.len >= 5) {
        uint16_t pedal = ((uint16_t)m.buf[3] << 2) | (m.buf[4] >> 6);
        if (m.buf[4] & 0x20) pedal = PEDAL_MAX;  // キックダウン域
        setDidValue(0x111E, pedalToThrottleDid(pedal), now);
      }
      break;
    }
    case ID_RPM: {  // B2-3 = rpm x8 → DID 1201 は rpm / 12.5 → raw / 100
      if (m.len >= 4) {
        uint32_t raw = ((uint32_t)m.buf[2] << 8) | m.buf[3];
        setDidValue(0x1201, raw / 100, now);
      }
      break;
    }
    default:
      break;
  }
}

// ============================================================================
// Teensy -> ECM ポーリング
// ============================================================================
static bool handlePollResponse(const CAN_message_t &m, uint32_t now) {
  if (!g_poll.pending || m.id != ID_ECM_RES) return false;

  const uint8_t pci = m.buf[0];
  if ((pci & 0xF0) != 0x00) return false;  // シングルフレームのみ対象
  const uint8_t sfLen = pci & 0x0F;
  if (sfLen < 3) return false;

  if (m.buf[1] == SID_READ_DID_POS) {
    const uint16_t did = ((uint16_t)m.buf[2] << 8) | m.buf[3];
    if (did != g_poll.did) return false;
    DidEntry *e = findDid(did);
    if (e) {
      uint8_t n = sfLen - 3;
      if (n > e->len) n = e->len;
      for (uint8_t i = 0; i < n; i++) e->value[i] = m.buf[4 + i];
      e->valid     = true;
      e->updatedMs = now;
    }
    g_poll.pending = false;
    g_stats.pollOk++;
    if (g_verbose) printFrame("POLL<", m);
    return true;
  }

  if (m.buf[1] == SID_NEGATIVE && m.buf[2] == SID_READ_DID) {
    if (m.buf[3] == NRC_PENDING) {  // 応答保留 → 待ち時間延長
      g_poll.sentMs    = now;
      g_poll.timeoutMs = POLL_PENDING_MS;
      return true;
    }
    g_poll.pending = false;
    g_stats.pollNeg++;
    Serial.printf("POLL negative response DID %04X NRC %02X\n", g_poll.did, m.buf[3]);
    return true;
  }
  return false;
}

static void servicePolling(uint32_t now) {
  if (g_mode != Mode::Proxy || !g_pollEnabled) return;

  if (g_poll.pending) {
    if (now - g_poll.sentMs < g_poll.timeoutMs) return;
    g_poll.pending = false;
    g_stats.pollTimeout++;
    if (g_verbose) Serial.printf("POLL timeout DID %04X\n", g_poll.did);
  }

  // R-VIT の未知リクエストを素通し中は ECU を占有しないよう待つ
  if (!timeReached(now, g_passthroughUntil)) return;
  if (now - g_lastPollSentMs < POLL_MIN_GAP_MS) return;

  // 最も期限を過ぎている DID を選ぶ
  DidEntry *best = nullptr;
  uint32_t bestOverdue = 0;
  for (auto &e : g_dids) {
    if (e.src != Source::Poll) continue;
    if (!isRequested(e, now)) continue;  // R-VIT が表示していない項目は問い合わせない
    uint32_t overdue;
    if (!e.everPolled) {
      overdue = UINT32_MAX;
    } else {
      uint32_t interval = e.pollIntervalMs;
      if (!e.valid && interval > POLL_RETRY_MS) interval = POLL_RETRY_MS;
      const uint32_t elapsed  = now - e.lastPollMs;
      if (elapsed < interval) continue;
      overdue = elapsed - interval;
    }
    if (!best || overdue > bestOverdue) {
      best = &e;
      bestOverdue = overdue;
    }
  }
  if (!best) return;

  CAN_message_t req;
  req.id     = ID_ECM_REQ;
  req.len    = 8;
  req.buf[0] = 0x03;
  req.buf[1] = SID_READ_DID;
  req.buf[2] = (uint8_t)(best->did >> 8);
  req.buf[3] = (uint8_t)(best->did & 0xFF);
  for (uint8_t i = 4; i < 8; i++) req.buf[i] = PAD_BYTE;
  sendCar(req);

  best->everPolled = true;
  best->lastPollMs = now;
  g_poll           = {true, best->did, now, POLL_TIMEOUT_MS};
  g_lastPollSentMs = now;
  g_stats.pollReq++;
  if (g_verbose) printFrame("POLL>", req);
}

// ============================================================================
// R-VIT への代理応答
// ============================================================================
static void sendProxyResponse(DidEntry &e) {
  CAN_message_t r;
  r.id     = ID_ECM_RES;
  r.len    = 8;
  r.buf[0] = 3 + e.len;          // SF 長 = SID + DID(2) + データ
  r.buf[1] = SID_READ_DID_POS;
  r.buf[2] = (uint8_t)(e.did >> 8);
  r.buf[3] = (uint8_t)(e.did & 0xFF);
  for (uint8_t i = 0; i < 4; i++) r.buf[4 + i] = (i < e.len) ? e.value[i] : PAD_BYTE;
  sendRvit(r);
  e.proxyCount++;
  g_stats.proxyRes++;
  if (g_verbose) printFrame("PROXY<", r);
}

static void reportUnknown(const CAN_message_t &m, uint32_t now, bool forwarded) {
  if (now - g_unknownWindowStart >= 1000) {
    g_unknownWindowStart = now;
    g_unknownWindowCount = 0;
  }
  if (g_unknownWindowCount++ >= UNKNOWN_LOG_PER_SEC) return;
  printFrame(forwarded ? "UNK>CAR" : "UNK DROP", m);
}

// ============================================================================
// 受信ハンドラ（loop() 内の events() から呼ばれる）
// ============================================================================
static void onCarFrame(const CAN_message_t &m) {
  const uint32_t now = millis();
  g_stats.carRx++;

  recordBroadcast(m, now);
  decodeBroadcast(m, now);  // どのモードでもキャッシュは更新する

  if (g_mode == Mode::Bridge) {
    sendRvit(cloneFrame(m));
    if (g_verbose && m.id >= ID_DIAG_RES_MIN && m.id <= ID_DIAG_RES_MAX) printFrame("CAR>RVIT", m);
    return;
  }

  // ---- Proxy モード ----
  if (m.id >= ID_DIAG_RES_MIN && m.id <= ID_DIAG_RES_MAX) {
    if (handlePollResponse(m, now)) return;           // 自分のポーリング応答
    if (!timeReached(now, g_passthroughUntil)) {      // R-VIT の素通しリクエストへの応答
      sendRvit(cloneFrame(m));
      g_stats.passToRvit++;
      printFrame("PASS<CAR", m);
    }
    return;
  }

  if (g_forwardBcast && m.id < ID_DIAG_MIN) {
    sendRvit(cloneFrame(m));
    g_stats.bcastFwd++;
  }
}

static void onRvitFrame(const CAN_message_t &m) {
  const uint32_t now = millis();
  g_stats.rvitRx++;
  g_rateWindowCount++;

  digitalWriteFast(PIN_LED, HIGH);
  g_ledOn    = true;
  g_ledOffAt = now + LED_ON_MS;

  if (g_verbose) printFrame("RVIT>", m);

  if (g_mode == Mode::Bridge) {
    sendCar(cloneFrame(m));
    return;
  }

  // ---- Proxy モード ----
  // 03 22 DIDh DIDl : ReadDataByIdentifier（シングルフレーム）
  if (m.id == ID_ECM_REQ && m.len >= 4 && m.buf[0] == 0x03 && m.buf[1] == SID_READ_DID) {
    const uint16_t did = ((uint16_t)m.buf[2] << 8) | m.buf[3];
    DidEntry *e = findDid(did);
    if (e) {
      e->requested     = true;
      e->lastRequestMs = now;
      sendProxyResponse(*e);
      return;
    }
  }

  // 未知のリクエスト（起動時の車両確認、フロー制御フレーム等を含む）
  if (g_passUnknown) {
    sendCar(cloneFrame(m));
    g_stats.passToCar++;
    g_passthroughUntil = now + PASSTHROUGH_WINDOW_MS;
    reportUnknown(m, now, true);
  } else {
    g_stats.unknownDrop++;
    reportUnknown(m, now, false);
  }
}

// ============================================================================
// シリアルコンソール
// ============================================================================
static const char *modeName(Mode m) { return m == Mode::Proxy ? "PROXY" : "BRIDGE"; }
static const char *onOff(bool b) { return b ? "ON" : "OFF"; }

static void printHelp() {
  Serial.println();
  Serial.println("---- can-proxy commands ----");
  Serial.println(" h / ?  : help");
  Serial.println(" s      : status");
  Serial.println(" a      : auto status (1s) ON/OFF");
  Serial.println(" p      : PROXY mode  (R-VIT を遮断して代理応答)");
  Serial.println(" b      : BRIDGE mode (両方向すべて素通し = 直結と同等)");
  Serial.println(" q      : Teensy -> ECM polling ON/OFF");
  Serial.println(" u      : unknown request passthrough ON/OFF");
  Serial.println(" f      : forward vehicle broadcast to R-VIT ON/OFF (PROXY)");
  Serial.println(" v      : verbose ON/OFF");
  Serial.println(" d      : vehicle broadcast latest frame dump (10s) ON/OFF");
  Serial.println(" D      : vehicle broadcast latest frame dump (now)");
  Serial.println(" r      : reset counters");
  Serial.println();
}

static void printPhysical(const DidEntry &e) {
  switch (e.did) {
    case 0x1101: Serial.printf("%4d degC", (int)e.value[0] - 50); break;
    case 0x1102: Serial.printf("%4u km/h", (unsigned)e.value[0] * 2); break;
    case 0x1201: Serial.printf("%6.1f rpm", (double)(((e.value[0] << 8) | e.value[1]) * 12.5f)); break;
    default:     Serial.print("-"); break;
  }
}

static void printStatus() {
  const uint32_t now = millis();
  Serial.println();
  Serial.printf("==== can-proxy status  uptime %lu.%03lus ====\n",
                (unsigned long)(now / 1000), (unsigned long)(now % 1000));
  Serial.printf("mode=%s poll=%s passUnknown=%s fwdBcast=%s verbose=%s\n",
                modeName(g_mode), onOff(g_pollEnabled), onOff(g_passUnknown),
                onOff(g_forwardBcast), onOff(g_verbose));
  Serial.printf("CAR : rx=%lu tx=%lu txFail=%lu\n",
                (unsigned long)g_stats.carRx, (unsigned long)g_stats.carTx, (unsigned long)g_stats.carTxFail);
  Serial.printf("RVIT: rx=%lu tx=%lu txFail=%lu  rx/s=%lu\n",
                (unsigned long)g_stats.rvitRx, (unsigned long)g_stats.rvitTx,
                (unsigned long)g_stats.rvitTxFail, (unsigned long)g_rvitReqPerSec);
  Serial.printf("proxy=%lu passToCar=%lu passToRvit=%lu unknownDrop=%lu bcastFwd=%lu\n",
                (unsigned long)g_stats.proxyRes, (unsigned long)g_stats.passToCar,
                (unsigned long)g_stats.passToRvit, (unsigned long)g_stats.unknownDrop,
                (unsigned long)g_stats.bcastFwd);
  Serial.printf("poll: req=%lu ok=%lu neg=%lu timeout=%lu\n",
                (unsigned long)g_stats.pollReq, (unsigned long)g_stats.pollOk,
                (unsigned long)g_stats.pollNeg, (unsigned long)g_stats.pollTimeout);
  Serial.println(" DID  name        src   raw     age(ms)  proxied  physical");
  for (const auto &e : g_dids) {
    char raw[8];
    if (e.len == 1) snprintf(raw, sizeof(raw), "%02X", e.value[0]);
    else            snprintf(raw, sizeof(raw), "%02X %02X", e.value[0], e.value[1]);
    char age[12];
    if (e.valid) snprintf(age, sizeof(age), "%lu", (unsigned long)(now - e.updatedMs));
    else         snprintf(age, sizeof(age), "default");
    const char *src = e.src == Source::Broadcast ? "bcast" : (isRequested(e, now) ? "poll" : "idle");
    Serial.printf(" %04X %-11s %-5s %-7s %8s %8lu  ",
                  e.did, e.name, src, raw, age,
                  (unsigned long)e.proxyCount);
    printPhysical(e);
    Serial.println();
  }
  Serial.println();
}

static void setMode(Mode m) {
  g_mode             = m;
  g_poll.pending     = false;
  g_passthroughUntil = millis();
  Serial.printf("mode -> %s\n", modeName(g_mode));
}

static void handleSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    switch (c) {
      case 'h': case '?': printHelp(); break;
      case 's': printStatus(); break;
      case 'a': g_autoStatus = !g_autoStatus; Serial.printf("autoStatus=%s\n", onOff(g_autoStatus)); break;
      case 'p': setMode(Mode::Proxy); break;
      case 'b': setMode(Mode::Bridge); break;
      case 'q': g_pollEnabled = !g_pollEnabled; g_poll.pending = false;
                Serial.printf("poll=%s\n", onOff(g_pollEnabled)); break;
      case 'u': g_passUnknown = !g_passUnknown; Serial.printf("passUnknown=%s\n", onOff(g_passUnknown)); break;
      case 'f': g_forwardBcast = !g_forwardBcast; Serial.printf("fwdBcast=%s\n", onOff(g_forwardBcast)); break;
      case 'v': g_verbose = !g_verbose; Serial.printf("verbose=%s\n", onOff(g_verbose)); break;
      case 'd': g_bcastDump = !g_bcastDump; Serial.printf("bcastDump=%s\n", onOff(g_bcastDump)); break;
      case 'D': printBcastDump(millis()); break;
      case 'r': {
        g_stats = {};
        for (auto &e : g_dids) e.proxyCount = 0;
        Serial.println("counters reset");
        break;
      }
      default: break;  // 改行等は無視
    }
  }
}

// ============================================================================
// setup / loop
// ============================================================================
template <typename Bus>
static void setupBus(Bus &bus, void (*handler)(const CAN_message_t &)) {
  bus.begin();
  bus.setBaudRate(CAN_BAUD);
  bus.setMaxMB(16);
  bus.enableFIFO();           // 全 ID 受信
  bus.enableFIFOInterrupt();
  bus.onReceive(handler);
}

// ============================================================================
// USB ソフト再接続（ケーブル抜き差しの代わり）
// ============================================================================
//   書き込み後、ハブ経由だと Windows が Teensy を列挙しないことがある。
//   起動後しばらくホストから USB コンフィギュレーションされなければ、
//   USBCMD.RS をクリアして D+ プルアップを外し（= 切断）、少し待って再セットする（= 再接続）。
//   USB 未接続（車載）でも数回試すだけなので CAN 動作には影響しない。
extern "C" volatile uint8_t usb_configuration;  // cores/teensy4/usb.c : 非 0 = ホストに認識済み

static constexpr uint32_t USB_REATTACH_FIRST_MS = 4000;   // 起動後この時間未認識なら 1 回目
static constexpr uint32_t USB_REATTACH_RETRY_MS = 10000;  // 以後の再試行間隔
static constexpr uint32_t USB_DETACH_MS         = 500;    // 切断している時間
static constexpr uint8_t  USB_REATTACH_MAX      = 3;
static uint8_t g_usbReattachCount = 0;

static void serviceUsbReattach(uint32_t now) {
  static bool     done      = false;
  static bool     detached  = false;
  static uint32_t nextAt    = USB_REATTACH_FIRST_MS;
  static uint32_t attachAt  = 0;
  if (done) return;

  if (detached) {
    if ((int32_t)(now - attachAt) >= 0) {
      USB1_USBCMD |= USB_USBCMD_RS;  // プルアップ ON -> ホストが再列挙
      detached = false;
      nextAt   = now + USB_REATTACH_RETRY_MS;
    }
    return;
  }
  if (usb_configuration) { done = true; return; }  // 一度認識されたら以後は何もしない
  if ((int32_t)(now - nextAt) < 0) return;
  if (g_usbReattachCount >= USB_REATTACH_MAX) { done = true; return; }

  g_usbReattachCount++;
  USB1_USBCMD &= ~USB_USBCMD_RS;     // プルアップ OFF -> ホストからは抜かれたように見える
  detached = true;
  attachAt = now + USB_DETACH_MS;
}

#ifndef CAN_LOOPBACK_TEST

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWriteFast(PIN_LED, LOW);

  // 車載では PC 未接続なので USB シリアルを待たずに CAN を即起動する。
  // バナーはモニターが接続された時点で loop() から表示する。
  Serial.begin(115200);

  setupBus(canCar, onCarFrame);
  setupBus(canRvit, onRvitFrame);
}

static void printBanner() {
  Serial.println();
  Serial.println("=== can-proxy : Teensy 4.1 R-VIT proxy ===");
  Serial.println(" CAN1 (vehicle) : TX=pin22 RX=pin23  500kbps");
  Serial.println(" CAN3 (R-VIT)   : TX=pin31 RX=pin30  500kbps");
  Serial.printf(" uptime=%lums  mode=%s  usb_reattach=%u  (h でヘルプ)\n",
                (unsigned long)millis(), modeName(g_mode), g_usbReattachCount);
  if (CrashReport) Serial.print(CrashReport);  // 前回クラッシュ情報があれば表示
}

// モニター（DTR）の接続を検出してバナーを出す
static void serviceSerialConnect() {
  static bool connected = false;
  const bool now = (bool)Serial;
  if (now && !connected) printBanner();
  connected = now;
}

void loop() {
  canCar.events();
  canRvit.events();

  serviceSerialConnect();

  const uint32_t now = millis();

  serviceUsbReattach(now);
  servicePolling(now);

  if (g_ledOn && timeReached(now, g_ledOffAt)) {
    digitalWriteFast(PIN_LED, LOW);
    g_ledOn = false;
  }

  if (now - g_rateWindowStart >= 1000) {
    g_rvitReqPerSec   = g_rateWindowCount;
    g_rateWindowCount = 0;
    g_rateWindowStart = now;
  }

  if (g_autoStatus && now - g_lastAutoStatus >= AUTO_STATUS_MS) {
    g_lastAutoStatus = now;
    printStatus();
  }

  if (g_bcastDump && now - g_lastBcastDump >= BCAST_DUMP_MS) {
    g_lastBcastDump = now;
    printBcastDump(now);
  }

  handleSerial();
}

#else  // CAN_LOOPBACK_TEST

// ============================================================================
// CAN1 <-> CAN3 双方向通信テスト
// ============================================================================
//   配線: CAN Pal #1 と #2 の CANH 同士、CANL 同士を直結する（車両・R-VIT は外す）。
//         終端は少なくとも片方 ON（両方 ON = 60Ω が理想）。
//   動作: CAN1 -> CAN3 (ID 0x101) と CAN3 -> CAN1 (ID 0x301) を同時に定周期送信し、
//         受信側で 連番欠落・データ化け・遅延 を検査する。1 秒ごとに結果を表示。
//   データ: B0-3 = 連番 (LE), B4-6 = 連番から作る検査パターン, B7 = B0-6 の和
static constexpr uint32_t TEST_ID_1TO3          = 0x101;  // CAN1 が送信 / CAN3 が受信
static constexpr uint32_t TEST_ID_3TO1          = 0x301;  // CAN3 が送信 / CAN1 が受信
static constexpr uint32_t TEST_REPORT_MS        = 1000;
static constexpr uint32_t TEST_BURST_FRAMES     = 500;    // バースト時の各方向フレーム数
static constexpr uint32_t TEST_BURST_TIMEOUT_MS = 3000;

struct LinkTest {
  const char *name;
  uint32_t    id;
  uint32_t    nextSeq;    // 次に送る連番
  uint32_t    expectSeq;  // 次に受信するはずの連番
  uint32_t    tx, txFull, rxOk, lost, bad, stale;
  uint32_t    latMinUs, latMaxUs, latCount;
  uint64_t    latSumUs;
  uint32_t    burstLeft;
  uint32_t    sentUs[256];  // 送信時刻（連番の下位 8bit で索引）
};

struct LinkSnap {
  uint32_t tx, txFull, rxOk, lost, bad;
};

static LinkTest t13 = {"CAN1->CAN3", TEST_ID_1TO3};
static LinkTest t31 = {"CAN3->CAN1", TEST_ID_3TO1};
static LinkSnap g_snap13 = {}, g_snap31 = {};    // 前回レポート時点
static LinkSnap g_burst13 = {}, g_burst31 = {};  // バースト開始時点

static uint32_t g_testIntervalMs = 50;
static bool     g_testRunning    = true;
static bool     g_testAutoReport = true;
static bool     g_testVerbose    = false;
static bool     g_burstActive    = false;
static uint32_t g_burstStartMs   = 0;
static uint32_t g_lastSendMs     = 0;
static uint32_t g_lastReportMs   = 0;
static uint32_t g_otherRx1 = 0, g_otherRx3 = 0;

static LinkSnap snap(const LinkTest &t) { return {t.tx, t.txFull, t.rxOk, t.lost, t.bad}; }

static void resetLink(LinkTest &t) {
  // 連番は継続（送信中のフレームを欠落扱いしないため）
  t.tx = t.txFull = t.rxOk = t.lost = t.bad = t.stale = 0;
  t.latMinUs = t.latMaxUs = t.latCount = 0;
  t.latSumUs = 0;
}

static void fillPayload(uint8_t *b, uint32_t seq) {
  b[0] = (uint8_t)seq;
  b[1] = (uint8_t)(seq >> 8);
  b[2] = (uint8_t)(seq >> 16);
  b[3] = (uint8_t)(seq >> 24);
  b[4] = (uint8_t)~b[0];
  b[5] = (uint8_t)(b[1] ^ 0x5A);
  b[6] = (seq & 1) ? 0xAA : 0x55;  // ビット交互パターン
  b[7] = (uint8_t)(b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6]);
}

template <typename Bus>
static bool testSend(Bus &bus, LinkTest &t) {
  CAN_message_t m;
  m.id  = t.id;
  m.len = 8;
  fillPayload(m.buf, t.nextSeq);
  t.sentUs[t.nextSeq & 0xFF] = micros();
  if (!bus.write(m)) return false;  // 送信キュー満杯
  t.nextSeq++;
  t.tx++;
  return true;
}

static void testReceive(LinkTest &t, const CAN_message_t &m) {
  const uint32_t nowUs = micros();

  digitalWriteFast(PIN_LED, HIGH);
  g_ledOn    = true;
  g_ledOffAt = millis() + LED_ON_MS;

  uint8_t expect[8];
  const uint32_t seq = (uint32_t)m.buf[0] | ((uint32_t)m.buf[1] << 8) |
                       ((uint32_t)m.buf[2] << 16) | ((uint32_t)m.buf[3] << 24);
  fillPayload(expect, seq);
  if (m.len != 8 || m.flags.extended || memcmp(expect, m.buf, 8) != 0) {
    t.bad++;
    printFrame("BAD", m);
    return;
  }

  const int32_t gap = (int32_t)(seq - t.expectSeq);
  if (gap < 0) {  // 重複・順序違い
    t.stale++;
    if (g_testVerbose) Serial.printf("%s stale seq=%lu expect=%lu\n", t.name, (unsigned long)seq,
                                     (unsigned long)t.expectSeq);
    return;
  }
  if (gap > 0) {
    t.lost += (uint32_t)gap;
    if (g_testVerbose) Serial.printf("%s lost %ld (seq=%lu)\n", t.name, (long)gap, (unsigned long)seq);
  }
  t.expectSeq = seq + 1;
  t.rxOk++;

  if (t.nextSeq - seq <= 256) {  // 送信時刻がまだ上書きされていない
    const uint32_t lat = nowUs - t.sentUs[seq & 0xFF];
    if (t.latCount == 0 || lat < t.latMinUs) t.latMinUs = lat;
    if (lat > t.latMaxUs) t.latMaxUs = lat;
    t.latSumUs += lat;
    t.latCount++;
  }
}

static void testOnCan1(const CAN_message_t &m) {
  if (m.id == TEST_ID_3TO1) {
    testReceive(t31, m);
  } else {
    g_otherRx1++;
    if (g_testVerbose) printFrame("CAN1 ?", m);
  }
}

static void testOnCan3(const CAN_message_t &m) {
  if (m.id == TEST_ID_1TO3) {
    testReceive(t13, m);
  } else {
    g_otherRx3++;
    if (g_testVerbose) printFrame("CAN3 ?", m);
  }
}

// FlexCAN のエラーカウンタとエラーフラグを直接読む
static void printBusState(const char *name, uint32_t base) {
  const uint32_t ecr = FLEXCANb_ECR(base);
  const uint32_t esr = FLEXCANb_ESR1(base);
  const uint32_t flt = (esr >> 4) & 0x3;
  Serial.printf("  %s: TEC=%3lu REC=%3lu %-10s", name, (unsigned long)(ecr & 0xFF),
                (unsigned long)((ecr >> 8) & 0xFF),
                flt == 0 ? "ErrActive" : flt == 1 ? "ErrPassive" : "BusOff");
  if (esr & (1UL << 13)) Serial.print(" ACK_ERR");
  if (esr & (1UL << 14)) Serial.print(" BIT0_ERR");
  if (esr & (1UL << 15)) Serial.print(" BIT1_ERR");
  if (esr & (1UL << 12)) Serial.print(" CRC_ERR");
  if (esr & (1UL << 11)) Serial.print(" FRM_ERR");
  if (esr & (1UL << 10)) Serial.print(" STF_ERR");
  Serial.println();
}

static void printLink(const LinkTest &t, const LinkSnap &cur, const LinkSnap &prev) {
  Serial.printf("  %s: tx=%lu(+%lu) ok=%lu(+%lu) lost=%lu bad=%lu stale=%lu txFull=%lu",
                t.name, (unsigned long)cur.tx, (unsigned long)(cur.tx - prev.tx),
                (unsigned long)cur.rxOk, (unsigned long)(cur.rxOk - prev.rxOk),
                (unsigned long)t.lost, (unsigned long)t.bad, (unsigned long)t.stale,
                (unsigned long)t.txFull);
  if (t.latCount) {
    Serial.printf("  lat avg/min/max=%lu/%lu/%lu us", (unsigned long)(t.latSumUs / t.latCount),
                  (unsigned long)t.latMinUs, (unsigned long)t.latMaxUs);
  }
  Serial.println();
}

// 前回レポートからの区間で 受信あり & 欠落・化けなし なら OK
static bool linkHealthy(const LinkSnap &cur, const LinkSnap &prev) {
  return cur.rxOk > prev.rxOk && cur.lost == prev.lost && cur.bad == prev.bad;
}

static void testReport(uint32_t now) {
  const LinkSnap n13 = snap(t13), n31 = snap(t31);
  const bool ok13 = linkHealthy(n13, g_snap13);
  const bool ok31 = linkHealthy(n31, g_snap31);
  const bool idle = !g_testRunning && !g_burstActive;

  Serial.printf("[%8.1fs] %s  interval=%lums%s\n", now / 1000.0,
                idle ? "PAUSED" : (ok13 && ok31) ? "PASS" : "FAIL",
                (unsigned long)g_testIntervalMs, g_burstActive ? "  (burst)" : "");
  printLink(t13, n13, g_snap13);
  printLink(t31, n31, g_snap31);
  printBusState("CAN1", (uint32_t)CAN1);
  printBusState("CAN3", (uint32_t)CAN3);
  if (g_otherRx1 || g_otherRx3) {
    Serial.printf("  other IDs: CAN1=%lu CAN3=%lu\n", (unsigned long)g_otherRx1, (unsigned long)g_otherRx3);
  }

  if (!idle) {
    if (n13.rxOk == g_snap13.rxOk) {
      Serial.println("  >> CAN1->CAN3 受信なし: CAN Pal #1 TX(pin22) / #2 RX(pin30)、CANH-CANL 配線、終端、電源を確認");
    }
    if (n31.rxOk == g_snap31.rxOk) {
      Serial.println("  >> CAN3->CAN1 受信なし: CAN Pal #2 TX(pin31) / #1 RX(pin23)、CANH-CANL 配線、終端、電源を確認");
    }
    if (n13.txFull != g_snap13.txFull || n31.txFull != g_snap31.txFull) {
      Serial.println("  >> 送信キュー満杯: 送信が完了していない（ACK_ERR なら相手がバスを受信できていない）");
    }
  }
  Serial.println();

  g_snap13 = n13;
  g_snap31 = n31;
}

static void testStartBurst(uint32_t now) {
  if (g_burstActive) return;
  t13.burstLeft  = TEST_BURST_FRAMES;
  t31.burstLeft  = TEST_BURST_FRAMES;
  g_burst13      = snap(t13);
  g_burst31      = snap(t31);
  g_burstStartMs = now;
  g_burstActive  = true;
  Serial.printf("burst start: %lu frames x 2 directions\n", (unsigned long)TEST_BURST_FRAMES);
}

static void testServiceBurst(uint32_t now) {
  // 送信キューが空いている分だけ詰める（満杯は次の loop で再試行）
  while (t13.burstLeft && testSend(canCar, t13)) t13.burstLeft--;
  while (t31.burstLeft && testSend(canRvit, t31)) t31.burstLeft--;

  const bool sentAll = !t13.burstLeft && !t31.burstLeft;
  const bool rcvAll  = t13.expectSeq == t13.nextSeq && t31.expectSeq == t31.nextSeq;
  const uint32_t elapsed = now - g_burstStartMs;
  if (!(sentAll && rcvAll) && elapsed < TEST_BURST_TIMEOUT_MS) return;

  const uint32_t ok13 = t13.rxOk - g_burst13.rxOk, ok31 = t31.rxOk - g_burst31.rxOk;
  const uint32_t ms   = elapsed ? elapsed : 1;
  Serial.printf("burst %s: %lu ms  %s ok=%lu lost=%lu  %s ok=%lu lost=%lu  total %.0f frames/s\n",
                (sentAll && rcvAll) ? "done" : "TIMEOUT", (unsigned long)elapsed,
                t13.name, (unsigned long)ok13, (unsigned long)(t13.lost - g_burst13.lost),
                t31.name, (unsigned long)ok31, (unsigned long)(t31.lost - g_burst31.lost),
                (ok13 + ok31) * 1000.0 / ms);
  t13.burstLeft = t31.burstLeft = 0;
  g_burstActive = false;
}

static void testPrintBanner() {
  Serial.println();
  Serial.println("=== can-proxy : CAN1 <-> CAN3 LOOPBACK TEST ===");
  Serial.println(" 配線: CAN Pal #1/#2 の CANH 同士・CANL 同士を直結（車両・R-VIT は外す）");
  Serial.println(" 終端: 少なくとも片方 ON（両方 ON 推奨）");
  Serial.printf(" CAN1->CAN3 ID 0x%03lX / CAN3->CAN1 ID 0x%03lX  %lukbps  uptime=%lums\n",
                (unsigned long)TEST_ID_1TO3, (unsigned long)TEST_ID_3TO1,
                (unsigned long)(CAN_BAUD / 1000), (unsigned long)millis());
  Serial.println(" (h でヘルプ)");
  if (CrashReport) Serial.print(CrashReport);
}

static void testPrintHelp() {
  Serial.println();
  Serial.println("---- loopback test commands ----");
  Serial.println(" h / ?  : help");
  Serial.println(" s      : 今すぐ結果表示");
  Serial.println(" a      : 1秒ごとの自動表示 ON/OFF");
  Serial.println(" p      : 定周期送信 一時停止/再開");
  Serial.println(" + / -  : 送信間隔 x2 / /2 (1-1000ms)");
  Serial.println(" b      : バースト（各方向 500 フレームを最速で送信）");
  Serial.println(" v      : verbose ON/OFF（欠落やテスト以外の ID を表示）");
  Serial.println(" r      : カウンタリセット");
  Serial.println();
}

static void testHandleSerial(uint32_t now) {
  while (Serial.available()) {
    switch ((char)Serial.read()) {
      case 'h': case '?': testPrintHelp(); break;
      case 's': testReport(now); break;
      case 'a': g_testAutoReport = !g_testAutoReport;
                Serial.printf("autoReport=%s\n", g_testAutoReport ? "ON" : "OFF"); break;
      case 'p': g_testRunning = !g_testRunning;
                Serial.printf("periodic send=%s\n", g_testRunning ? "ON" : "OFF"); break;
      case '+': if (g_testIntervalMs < 1000) g_testIntervalMs *= 2;
                if (g_testIntervalMs > 1000) g_testIntervalMs = 1000;
                Serial.printf("interval=%lums\n", (unsigned long)g_testIntervalMs); break;
      case '-': g_testIntervalMs /= 2;
                if (g_testIntervalMs < 1) g_testIntervalMs = 1;
                Serial.printf("interval=%lums\n", (unsigned long)g_testIntervalMs); break;
      case 'b': testStartBurst(now); break;
      case 'v': g_testVerbose = !g_testVerbose;
                Serial.printf("verbose=%s\n", g_testVerbose ? "ON" : "OFF"); break;
      case 'r': resetLink(t13); resetLink(t31);
                g_snap13 = g_snap31 = {};
                g_otherRx1 = g_otherRx3 = 0;
                Serial.println("counters reset"); break;
      default: break;
    }
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWriteFast(PIN_LED, LOW);
  Serial.begin(115200);

  setupBus(canCar, testOnCan1);
  setupBus(canRvit, testOnCan3);
}

void loop() {
  canCar.events();
  canRvit.events();

  static bool connected = false;  // モニター接続でバナー表示
  if ((bool)Serial && !connected) testPrintBanner();
  connected = (bool)Serial;

  const uint32_t now = millis();

  serviceUsbReattach(now);

  if (g_burstActive) {
    testServiceBurst(now);
  } else if (g_testRunning && now - g_lastSendMs >= g_testIntervalMs) {
    g_lastSendMs = now;
    if (!testSend(canCar, t13)) t13.txFull++;
    if (!testSend(canRvit, t31)) t31.txFull++;
  }

  if (g_ledOn && timeReached(now, g_ledOffAt)) {
    digitalWriteFast(PIN_LED, LOW);
    g_ledOn = false;
  }

  if (g_testAutoReport && now - g_lastReportMs >= TEST_REPORT_MS) {
    g_lastReportMs = now;
    testReport(now);
  }

  testHandleSerial(now);
}

#endif  // CAN_LOOPBACK_TEST
