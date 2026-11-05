<#
monitor_upload.ps1
プロジェクトルートで実行する想定。Windows PowerShell 5.1 で安全に動作するように
ASCII ベースで記述しています。

使い方:
  .\monitor_upload.ps1                # 本番 (teensy41)
  .\monitor_upload.ps1 -Env cantest   # CAN1 <-> CAN3 双方向通信テスト (CAN_LOOPBACK_TEST)
  .\monitor_upload.ps1 -Env diag      # 切り分け用 USB シリアルテスト (CAN 不使用)
  .\monitor_upload.ps1 -Env diag_old  # 同上を旧プラットフォーム (teensy@4.18.0) でビルド
  .\monitor_upload.ps1 -HubRestart    # COM ポートが出ないとき Teensy の親ハブを管理者権限で再起動
                                       (UAC 確認が出る。同じハブにつながる他の機器も一瞬切断される)

手順:
  1. Teensy の COM ポートがあれば 134 baud で開いて自動でブートローダーに入れる (ボタン不要)
     COM ポートが無い / 別のモニターが使用中のときは、
     "Waiting for Teensy device..." が出たら Teensy のボタンを 1 回押す
  2. 書き込み後、ファームが未認識を検出すると USB を自動で切断→再接続する (起動 4 秒後〜)
     それでも出なければ -HubRestart 指定時はハブ再起動、未指定なら "Unplug and replug" の案内で拜き差し
  3. COM ポートを自動検出してモニター開始。ログは logs\serial_<日時>.log (UTF-8)
     終了は Ctrl+C

cantest の配線:
  CAN Pal #1 / #2 の CANH 同士・CANL 同士を直結 (車両・R-VIT は外す)
  終端は少なくとも片方 ON (両方 ON 推奨)

cantest のシリアルコマンド:
  h / ?  : ヘルプ
  s      : 今すぐ結果表示
  a      : 1 秒ごとの自動表示 ON/OFF
  p      : 定周期送信 一時停止/再開
  + / -  : 送信間隔 x2 / /2 (1-1000ms, 初期 50ms)
  b      : バースト (各方向 500 フレームを最速で送信し frames/s と欠落を表示)
  v      : verbose ON/OFF (欠落やテスト以外の ID を表示)
  r      : カウンタリセット

cantest の見方:
  PASS/FAIL は直近 1 秒で 受信あり & 欠落・化けなし かどうか
  ACK_ERR で TEC が増え続ける -> 相手側がバスを受信できていない
                                 (CANH/CANL 配線・相手側 RX ピン・CAN Pal 電源を確認)
  両方向とも受信なし          -> まず CAN Pal の電源 (3.3V/GND) を確認
#>
param(
    [Alias('Env')]
    [string]$PioEnv = 'teensy41',
    [switch]$HubRestart
)
Write-Host ("Target env: " + $PioEnv) -ForegroundColor Cyan

# logs ディレクトリを安全に作成
$dir = '.\logs'
if (-not (Test-Path $dir -PathType Container)) {
    New-Item -ItemType Directory -Path $dir | Out-Null
}

# タイムスタンプ付きファイル名
# StreamWriter は .NET のカレントディレクトリ基準になるため絶対パスにする
$ts = Get-Date -Format 'yyyyMMddHHmmss'
$file = Join-Path (Resolve-Path $dir).Path ("serial_$ts.log")

# pio を探す（PATH -> 既知の場所 -> python -m platformio）
$pioCmd = $null
try { $cmd = Get-Command pio -ErrorAction Stop; $pioCmd = $cmd.Path } catch {}
if (-not $pioCmd) {
    $candidate1 = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe'
    if (Test-Path $candidate1) { $pioCmd = $candidate1 }
    else {
        $candidate2 = Join-Path $env:USERPROFILE 'AppData\Roaming\Python\Scripts\platformio.exe'
        if (Test-Path $candidate2) { $pioCmd = $candidate2 }
    }
}

if (-not $pioCmd) {
    Write-Host "pio not found in PATH; will try 'python -m platformio'" -ForegroundColor Yellow
    $usePython = $true
} else {
    Write-Host ("Using pio: " + $pioCmd) -ForegroundColor Green
    $usePython = $false
}

function Invoke-Pio([string[]]$PioArgs) {
    # Out-Host で画面に流す（関数の戻り値に pio の出力が混ざらないようにする）
    if ($usePython) { python -m platformio @PioArgs | Out-Host } else { & $pioCmd @PioArgs | Out-Host }
    return $LASTEXITCODE
}

# Teensy が最後につながっていた USB ハブを再起動する（ケーブル抜き差しの代わり）。
# デバイス再起動は管理者権限が必要なので pnputil を UAC 昇格して実行する。
function Restart-TeensyHub {
    $hub = $null
    $devs = Get-PnpDevice -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -match '^USB\\VID_16C0&PID_04' }
    foreach ($d in $devs) {
        foreach ($key in 'DEVPKEY_Device_Parent', 'DEVPKEY_Device_LastKnownParent') {
            $p = (Get-PnpDeviceProperty -InstanceId $d.InstanceId -KeyName $key -ErrorAction SilentlyContinue).Data
            if ($p -match '^USB\\') { $hub = $p; break }
        }
        if ($hub) { break }
    }
    if (-not $hub) {
        Write-Host "Parent hub of Teensy not found; cannot restart hub." -ForegroundColor Yellow
        return
    }
    $others = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object {
        $_.InstanceId -notmatch 'VID_16C0' -and
        (Get-PnpDeviceProperty -InstanceId $_.InstanceId -KeyName DEVPKEY_Device_Parent -ErrorAction SilentlyContinue).Data -eq $hub
    }
    # インスタンス ID（ポート位置・シリアル番号を含む）はログ/スクショに残さないよう VID/PID だけ表示する
    $hubLabel = ($hub -split '\\')[1]
    Write-Host ("Restarting USB hub " + $hubLabel + " (UAC prompt)...") -ForegroundColor Cyan
    foreach ($o in $others) { Write-Host ("  also affected: " + $o.FriendlyName) -ForegroundColor Yellow }
    try {
        $pr = Start-Process -FilePath 'pnputil.exe' -ArgumentList '/restart-device', ('"' + $hub + '"') `
            -Verb RunAs -Wait -PassThru -WindowStyle Hidden -ErrorAction Stop
        if ($pr.ExitCode -ne 0) {
            Write-Host ("pnputil exit " + $pr.ExitCode + " (/restart-device needs Windows 10 2004 or later)") -ForegroundColor Yellow
        }
    } catch {
        Write-Host ("Hub restart cancelled/failed: " + $_.Exception.Message) -ForegroundColor Yellow
    }
}

# 先にビルドだけ行う（ブートローダーに入れてから書き込みまでの時間を短くするため）
$rc = Invoke-Pio @('run', '-e', $PioEnv)
if ($rc -ne 0) {
    Write-Host ("build failed (exit " + $rc + ")") -ForegroundColor Red
    exit $rc
}

# ソフトリブート: teensy_loader_cli は Windows でソフトリブート非対応なので自前で行う。
# Teensy の USB シリアルは 134 baud で開かれるとブートローダー (HalfKay) に入る（Teensyduino の teensy_reboot と同じ方式）。
# COM ポートが無い（RawHID 状態・未接続・モニター使用中など）場合は従来どおりボタン押下で書き込む。
$tdev = Get-CimInstance Win32_PnPEntity -Filter "PNPClass='Ports'" -ErrorAction SilentlyContinue |
    Where-Object { $_.PNPDeviceID -match 'VID_16C0&PID_0483' -and $_.Name -match '\((COM\d+)\)' } |
    Select-Object -First 1
if ($tdev) {
    $null = $tdev.Name -match '\((COM\d+)\)'
    $rebootPort = $Matches[1]
    Write-Host ("Soft reboot via " + $rebootPort + " (134 baud)...") -ForegroundColor Cyan
    try {
        $sp = New-Object System.IO.Ports.SerialPort $rebootPort, 134
        $sp.Open()
        Start-Sleep -Milliseconds 100
        $sp.Close()
    } catch {
        Write-Host ("Soft reboot failed: " + $_.Exception.Message) -ForegroundColor Yellow
        Write-Host "(another monitor may be holding the port) Press the Teensy button when prompted." -ForegroundColor Yellow
    }
} else {
    Write-Host "Teensy COM port not found; press the Teensy button when 'Waiting for Teensy device...' appears." -ForegroundColor Yellow
    # VID_1FC9&PID_0135 = i.MX RT1062 の ROM ブートローダー (NXP Serial Downloader)。
    # フラッシュ内容が起動できない状態（書き込み中断など）だと IMXRT がこれになり、Teensy Loader からは見えない
    $nxpRom = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -match 'VID_1FC9&PID_0135' }
    if ($nxpRom) {
        Write-Host "NXP ROM boot mode (VID_1FC9&PID_0135) detected: flash image may be invalid." -ForegroundColor Yellow
        Write-Host "  Press the Teensy button (repeat if needed) until the red LED stays dim, or hold 15s to restore blink." -ForegroundColor Yellow
    }
}

# upload 実行（ビルド済みなので書き込みのみ）
$rc = Invoke-Pio @('run', '-e', $PioEnv, '-t', 'upload')

if ($rc -ne 0) {
    Write-Host ("upload failed (exit " + $rc + ")") -ForegroundColor Red
    exit $rc
}

# monitor を実行してログへ追記（画面表示＋ファイル保存）
# pyserial (miniterm) を直接使って対話型の問題（UserSideException）を回避します。
Write-Host "Starting serial monitor (logging to $file)..." -ForegroundColor Green

$enc = New-Object System.Text.UTF8Encoding($false)   # $true にすると BOM 付き
$sw  = New-Object System.IO.StreamWriter($file, $true, $enc)

try {
    # platformio.ini からボーレートとポートを取得する
    $baud = 115200
    $port = $null
    if (Test-Path "platformio.ini") {
        $iniContent = Get-Content "platformio.ini" -Raw
        if ($iniContent -match '(?m)^\s*monitor_speed\s*=\s*(\d+)') {
            $baud = [int]$Matches[1]
        }
        if ($iniContent -match '(?m)^\s*monitor_port\s*=\s*([^\r\n;]+)') {
            $port = $Matches[1].Trim()
        }
    }

    # monitor_port 未指定なら Teensy (VID 16C0 / PID 0483 = USB Serial) の COM ポートを自動検出
    # 書き込み直後は再列挙に時間がかかる（初回はドライバ登録もある）。
    # ファーム側が起動 4 秒後から USB 再接続を試すので、まず 10 秒待つ。
    # それでも出なければハブ再起動 (-HubRestart) か抜き差しの案内を出して最大 60 秒待つ
    if (-not $port) {
        Write-Host "Waiting for Teensy COM port..." -ForegroundColor Cyan
        for ($i = 0; $i -lt 120 -and -not $port; $i++) {
            if ($i -eq 20) {
                if ($HubRestart) {
                    Restart-TeensyHub
                } else {
                    Write-Host "COM port not yet found. Unplug and replug the Teensy USB cable (or rerun with -HubRestart)." -ForegroundColor Yellow
                }
            }
            $dev = Get-CimInstance Win32_PnPEntity -Filter "PNPClass='Ports'" -ErrorAction SilentlyContinue |
                Where-Object { $_.PNPDeviceID -match 'VID_16C0&PID_0483' -and $_.Name -match '\((COM\d+)\)' } |
                Select-Object -First 1
            if ($dev) {
                $null = $dev.Name -match '\((COM\d+)\)'
                $port = $Matches[1]
            } else {
                Start-Sleep -Milliseconds 500
            }
        }
        if (-not $port) {
            Write-Host "Teensy COM port not found." -ForegroundColor Red
            exit 1
        }
        Write-Host ("Found Teensy on " + $port) -ForegroundColor Green
    }

    # プラットフォーム固有の python (PlatformIOの仮想環境内) またはシステムの python を探す
    $pyCmd = "python"
    $candidatePy = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'
    if (Test-Path $candidatePy) {
        $pyCmd = $candidatePy
    }

    # miniterm は UTF-8 で出力するが、PowerShell 5.1 はパイプ受信時に [Console]::OutputEncoding
    # (日本語環境では CP932) で解釈して文字化けするので、一時的に UTF-8 にする
    $prevOutEnc = [Console]::OutputEncoding
    [Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
    $env:PYTHONIOENCODING = 'utf-8'

    # pyserial miniterm を実行 (--exit-char 3 を追加して Ctrl+C で終了できるようにする)
    # 2>&1 で stderr の行は ErrorRecord になるので、元の文字列 (TargetObject) を取り出す
    & $pyCmd -m serial.tools.miniterm --exit-char 3 --eol LF $port $baud 2>&1 | ForEach-Object {
        if ($_ -is [System.Management.Automation.ErrorRecord]) {
            $line = [string]$_.TargetObject
        } else {
            $line = [string]$_
        }
        Write-Host $line
        $sw.WriteLine($line)
        $sw.Flush()
    }
}
finally {
    if ($prevOutEnc) { [Console]::OutputEncoding = $prevOutEnc }
    $sw.Close()
}