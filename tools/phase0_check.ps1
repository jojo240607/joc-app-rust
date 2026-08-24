<#
.SYNOPSIS
  P3-D2 真机联调 Phase 0 环境自检（COM12 通信 / COM8 调试 / ST-Link SWD 烧录）。

.DESCRIPTION
  依序执行 6 步自检，每步输出 PASS/FAIL 横幅，最后汇总退出码（失败数）：
    0.1 端口枚举：COM12(STM VCP) + COM8(调试口) 可见；pyserial 可用
    0.2 工具链检查：build_app.py --check（cargo / arm-none-eabi-gcc / objcopy / ABI 版本）
    0.3 烧录当前固件（App 分区 0x08060000，openocd SWD）
    0.4 运行板子 + 抓 COM8 启动日志（校验 app mounted / flyctrl 任务 / hb 心跳）
    0.5 下行 MAVLink 帧验证（verify_downlink.py：CRC / seq_dups / seq_jumps）
    0.6 上行 ARM/DISARM 验证（groundctrl-tools：COMMAND_ACK ACCEPTED）
    0.7 （可选，-RunStep7）原始流 dump_usb.py

.NOTES
  前置：板子需同时接 ST-Link(SWD，烧录/复位) 与 板载 USB(枚举为 COM12)。
        验证下行/上行期间【不要】挂 OpenOCD/GDB（halt 会让 CDC 端口掉线）。
        跑前关闭占用 COM12 的进程（QGC / 地面站 / 之前的 python）。
  备注：verify_structure.py 已过时（只认旧 3 帧循环 [HB21][LP40][SS43]），
        本脚本统一用 verify_downlink.py（兼容当前 6 帧下行 [0,32,1,30,74,33]）。

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File tools\phase0_check.ps1
  powershell -ExecutionPolicy Bypass -File tools\phase0_check.ps1 -SkipFlash -SkipRun
  powershell -ExecutionPolicy Bypass -File tools\phase0_check.ps1 -SkipFlash -SkipRun -SkipUplink
  powershell -ExecutionPolicy Bypass -File tools\phase0_check.ps1 -ComPort COM12 -CaptureSeconds 20
#>
[CmdletBinding()]
param(
    [string]$ComPort      = "COM12",   # 通信串口（USB CDC）
    [string]$DebugPort    = "COM8",    # 调试日志串口
    [int]$RunSeconds      = 12,        # run_app.py 监听 COM8 时长
    [int]$CaptureSeconds  = 15,        # verify_downlink.py 捕获时长
    [switch]$SkipFlash,                # 跳过 0.3 烧录
    [switch]$SkipRun,                  # 跳过 0.4 运行（需板子已在跑）
    [switch]$SkipDownlink,             # 跳过 0.5 下行验证
    [switch]$SkipUplink,               # 跳过 0.6 上行验证
    [switch]$RunStep7                  # 可选执行 0.7 原始流 dump
)
$ErrorActionPreference = "Stop"

# ── 路径 ─────────────────────────────────────────────
$Root     = "D:\project\mcu\oop\joc-app-rust"
$Tools    = Join-Path $Root "tools"
$GcsTools = "D:\project\mcu\oop\groundctrl\target\release\groundctrl-tools.exe"

$script:PassCount = 0
$script:FailCount = 0

# ── 辅助函数 ─────────────────────────────────────────
function Show-Banner {
    param([string]$Title)
    Write-Host ""
    Write-Host ("=" * 62) -ForegroundColor Cyan
    Write-Host ("  " + $Title) -ForegroundColor Cyan
    Write-Host ("=" * 62) -ForegroundColor Cyan
}

function Write-StepResult {
    param([bool]$Pass, [string]$Msg)
    if ($Pass) { $script:PassCount++; Write-Host "[PASS] $Msg" -ForegroundColor Green }
    else       { $script:FailCount++; Write-Host "[FAIL] $Msg" -ForegroundColor Red }
}

function Invoke-Py {
    param([string]$Script, [string[]]$ArgList, [string]$Cwd = $null)
    $oldLoc = Get-Location
    $oldEap = $ErrorActionPreference
    if ($Cwd) { Set-Location $Cwd }
    try {
        # PS 5.1：原生程序写 stderr（如 cargo 警告）在 EAP=Stop 下会抛
        # NativeCommandError 直接中止脚本，这里临时放宽到 Continue。
        $ErrorActionPreference = "Continue"
        $out = & python $Script @ArgList 2>&1 | Out-String
        return [pscustomobject]@{ ExitCode = $LASTEXITCODE; Output = $out }
    } finally {
        $ErrorActionPreference = $oldEap
        Set-Location $oldLoc
    }
}

function Invoke-WithTimeout {
    # 以独立进程运行可执行文件，最多等 $TimeoutSec 秒；超时则强杀进程并返回
    # ExitCode=-1（避免 groundctrl-tools 卡死时把整个脚本拖住、更怕僵尸进程
    # 继续霸占 COM 句柄）。输出大小有限，直接同步读 stdout/stderr。
    param([string]$FilePath, [string[]]$ArgList, [int]$TimeoutSec = 30)
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $FilePath
    $psi.UseShellExecute = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.CreateNoWindow = $true
    # PS 5.1 无 ArgumentList 属性，参数无空格直接用空格拼接
    $psi.Arguments = ($ArgList -join " ")
    $p = [System.Diagnostics.Process]::Start($psi)
    if (-not $p.WaitForExit($TimeoutSec * 1000)) {
        try { $p.Kill() } catch { }
        $p.WaitForExit()
        $stdout = $p.StandardOutput.ReadToEnd()
        $stderr = $p.StandardError.ReadToEnd()
        return [pscustomobject]@{ ExitCode = -1; Output = "[TIMEOUT ${TimeoutSec}s]" + $stdout + $stderr }
    }
    $stdout = $p.StandardOutput.ReadToEnd()
    $stderr = $p.StandardError.ReadToEnd()
    return [pscustomobject]@{ ExitCode = $p.ExitCode; Output = $stdout + $stderr }
}

Write-Host "Phase 0 环境自检  COM=$ComPort  DBG=$DebugPort" -ForegroundColor Cyan

# ── 0.1 端口枚举 ─────────────────────────────────────
Show-Banner "STEP 0.1  端口枚举（$ComPort=通信 / $DebugPort=调试）"
# 用 .NET 直读，避免 Get-CimInstance(WMI) 在 CDC 设备异常时挂起
$devIds = @([System.IO.Ports.SerialPort]::GetPortNames())
Write-Host ("检测到 COM 口：" + ($(if ($devIds.Count) { $devIds -join ", " } else { "(无)" })))
$hasCom = $devIds -contains $ComPort
$hasDbg = $devIds -contains $DebugPort
Write-StepResult ($hasCom -and $hasDbg) "$ComPort=$hasCom $DebugPort=$hasDbg（期望均为可见）"

Show-Banner "STEP 0.1b  pyserial 版本"
$oldEap = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$pyOut = & python -c "import serial; print('pyserial', serial.__version__)" 2>&1 | Out-String
$pyExit = $LASTEXITCODE
$ErrorActionPreference = $oldEap
Write-Host $pyOut
Write-StepResult (($pyExit -eq 0) -and ($pyOut -match "pyserial\s+3\.\d+")) "pyserial 3.x 可用"

# ── 0.2 工具链检查 ───────────────────────────────────
Show-Banner "STEP 0.2  工具链检查（cargo / arm-none-eabi-gcc / objcopy / ABI）"
$r = Invoke-Py "build_app.py" @("--check") $Root
Write-Host $r.Output
Write-StepResult (($r.ExitCode -eq 0) -and ($r.Output -match "\[OK\] 工具链就绪")) "工具链就绪 / RTOS_ABI_VERSION 匹配"

# ── 0.3 烧录 ─────────────────────────────────────────
Show-Banner "STEP 0.3  烧录当前固件（App 分区 0x08060000，openocd SWD）"
if ($SkipFlash) {
    Write-Host "[SKIP]" -ForegroundColor Yellow
} else {
    $r = Invoke-Py "flash_app.py" @() $Root
    Write-Host $r.Output
    $ok = ($r.Output -match "\[OK\] App 分区烧录完成") -or ($r.Output -match "wrote")
    Write-StepResult $ok "App 分区烧录完成"
}

# ── 0.4 运行 + COM8 日志 ─────────────────────────────
Show-Banner "STEP 0.4  运行板子 + 抓 $DebugPort 启动日志（${RunSeconds}s）"
if ($SkipRun) {
    Write-Host "[SKIP]（需确认板子已在运行）" -ForegroundColor Yellow
} else {
    $r = Invoke-Py "run_app.py" @($DebugPort, "115200", "$RunSeconds") $Root
    Write-Host $r.Output
    # 用 \w+ 匹配 True/False，避免吞入行尾的 "]"
    $m = [regex]::Match($r.Output, "HAS_APP=(\w+)\s+HAS_FC=(\w+)\s+HAS_HB=(\w+)")
    $ok = $m.Success -and ($m.Groups[1].Value -eq "True") -and ($m.Groups[2].Value -eq "True") -and ($m.Groups[3].Value -eq "True")
    Write-StepResult $ok "app mounted / flyctrl 任务 / hb 心跳 均出现"
}

# ── 0.5 下行验证 ─────────────────────────────────────
Show-Banner "STEP 0.5  下行 MAVLink 帧验证（${CaptureSeconds}s）"
if ($SkipDownlink) {
    Write-Host "[SKIP]" -ForegroundColor Yellow
} else {
    $r = Invoke-Py (Join-Path $Tools "verify_downlink.py") @($ComPort, "$CaptureSeconds") $Tools
    Write-Host $r.Output
    $m = [regex]::Match($r.Output, "frames=(\d+)\s+crc_ok=(\d+)\s+crc_bad=(\d+)\s+seq_dups=(\d+)\s+seq_jumps=(\d+)")
    $ok = $m.Success -and ([int]$m.Groups[1].Value -gt 0) -and ([int]$m.Groups[3].Value -eq 0) -and ([int]$m.Groups[4].Value -eq 0) -and ([int]$m.Groups[5].Value -eq 0)
    Write-StepResult $ok "frames>0 / crc_bad=0 / seq_dups=0 / seq_jumps=0"
}

# ── 0.6 上行验证 ─────────────────────────────────────
Show-Banner "STEP 0.6  上行 ARM/DISARM 验证（groundctrl-tools）"
if ($SkipUplink) {
    Write-Host "[SKIP]" -ForegroundColor Yellow
} elseif (-not (Test-Path $GcsTools)) {
    Write-StepResult $false "未找到 groundctrl-tools.exe，需先构建：cargo build -p groundctrl-tools"
} else {
    # 超时兜底 25s（ARM 单次运行 8s + 启动开销，留足余量）；超时返回 ExitCode=-1
    Write-Host ">>> ARM ..." -ForegroundColor DarkYellow
    $a = Invoke-WithTimeout $GcsTools @("--port", $ComPort, "--baud", "115200", "--send-cmd", "ARM", "--duration", "8") 25
    Write-Host $a.Output
    Write-Host ">>> DISARM ..." -ForegroundColor DarkYellow
    $d = Invoke-WithTimeout $GcsTools @("--port", $ComPort, "--baud", "115200", "--send-cmd", "DISARM", "--duration", "8") 25
    Write-Host $d.Output
    $ok = ($a.ExitCode -eq 0) -and ($a.Output -match "ACCEPTED") -and ($d.ExitCode -eq 0) -and ($d.Output -match "ACCEPTED")
    if ($a.ExitCode -lt 0) { Write-Host "（ARM 超时，已强杀 groundctrl-tools）" -ForegroundColor Red }
    if ($d.ExitCode -lt 0) { Write-Host "（DISARM 超时，已强杀 groundctrl-tools）" -ForegroundColor Red }
    Write-StepResult $ok "ARM / DISARM 均收到 COMMAND_ACK ACCEPTED"
}

# ── 0.7 原始流（可选）────────────────────────────────
if ($RunStep7) {
    Show-Banner "STEP 0.7  （可选）原始流 dump_usb.py"
    $r = Invoke-Py (Join-Path $Tools "dump_usb.py") @($ComPort, "6", "8") $Tools
    Write-Host $r.Output
    Write-StepResult ($r.ExitCode -eq 0) "已抓原始流，请人工核对帧内容"
}

# ── 汇总 ─────────────────────────────────────────────
Write-Host ""
Write-Host ("=" * 62) -ForegroundColor Cyan
Write-Host ("  汇总：PASS=$($script:PassCount)  FAIL=$($script:FailCount)") -ForegroundColor Cyan
Write-Host ("=" * 62) -ForegroundColor Cyan
if ($script:FailCount -eq 0) {
    Write-Host "[PASS] Phase 0 环境自检全部通过，可开始 HIL 前置实现（MCU hil feature + PC hil_link.rs）" -ForegroundColor Green
} else {
    Write-Host "[FAIL] 存在失败项，请按上面提示修复后重跑" -ForegroundColor Red
}
exit $script:FailCount
