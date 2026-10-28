<#
.SYNOPSIS
    Start the SO-ARM101 vision pick-and-place stack, each part in its own window.

.DESCRIPTION
    Starts, in order, waiting for each to answer before the next:
      1. sim2hw\ik_server.py           MuJoCo IK server        http://127.0.0.1:7842
      2. sim2hw\hw_bridge.py           real-arm bridge (only with -Port)  http://127.0.0.1:7843
      3. CV-seg\Robotic_Arm-main\object_detector_standalone.py   camera window
    Uses the .venv made by setup.ps1 (falls back to python on PATH). Close a
    window or press Ctrl+C in it to stop that part.

    If scripts are blocked, use: powershell -ExecutionPolicy Bypass -File .\run.ps1 ...

.PARAMETER Port
    Serial port of the arm (for example COM4). Without it nothing moves the
    real arm: the IK server and detector run in simulation only.

.PARAMETER Detector
    pick-place (default with -Port): select an object and press a to pick it.
    preview (default without -Port): IK reachability checks only (press p).
    detect: camera detection only.  none: do not start the detector.

.PARAMETER Camera
    Camera index for the detector (default 1).

.PARAMETER Viewer
    Open the interactive MuJoCo 3-D viewer window.

.PARAMETER Dashboard
    Also serve and open the browser dashboard (sim2hw\so101_dashboard.html).

.PARAMETER IkArgs
    Extra arguments for ik_server.py as one string.

.PARAMETER BridgeArgs
    Extra arguments for hw_bridge.py as one string, e.g. "--log-health 1 --p-gain 20".

.PARAMETER DetectorArgs
    Extra arguments for the detector as one string, e.g. "--learning-mode off".

.PARAMETER DryRun
    Print the commands without starting anything.

.EXAMPLE
    .\run.ps1 -Port COM4
.EXAMPLE
    .\run.ps1
.EXAMPLE
    .\run.ps1 -Port COM4 -Camera 0 -Viewer -BridgeArgs "--log-health 1"
.EXAMPLE
    .\run.ps1 -Port COM4 -Detector none -Dashboard
#>
[CmdletBinding()]
param(
    [string]$Port = "",
    [ValidateSet("", "pick-place", "preview", "detect", "none")]
    [string]$Detector = "",
    [int]$Camera = 1,
    [switch]$Viewer,
    [switch]$Dashboard,
    [string[]]$IkArgs = @(),
    [string[]]$BridgeArgs = @(),
    [string[]]$DetectorArgs = @(),
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$Root = $PSScriptRoot
$Sim2hw = Join-Path $Root "sim2hw"
$DetectorScript = Join-Path $Root "CV-seg\Robotic_Arm-main\object_detector_standalone.py"
$IkUrl = "http://127.0.0.1:7842"
$BridgeUrl = "http://127.0.0.1:7843"

# ---- Python ----------------------------------------------------------------
$Py = Join-Path $Root ".venv\Scripts\python.exe"
if (-not (Test-Path $Py)) {
    $found = Get-Command python -ErrorAction SilentlyContinue
    if (-not $found) { throw "No .venv and no python on PATH. Run .\setup.ps1 first." }
    $Py = $found.Source
    Write-Warning "No .venv found; using $Py. Run .\setup.ps1 to create the environment."
}

if (-not $Detector) { $Detector = if ($Port) { "pick-place" } else { "preview" } }
if ($Port) {
    $Port = $Port.ToUpper()
    $available = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($available -notcontains $Port) {
        Write-Warning "$Port is not among the serial ports found ($($available -join ', ')). Check the cable, or find it with lerobot-find-port."
    }
} elseif ($Detector -eq "pick-place") {
    Write-Warning "-Detector pick-place without -Port: pick/place runs in simulation only (no real arm)."
}

# ---- helpers ---------------------------------------------------------------
function Quote([string]$Text) { "'" + ($Text -replace "'", "''") + "'" }

function Split-ExtraArgs([string[]]$Values) {
    # Accept "--log-health 1", '--log-health','1' or (via powershell -File,
    # which joins lists with commas) "--log-health,1". Quotes keep spaces.
    $out = @()
    foreach ($value in $Values) {
        foreach ($m in [regex]::Matches("$value", '"([^"]*)"|''([^'']*)''|[^\s,]+')) {
            if ($m.Groups[1].Success) { $out += $m.Groups[1].Value }
            elseif ($m.Groups[2].Success) { $out += $m.Groups[2].Value }
            else { $out += $m.Value }
        }
    }
    return ,$out
}
$IkArgs = Split-ExtraArgs $IkArgs
$BridgeArgs = Split-ExtraArgs $BridgeArgs
$DetectorArgs = Split-ExtraArgs $DetectorArgs

function Start-InWindow([string]$Title, [string]$WorkDir, [string]$Script, [string[]]$Arguments) {
    $argText = ($Arguments | ForEach-Object { Quote $_ }) -join " "
    $command = "`$Host.UI.RawUI.WindowTitle = $(Quote $Title); " +
               "`$env:PYTHONIOENCODING = 'utf-8'; " +
               "Set-Location -LiteralPath $(Quote $WorkDir); " +
               "& $(Quote $Py) $(Quote $Script) $argText; " +
               "Write-Host ''; Write-Host ($(Quote "$Title stopped (exit code ") + `$LASTEXITCODE + ').') -ForegroundColor Yellow"
    Write-Host ("  {0,-16} {1} {2}" -f $Title, (Split-Path $Script -Leaf), ($Arguments -join " "))
    if ($DryRun) { return }
    # -EncodedCommand avoids quoting problems with spaces in the paths.
    $encoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($command))
    Start-Process powershell.exe -ArgumentList @("-NoExit", "-NoProfile", "-EncodedCommand", $encoded) | Out-Null
}

function Wait-ForHttp([string]$Url, [int]$Seconds, [string]$Name) {
    if ($DryRun) { return $true }
    $deadline = (Get-Date).AddSeconds($Seconds)
    Write-Host "  waiting for $Name at $Url ..." -NoNewline
    while ((Get-Date) -lt $deadline) {
        try {
            Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec 2 | Out-Null
            Write-Host " ready" -ForegroundColor Green
            return $true
        } catch {
            Start-Sleep -Milliseconds 500
            Write-Host "." -NoNewline
        }
    }
    Write-Host " not answering" -ForegroundColor Yellow
    return $false
}

# ---- 1. IK server ----------------------------------------------------------
Write-Host ""
Write-Host "Starting the SO-ARM101 stack$(if ($DryRun) { ' (dry run)' })" -ForegroundColor Cyan
$ik = @() + $IkArgs
if ($Viewer) { $ik = @("--viewer") + $ik }
Start-InWindow "IK server" $Sim2hw (Join-Path $Sim2hw "ik_server.py") $ik
if (-not (Wait-ForHttp "$IkUrl/state" 60 "the IK server")) {
    Write-Warning "The IK server did not start; check its window for the error."
}

# ---- 2. hardware bridge ----------------------------------------------------
if ($Port) {
    Start-InWindow "HW bridge" $Sim2hw (Join-Path $Sim2hw "hw_bridge.py") (@("--port", $Port) + $BridgeArgs)
    if (-not (Wait-ForHttp "$BridgeUrl/ping" 30 "the hardware bridge")) {
        Write-Warning "The hardware bridge did not start; check its window (port in use? arm powered?)."
    }
}

# ---- 3. object detector ----------------------------------------------------
if ($Detector -ne "none") {
    $det = @("--camera", "$Camera")
    if ($Detector -eq "pick-place") { $det += "--ik-pick-place" }
    if ($Detector -eq "preview") { $det += "--ik-preview" }
    Start-InWindow "Object detector" $Root $DetectorScript ($det + $DetectorArgs)
}

# ---- optional browser dashboard --------------------------------------------
if ($Dashboard) {
    Start-InWindow "Dashboard web" $Sim2hw "-m" @("http.server", "8000")
    if (-not $DryRun) {
        Start-Sleep -Seconds 2
        Start-Process "http://127.0.0.1:8000/so101_dashboard.html"
    }
}

Write-Host ""
Write-Host "Running:" -ForegroundColor Green
Write-Host "  IK server + coordinate control  $IkUrl/"
if ($Port) { Write-Host "  hardware bridge                 $BridgeUrl  (health: $BridgeUrl/health)" }
if ($Dashboard) { Write-Host "  dashboard                       http://127.0.0.1:8000/so101_dashboard.html" }
if ($Detector -ne "none") { Write-Host "  object detector                 camera $Camera, mode $Detector" }
Write-Host "Close a window (or Ctrl+C in it) to stop that part."
