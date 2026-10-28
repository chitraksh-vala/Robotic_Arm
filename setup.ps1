<#
.SYNOPSIS
    Set up the Python environment for the SO-ARM101 vision pick-and-place stack.

.DESCRIPTION
    Creates a virtual environment in .venv (next to this script), installs
    requirements.txt and, unless -SimOnly, requirements-hardware.txt (LeRobot,
    for the real arm). Then it replaces LeRobot's window-less OpenCV build with
    the normal one and checks that everything imports and the MuJoCo scene loads.

    Run from PowerShell in this folder. If scripts are blocked, use:
        powershell -ExecutionPolicy Bypass -File .\setup.ps1

.PARAMETER SimOnly
    Skip LeRobot (no real arm; avoids the large PyTorch download).

.PARAMETER Python
    Python interpreter to build the environment from. Default: the newest
    suitable one found (py launcher 3.13/3.12, then python on PATH).

.PARAMETER VenvPath
    Where to create the environment. Default: .venv in this folder.

.PARAMETER Recreate
    Delete and rebuild an existing environment.

.EXAMPLE
    .\setup.ps1
.EXAMPLE
    .\setup.ps1 -SimOnly
.EXAMPLE
    .\setup.ps1 -Python "C:\Program Files\Python313\python.exe" -Recreate
#>
[CmdletBinding()]
param(
    [switch]$SimOnly,
    [string]$Python = "",
    [string]$VenvPath = "",
    [switch]$Recreate
)

$ErrorActionPreference = "Stop"
$Root = $PSScriptRoot
if (-not $VenvPath) { $VenvPath = Join-Path $Root ".venv" }
# LeRobot 0.6 requires Python 3.12+; the simulation alone works from 3.10.
$MinMinor = if ($SimOnly) { 10 } else { 12 }
$MaxMinor = 13

function Write-Step([string]$Text) { Write-Host ""; Write-Host "==> $Text" -ForegroundColor Cyan }

function Invoke-Checked([string]$Exe, [string[]]$Arguments) {
    & $Exe @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed (exit $LASTEXITCODE): $Exe $($Arguments -join ' ')"
    }
}

function Get-PythonMinor([string]$Exe, [string[]]$Prefix) {
    # Returns the 3.x minor version, or -1 if this is not a usable Python 3.
    try {
        $out = & $Exe @Prefix -c "import sys; print(sys.version_info[0], sys.version_info[1])"
    } catch {
        return -1
    }
    if ($LASTEXITCODE -ne 0 -or -not $out) { return -1 }
    $parts = ("$out").Trim() -split "\s+"
    if ($parts.Count -ne 2 -or $parts[0] -ne "3") { return -1 }
    return [int]$parts[1]
}

# ---- 1. find a Python ------------------------------------------------------
Write-Step "Looking for Python 3.$MinMinor-3.$MaxMinor"
$candidates = @()
if ($Python) {
    $candidates += ,@($Python, @())
} else {
    if (Get-Command py -ErrorAction SilentlyContinue) {
        for ($minor = $MaxMinor; $minor -ge $MinMinor; $minor--) {
            $candidates += ,@("py", @("-3.$minor"))
        }
    }
    foreach ($name in @("python", "python3")) {
        if (Get-Command $name -ErrorAction SilentlyContinue) { $candidates += ,@($name, @()) }
    }
}
$BaseExe = $null; $BasePrefix = @(); $BaseMinor = -1
foreach ($candidate in $candidates) {
    $minor = Get-PythonMinor $candidate[0] $candidate[1]
    if ($minor -ge $MinMinor -and $minor -le $MaxMinor) {
        $BaseExe = $candidate[0]; $BasePrefix = $candidate[1]; $BaseMinor = $minor
        break
    }
}
if (-not $BaseExe) {
    $need = if ($SimOnly) { "3.10 to 3.13" } else { "3.12 or 3.13 (LeRobot needs 3.12+)" }
    throw "No suitable Python found. Install Python $need from python.org (tick 'Add to PATH'), " +
          "or pass -Python <path-to-python.exe>. Use -SimOnly to allow Python 3.10/3.11."
}
Write-Host "Using Python 3.$BaseMinor ($BaseExe $($BasePrefix -join ' '))"

# ---- 2. virtual environment ------------------------------------------------
$VenvPy = Join-Path $VenvPath "Scripts\python.exe"
if ($Recreate -and (Test-Path $VenvPath)) {
    Write-Step "Removing the existing environment $VenvPath"
    Remove-Item -Recurse -Force $VenvPath
}
if (-not (Test-Path $VenvPy)) {
    Write-Step "Creating the virtual environment $VenvPath"
    Invoke-Checked $BaseExe ($BasePrefix + @("-m", "venv", $VenvPath))
} else {
    Write-Step "Reusing the virtual environment $VenvPath (use -Recreate to rebuild)"
}
$venvMinor = Get-PythonMinor $VenvPy @()
if ($venvMinor -lt $MinMinor) {
    throw "The existing environment uses Python 3.$venvMinor, but 3.$MinMinor+ is needed. Run again with -Recreate."
}

# ---- 3. packages -----------------------------------------------------------
Write-Step "Upgrading pip"
Invoke-Checked $VenvPy @("-m", "pip", "install", "--upgrade", "pip")

$reqFile = if ($SimOnly) { "requirements.txt" } else { "requirements-hardware.txt" }
Write-Step "Installing $reqFile$(if (-not $SimOnly) { ' (includes PyTorch via LeRobot: this can take a while)' })"
Invoke-Checked $VenvPy @("-m", "pip", "install", "-r", (Join-Path $Root $reqFile))

# LeRobot pulls opencv-python-headless, which overwrites cv2 with a build that
# cannot open windows (the detector would fail at cv2.imshow).
$installed = & $VenvPy -m pip list --format=freeze
if ($installed -match "^opencv-python-headless==") {
    Write-Step "Replacing opencv-python-headless with the windowed OpenCV build"
    Invoke-Checked $VenvPy @("-m", "pip", "uninstall", "-y", "opencv-python-headless")
    Invoke-Checked $VenvPy @("-m", "pip", "install", "--force-reinstall", "--no-deps", "opencv-python>=4.9,<4.14")
}

# ---- 4. checks -------------------------------------------------------------
Write-Step "Checking the installation"
$scene = Join-Path $Root "sim2hw\scene_full.xml"
$check = "import mujoco, numpy, cv2, PIL; " +
         "m = mujoco.MjModel.from_xml_path(r'$scene'); " +
         "gui = any(('Win32 UI' in l or 'GTK' in l) and 'YES' in l for l in cv2.getBuildInformation().splitlines()); " +
         "print('  mujoco', mujoco.__version__, '| numpy', numpy.__version__, '| opencv', cv2.__version__, '| Pillow', PIL.__version__); " +
         "print('  MuJoCo scene loads:', m.nbody, 'bodies'); " +
         "print('  OpenCV can open windows:', gui); " +
         "raise SystemExit(0 if gui else 3)"
& $VenvPy -c $check
if ($LASTEXITCODE -eq 3) {
    Write-Warning "OpenCV has no window support; the detector window will not open. Run: $VenvPy -m pip install --force-reinstall opencv-python"
} elseif ($LASTEXITCODE -ne 0) {
    throw "The import check failed (see the error above)."
}
if (-not $SimOnly) {
    Invoke-Checked $VenvPy @("-c", "from lerobot.motors.feetech import FeetechMotorsBus; import lerobot; print('  LeRobot', lerobot.__version__, 'with the Feetech driver')")
}

# ---- 5. next steps ---------------------------------------------------------
$scripts = Join-Path $VenvPath "Scripts"
Write-Host ""
Write-Host "Setup complete." -ForegroundColor Green
Write-Host ""
Write-Host "Next steps:"
if (-not $SimOnly) {
    Write-Host "  1. Find the arm's serial port:      & '$scripts\lerobot-find-port.exe'"
    Write-Host "  2. Calibrate the arm once (if not done before; the bridge uses id 'follower'):"
    Write-Host "       & '$scripts\lerobot-calibrate.exe' --robot.type=so101_follower --robot.port=COM4 --robot.id=follower"
    Write-Host "  3. Start everything:                .\run.ps1 -Port COM4"
} else {
    Write-Host "  Start the simulation and detector:  .\run.ps1"
}
Write-Host ""
Write-Host "See README.md (quick start), sim2hw\README.md and CV-seg\Robotic_Arm-main\README.md."
