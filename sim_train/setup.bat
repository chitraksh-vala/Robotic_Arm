@echo off
REM Setup script for SO Arm Simulator on Windows

echo.
echo ====================================
echo SO Arm Simulator - Setup Script
echo ====================================
echo.

REM Check Python installation
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.8+ from https://www.python.org/
    pause
    exit /b 1
)

echo [1/3] Installing Python dependencies...
pip install -r requirements.txt
if errorlevel 1 (
    echo ERROR: Failed to install dependencies
    pause
    exit /b 1
)
echo [OK] Dependencies installed

echo.
echo [2/3] Creating directories...
if not exist recordings mkdir recordings
if not exist mujoco_model mkdir mujoco_model
echo [OK] Directories created

echo.
echo [3/3] Checking MuJoCo installation...
python -c "import mujoco; print(f'MuJoCo version: {mujoco.__version__}')" >nul 2>&1
if errorlevel 1 (
    echo WARNING: MuJoCo not found or import failed
    echo Please make sure MuJoCo is properly installed
    echo Visit: https://mujoco.org/
) else (
    echo [OK] MuJoCo found
)

echo.
echo ====================================
echo Setup Complete!
echo ====================================
echo.
echo Next steps:
echo   1. Connect your DualSense controller
echo   2. Run: cd src
echo   3. Run: python simulator.py
echo.
echo For detailed instructions, see README.md
echo.
pause
