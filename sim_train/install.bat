@echo off
REM Simple installer for SO Arm Simulator

echo Installing SO Arm Simulator dependencies...
echo.

python -m pip install --upgrade pip setuptools wheel
if errorlevel 1 (
    echo Warning: pip upgrade had issues, continuing anyway...
)

echo.
echo Installing packages from requirements.txt...
pip install -r requirements.txt

if errorlevel 1 (
    echo.
    echo Error installing dependencies. Try:
    echo   pip install --no-cache-dir -r requirements.txt
    pause
    exit /b 1
)

echo.
echo ✓ Installation complete!
echo.
echo Next: python validate.py
pause
