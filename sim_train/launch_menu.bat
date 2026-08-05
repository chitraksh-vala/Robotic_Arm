@echo off
REM Launch Menu for SO Arm Simulator

:menu
cls
echo.
echo ====================================
echo SO Arm Simulator - Launch Menu
echo ====================================
echo.
echo Select an option:
echo.
echo [1] Run Simulator (Interactive)
echo [2] Run Simulator (Headless)
echo [3] List Recordings
echo [4] Analyze Recording
echo [5] Playback Recording
echo [6] Training Example
echo [7] Validate Installation
echo [8] Open Documentation
echo [9] Exit
echo.
set /p choice="Enter your choice (1-9): "

if "%choice%"=="1" goto simulator
if "%choice%"=="2" goto headless
if "%choice%"=="3" goto list_recordings
if "%choice%"=="4" goto analyze
if "%choice%"=="5" goto playback
if "%choice%"=="6" goto training
if "%choice%"=="7" goto validate
if "%choice%"=="8" goto docs
if "%choice%"=="9" goto exit

echo Invalid choice. Please try again.
timeout /t 2 >nul
goto menu

:simulator
cls
echo Starting interactive simulator...
cd src
python simulator.py
cd ..
pause
goto menu

:headless
cls
set /p duration="Enter simulation duration (seconds, default 60): "
if "%duration%"=="" set duration=60
cd src
python simulator.py --headless --duration %duration%
cd ..
pause
goto menu

:list_recordings
cls
cd src
python motion_utils.py list
cd ..
pause
goto menu

:analyze
cls
cd src
echo.
echo Available recordings:
python motion_utils.py list
echo.
set /p file="Enter recording filename or full path: "
python motion_utils.py analyze %file%
cd ..
pause
goto menu

:playback
cls
cd src
echo.
echo Available recordings:
python playback.py --list
echo.
set /p file="Enter recording filename or full path: "
python playback.py %file%
cd ..
pause
goto menu

:training
cls
cd src
python training_example.py
cd ..
pause
goto menu

:validate
cls
python validate.py
pause
goto menu

:docs
cls
echo Opening documentation...
start README.md
start QUICKSTART.md
goto menu

:exit
exit /b 0
