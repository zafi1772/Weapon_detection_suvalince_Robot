@echo off
title Rover + Weapon Detection System
color 0A

echo.
echo  === ROVER + WEAPON DETECTION SYSTEM ===
echo.

if not exist ".venv\Scripts\python.exe" goto NO_VENV
if not exist "weapon_detection_yolov12.pt" goto NO_MODEL

:: To force a specific Arduino port uncomment the next line:
:: set ARDUINO_PORT=COM5

echo  Starting app... browser opens at http://localhost:5000 in 4 sec.
echo  Press Ctrl+C to stop.
echo.
echo  Local   : http://localhost:5000
echo  Network : http://%COMPUTERNAME%:5000
echo.

start /min "" cmd /c "ping -n 5 127.0.0.1 >nul & start http://localhost:5000"

.venv\Scripts\python.exe app.py

echo.
echo  Application stopped.
pause
exit /b 0

:NO_VENV
echo  ERROR: .venv not found. Run install.bat first.
echo.
pause
exit /b 1

:NO_MODEL
echo  ERROR: weapon_detection_yolov12.pt not found. Place it in this folder.
echo.
pause
exit /b 1
