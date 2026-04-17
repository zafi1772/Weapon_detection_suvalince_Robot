@echo off
title Installation
color 0A

echo.
echo  === ROVER + WEAPON DETECTION - INSTALLATION ===
echo.

echo  [0/5] Checking Python...
python --version
if errorlevel 1 goto NO_PYTHON
echo  [OK]  Python found.
echo.
goto PYTHON_OK

:NO_PYTHON
echo.
echo  ERROR: Python is not installed or not in PATH.
echo  Download Python 3.10+ from: https://www.python.org/downloads/
echo  During install tick "Add Python to PATH"
echo.
pause
exit /b 1

:PYTHON_OK
echo  [1/5] Creating virtual environment (.venv)...
if exist ".venv" goto VENV_EXISTS
python -m venv .venv
if errorlevel 1 goto VENV_FAIL
echo  [OK]  Virtual environment created.
goto VENV_DONE

:VENV_EXISTS
echo  [OK]  Virtual environment already exists.

:VENV_DONE
echo.

echo  [2/5] Upgrading pip...
.venv\Scripts\python.exe -m pip install --upgrade pip --quiet
echo  [OK]  pip ready.
echo.

echo  [3/5] Installing packages...
echo        flask, opencv-python, numpy, pyserial, ultralytics
echo.
echo        NOTE: First run downloads PyTorch ~800MB. May take 5-15 min.
echo.
.venv\Scripts\pip.exe install flask opencv-python numpy pyserial ultralytics --upgrade
if errorlevel 1 goto INSTALL_FAIL
echo.
echo  [OK]  All packages installed.
echo.

echo  [4/5] Verifying imports...
.venv\Scripts\python.exe -c "import flask; print('  flask ' + flask.__version__ + ' OK')"
if errorlevel 1 goto VERIFY_FAIL
.venv\Scripts\python.exe -c "import cv2; print('  opencv ' + cv2.__version__ + ' OK')"
if errorlevel 1 goto VERIFY_FAIL
.venv\Scripts\python.exe -c "import numpy; print('  numpy ' + numpy.__version__ + ' OK')"
if errorlevel 1 goto VERIFY_FAIL
.venv\Scripts\python.exe -c "import serial; print('  pyserial ' + serial.__version__ + ' OK')"
if errorlevel 1 goto VERIFY_FAIL
.venv\Scripts\python.exe -c "import ultralytics; print('  ultralytics ' + ultralytics.__version__ + ' OK')"
if errorlevel 1 goto VERIFY_FAIL
echo.

echo  [5/5] Checking model files...
if exist "weapon_detection_yolov12.pt" (
    echo  [OK]  weapon_detection_yolov12.pt found.
) else (
    echo  [WARN] weapon_detection_yolov12.pt NOT found - place it here before running.
)
if exist "yolo12n.pt" (
    echo  [OK]  yolo12n.pt found.
) else (
    echo  [INFO] yolo12n.pt missing - will auto-download on first run.
)
echo.

echo  === INSTALLATION COMPLETE - Run run.bat to start ===
echo.
pause
exit /b 0

:VENV_FAIL
echo  ERROR: Failed to create virtual environment.
pause
exit /b 1

:INSTALL_FAIL
echo  ERROR: Package install failed. Check internet connection and retry.
pause
exit /b 1

:VERIFY_FAIL
echo  ERROR: Import check failed. Re-run install.bat.
pause
exit /b 1
