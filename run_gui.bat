@echo off
title ESP32-S3 Robot Control GUI Launcher
color 0A

echo.
echo  =========================================
echo  ESP32-S3 2-DOF Robotic Arm Control GUI
echo  =========================================
echo.

echo [1] Install Dependencies
echo [2] Run Robot Control GUI
echo [3] Exit
echo.

set /p choice="Select option (1-3): "

if "%choice%"=="1" goto install
if "%choice%"=="2" goto run
if "%choice%"=="3" goto exit
goto menu

:install
echo.
echo Installing dependencies...
python install_requirements.py
pause
goto menu

:run
echo.
echo Starting Robot Control GUI...
python robot_control_gui.py
pause
goto menu

:exit
exit

:menu
goto menu