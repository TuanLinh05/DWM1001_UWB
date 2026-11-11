@echo off
wsl.exe -d Ubuntu-22.04 --cd "%~dp0IMGUI_UWB_CALIB" -- bash ./run_wsl.sh
if errorlevel 1 pause
