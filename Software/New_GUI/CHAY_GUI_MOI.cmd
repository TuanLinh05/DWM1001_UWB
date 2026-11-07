@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
    py -3.12 main_gui.py %*
) else (
    python main_gui.py %*
)
endlocal
