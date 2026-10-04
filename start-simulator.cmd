@echo off
setlocal
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0start-simulator.ps1" %*
if errorlevel 1 (
    echo.
    echo The simulator could not start. Read the message above for details.
    pause
    exit /b 1
)
