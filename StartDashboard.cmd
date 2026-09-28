@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0PortSim\Scripts\LaunchDashboard.ps1" -Background
if errorlevel 1 pause
