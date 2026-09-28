@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0PortSim\Scripts\LaunchPortSim.ps1"
if errorlevel 1 pause
