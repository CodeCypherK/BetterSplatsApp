@echo off
REM Capture receiver for the BetterSplats iPhone app — same protocol as
REM PhoneStreamer. Sessions land in .\incoming\<name>\
cd /d "%~dp0"
py capture_receiver.py
if errorlevel 1 python capture_receiver.py
pause
