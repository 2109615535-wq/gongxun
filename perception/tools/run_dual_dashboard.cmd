@echo off
setlocal
cd /d "%~dp0.."
if not defined VISION_PYTHON set "VISION_PYTHON=python"
echo Open http://127.0.0.1:8080 in your browser.
"%VISION_PYTHON%" -m rescue_vision.dashboard %*
if errorlevel 1 pause
