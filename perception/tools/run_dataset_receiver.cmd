@echo off
setlocal
cd /d "%~dp0.."
if not defined VISION_PYTHON set "VISION_PYTHON=python"
"%VISION_PYTHON%" -m rescue_vision.dataset_receive %*
if errorlevel 1 pause
