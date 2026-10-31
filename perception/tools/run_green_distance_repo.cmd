@echo off
python "%~dp0detect_green_distance.py" --weights "%~dp0..\models\best.pt" --check-report "%~dp0..\calibration\20261030_180403_198681_check_report.json" --mapping "%~dp0..\calibration\20261030_180026_563174_unvalidated.json" %*
exit /b %errorlevel%
