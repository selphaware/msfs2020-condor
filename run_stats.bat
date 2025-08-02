@echo off
:iLoop
echo.
python stats.py
TIMEOUT /T 15 /NOBREAK
goto iLoop
