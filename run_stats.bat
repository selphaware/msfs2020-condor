@echo off
:iLoop
python stats.py
TIMEOUT /T 5 /NOBREAK
goto iLoop
