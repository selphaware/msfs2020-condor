@echo off
:iLoop
python stats.py
TIMEOUT /T 15 /NOBREAK
goto iLoop
