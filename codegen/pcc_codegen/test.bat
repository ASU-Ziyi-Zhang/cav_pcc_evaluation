@echo off
REM Check if Python or Python3 are installed on path and are executable

WHERE python >NUL 2>&1
IF %ERRORLEVEL% EQU 0 (
    python test.py
    exit /b 0
)

WHERE python3 >NUL 2>&1
IF %ERRORLEVEL% EQU 0 (
    python3 test.py
    exit /b 0
)

echo Could not find Python in path to run test.py
exit /b 1