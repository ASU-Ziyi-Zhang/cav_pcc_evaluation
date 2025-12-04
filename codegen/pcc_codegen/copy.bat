@echo off
REM Check filename
SET DEST=..\..\

REM Default file
SET FILE=libpcc_so.so

REM Detect OS type
ver | findstr /i "Windows" >NUL
IF %ERRORLEVEL% EQU 0 (
    echo Windows detected.
    SET FILE=pcc_so.dll
) ELSE (
    echo Non-Windows OS detected.
    SET FILE=libpcc_so.so
)

REM Copy the file to specified directories
echo Copying file %FILE% to %DEST%
copy lib\%FILE% %DEST%

REM Check if copy was successful
IF %ERRORLEVEL% NEQ 0 (
    echo Copy Code: %ERRORLEVEL% - Unsuccessful
)