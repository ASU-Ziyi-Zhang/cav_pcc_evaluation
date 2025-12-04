@echo off
REM Check for cmake installation
WHERE cmake >NUL 2>&1
IF %ERRORLEVEL% NEQ 0 (
    echo cmake could not be found! Exiting controller installation. Please install/fix cmake and then try again.
    
    pause
    exit /b 1
)

REM PCC
cd pcc_codegen
call install.bat
call copy.bat

copy _cppwrapper.py ..\..\src\pcc_cppwrapper.py

cd ..

REM CAV Eco-approach
cd cavctrl_codegen
call install.bat
call copy.bat

copy _cwrapper.py ..\..\src\cav_cwrapper.py

cd ..