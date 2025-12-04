#!/bin/sh

## Check for cmake install
if ! command -v cmake 2>&1 >/dev/null
then
    echo "cmake could not be found! Exiting controller installation. Please install/fix cmake and then try again."
    
else
    ## PCC
    cd pcc_codegen
    . install.sh
    . copy.sh

    cp _cppwrapper.py ../../src/pcc_cppwrapper.py

    cd ..

    ## CAV Eco-approach
    cd cavctrl_codegen
    . install.sh
    . copy.sh

    cp _cwrapper.py ../../src/cav_cwrapper.py

    cd ..

fi