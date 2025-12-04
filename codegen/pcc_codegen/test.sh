#!/bin/sh

# Check if python or python3 are installed on path and are executable
# [ -x "$(command -v foo)" ]

# echo "$(dirname "$0")"
# cd "$(dirname "$0")"

if [ -x "$(command -v python)" ]; then
    python test.py

elif [ -x "$(command -v python3)" ]; then
    python3 test.py

else
    echo "Could not find Python in path to run test.py"

fi