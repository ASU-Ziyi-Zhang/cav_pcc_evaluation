
# Check filename
DEST=../../

FILE=libcav_so.so # Default

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Linux GNU detected."
    FILE=libcav_so.so

elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Mac OSX detected."
    FILE=libcav_so.so

elif [[ "$OSTYPE" == "cygwin" ]]; then
    echo "POSIX/Linux for Windows detected."
    FILE=libcav_so.so

elif [[ "$OSTYPE" == "msys" ]]; then
    echo "msys detected."
    FILE=cav_so.dll

elif [[ "$OSTYPE" == "win32" ]]; then
    echo "win32 detected."
    FILE=cav_so.dll

elif [[ "$OSTYPE" == "freebsd"* ]]; then
    echo "freebsd detected."
    FILE=libcav_so.so
    
fi

# Copy the file to specified directories
echo "Copying file $FILE to $DEST"

cp lib/$FILE $DEST

# Check if it was successful
if [ $? != 0 ]; then # last command: echo
    echo "Copy Code: $? - Unsuccessful" # last command: [
fi