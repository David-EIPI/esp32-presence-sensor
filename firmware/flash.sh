. ~/esp/v5.5.1/export.sh
export IDF_TARGET=esp32h2
if [ "$1" == "-e" ]; then
    idf.py -p /dev/ttyACM0 erase-flash
fi
idf.py -p /dev/ttyACM0 flash && \
idf.py -p /dev/ttyACM0 monitor


