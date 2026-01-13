. ~/esp/v5.5.1/export.sh
export IDF_TARGET=esp32h2
#idf.py gdb
#xtensa-esp32-elf-gdb -q -x build/gdbinit/symbols -x build/gdbinit/prefix_map -x build/gdbinit/connect build/blink.elf
openocd -f board/esp32h2-builtin.cfg
