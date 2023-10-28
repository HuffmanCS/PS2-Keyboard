This /src directory contains the source code for the "PS/2 Keyboard From Scratch" project, as well as some of the files generated during compilation by SDCC for reference. 

Small Device C Compiler (SDCC) must be installed before compilation is possible.
Minipro software and a compatible EEPROM programmer should be considered also for uploading the binary into the MCU.

Compilation may be done in Terminal via...
sdcc keyboard.c

The resulting hex file may be burnt into the MCU in Terminal via...
minipro -w keyboard.ihx -p AT89S52@DIP40 

