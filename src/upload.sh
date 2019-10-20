#!/bin/bash
/home/corentin/.platformio/packages/tool-stm32duino/stm32flash/stm32flash -w "/home/corentin/Documents/PlatformIO/Projects/Drone_pilot/.pio/build/bluepill_f103c8/firmware.bin" -v -g 0x0 /dev/ttyUSB0 -b 115200
