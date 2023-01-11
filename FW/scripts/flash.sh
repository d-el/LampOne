#!/bin/bash

# save current dir
cwd=$(pwd)
# cd to script dir
DIR="$(cd "$(dirname "$0")" && pwd)"
cd $DIR

arm-none-eabi-gdb	-ex "target remote localhost:1234" \
					-ex "monitor reset halt" \
					-ex "load" \
					-ex "monitor reset" \
					-ex "quit" \
					../build/GClock.elf

# restore current dir
cd $cwd
