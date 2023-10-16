# LampOne

## Overview
Firmware of led matrix driver:  
   * constant current mode  
   * MCU: STM32G030F  
   * RS485 ModBus  
   * smooth button control  

## Requirements
toolchain arm-none-eabi 10.3 or higher  
gcc / g++ 7.5.0 or higher  
cmake 3.14 or higher  
libjsoncpp-dev  

## Build project
>mkdir build  
>cd build  
>cmake .. -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake  
>make -j8  

### Start debug server
>./scripts/gdb-serv.sh openocd-jlink

### Start debug client
>./scripts/gdb-client.sh
