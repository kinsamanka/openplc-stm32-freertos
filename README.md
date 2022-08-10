# openplc-stm32-freertos
OpenPLC on STM32 using FreeRTOS

### Prerequisites
- git
- [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation/index.html)
- serial port

## Instructions for BluePill boards
- Clone repo
   ``` bash
   git clone https://github.com/kinsamanka/openplc-stm32-freertos.git
   cd openplc-stm32-freertos
   git submodule init
   git submodule update --depth 1
   ```
- Reset board to DFU mode (set BOOT0 to 1)
- Compile and flash bootloader
   ```
   pio run -e bootloader -t upload
   ```
- Save plc program as `plc_prog.st` under the current directory
- Compile and flash firmware
   ```
   pio run -e bluepill -t upload
   ```
