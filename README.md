# openplc-stm32-freertos
OpenPLC on STM32 using FreeRTOS

### Prerequisites
- git
- [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation/index.html)
- st-link programmer

## Instructions
- Clone repo
   ``` bash
   git clone https://github.com/kinsamanka/openplc-stm32-freertos.git
   cd openplc-stm32-freertos
   git submodule init
   git submodule update --depth 1
   ```
- Save plc program as `plc_prog.st` under the current directory
- Compile firmware
   ```
   pio run
   ```
- Flash firmware
   ```
   pio run -t upload
   ```
