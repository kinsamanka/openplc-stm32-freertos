#ifndef CONFIG_H
#define CONFIG_H

#ifndef MAX_REQUEST
#define MAX_REQUEST             64
#endif

#ifndef MAX_RESPONSE
#define MAX_RESPONSE            64
#endif

#ifndef HOLDING_REG_COUNT
#define HOLDING_REG_COUNT       8
#endif

#ifndef INPUT_REG_COUNT
#define INPUT_REG_COUNT         8
#endif

#ifndef COIL_COUNT
#define COIL_COUNT              16
#endif

#ifndef DISCRETE_COUNT
#define DISCRETE_COUNT          8
#endif

#ifndef SLAVE_ADDRESS
#define SLAVE_ADDRESS           1
#endif

#ifndef SLAVE_BAUD_RATE
#define SLAVE_BAUD_RATE         19200
#endif

#ifndef SLAVE_PARITY
#define SLAVE_PARITY            USART_PARITY_EVEN
#endif

#define configMODBUS_PORT       502

/* MAC address configuration. */
#define configMAC_ADDR0         0x00
#define configMAC_ADDR1         0x12
#define configMAC_ADDR2         0x13
#define configMAC_ADDR3         0x10
#define configMAC_ADDR4         0x15
#define configMAC_ADDR5         0x11

/* IP address configuration. */
#define configIP_ADDR0          192
#define configIP_ADDR1          168
#define configIP_ADDR2          1
#define configIP_ADDR3          2

/* Netmask configuration. */
#define configNET_MASK0         255
#define configNET_MASK1         255
#define configNET_MASK2         255
#define configNET_MASK3         0

/* Start of bootloader region */
#ifndef BOOTLOADER_ADDR
#define BOOTLOADER_ADDR         0x1FFFF000UL
#endif

#endif
