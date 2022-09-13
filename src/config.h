#ifndef CONFIG_H
#define CONFIG_H

#define NUM(a) (sizeof(a) / sizeof(*a))
#define ct_assert(e) ((void)sizeof(char[1 - 2*!(e)]))

#ifndef MAX_REQUEST
#if RAM_SIZE > 8192
#define MAX_REQUEST             256
#else
#define MAX_REQUEST             64
#endif
#endif

#ifndef MAX_RESPONSE
#if RAM_SIZE > 8192
#define MAX_RESPONSE            256
#else
#define MAX_RESPONSE            64
#endif
#endif

#ifndef HOLDING_REG_COUNT
#define HOLDING_REG_COUNT       16
#endif

#ifndef INPUT_REG_COUNT
#define INPUT_REG_COUNT         8
#endif

#ifndef COIL_COUNT
#define COIL_COUNT              16
#endif

#ifndef DISCRETE_COUNT
#define DISCRETE_COUNT          16
#endif

#ifdef MODBUS_MASTER

#undef MODBUS_MASTER
#define MODBUS_MASTER           1

#ifndef SLAVE_HOLDING_REG_COUNT
#define SLAVE_HOLDING_REG_COUNT 16
#endif

#ifndef SLAVE_INPUT_REG_COUNT
#define SLAVE_INPUT_REG_COUNT   16
#endif

#ifndef SLAVE_COIL_COUNT
#define SLAVE_COIL_COUNT        16
#endif

#ifndef SLAVE_DISCRETE_COUNT
#define SLAVE_DISCRETE_COUNT    16
#endif

#define QW_BASE                 HOLDING_REG_COUNT
#define IW_BASE                 INPUT_REG_COUNT
#define QX_BASE                 (COIL_COUNT / 8)
#define IX_BASE                 (DISCRETE_COUNT / 8)

#else

#define MODBUS_MASTER           0

#ifdef SLAVE_HOLDING_REG_COUNT
#undef SLAVE_HOLDING_REG_COUNT
#endif

#ifdef SLAVE_INPUT_REG_COUNT
#undef SLAVE_INPUT_REG_COUNT
#endif

#ifdef SLAVE_COIL_COUNT
#undef SLAVE_COIL_COUNT
#endif

#ifdef SLAVE_DISCRETE_COUNT
#undef SLAVE_DISCRETE_COUNT
#endif

#define SLAVE_HOLDING_REG_COUNT 0
#define SLAVE_INPUT_REG_COUNT   0
#define SLAVE_COIL_COUNT        0
#define SLAVE_DISCRETE_COUNT    0

#define QW_BASE                 0
#define IW_BASE                 0
#define QX_BASE                 0
#define IX_BASE                 0

#endif

#define QW_COUNT                (HOLDING_REG_COUNT + SLAVE_HOLDING_REG_COUNT)
#define IW_COUNT                (INPUT_REG_COUNT + SLAVE_INPUT_REG_COUNT)
#define QX_COUNT                (COIL_COUNT + SLAVE_COIL_COUNT)
#define IX_COUNT                (DISCRETE_COUNT + SLAVE_DISCRETE_COUNT)

#ifndef SLAVE_ADDRESS
#define SLAVE_ADDRESS           1
#endif

#ifndef SLAVE_BAUD_RATE
#define SLAVE_BAUD_RATE         57600
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

#ifndef BOOTLOADER_MAGIC
#define BOOTLOADER_MAGIC        ("\xff" "\xff" "BOOTLOADER" "\xff" "\xff")
#endif

#endif
