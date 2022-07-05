#ifndef __UIP_CONF_H__
#define __UIP_CONF_H__

#include <inttypes.h>

typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef unsigned short uip_stats_t;

#define UIP_CONF_MAX_CONNECTIONS    4
#define UIP_CONF_MAX_LISTENPORTS    4
#define UIP_CONF_BUFFER_SIZE        420
#define UIP_CONF_BYTE_ORDER         UIP_LITTLE_ENDIAN
#define UIP_CONF_LOGGING            0
#define UIP_CONF_UDP                0
#define UIP_CONF_UDP_CHECKSUMS      1
#define UIP_CONF_STATISTICS         1
#define UIP_CONF_IPV6               0
#define UIP_ARCH_ADD32              0
#define UIP_ARCH_CHKSUM             0

#include "modbus_tcp.h"

#endif                          /* __UIP_CONF_H__ */
