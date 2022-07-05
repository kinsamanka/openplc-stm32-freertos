#ifndef W5500_CONNECTOR_H
#define W5500_CONNECTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    int w5500_init(const uint8_t * address);
    uint16_t w5500_sendFrame(const uint8_t * data, uint16_t datalen);
    uint16_t w5500_readFrame(uint8_t * buffer, uint16_t bufsize);

#ifdef __cplusplus
}
#endif
#endif
