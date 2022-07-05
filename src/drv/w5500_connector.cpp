#include "w5500_connector.h"
#include "w5500.h"

static Wiznet5500 w5500;

int w5500_init(const uint8_t * address)
{
    return w5500.begin(address);
}

uint16_t w5500_sendFrame(const uint8_t * data, uint16_t datalen)
{
    return w5500.sendFrame(data, datalen);
}

uint16_t w5500_readFrame(uint8_t * buffer, uint16_t bufsize)
{
    return w5500.readFrame(buffer, bufsize);
}
