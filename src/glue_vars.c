#include "config.h"
#include "iec_std_lib.h"

#define CHECK_BOUNDS(z, a, b, c, d) enum { var##d = 1/(!!(8*a+b < c)) }

#define ALLOC_QX(type, name, x, y) \
    CHECK_BOUNDS("error: Address exceeded COIL COUNT", \
            x, y, COIL_COUNT, name); \
    IEC_##type* name = &QX[x*8 + y];
#define ALLOC_IX(type, name, x, y) \
    CHECK_BOUNDS("error: Address exceeded INPUT COUNT", \
            x, y, DISCRETE_COUNT, name); \
    IEC_##type* name = &IX[x*8 + y];
#define ALLOC_IW(type, name, x) \
    CHECK_BOUNDS("error: Address exceeded INPUT REGISTER COUNT", \
            0, x, INPUT_REG_COUNT, name); \
    IEC_##type* name = &IW[x];
#define ALLOC_QW(type, name, x) \
    CHECK_BOUNDS("error: Address exceeded HOLDING REGISTER COUNT", \
            0, x, HOLDING_REG_COUNT, name); \
    IEC_##type* name = &QW[x];

#define __LOCATED_VAR(type, name, pre, suf, loc, ...) \
    ALLOC_##pre ## suf(type,name,loc, ## __VA_ARGS__)

IEC_UINT QW[HOLDING_REG_COUNT];
IEC_UINT IW[INPUT_REG_COUNT];
IEC_BOOL QX[COIL_COUNT];
IEC_BOOL IX[DISCRETE_COUNT];

#include "generated/LOCATED_VARIABLES.h"
