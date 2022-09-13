#include "config.h"
#include "iec_std_lib.h"

#define BIT_POS(x, y)               (x*8 + y)
#define NORM(x, y)                  ((x >= 100) ? (x - 100 + y) : x)
#define CHECK_BOUNDS(z, a, b, c)    enum { var##c = 1/(!!(a < b)) }

#define ALLOC_QX(type, name, x, y) \
    CHECK_BOUNDS("error: Address exceeded COIL COUNT", \
            BIT_POS(NORM(x, QX_BASE), y), QX_COUNT, name); \
    IEC_##type* name = &QX[BIT_POS(NORM(x, QX_BASE), y)];

#define ALLOC_IX(type, name, x, y) \
    CHECK_BOUNDS("error: Address exceeded INPUT COUNT", \
            BIT_POS(NORM(x, IX_BASE), y), IX_COUNT, name); \
    IEC_##type* name = &IX[BIT_POS(NORM(x, IX_BASE), y)];

#define ALLOC_IW(type, name, x) \
    CHECK_BOUNDS("error: Address exceeded INPUT REGISTER COUNT", \
            NORM(x, IW_BASE), IW_COUNT, name); \
    IEC_##type* name = &IW[NORM(x, IW_BASE)];

#define ALLOC_QW(type, name, x) \
    CHECK_BOUNDS("error: Address exceeded HOLDING REGISTER COUNT", \
            NORM(x, QW_BASE), QW_COUNT, name); \
    IEC_##type* name = &QW[NORM(x, QW_BASE)];

#define __LOCATED_VAR(type, name, pre, suf, loc, ...) \
    ALLOC_##pre ## suf(type,name,loc, ## __VA_ARGS__)

IEC_UINT QW[QW_COUNT];
IEC_UINT IW[IW_COUNT];
IEC_BOOL QX[QX_COUNT];
IEC_BOOL IX[IX_COUNT];

#include "generated/LOCATED_VARIABLES.h"
