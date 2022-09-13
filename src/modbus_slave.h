#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

/*
 *
 * IX_TYPE = discrete input
 * QX_TYPE = coil
 * IW_TYPE = input register
 * QW_TYPE = holding register
 *
 */


/* read only */
#define SLAVE_DISCRETE_INPUT        { \
                                        { \
                                            .address = 1, \
                                            .index   = 0, \
                                            .count   = 4, \
                                            .type    = IX_TYPE, \
                                        }, \
                                        { \
                                            .address = 1, \
                                            .index   = 4, \
                                            .count   = 4, \
                                            .type    = IX_TYPE, \
                                        }, \
                                        { \
                                            .address = 1, \
                                            .index   = 0, \
                                            .count   = 8, \
                                            .type    = QX_TYPE, \
                                        }, \
                                    }

#define SLAVE_INPUT_REGISTER        { \
                                        { \
                                            .address = 1, \
                                            .index   = 0, \
                                            .count   = 8, \
                                            .type    = IW_TYPE, \
                                        }, \
                                        { \
                                            .address = 1, \
                                            .index   = 0, \
                                            .count   = 4, \
                                            .type    = QW_TYPE, \
                                        }, \
                                    }

/* write only */
#define SLAVE_COIL                  { \
                                        { \
                                            .address = 1, \
                                            .index   = 5, \
                                            .count   = 4, \
                                        }, \
                                        { \
                                            .address = 1, \
                                            .index   = 9, \
                                            .count   = 4, \
                                        }, \
                                    }

#define SLAVE_HOLDING_REGISTER      { \
                                        { \
                                            .address = 1, \
                                            .index   = 4, \
                                            .count   = 4, \
                                        }, \
                                        { \
                                            .address = 1, \
                                            .index   = 4, \
                                            .count   = 4, \
                                        }, \
                                    }

#endif
