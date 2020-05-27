#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

#include "serial_num.h"
#include "uDACS_pins.h"
#include "subbus.h"

#ifdef uDACS_B

#define CMD_BASE_ADDR 0x30
#define CMD_HIGH_ADDR 0x30

extern subbus_driver_t sb_cmd;
#endif

#endif
