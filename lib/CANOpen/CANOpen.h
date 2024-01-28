#pragma once
#include "config.h"
#include <FlexCAN_T4.h>

//CANOpen States
#define co_NMT_operational 0x01
#define co_NMT_stop 0x02
#define co_NMT_preop 0x80
#define co_NMT_resetapp 0x81
#define co_NMT_resetcom 0x82

#define co_state_bootup 0x00
#define co_state_stopped 0x04
#define co_state_op 0x05
#define co_state_preop 0x7f