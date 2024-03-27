#pragma once
#include <Arduino.h>
#define CAN_Struct_size 30

typedef struct {
    CAN_message_t Frame[CAN_Struct_size];
} CAN_Struct;
