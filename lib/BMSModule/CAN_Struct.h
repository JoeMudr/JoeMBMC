#pragma once
#include <Arduino.h>
#define CAN_Struct_size 30

typedef struct {
    CAN_message_t Frame[CAN_Struct_size];
} CAN_Struct;

CAN_Struct clearCANStruct(); // returns an empty initialized CAN_Struct

inline CAN_Struct clearCANStruct(){
    CAN_Struct msg;
    for(byte i = 0; i < CAN_Struct_size; i++){
        msg.Frame[i].flags.extended = 0;
        msg.Frame[i].len = 0;
        msg.Frame[i].id = 0;
        for (byte x = 0; x < 8; x++){
            msg.Frame[i].buf[x] = 0;
        }
    }
    return msg;
}