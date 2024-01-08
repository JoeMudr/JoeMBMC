#pragma once

#include <Arduino.h>

//Set to the proper port for your USB connection - SerialUSB on Due (Native) or Serial for Due (Programming) or Teensy
#define SERIALCONSOLE Serial

//Define this to be the serial port the Tesla BMS modules are connected to.
//On the Due you need to use a USART port (Serial1, Serial2, Serial3) and update the call to serialSpecialInit if not Serial1
//Serial3 for teensy
#define SERIALBMS  Serial2

// Secondary CAN / Serial
#define DisplaySerial Serial4

#define REG_DEV_STATUS      0
#define REG_GPAI            1
#define REG_VCELL1          3
#define REG_VCELL2          5
#define REG_VCELL3          7
#define REG_VCELL4          9
#define REG_VCELL5          0xB
#define REG_VCELL6          0xD
#define REG_TEMPERATURE1    0xF
#define REG_TEMPERATURE2    0x11
#define REG_ALERT_STATUS    0x20
#define REG_FAULT_STATUS    0x21
#define REG_COV_FAULT       0x22
#define REG_CUV_FAULT       0x23
#define REG_ADC_CTRL        0x30
#define REG_IO_CTRL         0x31
#define REG_BAL_CTRL        0x32
#define REG_BAL_TIME        0x33
#define REG_ADC_CONV        0x34
#define REG_ADDR_CTRL       0x3B

#define MAX_MODULE_ADDR     0x3E

#define EEPROM_VERSION      0x20   //update any time EEPROM struct below is changed.
#define EEPROM_PAGE         0

typedef struct {
  byte version;
  byte checksum;
  byte batteryID;  //which battery ID should this board associate as on the CAN bus
  float OverVSetpoint; // in mV
  float UnderVSetpoint; // in mV
  float DischHys;
  float ChargeVSetpoint;
  float ChargeVsetpoint_toll;
  float UnderVDerateSetpoint;
  float ChargeHys;
  float StoreVsetpoint;
  float WarnOff;
  float OverTSetpoint;
  float UnderTSetpoint;
  float UnderTDerateSetpoint;
  float OverTDerateSetpoint;
  float WarnToff;
  float CellGap;
  byte IgnoreTemp;
  float IgnoreVolt;
  float balanceVoltage;
  float balanceHyst;
  int Scells;
  int Pstrings;
  int CAP; // in Ah
  int designCAP; // in Ah
  int CAP_Wh;
  uint16_t ChargerChargeCurrentMax;
  uint16_t PackChargeCurrentMax;
  uint16_t chargecurrent2max;
  uint16_t chargecurrentend;
  uint16_t PackDisCurrentMax ;
  int socvolt[4];
  bool invertcur;
  int cursens;
  bool voltsoc;
  int Pretime;
  int Precurrent;
  float convhigh;
  float convlow;
  int32_t changecur;
  uint16_t offset1;
  uint16_t offset2;
  int balanceDuty;
  bool ESSmode;
  byte gaugelow;
  byte gaugehigh;
  int ncur;
  int chargertype;
  byte nchargers;
  uint16_t CurDead;
  float DisTaper;
  byte ChargerDirect;
  int mctype;
  byte SerialCan; // bool
  unsigned long error_delay;
  int Temp_Cap_Map[2][5] = {}; // [0][x] Temperature; [1][x] Percentage of Capacity
  byte Out_Map[2][9] = {}; // OUT1 = Out_Map[x][1]...; [0][x] Functions; [1][x] PWM
  uint32_t CAN1_Speed;
  uint32_t CAN2_Speed;
  uint16_t CAN1_Interval;
  uint16_t CAN2_Interval;
  byte CAN_Map[2][5]; // index[0]: 0 = BMC-Output; 1 = BMS Communication; 2 = Chargers; 3 = Current Sensors; 4 = MotorController // Values: 0 = not set; 1 = Can1; 2 = Can2; 3 = Can1 & Can2
} EEPROMSettings;
