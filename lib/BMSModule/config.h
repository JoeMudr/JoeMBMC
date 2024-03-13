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

//Tesla BMS_Module Comms
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

//VW BMS_Module Comms
//??

// Values for Modules
#define MAX_MODULE_ADDR     62
#define MAX_CELL_No         13
#define MAX_CELL_No_Tesla   6
#define MAX_CELL_No_VW      13
#define MAX_Temp_Sens       3

#define EEPROM_VERSION      0x25   //update any time EEPROM struct below is changed.
#define EEPROM_PAGE         0

//BMS types
enum BMS_t {BMS_Dummy, BMS_Tesla, BMS_VW_MEB, BMS_VW_eGolf, BMS_BMW_I3, BMS_Type_MAX};

//BMC status values
enum {Stat_Boot, Stat_Ready, Stat_Drive, Stat_Charge, Stat_Precharge, Stat_Error, Stat_Warning, Stat_Debug, Stat_Charged, Stat_Healthy};

//Current sensor values
enum {Sen_Undefined, Sen_Analoguesing, Sen_Analoguedual, Sen_Canbus};

//Charger Types
enum {Charger_No, Charger_BrusaNLG5, Charger_ChevyVolt, Charger_Eltek, Charger_Elcon, Charger_Victron, Charger_Coda};

//Motor Controllers
enum {MC_No, Curtis};

//Menu options
enum {Menu_Start, Menu_Main, Menu_Battery, Menu_CurSen, Menu_Charger, Menu_MC, Menu_Outputs, Menu_Alarms, Menu_IgnVal, Menu_CAN, Menu_Exp, Menu_Debug};
#define Menu_Quit 113

/*
CAN mapping
CAN_BMC       // general BMC-Output
CAN_BMS       // Control of OEM Battery Managment Boards
CAN_Charger   // Charger control
CAN_Curr_Sen  // Current Sensor Bus
CAN_MC        // Motor Controller
*/
enum {CAN_BMC, CAN_BMS, CAN_Charger, CAN_Curr_Sen, CAN_MC};

typedef struct {
  byte version;
  byte checksum;
  uint16_t ReadTimeout;
  byte batteryID;  //which battery ID should this board associate as on the CAN bus
  BMS_t BMSType; // see BMS_Type above
  uint16_t OverVSetpoint; // in mV
  uint16_t UnderVSetpoint; // in mV
  uint16_t DischHys; // in mV
  uint16_t ChargeVSetpoint; // in mV
  uint16_t ChargeVsetpoint_toll; // in mV
  uint16_t UnderVDerateSetpoint; // in mV
  uint16_t ChargeHys; // in mV
  uint16_t StoreVsetpoint; // in mV
  uint16_t WarnVoltageOffset; // in mV
  
  int16_t OverTSetpoint; // in 0,1°C
  int16_t UnderTSetpoint; // in 0,1°C
  int16_t UnderTDerateSetpoint; // in 0,1°C
  int16_t OverTDerateSetpoint; // in 0,1°C
  int16_t WarnTempOffset; // in 0,1°C

  uint16_t CellGap; // in mV
  byte useTempSensor;
  uint16_t IgnoreVolt; // in mV
  uint16_t balanceVoltage; // in mV
  uint16_t balanceHyst; // in mV
  uint16_t Scells;
  uint16_t Pstrings;
  uint16_t CAP; // in Ah
  uint16_t designCAP; // in Ah
  uint32_t CAP_Wh;
  uint16_t ChargerChargeCurrentMax; // in 0,1 A
  uint16_t PackChargeCurrentMax; // in 0,1 A
  uint16_t chargecurrent2max; // in 0,1 A
  uint16_t chargecurrentend; // in 0,1 A
  uint16_t PackDisCurrentMax ; // in 0,1 A
  int socvolt[4]; // in mV
  bool invertcur;
  int cursens;
  bool voltsoc;
  int Pretime;
  int Precurrent; // in mA
  uint16_t convhigh;
  uint16_t convlow;
  int32_t changecur; // in mA
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
  uint16_t DisTaper;
  byte ChargerDirect;
  int mctype;
  byte SerialCan; // bool
  unsigned long error_delay;
  int16_t Temp_Cap_Map[2][5] = {}; // [0][x] Temperature in 0,1°C; [1][x] Percentage of Capacity
  byte Out_Map[2][9] = {}; // OUT1 = Out_Map[x][1]...; [0][x] Functions; [1][x] PWM
  uint32_t CAN1_Speed;
  uint32_t CAN2_Speed;
  uint16_t CAN1_Interval;
  uint16_t CAN2_Interval;
  byte CAN_Map[2][5]; // index[0]: 0 = BMC-Output; 1 = BMS Communication; 2 = Chargers; 3 = Current Sensors; 4 = MotorController // Values: 0 = not set; 1 = Can1; 2 = Can2; 3 = Can1 & Can2
} EEPROMSettings;
