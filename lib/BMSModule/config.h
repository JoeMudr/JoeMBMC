#pragma once
#include <Arduino.h>

// Set to the proper port for your USB connection - SerialUSB on Due (Native) or Serial for Due (Programming) or Teensy
#define Serial_USB Serial

// Define this to be the serial port the Tesla BMS modules are connected to.
// On the Due you need to use a USART port (Serial1, Serial2, Serial3) and update the call to serialSpecialInit if not Serial1
// Serial3 for teensy 3.2; Serial 2 for Teensy 4.x
#define Serial_BMS  Serial2

// Serial port for BlueTooth module
#define Serial_BT   Serial6

// Serial port for Display
#define Serial_Display Serial4

Stream* activeSerial = &Serial_USB;

// Tesla BMS_Module Comms
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

// VW BMS_Module Comms
// ??

// Values for Modules
#define MAX_MODULE_ADDR     62
#define MAX_CELL_No         13
#define MAX_CELL_No_Tesla   6
//#define MAX_CELL_No_VW      13
//#define MAX_CELL_No_BMW     12
#define MAX_Temp_Sens       3

// BMS types
enum BMS_t {
    BMS_Dummy, 
    BMS_Tesla, 
    BMS_VW_MEB, 
    BMS_VW_eGolf, 
    BMS_BMW_I3, 
    BMS_BMW_MiniE, 
    BMS_Type_MAX
};

// BMC status values
enum {
  Stat_Boot, 
  Stat_Ready, 
  Stat_Drive, 
  Stat_Charge, 
  Stat_Precharge, 
  Stat_Error, 
  Stat_Warning, 
  Stat_Debug, 
  Stat_Charged, 
  Stat_Idle,
  Stat_Discharge
};

// Current sensor values
enum {
  Sen_Undefined, 
  Sen_AnalogueSing, 
  Sen_AnalogueDual, 
  Sen_Canbus
};

// Charger Types
enum {
  Charger_No, 
  Charger_BrusaNLG5, 
  Charger_ChevyVolt, 
  Charger_Eltek, 
  Charger_Elcon, 
  Charger_Victron, 
  Charger_Coda
};

// Motor Controllers
enum {MC_No, Curtis};

// Menu options
enum {
    Menu_Start, 
    Menu_Main, 
    Menu_Battery, 
    Menu_CurSen, 
    Menu_Charger, 
    Menu_MC, 
    Menu_Outputs, 
    Menu_Alarms, 
    Menu_IgnVal, 
    Menu_CAN, 
    Menu_Misc, 
    Menu_Debug
};
#define Menu_Quit 113

// Warnings & Alarms
enum {
  WarnAlarm_Warning = 1,
  WarnAlarm_Alarm = 2
};

enum {
  WarnAlarm_Dummy, 
  WarnAlarm_VLow, 
  WarnAlarm_Vhigh, 
  WarnAlarm_TLow, 
  WarnAlarm_THigh, 
  WarnAlarm_ChargeTLow, 
  WarnAlarm_ChargeTHigh, 
  WarnAlarm_CurHigh,
  WarnAlarm_ChargeCurHigh,
  WarnAlarm_External, 
  WarnAlarm_CellGap,
};

/*
CAN mapping
CAN_BMC_std   // general BMC-Output SMA/Victron/Polyontech
CAN_BMC_HV    // general BMC-Output Polyontech HV
CAN_BMS       // Control of OEM Battery Managment Boards
CAN_Charger   // Charger control
CAN_Curr_Sen  // Current Sensor Bus
CAN_MC        // Motor Controller
*/
enum {
  CAN_BMC_std, 
  CAN_BMC_HV, 
  CAN_BMS, 
  CAN_Charger, 
  CAN_Curr_Sen, 
  CAN_MC, CAN_MAP_MAX
};

#define EEPROM_VERSION      40   //update any time EEPROM struct below is changed.
#define EEPROM_PAGE         0

typedef struct {
  byte version;
  byte checksum;
  uint16_t ReadTimeout;
  byte batteryID;       //which battery ID should this board associate as on the CAN bus
  BMS_t BMSType;        // see BMS_Type above

  uint16_t OverVAlarm;           // in mV
  uint16_t OverVWarn;            // in mV
  uint16_t UnderVWarn;           // in mV
  uint16_t UnderVAlarm;          // in mV
  uint16_t DischHys;             // mV Discharge voltage offset [ToDo] not used
  uint16_t ChargeVSetpoint;      // in mV
  uint16_t ChargeVsetpoint_toll; // in mV
  uint16_t ChargeHys;            // mV drop required for charger to kick back on
  uint16_t StoreVsetpoint;       // mV storage mode charge max
  uint16_t CellGap;              // mV max delta between high and low cell

  int16_t OverTAlarm;        // in 0,1°C
  int16_t OverTWarn;         // in 0,1°C
  int16_t UnderTWarn;        // in 0,1°C
  int16_t UnderTAlarm;       // in 0,1°C
  int16_t ChargeOverTAlarm;  // in 0,1°C
  int16_t ChargeOverTWarn;   // in 0,1°C
  int16_t ChargeUnderTWarn;  // in 0,1°C  
  int16_t ChargeUnderTAlarm; // in 0,1°C

  byte useTempSensor;      // 0 - use both sensors, 1 or 2 only use that sensor
  uint16_t IgnoreVolt;     // mV Cells under this Voltage will be ignored
  uint16_t balanceVoltage; // in mV
  uint16_t balanceHyst;    // in mV
  uint16_t Scells;         // Cells in series
  uint16_t Pstrings;       // strings in parallel
  uint16_t CAP;            // Ah Battery Capacity
  uint16_t designCAP;      // Ah Battery Design Capacity
  uint32_t CAP_Wh;
  
  uint16_t ChargeOverCurrAlarm; // in 0,1 A max charge current
  uint16_t ChargeOverCurrWarn;  // in 0,1 A max charge current
  uint16_t ChargeCurrentEnd;    // in 0,1 A end charge current
  uint16_t OverCurrAlarm;       // in 0,1 A max discharge current
  uint16_t OverCurrWarn;        // in 0,1 A max discharge current

  uint16_t ChargerChargeCurrentMax; // in 0,1 A max Charger charge current
  int ChargerType;                  // 0 - No Charger, 1 - Brusa NLG5xx, 2 - Volt charger ...
  byte nChargers;                   // number of chargers
  byte ChargerDirect;               // charger with or without contactors to HV
 
  bool voltsoc;   // SOC purely voltage based
  int socvolt[4]; // in mV Voltage and SOC curve for voltage based SOC calc

  int CurSenType;                 // Sensor Type
  int CurSenMultiplier;           // number of multiples to use for current measurement
  bool CurSenInvert;              // Invert current sensor direction
  uint16_t analogueSen1_convlow;  // mV/A current sensor low range channel (*100)
  uint16_t analogueSen2_convhigh; // mV/A current sensor high range channel (*100)
  uint16_t analogueSen1_offset;   // mV mid point of channel 1
  uint16_t analogueSen2_offset;   // mV mid point of channel 2
  int32_t analogueSen_ChangeCur;  // in mA change over point
  uint16_t analogueSen_CurDead;   // mV of dead band on current sensor

  int balanceDuty;
  bool ESSmode; //activate ESS mode
  int mctype;   // type of Motor Controller
  byte secondarySerial; // Serial canbus or display: 0-display 1-canbus expansion 2-Bluetooth App
  unsigned long error_delay; //time before Error_Stat shuts everything off
  int16_t Temp_CAP_Map[2][5] = {}; // [0][x] Temperature in 0,1°C; [1][x] Percentage of Capacity
  byte Out_Map[2][9] = {}; // OUT1 = Out_Map[x][1]...; [0][x] Functions; [1][x] PWM
  int PreTime;    // ms of precharge time
  int PreCurrent; // mA before closing main contator
  byte GaugeLow;  // empty fuel gauge pwm
  byte GaugeHigh; // full fuel gauge pwm

  uint32_t CAN1_Speed;
  uint32_t CAN2_Speed;
  uint16_t CAN1_Interval;
  uint16_t CAN2_Interval;
  byte CAN_Map[2][CAN_MAP_MAX]; // index[0]: 0 = BMC-Output_std; 1 = BMC-Output_HV; 2 = BMS Communication; 3 = Chargers; 4 = Current Sensors; 5 = MotorController // Values: 0 = not set; 1 = Can1; 2 = Can2; 3 = Can1 & Can2 // index[1] Interval
} EEPROMSettings;
