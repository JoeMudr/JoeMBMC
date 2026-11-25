#include "Globals.h"


/////Version Identifier/////////
// in format YYMMDD, e.g. 26.05.2020 -> 260520
uint32_t firmver = 260509; 

uint32_t Uptime = 0;

BMSManager bms;
EEPROMSettings settings;

//Curent filter//
byte filterFrequency = 5 ;
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

// uint32_t LED_Timer = 0;

byte BMC_Stat = 0;

//Serial Menu
bool menu_load = 0;

//Errors
uint32_t error_timer = 0;

byte Settings_unsaved = 0;

//variables for output control
uint16_t pwmfreq = 18000;//pwm frequency

uint16_t chargecurrent = 0;       // in 0,1A
uint16_t chargecurrentFactor = 0; // in%, correction factor, when using multiple chargers
uint16_t discurrent = 0;          // in 0,1A

/*
// SMA/Victron style Alarms & Warnings
// even Bits --> Alarm / Warning raised
// odd Bits --> Alarm / Warning cleared
[0] (7/6) hight Temp.          (5/4) low Voltage           (3/2) high Voltage           (1/0) general Alarm
[1] (7/6) high current         (5/4) low Temp. charge      (3/2) high Temp. charge      (1/0) low Temp.
[2] (7/6) Internal Failure     (5/4) battery short circuit (3/2) contactor problem      (1/0) high current charge
[3] (1/0) Cell Imbalance
*/
byte alarm[4],warning[4] = {0, 0, 0, 0};

// uint32_t warning_timer = 0;

unsigned char bmcname[8] = {'J', 'O', 'E', 'M', ' ', 'B', 'M', 'C'};
unsigned char bmcmanu[8] = {'J', 'O', 'E', 'M', ' ', 'T', 'E', 'C'};

//int32_t ISAVoltage1, ISAVoltage2, ISAVoltage3 = 0; //mV only with ISAscale sensor

uint32_t BMSLastRead = 0; // [ToDo] not used

//variables for current/capacity calulation
uint16_t Sen_AnalogueRawValue;
int32_t currentact = 0; // mA
int32_t currentlast = 0; // mA
int32_t RawCur;
int32_t mampsecond = 0; // Range 0 = full to settings.cap * -1 = empty
volatile int32_t mampsecondTimer = 0;
byte Sen_Analogue_active_Nr = 1; // active analogue sensor: 1 = Sensor 1; 2 = Sensor 2
int16_t TCAP = 0; //Temperature corrected Capacity in Ah including settings.Pstrings!
int16_t TCAP_Wh = 0;
//Variables for SOC calc
byte SOC = 100; //State of Charge

//charger variables Prusa
byte maxac1 = 16; //Shore power 16A per charger; only used for prusa charger
byte maxac2 = 10; //Generator Charging; only used for prusa charger
const int16_t chargerid1 = 0x618; //bulk chargers (brusa)
const int16_t chargerid2 = 0x638; //finishing charger (brusa)
uint16_t chargerendbulk = 0; //mV before Charge Voltage to turn off the bulk charger/s
uint16_t chargerend = 0; //mV before Charge Voltage to turn off the finishing charger/s


bool v_CAN_Charger_Active = false; // global variable for CAN-Based chargers
u_int32_t v_CAN_Charger_Active_Timer = 0; // Reset-timer for v_CAN_Charger_Active


//variables
byte storagemode = 0;
byte precharged = 0;

byte debug_output_counter = 0;

//Output Control
byte cont_pulltime = 255;

/*
Array for looping through outputs
OUT1 = 1, OUT2 = 2 ...
*/
const byte *Outputs[9] = {0,&OUT1,&OUT2,&OUT3,&OUT4,&OUT5,&OUT6,&OUT7,&OUT8};

/*
Array for function states & according timers for PWMs (full pull->reduced power)
[0][x] Function State (High/Low); [1][x] timer
*/
byte Out_States[2][Out_States_Max] = {{0,0,0,0,0,0,0,0,0,0,0,0,0},{0,cont_pulltime,cont_pulltime,cont_pulltime,0,0,0,0,0,0,0,0,0}}; 

/*
Text desctiptions for output funktions:
0 - undefined, 
1 - precharge Contactor, 
2 - positive Contactor, 
3 - negative Contactor, 
4 - Discharge enable, 
5 - Charge Enable, 
6 - Error, 
7 - Warning, 
8 - Error & Warning 
9 - Heating enable, 
10 - Cooling enable, 
11 - Gauge
12 - Balance enable
*/
String Out_Functions[] = {
  "undefined", 
  "precharge Cont.", 
  "positive Cont.", 
  "negative Cont.", 
  "Discharge enable", 
  "Charge enable", 
  "Error", 
  "Warning", 
  "Error & Warning",
  "Heating enable", 
  "Cooling enable", 
  "Gauge",
  "Balance enable"
};

// Debugging modes
bool debug = 1;
bool debug_Input = 0;   // read digital inputs
bool debug_Output = 0;  // check outputs
bool debug_CAN1 = 0;    // view can frames
bool debug_CAN2 = 0;    // view can frames
bool debug_Gauge = 0;
byte  debug_Gauge_val = 0;
uint32_t debug_Gauge_timer = 0;
bool debug_Cur = 0;
bool debug_CSV = 0;
byte debug_Digits = 2; // amount of digits behind decimal for voltage reading

// TCU Variables
byte TCU_Pump = 0;
byte TCU_Cool = 0;
byte TCU_Heat = 0;
byte TCU_REL4 = 0;

IntervalTimer Timer_mAmpSec;

// CAN
FlexCAN_T4<CAN1> can1;
FlexCAN_T4<CAN2> can2;

ADC *adc = new ADC();
Stream* activeSerial = &Serial_USB;