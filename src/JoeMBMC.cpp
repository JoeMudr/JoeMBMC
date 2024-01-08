/*
  This software is heavily based on the work of Simp ECO / tomdebree. 
  Although most of his code has been completely rewritten, respect is due to him.
  Hence the original Copyright:

  Copyright (c) 2019 Simp ECO Engineering [& JoeM]
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <JoeMBMC.h>
#include <BMSModuleManager.h>
#include <Arduino.h>
#include <config.h>
#include <ADC.h> //https://github.com/pedvide/ADC
#include <EEPROM.h>
#include <FlexCAN_T4.h> //https://github.com/tonton81/FlexCAN_T4
#include <SPI.h>
#include <Filters.h>//https://github.com/JonHub/Filters
#include <Watchdog_t4.h>

/*
//[ToDo] Reboot
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
#define CPU_REBOOT WRITE_RESTART(0x5FA0004)
*/

/////Version Identifier/////////
int firmver = 240102;

BMSModuleManager bms;
EEPROMSettings settings;

//Curent filter//
float filterFrequency = 5.0 ;
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );

//BMC wiring//
const int ACUR2 = A0; // current 1
const int ACUR1 = A1; // current 2
const int IN1_Key = 19; // input 1 - high active
const int IN2_Gen = 18; // input 2 - high active
const int IN3_AC = 20; // input 3 - high active
const int IN4 = 21; // input 4 - high active
const int OUT1 = 6;// output 1 - high active
const int OUT2 = 5;// output 2 - high active
const int OUT3 = 4;// output 3 - high active
const int OUT4 = 3;// output 4 - high active
const int OUT5 = 11;// output 5 - Low active / PWM
const int OUT6 = 12;// output 6 - Low active / PWM
const int OUT7 = 10; // output 7 - Low active / PWM
const int OUT8 = 9; // output 8 - Low active / PWM
const int led = 13;

byte BMC_Stat = 0;

//BMC status values
#define Stat_Boot 0
#define Stat_Ready 1
#define Stat_Drive 2
#define Stat_Charge 3
#define Stat_Precharge 4
#define Stat_Error 5
#define Stat_Warning 6 //not used anymore!
#define Stat_Debug 7
#define Stat_Charged 8
#define Stat_Healthy 9

//Current sensor values
#define Sen_Undefined 0
#define Sen_Analoguedual 1
#define Sen_Canbus 2
#define Sen_Analoguesing 3

//Charger Types
#define Charger_No 0
#define Charger_BrusaNLG5 1
#define Charger_ChevyVolt 2
#define Charger_Eltek 3
#define Charger_Elcon 4
#define Charger_Victron 5
#define Charger_Coda 6

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

//Motor Controllers
#define MC_No 0
#define Curtis 1

//Serial Menu
bool menu_load = 0;
int menu_option = 0;
byte menu_current = 0;

#define Menu_Start 0
#define Menu_Main 1
#define Menu_Battery 2
#define Menu_CurSen 3
#define Menu_Charger 4
#define Menu_MC 5
#define Menu_Outputs 6
#define Menu_Alarms 7
#define Menu_IgnVal 8
#define Menu_CAN 9
#define Menu_Exp 10
#define Menu_Debug 11
#define Menu_Quit 113

//Errors
unsigned long error_timer = 0;

byte Settings_unsaved= 0;

//variables for output control
unsigned long Pretimer;
uint16_t pwmfreq = 18000;//pwm frequency

uint16_t chargecurrent = 0; // in 0,1A
uint16_t chargecurrentlast = 0; // in 0,1A
float chargecurrentFactor = 0.0f; //correction factor, when using multiple chargers
uint16_t discurrent = 0;

/*
// SMA style Alarms & Warnings
// even Bits --> Alarm / Warning raised
// odd Bits --> Alarm / Warning cleared
[0] (7/6) hight Temp.          (5/4) low Voltage           (3/2) high Voltage           (1/0) general Alarm
[1] (7/6) high current         (5/4) low Temp. charge      (3/2) high Temp. charge      (1/0) low Temp.
[2] (7/6) Internal Failure     (5/4) battery short circuit (3/2) contactor problem      (1/0) high current charge
[3] (1/0) Cell Imbalance
*/
unsigned char alarm[4],warning[4] = {0, 0, 0, 0};
//unsigned char warning[4] = {0, 0, 0, 0};


unsigned long warning_timer = 0;

unsigned char bmcname[8] = {'J', 'O', 'E', 'M', ' ', 'B', 'M', 'C'};
unsigned char bmcmanu[8] = {'J', 'O', 'E', 'M', ' ', 'T', 'E', 'C'};

signed long ISAVoltage1, ISAVoltage2, ISAVoltage3 = 0; //mV only with ISAscale sensor

//variables for current/capacity calulation
uint16_t Sen_AnalogueRawValue;
float currentact = 0.0f; //mA
float currentlast = 0.0f; 
float currentavg = 0.0f; //mA
float currentavg_array[60];
byte currentavg_counter = 0;
float RawCur;
long mampsecond = 0; // Range 0 = full to settings.cap * -1 = empty
long mampsecondTimer = 0;
int64_t mWs = 0; // Capacity in mWs (see mampsecond)
long mWsTimer = 0;
unsigned long lastTime;
unsigned long looptime, looptime1, UnderTime, cleartime, baltimer = 0; //ms
int Sen_Analogue_Num = 1; // 1 = Sensor 1; 2 = Sensor 2
int TCAP = 0; //Temperature corrected Capacity in Ah including settings.Pstrings! settings.CAP * settings.Pstrings * -1 + (CAP_Temp_alteration() / 3600000);
int TCAP_Wh = 0;
//Variables for SOC calc
byte SOC = 100; //State of Charge
byte SOCset = 0;

//charger variables
byte maxac1 = 16; //Shore power 16A per charger
byte maxac2 = 10; //Generator Charging
int chargerid1 = 0x618; //bulk chargers (brusa)
int chargerid2 = 0x638; //finishing charger (brusa)
float chargerendbulk = 0; //V before Charge Voltage to turn off the bulk charger/s
float chargerend = 0; //V before Charge Voltage to turn off the finishing charger/s

//variables
byte output_debug_counter = 0;
byte storagemode = 0;
byte precharged = 0;

//Output Control
byte cont_pulltime = 255;

/*
Array for looping through outputs
OUT1 = 1, OUT2 = 2 ...
*/
const int *Outputs[9] = {0,&OUT1,&OUT2,&OUT3,&OUT4,&OUT5,&OUT6,&OUT7,&OUT8};

/*
Array for function states & according timers for PWMs (full pull->reduced power)
[0][x] Function State (High/Low); [1][x] timer
*/
byte Out_States[2][12] = {{0,0,0,0,0,0,0,0,0,0,0,0},{0,cont_pulltime,cont_pulltime,cont_pulltime,0,0,0,0,0,0,0,0}}; 

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
*/
String Out_Functions[] = {
  "undefined       ", 
  "precharge Cont. ", 
  "positive Cont.  ", 
  "negative Cont.  ", 
  "Discharge enable", 
  "Charge enable   ", 
  "Error           ", 
  "Warning         ", 
  "Error & Warning ",
  "Heating enable  ", 
  "Cooling enable  ", 
  "Gauge           "
};

//IDs for Array above
#define Out_undefined 0
#define Out_Cont_Precharge 1
#define Out_Cont_Pos 2
#define Out_Cont_Neg 3
#define Out_Discharge 4
#define Out_Charge 5
#define Out_Error 6
#define Out_Warning 7
#define Out_Err_Warn 8
#define Out_Heating 9
#define Out_Cooling 10
#define Out_Gauge 11

u_int32_t cont_timer = 0;
unsigned long Out_PWM_fullpull_Timer = 0; // PWM Timer

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

//[ToDo] rewrite CAN-Send for 2 CAN busses
CAN_message_t outMsg;

// CAN mapping

#define CAN_BMC 0 // general BMC-Output
#define CAN_BMS 1 // Control of OEM Battery Managment Boards
#define CAN_Charger 2 // Charger control
#define CAN_Curr_Sen 3 // Current Sensor Bus
#define CAN_MC 4 // Motor Controller

//Debugging modes
bool debug = 1;
bool debug_Input = 0; //read digital inputs
bool debug_Output = 0; //check outputs
bool debug_CAN1 = 0; //view can frames
bool debug_CAN2 = 0; //view can frames
bool debug_Gauge = 0;
byte  debug_Gauge_val = 0;
unsigned long debug_Gauge_timer = 0;
bool debug_Cur = 0;
bool debug_CSV = 0;
int debug_Digits = 2; //amount of digits behind decimal for voltage reading

ADC *adc = new ADC(); // adc object

IntervalTimer Timer_mAmpSec;
IntervalTimer Timer_CO_SYNC;

void loadSettings(){
  settings.version = EEPROM_VERSION;
  settings.checksum = 2;
  settings.batteryID = 0x01; //in the future should be 0xFF to force it to ask for an address
  settings.OverVSetpoint = 4.2f;
  settings.ChargeVSetpoint = 4.15f;
  settings.UnderVSetpoint = 3.0f;
  settings.UnderVDerateSetpoint = 3.2f;
  settings.ChargeHys = 0.1f; // voltage drop required for charger to kick back on
  settings.WarnOff = 0.1f; //voltage offset to raise a warning
  settings.DischHys = 0.2f; // Discharge voltage offset
  settings.CellGap = 0.2f; //max delta between high and low cell
  settings.OverTSetpoint = 55.0f;
  settings.OverTDerateSetpoint = 45.0f;
  settings.UnderTSetpoint = 0.0f;
  settings.UnderTDerateSetpoint = 5.0f;
  settings.WarnToff = 5.0f; //temp offset before raising warning
  settings.IgnoreTemp = 0; // 0 - use both sensors, 1 or 2 only use that sensor
  settings.IgnoreVolt = 0.5;//
  settings.balanceVoltage = 3.7f;
  settings.balanceHyst = 0.01f;
  settings.balanceDuty = 50;
  settings.CAP = 230; //battery size in Ah
  settings.designCAP = 230; //battery design capazity in Ah
  settings.CAP_Wh = 25000;
  settings.Pstrings = 1; // strings in parallel used to divide voltage of pack
  settings.Scells = 30;//Cells in series
  settings.StoreVsetpoint = 3.8; // V storage mode charge max
  settings.PackDisCurrentMax = 5500; // max discharge current in 0.1A
  settings.DisTaper = 0.3f; //V offset to bring in discharge taper to Zero Amps at settings.UnderVDerateSetpoint
  settings.ChargerChargeCurrentMax = 320; //max charge current in 0.1A
  settings.PackChargeCurrentMax = 1000; //Pack max charge current in 0.1A for Error handling
  settings.chargecurrent2max = 150; //max charge current in 0.1A
  settings.chargecurrentend = 50; //end charge current in 0.1A
  settings.socvolt[0] = 3100; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[1] = 10; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[2] = 4100; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[3] = 90; //Voltage and SOC curve for voltage based SOC calc
  settings.invertcur = 0; //Invert current sensor direction
  settings.cursens = Sen_Canbus;
  settings.voltsoc = 0; //SOC purely voltage based
  settings.Pretime = 5000; //ms of precharge time
  settings.conthold = 50; //holding duty cycle for contactor 0-255
  settings.Precurrent = 1000; //ma before closing main contator
  settings.convhigh = 312; // mV/A current sensor high range channel
  settings.convlow = 1250; // mV/A current sensor low range channel
  settings.offset1 = 1650; //mV mid point of channel 1
  settings.offset2 = 1650;//mV mid point of channel 2
  settings.changecur = 140000;//mA change over point
  settings.gaugelow = 0; //empty fuel gauge pwm
  settings.gaugehigh = 255; //full fuel gauge pwm
  settings.ESSmode = 0; //activate ESS mode
  settings.ncur = 1; //number of multiples to use for current measurement
  settings.nchargers = 1; // number of chargers
  settings.chargertype = Charger_Elcon; // 0 - No Charger, 1 - Brusa NLG5xx, 2 - Volt charger ...
  settings.ChargerDirect = 1; // charger with or without contactors to HV
  settings.CurDead = 5;// mV of dead band on current sensor
  settings.mctype = 0; // type of Motor Controller
  settings.SerialCan = 0; //Serial canbus or display: 0-display 1-canbus expansion 2-Bluetooth App
  settings.error_delay = 10000; //time before Error_Stat shuts everything off
  settings.Temp_Cap_Map[0][0] = -20;
  settings.Temp_Cap_Map[0][1] = -10;
  settings.Temp_Cap_Map[0][2] = 0;
  settings.Temp_Cap_Map[0][3] = 25;
  settings.Temp_Cap_Map[0][4] = 40;
  settings.Temp_Cap_Map[1][0] = 38;
  settings.Temp_Cap_Map[1][1] = 69;
  settings.Temp_Cap_Map[1][2] = 78;
  settings.Temp_Cap_Map[1][3] = 90;
  settings.Temp_Cap_Map[1][4] = 100;
  for (size_t i = 0; i < 9; i++){
    settings.Out_Map[0][i] = 0;
    settings.Out_Map[1][i] = 0;
  }
  settings.CAN1_Interval = 100; //ms per message
  settings.CAN2_Interval = 100; //ms per message
  settings.CAN1_Speed = 500000;
  settings.CAN2_Speed = 500000;
  for (size_t i = 0; i < 5; i++){
  settings.CAN_Map[i] = 0;
  }
}

uint32_t lastUpdate;

WDT_T4<WDT1> watchdog;
WDT_timings_t watchdog_config;

uint32_t lastResetCause;

void setup(){

  // Save copy of Reset Status Register
  lastResetCause = SRC_SRSR;
  // Clear all Reset Status Register bits
  SRC_SRSR = (uint32_t)0x1FF;

  pinMode(IN1_Key, INPUT);
  pinMode(IN2_Gen, INPUT);  lastResetCause = SRC_SRSR;

  pinMode(IN3_AC, INPUT);
  pinMode(IN4, INPUT);
  pinMode(OUT1, OUTPUT); // 12V output
  pinMode(OUT2, OUTPUT); // 12V output
  pinMode(OUT3, OUTPUT); // 12V output
  pinMode(OUT4, OUTPUT); // 12V output
  pinMode(OUT5, OUTPUT); // pwm GND output
  pinMode(OUT6, OUTPUT); // pwm GND output
  pinMode(OUT7, OUTPUT); // pwm GND output
  pinMode(OUT8, OUTPUT); // pwm GND output
  pinMode(led, OUTPUT);

  analogWriteFrequency(OUT5, pwmfreq);
  analogWriteFrequency(OUT6, pwmfreq);
  analogWriteFrequency(OUT7, pwmfreq);
  analogWriteFrequency(OUT8, pwmfreq);

  EEPROM.get(0, settings);
  if (settings.version != EEPROM_VERSION){ loadSettings(); }
  //loadSettings();
  can1_start();
  can2_start();

   //if using enable pins on a transceiver they need to be set on

  adc->adc0->setAveraging(16); // set number of averages
  adc->adc0->setResolution(16); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc->adc0->startContinuous(ACUR1);

  //delay(2000);  //just for easy debugging. It takes a few seconds for USB to come up properly on most OS's  
  SERIALCONSOLE.begin(115200);
  SERIALCONSOLE.setTimeout(20);
  SERIALCONSOLE.println("Starting up...");
  SERIALCONSOLE.println("JoeM BMC");

  DisplaySerial.begin(115200); //display and can adpater canbus

  // Display reason the Teensy was last reset
  SERIALCONSOLE.println();
  SERIALCONSOLE.println("Reason for last Reset: ");
  Reset_Cause(lastResetCause);

  // enable WDT
  //watchdog_config.trigger = 5; // [ToDo] ?
  //watchdog_config.timeout = 1; //[ToDo]
  watchdog.begin(watchdog_config);


  SERIALBMS.begin(612500); //Tesla serial bus
#if defined (__arm__) && defined (__SAM3X8E__)
  serialSpecialInit(USART0, 612500); //required for Due based boards as the stock core files don't support 612500 baud.
#endif

  SERIALCONSOLE.println("Started serial interface to BMS.");
  lastUpdate = 0;
  BMS_revive(); 

  Timer_mAmpSec.begin(mAmpsec_calc, 500000);
 // Timer_CO_SYNC.begin(CO_SYNC,100000);
}

void loop(){
  CAN_read(); //everything CAN related happens here

  if ( settings.cursens == Sen_Analoguedual || settings.cursens == Sen_Analoguesing){
    currentact = SEN_AnalogueRead(currentlast);
  }

  if (millis() > 3000 && SERIALCONSOLE.available()){Menu();} 

  Alarm_Check();

  if (!debug_Output){
    //ContCon();
    if (settings.ESSmode){BMC_Stat = ESS_CondCheck(BMC_Stat);}
    else{BMC_Stat = Vehicle_CondCheck(BMC_Stat);}
    set_BMC_Status(BMC_Stat);
    set_OUTs();
  }

  if(debug_Cur){Current_debug();}

  if (millis() - looptime > 500){ // 0.5s loop
    looptime = millis();

    TCAP = settings.CAP * settings.Pstrings * -1 * CAP_Temp_alteration();
    TCAP_Wh = settings.CAP_Wh * -1 * CAP_Temp_alteration(); // [ToDo]Pstrings?

    //copy Ampseconds from Timer Funktion
    noInterrupts();
    mampsecond += mampsecondTimer;
    mWs += mWsTimer;
    if(mampsecond > 0){mampsecond = 0;}
    if(mWs > 0){mWs = 0;}
    mampsecondTimer = 0;
    mWsTimer = 0;
    interrupts();

    bms.getAllVoltTemp(); //has to be looped to get all values from modules

    if (!menu_load){ SERIALCONSOLEprint(); OUT_Debug();} //"debug" output on serial console
    if (debug_CSV){ bms.printAllCSV(millis(), currentact, SOC); }
    if (debug_Input){ Input_Debug(); }
    if (debug_Output){ Output_debug(); }
    else{ Gauge_update(); } //WDT tripped?!
    
    SOC_update();
    CAN_BMC_send(settings.CAN_Map[CAN_BMC]);
    
    Currentavg_Calc();

    if (settings.SerialCan == 0){ Dash_update(); }//Info on serial bus 2
    //if (settings.SerialCan == 2){ BT_update(); }// --> Bluetooth App
    
    //WDT reset;
    watchdog.feed();
  }

}
//[ToTest]
/* ==================================================================== */
// i.MX RT1060 Processor Reference Manual, 21.8.3 SRC Reset Status Register
void Reset_Cause(uint32_t resetStatusReg) {
    bool info = false;

    if (resetStatusReg & SRC_SRSR_TEMPSENSE_RST_B) {
        Serial.println("Temperature Sensor Software Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_WDOG3_RST_B) {
        Serial.println("IC Watchdog3 Timeout Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_JTAG_SW_RST) {
        Serial.println("JTAG Software Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_JTAG_RST_B) {
        Serial.println("High-Z JTAG Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_WDOG_RST_B) {
        Serial.println("IC Watchdog Timeout Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_IPP_USER_RESET_B) {
        Serial.println("Power-up Sequence (Cold Reset Event)");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_CSU_RESET_B) {
        Serial.println("Central Security Unit Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_LOCKUP_SYSRESETREQ) {
        Serial.println("CPU Lockup or Software Reset");
        info = true;
        /* Per datasheet: "SW needs to write a value to SRC_GPR5
         * before writing the SYSRESETREQ bit and use the SRC_GPR5
         * value to distinguish if the reset is caused by SYSRESETREQ
         * or CPU lockup."
         */
    }
    if (resetStatusReg & SRC_SRSR_IPP_RESET_B) {
        Serial.println("Power-up Sequence");
        info = true;
    }
    if (!info) {
        Serial.println("No status bits set in SRC Reset Status Register");
    }
}

void Alarm_Check(){
 ///warnings
  for(byte i = 0; i<4; i++){warning[i] = 0b10101010;} // reset to all OK

  if (bms.getHighCellVolt() > (settings.OverVSetpoint - settings.WarnOff)){ warning[0] ^= 0b00001100;}
  if (bms.getLowCellVolt() < (settings.UnderVSetpoint + settings.WarnOff)){
    warning[0] ^= 0b00110000;
    if (!warning_timer){warning_timer = millis() + 10000;}
    if (millis() > warning_timer){CAP_recalc();warning_timer = 0;} // recalculate capacity when lowest cell is under warning setpoint; reset Timer to prevent EEPROM Kill
  } else {warning_timer = 0;}

  if (bms.getHighTemperature() > (settings.OverTSetpoint - settings.WarnToff)){warning[0] ^= 0b11000000;}
  if (bms.getLowTemperature() < (settings.UnderTSetpoint + settings.WarnToff)){warning[1] ^= 0b00000011;}
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap){warning[3] ^= 0b00000011;}
  if (bms.seriescells() != settings.Scells * settings.Pstrings){warning[2] ^= 0b11000000;}
 
  //Errors
  for(byte i = 0; i<4; i++){alarm[i] = 0b10101010;} // reset to all OK

  if (bms.getHighCellVolt() > settings.OverVSetpoint){alarm[0] ^= 0b00001100;}
  if (bms.getLowCellVolt() < settings.UnderVSetpoint){alarm[0] ^= 0b00110000;}
  if (bms.getHighTemperature() > settings.OverTSetpoint){alarm[0] ^= 0b11000000;}
  if (bms.getLowTemperature() < settings.UnderTSetpoint){alarm[1] ^= 0b00000011;}
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap){alarm[3] ^= 0b00000011;}
  if (bms.seriescells() != settings.Scells * settings.Pstrings){alarm[2] ^= 0b11000000;}
}



byte Vehicle_CondCheck(byte tmp_status){ 
  // [ToDo] Funktion läuft ohne Beschränkung in der Loop. BMS-Modul-Werte werden aber nur alle 0,5s geholt. 
  // Eventuell diese Vergleiche auslagern bzw. mit BMB-Abfrage verknüpfen.

  // start with no Errors
  tmp_status = Stat_Ready;

  // Reset to Ready if all Inputs are LOW ?
  // if (digitalRead(IN1_Key) == LOW && digitalRead(IN2_Gen) == LOW && digitalRead(IN3_AC) == LOW && digitalRead(IN4) == LOW){tmp_status = Stat_Ready;}
  
  // [ToDo] alle "digitalRead(IN3)" durch Funktion zum Erkennen des Ladesteckers ersetzen. Diese sollte auch das erkennen über CAN enthalten.

  // detect KEY ON & AC OFF -> Drive
  if (digitalRead(IN1_Key) == HIGH && digitalRead(IN3_AC) == LOW){
    if (precharged) {tmp_status = Stat_Drive;}
    else {tmp_status = Stat_Precharge;}
  } 

  // detect Undervoltage before Charging / let Charging override this Error
  if (alarm[0] & 0b00010000) {tmp_status = Stat_Error;}

  //detect AC present & Check charging conditions
  if (digitalRead(IN3_AC) == HIGH){
    //start charging when Voltage is below Charge Voltage - ChargeHyst.
    if (bms.getHighCellVolt() < (settings.ChargeVSetpoint - settings.ChargeHys)){ 
      if (precharged || settings.ChargerDirect){tmp_status = Stat_Charge;}
      else {tmp_status = Stat_Precharge;}      
    }
    //Set 100% when Voltage gets above Charge Voltage 
    //Set charged when Voltage gets above Charge Voltage
    if (bms.getHighCellVolt() > (settings.ChargeVSetpoint)){
      SOC_charged();
      tmp_status = Stat_Charged;
    }

    // Set charged when not charging and Voltage is above Charge Voltage - ChargeHyst.
    if (bms.getHighCellVolt() > (settings.ChargeVSetpoint - settings.ChargeHys) && tmp_status != Stat_Charge){
      tmp_status = Stat_Charged;
    }    
  }
//SERIALCONSOLE.println(tmp_status);
  // Set Error depending on Error conditions, except Undervoltage to let Charging recover from Undervoltage
  // Undertemp is no Error in Drive & Charge (--> derate)
  //[ToDO] Fix Tlow
  if (alarm[0] & 0b01010001 || /*((alarm[1] & 0x01) && (tmp_status != Stat_Drive || tmp_status != Stat_Precharge || tmp_status != Stat_Charge)) ||*/ alarm[3] & 0b00000010){tmp_status = Stat_Error;}
//SERIALCONSOLE.println(tmp_status);

  // reset Error-Timer
  if (tmp_status != Stat_Error){error_timer = 0;}
  return tmp_status; 
}

byte ESS_CondCheck(byte tmp_status){
  //[ToDo] Add balancing "allways" except when current over certain value.
  // Precharge first
  if (precharged){tmp_status = Stat_Healthy;}
  else {tmp_status = Stat_Precharge;}  
  //tmp_status = Stat_Healthy;
  if (digitalRead(IN1_Key) == HIGH) { 
    //[ToDo] storagemode
  }

  if (bms.getHighCellVolt() > (settings.ChargeVSetpoint)){
    SOC_charged();
  }

  //Errors
  if (alarm[0] & 0b01010101 || (alarm[1] & 0b00000001)  || alarm[2] & 0b01010100){tmp_status = Stat_Error;}
  if (tmp_status != Stat_Error){error_timer = 0;}
  return tmp_status;
}

byte Warn_handle(){
  if (warning[0] & 0b01010101 || warning[1] & 0b01010101 || warning[2] & 0b01010101 || warning[3] & 0b01010101){
    set_OUT_States(Out_Err_Warn);
    set_OUT_States(Out_Warning);
    if(warning[0] & 0b01000000){ // high temp -> activate cooling
      set_OUT_States(Out_Cooling);
    }
    if(warning[1] & 0b00000001){ // low temp -> activate heating
      set_OUT_States(Out_Heating);
    }
    return 1;
  }
  return 0;
}

void set_BMC_Status(byte status){
  switch (status){
    case Stat_Boot:  //[ToDO] unnötig? Ändern in Wartungsmodus?
      set_OUT_States(); // all off
      Balancing(0);
    break;
    case Stat_Ready:
      precharged = 0;
      set_OUT_States(); // all off
      Balancing();
      Warn_handle();
    break;
    case Stat_Drive:
      set_OUT_States(); // all off
      set_OUT_States(Out_Cont_Pos);
      set_OUT_States(Out_Cont_Neg);
      set_OUT_States(Out_Gauge);
      DischargeCurrentLimit();
      Warn_handle();
      Balancing(0);
    break;
    case Stat_Precharge:
      set_OUT_States(); // all off
      Prechargecon();
      Balancing(0);
    break;    
    case Stat_Charge:
      set_OUT_States(); // all off
      CAP_recalc();
      if (Settings_unsaved){EEPROM.put(0, settings); Settings_unsaved = 0; SERIALCONSOLE.println("--Saved--");}
      set_OUT_States(Out_Charge); // enable charger
      if (digitalRead(IN1_Key) == HIGH){set_OUT_States(Out_Gauge);} // enable gauge if key is on
      Balancing();
      CAN_Charger_Send(settings.CAN_Map[CAN_Charger]);
      ChargeCurrentLimit();
      Warn_handle();
    break;
    case Stat_Charged:
      set_OUT_States(); // all off
      if (digitalRead(IN1_Key) == HIGH){set_OUT_States(Out_Gauge);} // enable gauge if key is on    
      Balancing();
      chargecurrentlast = 0;
      Warn_handle();
    break;
    case Stat_Error:
      if (!error_timer){ error_timer = millis() + settings.error_delay; }//10s delay before turning everything off
      if (millis() > error_timer){
        set_OUT_States(); // all off
        set_OUT_States(Out_Error);
        set_OUT_States(Out_Err_Warn);
        if (digitalRead(IN1_Key) == HIGH){set_OUT_States(Out_Gauge);} // enable gauge if key is on    
        Balancing(0);
        discurrent = 0;
        chargecurrent = 0;
      }
    break;
    case Stat_Healthy:
      set_OUT_States(); // all off
      set_OUT_States(Out_Discharge);
      set_OUT_States(Out_Cont_Pos);
      set_OUT_States(Out_Cont_Neg);
      set_OUT_States(Out_Gauge);
      Balancing();
      DischargeCurrentLimit();
      ChargeCurrentLimit();
      Warn_handle();
    break;
  } 
}

void BMS_revive(){
  if(millis() > 2000){digitalWrite(led, LOW);} //Bug which messes up Serial when changing the LED within 2000ms after power up.
  bms.renumberBoardIDs();
  bms.findBoards();
  bms.setPstrings(settings.Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
  bms.clearFaults();
  if(millis() > 2000){digitalWrite(led, HIGH);}
}

byte Balancing(byte active){
  if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst && active){
    bms.balanceCells(settings.balanceDuty, settings.balanceHyst, 0);
    return 1;
  } else {
    bms.StopBalancing();
    return 0;
  }
}

void SERIALCONSOLEprint(){
  Serial_clear();
  SERIALCONSOLE.print("Firmware: ");
  SERIALCONSOLE.println(firmver);
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("BMC Status: ");
  SERIALCONSOLE.print(BMC_Stat);
  SERIALCONSOLE.print(" ");
    switch (BMC_Stat){
      case (Stat_Boot): SERIALCONSOLE.print("Boot"); break;
      case (Stat_Ready): SERIALCONSOLE.print("Ready"); break;
      case (Stat_Precharge): SERIALCONSOLE.print("Precharge"); break;
      case (Stat_Drive): SERIALCONSOLE.print("Drive"); break;
      case (Stat_Charge): SERIALCONSOLE.print("Charge"); break;
      case (Stat_Charged): SERIALCONSOLE.print("Charged"); break;
      case (Stat_Error): SERIALCONSOLE.print("Error"); break;
      case (Stat_Healthy): SERIALCONSOLE.print("Healthy"); break;
    }
  if (Warn_handle()){SERIALCONSOLE.print(" (Warning!)");}
  SERIALCONSOLE.print(" | Cells: ");
  SERIALCONSOLE.print(bms.seriescells());
  SERIALCONSOLE.print("/");
  SERIALCONSOLE.print(settings.Scells*settings.Pstrings);
  if (digitalRead(IN3_AC) == HIGH){ SERIALCONSOLE.print(" | AC Present"); }
  if (digitalRead(IN1_Key) == HIGH){ SERIALCONSOLE.print(" | Key ON"); }
  if (Balancing()){ SERIALCONSOLE.print(" | Balancing Active"); }
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("In:");
  SERIALCONSOLE.print(digitalRead(IN1_Key));
  SERIALCONSOLE.print(digitalRead(IN2_Gen));
  SERIALCONSOLE.print(digitalRead(IN3_AC));
  SERIALCONSOLE.print(digitalRead(IN4));
  SERIALCONSOLE.print(" | Charge Current Limit : ");
  SERIALCONSOLE.print(float(chargecurrent) * settings.nchargers / 10);
  SERIALCONSOLE.print("(");
  SERIALCONSOLE.print(float(chargecurrent) / 10);
  SERIALCONSOLE.print(") A, Factor: ");
  SERIALCONSOLE.print(chargecurrentFactor);
  SERIALCONSOLE.print(" | DisCharge Current Limit : ");
  SERIALCONSOLE.print(discurrent / 10);
  SERIALCONSOLE.print("A");
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Vpack: ");
  SERIALCONSOLE.print(bms.getPackVoltage());
  SERIALCONSOLE.print("V| Vlow: ");
  SERIALCONSOLE.print(bms.getLowCellVolt());
  SERIALCONSOLE.print("V| Vhigh: ");
  SERIALCONSOLE.print(bms.getHighCellVolt());
  SERIALCONSOLE.print("V| DeltaV: ");
  int deltaV = (bms.getHighCellVolt() - bms.getLowCellVolt())*1000;
  SERIALCONSOLE.print(deltaV);
  SERIALCONSOLE.print("mV| tlow: ");
  SERIALCONSOLE.print(bms.getLowTemperature());
  SERIALCONSOLE.print("C| thigh: ");
  SERIALCONSOLE.print(bms.getHighTemperature());
  SERIALCONSOLE.print("C");
  SERIALCONSOLE.println();
  bms.printPackDetails(debug_Digits);
  SERIALCONSOLE.println();
  
  if (settings.cursens == Sen_Analoguedual){
    if (Sen_Analogue_Num == 1)
      { SERIALCONSOLE.print("Low Range "); }
    else if (Sen_Analogue_Num == 2)
      { SERIALCONSOLE.print("High Range"); }
  }
  
  if (settings.cursens == Sen_Analoguesing)
    {SERIALCONSOLE.print("Analogue Single ");}
  if (settings.cursens == Sen_Canbus)
    {SERIALCONSOLE.print("CANbus ");}
  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print(currentact);
  SERIALCONSOLE.print("mA");
  SERIALCONSOLE.print(" SOC: ");
  SERIALCONSOLE.print(SOC);
  SERIALCONSOLE.print("% ");
  SERIALCONSOLE.print(float(mampsecond) / 3600, 2);
  SERIALCONSOLE.print("mAh / ");
  SERIALCONSOLE.print(float(mWs) / 3600, 2);
  SERIALCONSOLE.print("mWh SOH: ");
  SERIALCONSOLE.print(SOH_calc());
  SERIALCONSOLE.print("% ");
  SERIALCONSOLE.print(settings.CAP);
  SERIALCONSOLE.print("/");
  SERIALCONSOLE.print(settings.designCAP);
  SERIALCONSOLE.print("Ah TCAP: ");
  SERIALCONSOLE.print(abs(TCAP));
  SERIALCONSOLE.print("Ah / TCAP_Wh: ");
  SERIALCONSOLE.print(abs(TCAP_Wh));
  SERIALCONSOLE.println("Wh");
  SERIALCONSOLE.print("Gauge PWM : ");
  SERIALCONSOLE.println(Gauge_update());
  SERIALCONSOLE.print("ETA : ");
  String eta = String(ETA() / 60) + "h " + String(ETA() % 60) + "m";
  SERIALCONSOLE.println(eta);
  SERIALCONSOLE.println();
  SERIALCONSOLE.println("Warning: ");
  for (byte i = 0; i < 4; i++){
    SERIALCONSOLE.println(warning[i], BIN);
  }
  SERIALCONSOLE.println();
  SERIALCONSOLE.println("Error: ");
  for (byte i = 0; i < 4; i++){
    SERIALCONSOLE.println(alarm[i], BIN);
  }
  SERIALCONSOLE.println();  
  if (alarm[2] & 0b01000000){
    SERIALCONSOLE.println("  ");
    SERIALCONSOLE.print("   !!! Internal Error / Series Cells Fault !!!");
    SERIALCONSOLE.println("  ");
  }
}

//called by Timer
void mAmpsec_calc(){ 
  unsigned long nowTime = millis();
  u_int16_t tmpTime = 0;
  long tmpmampsecond = 0;
  if((nowTime - lastTime) < 0){tmpTime = 4294967295 - lastTime + nowTime;} // Catch millis() overflow [ToTest]
  else{tmpTime = nowTime - lastTime;}
  tmpmampsecond = ((currentact+currentlast)/2 * tmpTime / 1000);
  mampsecondTimer += tmpmampsecond;
  mWsTimer += tmpmampsecond * bms.getPackVoltage();
  currentlast = currentact;
  lastTime = millis();  
}

float SEN_AnalogueRead(float tmp_currrentlast){
  float tmp_current = 0;
  switch (settings.cursens){
    case Sen_Analoguesing:
      Sen_Analogue_Num = 1;
      adc->adc0->startContinuous(ACUR1);   
    break;
    case Sen_Analoguedual:
      if (tmp_currrentlast < settings.changecur && tmp_currrentlast > (settings.changecur * -1)){
        if (Sen_Analogue_Num == 2){adc->adc0->stopContinuous();}
        Sen_Analogue_Num = 1;
        adc->adc0->startContinuous(ACUR1);
      }else{
        if (Sen_Analogue_Num == 1){adc->adc0->stopContinuous();}
        Sen_Analogue_Num = 2;
        adc->adc0->startContinuous(ACUR2);
      }    
    break;
  }

  Sen_AnalogueRawValue= (uint16_t)adc->adc0->analogReadContinuous(); 

  switch (Sen_Analogue_Num){
    case 1:
      RawCur = int16_t((Sen_AnalogueRawValue* 3300 / adc->adc0->getMaxValue()) - settings.offset1) / (settings.convlow / 100 / 1000 / (5 / 3.3));
      if (abs((int16_t(Sen_AnalogueRawValue* 3300 / adc->adc0->getMaxValue()) - settings.offset1)) <  settings.CurDead) { RawCur = 0; }
    break;
    case 2:
      RawCur = int16_t((Sen_AnalogueRawValue* 3300 / adc->adc0->getMaxValue()) - settings.offset2) / (settings.convhigh / 100 / 1000 / (5 / 3.3));
    break;
    }      

  lowpassFilter.input(RawCur);
  tmp_current = lowpassFilter.output();
  tmp_current = settings.ncur * tmp_current;
  if (settings.invertcur){ tmp_current *= -1; }
  return tmp_current;

} 

float CAN_SEN_read(CAN_message_t MSG){
  signed long CANmilliamps = 0;
  switch (MSG.id){
    //LEM CAB
    // [ToDo] CAB1500 has same ID. ALso same Data?
    case 0x3c0: CANmilliamps = CAN_SEN_LEMCAB(MSG); break; //CAB 300-C/SP3-000
    case 0x3c1: CANmilliamps = CAN_SEN_LEMCAB(MSG); break; //CAB 300-C/SP3-001
    case 0x3c2: CANmilliamps = CAN_SEN_LEMCAB(MSG); break; //CAB 300-C/SP3-002 & CAB 300-C/SP3-010 & CAB-500 (all Versions)
    case 0x3c3: CANmilliamps = CAN_SEN_LEMCAB(MSG); break; //CAB 300-C/SP3-003
    case 0x3c4: CANmilliamps = CAN_SEN_LEMCAB(MSG); break; //CAB 300-C/SP3-004
    case 0x3c5: CANmilliamps = CAN_SEN_LEMCAB(MSG); break; //CAB 300-C/SP3-005
    case 0x3c6: CANmilliamps = CAN_SEN_LEMCAB(MSG); break; //CAB 300-C/SP3-006
    case 0x3c7: CANmilliamps = CAN_SEN_LEMCAB(MSG); break; //CAB 300-C/SP3-007
    case 0x3c8: CANmilliamps = CAN_SEN_LEMCAB(MSG); break; //CAB 300-C/SP3-008
    case 0x3c9: CANmilliamps = CAN_SEN_LEMCAB(MSG); break; //CAB 300-C/SP3-009
    //IsaScale
    case 0x521: CANmilliamps = MSG.buf[5] + (MSG.buf[4] << 8) + (MSG.buf[3] << 16) + (MSG.buf[2] << 24); break;
    case 0x522: ISAVoltage1 = MSG.buf[5] + (MSG.buf[4] << 8) + (MSG.buf[3] << 16) + (MSG.buf[2] << 24); break;
    case 0x523: ISAVoltage2 = MSG.buf[5] + (MSG.buf[4] << 8) + (MSG.buf[3] << 16) + (MSG.buf[2] << 24); break;    
    default: break;
  }

  //Victron Lynx  
  if (pgnFromCANId(MSG.id) == 0x1F214 && MSG.buf[0] == 0) // Check PGN and only use the first packet of each sequence
  {CANmilliamps = CAN_SEN_VictronLynx(MSG);}

  if (settings.invertcur){ CANmilliamps *= -1; }
  return CANmilliamps;
}

signed long CAN_SEN_LEMCAB(CAN_message_t MSG){
  /*
  80000000H = 0 mA,
  7FFFFFFFH = −1 mA,
  80000001H = 1 mA
  Byte 0 = MSB
  Byte 3 = LSB
  */
 
  if(MSG.buf[0]==0xff && MSG.buf[1]==0xff && MSG.buf[2]==0xff && MSG.buf[3]==0xff && MSG.buf[4] & 8){return 0;} // check error bit 32. 1 = Error [ToTest]
  signed long LEM_milliamps = 0;
  for (int i = 0; i < 4; i++){LEM_milliamps = (LEM_milliamps << 8) | MSG.buf[i];}
  LEM_milliamps -= 0x80000000;
  //SERIALCONSOLE.println(LEM_milliamps); //debug
  return LEM_milliamps;
}

signed long CAN_SEN_VictronLynx(CAN_message_t MSG){
  signed long Victron_milliamps = 0;
  if (MSG.buf[4] == 0xff && MSG.buf[3] == 0xff) return 0;
  int16_t current = (int)MSG.buf[4] << 8; // in 0.1A increments
  current |= MSG.buf[3];
  Victron_milliamps = current * 100;
  return Victron_milliamps;
}

void CAN_Debug_IN(CAN_message_t MSG, byte CanNr){
  int pgn = 0;
  //if ((MSG.id & 0x10000000) == 0x10000000){   // Determine if ID is standard (11 bits) or extended (29 bits)
  switch (CanNr)
  {
  case 1: SERIALCONSOLE.print("CAN1 "); break;
  case 2: SERIALCONSOLE.print("CAN2 "); break;
  }
  SERIALCONSOLE.print("IN ");
  if (MSG.flags.extended){ //[ToTest] use FlexCAN parameter
    SERIALCONSOLE.println("ext:");
    pgn = pgnFromCANId(MSG.id);
    SERIALCONSOLE.print(pgn, HEX);
    SERIALCONSOLE.print("  ");
  } else {
    SERIALCONSOLE.print("std:");
    SERIALCONSOLE.print(MSG.id, HEX);
    SERIALCONSOLE.print("  ");
  }
  for (int i = 0; i < MSG.len; i++) { // print the data
    SERIALCONSOLE.print(MSG.buf[i], HEX);
    SERIALCONSOLE.print(" ");
  }
  SERIALCONSOLE.println();
}

void Currentavg_Calc(){
  //[ToDo] überarbeiten! (glätten, reset bei Statuswechsel)
  currentavg_array[currentavg_counter] = currentact;
  currentavg_counter++;
  if(currentavg_counter > 59){currentavg_counter = 0;}
  float current_temp = 0;
  for (int i=0; i<60; i++){
    current_temp += currentavg_array[i];
  }
  currentavg = current_temp / 60;
}

void Current_debug(){
  if ( settings.cursens == Sen_Analoguedual || settings.cursens == Sen_Analoguesing){
    if (Sen_Analogue_Num == 1){
      SERIALCONSOLE.println();
      if (settings.cursens == Sen_Analoguedual)
      {SERIALCONSOLE.print("Low Range: ");}
      else
      {SERIALCONSOLE.print("Single In: ");}
      SERIALCONSOLE.print("Value ADC0: ");
      SERIALCONSOLE.print(Sen_AnalogueRawValue* 3300 / adc->adc0->getMaxValue()); //- settings.offset1)
      SERIALCONSOLE.print(" ");
      SERIALCONSOLE.print(settings.offset1);
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(int16_t(Sen_AnalogueRawValue* 3300 / adc->adc0->getMaxValue()) - settings.offset1);
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(RawCur);
      SERIALCONSOLE.print(" mA");
      SERIALCONSOLE.print("  ");
    }else if (Sen_Analogue_Num == 2){
      SERIALCONSOLE.println();
      SERIALCONSOLE.print("High Range: ");
      SERIALCONSOLE.print("Value ADC0: ");
      SERIALCONSOLE.print(Sen_AnalogueRawValue* 3300 / adc->adc0->getMaxValue()); //- settings.offset2)
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(settings.offset2);
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print((float(Sen_AnalogueRawValue* 3300 / adc->adc0->getMaxValue()) - settings.offset2));
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(RawCur);
      SERIALCONSOLE.print("mA");
      SERIALCONSOLE.print("  ");
    }
    SERIALCONSOLE.print(lowpassFilter.output());
    SERIALCONSOLE.print(" | ");
    SERIALCONSOLE.print(settings.changecur);
    SERIALCONSOLE.print(" | ");
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA  ");
  }
  if ( settings.cursens == Sen_Canbus ) {
    SERIALCONSOLE.println();
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA  ");
  }
}


int ETA(){ // return minutes
  //[ToDo] weiter glätten! mögl. vordef./errechneten Durchschnittswert einbeziehen?
  if(BMC_Stat == Stat_Charge){
    return abs(mampsecond / 1000 / 60 / abs(currentavg/1000));
  }else{
    if (currentavg >= 0){
      return 0;
    }else {
      return abs((TCAP * 60 - mampsecond / 1000 / 60) / abs(currentavg/1000)); //[ToTest]
    }
  }
}

int SOH_calc(){
  return round(float(settings.CAP) / float(settings.designCAP) * 100);
}

void SOC_update(){
  //[ToDo]umbau auf Wh?
  int SOC_tmp = 0;
  if (!SOCset && bms.getAvgCellVolt() > 0){
    if (millis() > 10000){
      SOC_tmp = map(uint16_t(bms.getAvgCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);
      SOC = constrain(SOC_tmp, 0, 100);// keep SOC bettween 0 and 100
      mampsecond = (100 - SOC) * TCAP * (3600000/100);
      SERIALCONSOLE.println((100 - SOC) * TCAP_Wh * (3600000/100));
      mWs = (100 - SOC) * TCAP_Wh * (3600000/100); // immer 0?
      SOCset = 1;
      if (!menu_load){
        SERIALCONSOLE.println();
        SERIALCONSOLE.print("--------SOC SET: ");
        SERIALCONSOLE.print(SOC);
        SERIALCONSOLE.print("%, ");
        SERIALCONSOLE.print(mampsecond);
        SERIALCONSOLE.print("mAs, ");
        SERIALCONSOLE.print(mWs);
        SERIALCONSOLE.print("mWs");        
        SERIALCONSOLE.print("--------");
      }
    }
  } else {
      SOC_tmp = round((TCAP - mampsecond / 3600000) * 100 / TCAP);
      SOC = constrain(SOC_tmp, 0, 100);// keep SOC bettween 0 and 100
  }
  if (settings.voltsoc){
    SOC_tmp = map(uint16_t(bms.getAvgCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);
    SOC = constrain(SOC_tmp, 0, 100);// keep SOC between 0 and 100
    mampsecond = (TCAP * 1000 - (SOC * TCAP * settings.Pstrings * 10)) * 3600;
    mWs = (TCAP_Wh * 1000 - (SOC * TCAP_Wh * 10)) * 3600; //pstrings?
  }
}

void SOC_charged(){
    SOC = 100;
    mampsecond = 0;
    mWs = 0;
}

/*
  Returns capacity in mAs to substract, corrected by temperature factor.
  Uses:
    bms.getLowTemperature()
    settings.Temp_Cap_Map[x][x]
*/
//long CAP_Temp_alteration(){
float CAP_Temp_alteration(){
  float tmp_map = 0;

  //temp below lowest Setpoint
  if (bms.getLowTemperature() < float(settings.Temp_Cap_Map[0][0])){ 
    return settings.Temp_Cap_Map[1][0] / 100;
  }
  //temp above highest setpoint.
  if(bms.getLowTemperature() >= float(settings.Temp_Cap_Map[0][4])) {
    return float(settings.Temp_Cap_Map[1][4]) / 100;
  }
  //everything in between
  for (int i = 1; i < 4; i++){
    if (bms.getLowTemperature() >= float(settings.Temp_Cap_Map[0][i]) && bms.getLowTemperature() < float(settings.Temp_Cap_Map[0][i+1])){
      tmp_map = map(bms.getLowTemperature(), settings.Temp_Cap_Map[0][i], settings.Temp_Cap_Map[0][i+1],settings.Temp_Cap_Map[1][i],settings.Temp_Cap_Map[1][i+1]);
      return tmp_map / 100;
    }  
  }
  return 1; // failsafe
}

void CAP_recalc(){
  if ((BMC_Stat == Stat_Charge && abs(mampsecond) > (settings.CAP * 3600 * 1000)) || ((warning[0] & 0x10) && bms.getAvgTemperature() > 20.00)){
    settings.CAP = round((abs(mampsecond) / 3600) / 1000 / settings.Pstrings);
    Settings_unsaved = 1;
  }
  /*
  if ((BMC_Stat == Stat_Charge && abs(mWs) > (settings.CAP_Wh * 3600 * 1000)) || ((warning[0] & 0x10) && bms.getAvgTemperature() > 20.00)){
    settings.CAP_Wh = round((abs(mWs) / 3600) / 1000);
    Settings_unsaved = 1;
  } 
  */ 
}

//START new Output Control

/*
Mapping functions to outputs
OUT1 = 1, ...
Functions see Out_Functions[]
PWM: 0 - 255
*/
void set_OUT_Mapping(byte OUT,byte Function,byte PWM){
  settings.Out_Map[0][OUT] = Function;
  settings.Out_Map[1][OUT] = PWM;
}

/*
Return current output mapping
Field 0 --> Funktion
Field 1 --> PWM
*/
byte get_OUT_Mapping(byte OUT, byte Field){
  return settings.Out_Map[Field][OUT];
}

/*
Return index/OutputNr. for specified function
0 if nothing is found
*/
byte find_OUT_Mapping(byte Function){
  for(int i = 1; i < 9; i++){
    if(settings.Out_Map[0][i] == Function){
      return i;
    }
  } 
  return 0;
}
/*
Change the function of the given output
OUT1 = 1 ...
*/
void cycle_OUT_Mapping(byte OUT){
  byte i = get_OUT_Mapping(OUT,0) + 1;
  if(OUT<5 && i==Out_Gauge){i++;} //Gauge not possible on 12V outputs
  while(find_OUT_Mapping(i)){i++;} //doubles not possible
  if(i>11){i=0;} //max number of different Output Types
  set_OUT_Mapping(OUT,i,get_OUT_Mapping(OUT,1));
}

/*
no parameter = all functions off
Use parameter (function) to activate corresponding function.
*/
void set_OUT_States(byte Function){
  if (Function == 254){for (byte i = 0; i < 12; i++){Out_States[0][i] = 0;}} // all off
  else {Out_States[0][Function] = 1;}
}

// Set output pins according to Out_States.
void set_OUTs(){
  u_int32_t tmp_timer = millis() - cont_timer;
  for (byte i = 1; i < 9;i++){
    if(i<5){digitalWrite(*Outputs[i], Out_States[0][settings.Out_Map[0][i]]);} // digitalWrite(Outn, HIGH/LOW)
    if(i>=5){
      if(i != find_OUT_Mapping(Out_Gauge)){
        if( // find contactors & check if ON and TIMER left for full pull
          (i == find_OUT_Mapping(Out_Cont_Neg) || i == find_OUT_Mapping(Out_Cont_Pos) || i == find_OUT_Mapping(Out_Cont_Precharge)) && 
          (Out_States[0][settings.Out_Map[0][i]] && Out_States[1][settings.Out_Map[0][i]])
          ){
          analogWrite(*Outputs[i], Out_States[0][settings.Out_Map[0][i]]*255); // analogWrite(OUTn, HIGH/LOW * PWM)
          if(Out_States[1][settings.Out_Map[0][i]] > tmp_timer){Out_States[1][settings.Out_Map[0][i]] -= tmp_timer;} //reduce Timer
          else{Out_States[1][settings.Out_Map[0][i]] = 0;} // timer to 0
        } else if ( // find contactors & check if OFF to reset TIMER
          (i == find_OUT_Mapping(Out_Cont_Neg) || i == find_OUT_Mapping(Out_Cont_Pos) || i == find_OUT_Mapping(Out_Cont_Precharge)) && 
          (!Out_States[0][settings.Out_Map[0][i]] && Out_States[1][settings.Out_Map[0][i]] != cont_pulltime)
          ) {
            Out_States[1][settings.Out_Map[0][i]] = cont_pulltime;
        } else {
          analogWrite(*Outputs[i], Out_States[0][settings.Out_Map[0][i]]*settings.Out_Map[1][i]);// analogWrite(OUTn, HIGH/LOW * PWM)
        }
      }
    } 
  }
  cont_timer = millis();
}

void OUT_Debug(){
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Function         ");  
  SERIALCONSOLE.print("State ");  
  SERIALCONSOLE.println("Timer");  
  SERIALCONSOLE.println("---------------------------");  
  for(byte i = 1; i < 12; i++){
    SERIALCONSOLE.print(Out_Functions[i]);
    SERIALCONSOLE.print(" ");
    SERIALCONSOLE.print(Out_States[0][i]);
    SERIALCONSOLE.print("      ");
    SERIALCONSOLE.print(Out_States[1][i]);
    SERIALCONSOLE.println();
  } 
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("OUT ");  
  SERIALCONSOLE.print("Function         ");  
  SERIALCONSOLE.println("PWM");  
  SERIALCONSOLE.println("------------------------");  
  for(byte i = 1; i < 9; i++){
    SERIALCONSOLE.print(i);
    SERIALCONSOLE.print("   ");
    SERIALCONSOLE.print(Out_Functions[settings.Out_Map[0][i]]);
    SERIALCONSOLE.print(" ");
    if(i>4){
      SERIALCONSOLE.print(settings.Out_Map[1][i]);
    }
    SERIALCONSOLE.println();
  }
}

//END new Output Control

byte Gauge_update(){ 
  byte Gauge_PWM = 0;
  if(!find_OUT_Mapping(Out_Gauge)){return 0;}
  if (debug_Gauge) {
    if (!debug_Gauge_timer){ debug_Gauge_timer = millis()+30000; }
    if (millis() < debug_Gauge_timer){ Gauge_PWM = debug_Gauge_val;}
    else {
      debug_Gauge = 0;
      debug_Gauge_val = 0;
      debug_Gauge_timer = 0;
      Menu(); //go back to the menu since we came from there
    }
  } else { 
    Gauge_PWM = map(SOC, 0, 100, settings.gaugelow, settings.gaugehigh) * Out_States[0][Out_Gauge];
  }
  analogWrite(*Outputs[find_OUT_Mapping(Out_Gauge)], Gauge_PWM);
  return Gauge_PWM;
}

void Prechargecon(){
  if (!Pretimer){Pretimer = millis();}
  Out_States[0][Out_Cont_Neg] = 1;    
  if (Pretimer + settings.Pretime > millis()){ //  Add later (|| currentact < settings.Precurrent)
    Out_States[0][Out_Cont_Precharge] = 1; 
    precharged = 0;
  }else{ 
    Out_States[0][Out_Cont_Pos] = 1;
    Out_States[0][Out_Cont_Precharge] = 0;
    Pretimer = 0;
    precharged = 1;
  }
}

void CurrentOffsetCalc(){
  byte x = 0;
  adc->adc0->startContinuous(ACUR1);
  Sen_Analogue_Num = 1;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20){
    settings.offset1 = settings.offset1 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  settings.offset1 = settings.offset1 / 21;
  SERIALCONSOLE.print(settings.offset1);
  SERIALCONSOLE.print(" current offset 1 calibrated ");
  SERIALCONSOLE.println("  ");
  x = 0;
  adc->adc0->startContinuous(ACUR2);
  Sen_Analogue_Num = 2;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20){
    settings.offset2 = settings.offset2 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  settings.offset2 = settings.offset2 / 21;
  SERIALCONSOLE.print(settings.offset2);
  SERIALCONSOLE.print(" current offset 2 calibrated ");
  SERIALCONSOLE.println("  ");
}

void Menu(){
  String menu_option_string;
  char menu_option_char;
  int menu_option_val = 0;
//  menu_option = 0;
   
  if (SERIALCONSOLE.available()){ //verhindert doppeltes Auslesen
    menu_option_string = SERIALCONSOLE.readString(); 
    menu_option_char = menu_option_string.charAt(0);
    menu_option = menu_option_string.toInt();
    if (menu_option == 0){ menu_option = menu_option_char; }
    //menu_option_val = split(menu_option_string,'=',1);
    for (uint16_t i = 0; i < menu_option_string.length(); i++) {
      if (menu_option_string.substring(i, i+1) == "=") { 
        menu_option_val = menu_option_string.substring(i+1).toInt();
        break;
      }
    }
  } else { menu_option = 0; }
  
  if (menu_option != 109 && menu_load == 0){return;}
  else {menu_load = 1;}
  
//  ##debug##  
//Serial.print("Menu:"); Serial.println(menu_current);
//Serial.print("Opt:"); Serial.println(menu_option);
//Serial.print("Val:"); Serial.println(menu_option_val);
//Serial.print("String: "); Serial.println(menu_option_string);

  switch (menu_current) {
    case Menu_Start:
      menu_current = Menu_Main;
      Serial_clear();
      SERIALCONSOLE.println("Main");
      SERIALCONSOLE.println("--------------------");
      SERIALCONSOLE.println("[1] Battery");
      SERIALCONSOLE.println("[2] Current Sensor");
      SERIALCONSOLE.println("[3] Charger");
      SERIALCONSOLE.println("[4] Motor Controller");
      SERIALCONSOLE.println("[5] Outputs");
      SERIALCONSOLE.println("[6] Alarms");
      SERIALCONSOLE.println("[7] Ignore Values");
      SERIALCONSOLE.println("[8] CAN-Bus");
      SERIALCONSOLE.println("[9] Experiental");
      SERIALCONSOLE.println("[10] Debug");
      SERIALCONSOLE.println("[11] Reboot");
      SERIALCONSOLE.println("[12] Load Defaults");
      SERIALCONSOLE.print("[13] Mode: ");
      if(settings.ESSmode){
        SERIALCONSOLE.println("Stationary");
      }else{
        SERIALCONSOLE.println("Vehicle");
      }
      SERIALCONSOLE.println("[q] Save & Quit");
    break;
    case Menu_Main:
      switch (menu_option){
        case 1: menu_current = Menu_Battery; Menu(); break;
        case 2: menu_current = Menu_CurSen; Menu(); break;
        case 3: menu_current = Menu_Charger; Menu(); break;
        case 4: menu_current = Menu_MC; Menu(); break;
        case 5: menu_current = Menu_Outputs; Menu(); break;
        case 6: menu_current = Menu_Alarms; Menu(); break;  
        case 7: menu_current = Menu_IgnVal; Menu(); break;
        case 8: menu_current = Menu_CAN; Menu(); break;  
        case 9: menu_current = Menu_Exp; Menu(); break;   
        case 10: menu_current = Menu_Debug; Menu(); break; 
        //case 11: CPU_REBOOT; break;        //[ToDo]
        case 12: loadSettings(); Menu(); SERIALCONSOLE.println("::::::Defaults loaded::::::"); break;    
        case 13: settings.ESSmode = !settings.ESSmode; menu_current = Menu_Start; Menu(); break;       
        case 120: (_reboot_Teensyduino_()); //-> hidden Program Mode (x)
        case Menu_Quit:
          EEPROM.put(0, settings); //save all change to eeprom
          menu_current = Menu_Start;
          menu_load = 0;
          SERIALCONSOLE.println("::::::Settings saved::::::");
        break;
      }
    break;
    case Menu_Battery:
      switch (menu_option){
        case 1: settings.OverVSetpoint = float(menu_option_val) / 1000; Menu(); break;
        case 2: settings.ChargeVSetpoint = float(menu_option_val) / 1000; Menu(); break;
        case 3: settings.UnderVSetpoint = float(menu_option_val) / 1000; Menu(); break;
        case 4: settings.UnderVDerateSetpoint = float(menu_option_val) / 1000; Menu(); break;
        case 5: settings.OverTSetpoint = menu_option_val; Menu(); break;
        case 6: settings.OverTDerateSetpoint = menu_option_val; Menu(); break;
        case 7: settings.UnderTSetpoint = menu_option_val; Menu(); break;    
        case 8: settings.UnderTDerateSetpoint = menu_option_val; Menu(); break;    
        case 9: settings.balanceVoltage = float(menu_option_val) / 1000; Menu(); break;   
        case 10: settings.balanceHyst = float(menu_option_val) / 1000; Menu(); break;   
        case 11: settings.PackDisCurrentMax = menu_option_val*10; Menu(); break;
        case 12: settings.designCAP = menu_option_val; Menu(); break;
        case 13: settings.CAP = menu_option_val; Menu(); break;
        case 14: 
          settings.Pstrings = menu_option_val; 
          bms.setPstrings(settings.Pstrings); 
          BMS_revive(); // reset BMS with new Settings
          Menu(); 
        break;     
        case 15: settings.Scells = menu_option_val; Menu(); break;  
        case 16: settings.socvolt[0] = menu_option_val; Menu(); break;  
        case 17: settings.socvolt[1] = menu_option_val; Menu(); break; 
        case 18: settings.socvolt[2] = menu_option_val; Menu(); break;                                                               
        case 19: settings.socvolt[3] = menu_option_val; Menu(); break;  
        case 20: settings.StoreVsetpoint = float(menu_option_val) / 1000; Menu(); break;  
        case 21: settings.Temp_Cap_Map[0][0] = menu_option_val; Menu(); break;
        case 22: settings.Temp_Cap_Map[1][0] = menu_option_val; Menu(); break;
        case 23: settings.Temp_Cap_Map[0][1] = menu_option_val; Menu(); break;
        case 24: settings.Temp_Cap_Map[1][1] = menu_option_val; Menu(); break;
        case 25: settings.Temp_Cap_Map[0][2] = menu_option_val; Menu(); break;
        case 26: settings.Temp_Cap_Map[1][2] = menu_option_val; Menu(); break;
        case 27: settings.Temp_Cap_Map[0][3] = menu_option_val; Menu(); break;
        case 28: settings.Temp_Cap_Map[1][3] = menu_option_val; Menu(); break;
        case 29: settings.Temp_Cap_Map[0][4] = menu_option_val; Menu(); break;
        case 30: settings.Temp_Cap_Map[1][4] = menu_option_val; Menu(); break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.println("Battery");
          SERIALCONSOLE.println("--------------------");
          SERIALCONSOLE.println();
          SERIALCONSOLE.println("Cell limits:");    
          SERIALCONSOLE.print("[1] Cell Overvoltage (mV): ");
          SERIALCONSOLE.println(settings.OverVSetpoint * 1000, 0);
          SERIALCONSOLE.print("[2] Charge Voltage Setpoint (mV): ");
          SERIALCONSOLE.println(settings.ChargeVSetpoint * 1000, 0);
          SERIALCONSOLE.print("[3] Cell Undervoltage (mV): ");
          SERIALCONSOLE.println(settings.UnderVSetpoint * 1000, 0);
          SERIALCONSOLE.print("[4] Derate at Undervoltage (mV): ");
          SERIALCONSOLE.println(settings.UnderVDerateSetpoint * 1000, 0);          
          SERIALCONSOLE.print("[5] Over Temperature (°C): ");
          SERIALCONSOLE.println(settings.OverTSetpoint);
          SERIALCONSOLE.print("[6] Derate at high Temperature (°C): ");
          SERIALCONSOLE.println(settings.OverTDerateSetpoint);
          SERIALCONSOLE.print("[7] Under Temperature (°C): ");
          SERIALCONSOLE.println(settings.UnderTSetpoint);
          SERIALCONSOLE.print("[8] Derate at low Temperature (°C): ");
          SERIALCONSOLE.println(settings.UnderTDerateSetpoint);
          SERIALCONSOLE.print("[9] Cell Balance Voltage (mV): ");
          SERIALCONSOLE.println(settings.balanceVoltage * 1000, 0);
          SERIALCONSOLE.print("[10] Balance Hysteresis (mV): ");
          SERIALCONSOLE.println(settings.balanceHyst * 1000, 0);
          SERIALCONSOLE.print("[11] Pack Max Discharge (A): ");
          SERIALCONSOLE.println(settings.PackDisCurrentMax * 0.1);
          SERIALCONSOLE.println();
          SERIALCONSOLE.println("Pack configuration:");         
          SERIALCONSOLE.print("[12] Pack design Capacity (Ah): ");
          SERIALCONSOLE.println(settings.designCAP);
          SERIALCONSOLE.print("[13] Pack current Capacity (Ah): ");
          SERIALCONSOLE.println(settings.CAP); 
          SERIALCONSOLE.print("[14] Cells in Parallel: ");
          SERIALCONSOLE.println(settings.Pstrings);
          SERIALCONSOLE.print("[15] Cells in Series: ");
          SERIALCONSOLE.println(settings.Scells );
          SERIALCONSOLE.println();
          SERIALCONSOLE.println("Values for Voltage based SOC:");
          SERIALCONSOLE.print("[16] Setpoint1 (mV): ");
          SERIALCONSOLE.println(settings.socvolt[0] );
          SERIALCONSOLE.print("[17] SOC @ Setpoint1 (%): ");
          SERIALCONSOLE.println(settings.socvolt[1] );
          SERIALCONSOLE.print("[18] Setpoint2 (mV): ");
          SERIALCONSOLE.println(settings.socvolt[2] );
          SERIALCONSOLE.print("[19] SOC @ Setpoint2 (%): ");
          SERIALCONSOLE.println(settings.socvolt[3] );
          SERIALCONSOLE.print("[20] Storage Setpoint (mV): ");
          SERIALCONSOLE.println(settings.StoreVsetpoint * 1000, 0 );
          SERIALCONSOLE.println();
          SERIALCONSOLE.println("Temperature based SOC adjustment:");
          for (byte i = 0; i < 5; i++){
            byte y = i*2;
            SERIALCONSOLE.print("["+String(21+y)+"]");
            SERIALCONSOLE.print(" ");
            SERIALCONSOLE.print("T");
            SERIALCONSOLE.print(i+1);
            SERIALCONSOLE.print("(°C): ");
            SERIALCONSOLE.println(settings.Temp_Cap_Map[0][i]);

            SERIALCONSOLE.print("["+String(22+y)+"]");
            SERIALCONSOLE.print(" ");
            SERIALCONSOLE.print("C");
            SERIALCONSOLE.print(i+1);
            SERIALCONSOLE.print("(%): ");
            SERIALCONSOLE.println(settings.Temp_Cap_Map[1][i]);
          }
          SERIALCONSOLE.println();                    
          SERIALCONSOLE.println("[q] Quit");          
      }
    break; 
    case Menu_CurSen:
      switch (menu_option){
        case 1: settings.invertcur = !settings.invertcur; Menu(); break;
        case 2: settings.voltsoc = !settings.voltsoc;  Menu(); break;
        case 3: settings.ncur = menu_option_val; Menu(); break;
        case 4: 
          settings.cursens ++; 
          if (settings.cursens > 3){settings.cursens = 0;}
          Menu();
        break;
        /*
        case 5:
          settings.curcan++;
          if (settings.curcan > CANSen_CurCanMax) {settings.curcan = 1;}      
          Menu();    
        break;         
        */
        case 6: settings.convlow = menu_option_val; Menu(); break;
        case 7: settings.convhigh = menu_option_val; Menu(); break;
        case 8: settings.changecur = menu_option_val * 1000; Menu(); break;
        case 9: settings.CurDead = menu_option_val; Menu(); break;                 
        case 10: CurrentOffsetCalc(); Menu();break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        case 100: debug_Cur = !debug_Cur; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.println("Current Sensor");
          SERIALCONSOLE.println("--------------------");
          SERIALCONSOLE.print("[1] Invert Current: ");
          SERIALCONSOLE.println(settings.invertcur);
          SERIALCONSOLE.print("[2] Voltage based SOC: ");
          SERIALCONSOLE.println(settings.voltsoc);
          SERIALCONSOLE.print("[3] Current Multiplier: ");
          SERIALCONSOLE.println(settings.ncur);
          SERIALCONSOLE.print("[4] Sensor Type: ");  
          switch (settings.cursens) {
            case Sen_Canbus:
              SERIALCONSOLE.println(" Canbus Current Sensor ");
            break;
            case Sen_Analoguesing:
              SERIALCONSOLE.println(" Analogue Single");
              SERIALCONSOLE.print("[6] Analog Sensor 1 (low) mV/A: ");
              SERIALCONSOLE.println(settings.convlow * 0.01, 2);
              SERIALCONSOLE.print("[8] Current Sensor switch over in A: ");
              SERIALCONSOLE.println(settings.changecur / 1000);
              SERIALCONSOLE.print("[9] Current Sensor Deadband in mV: ");
              SERIALCONSOLE.println(settings.CurDead); 
              SERIALCONSOLE.println("[10] Calibrate");
            break;
            case Sen_Analoguedual:
              SERIALCONSOLE.println(" Analogue Dual");
              SERIALCONSOLE.print("[6] Analog Sensor 1 (low) mV/A: ");
              SERIALCONSOLE.println(settings.convlow * 0.01, 2);
              SERIALCONSOLE.print("[7] Analog Sensor 2 (high) mV/A: ");
              SERIALCONSOLE.println(settings.convhigh * 0.01, 2);  
              SERIALCONSOLE.print("[8] Current Sensor switch over in A: ");
              SERIALCONSOLE.println(settings.changecur / 1000);                          
              SERIALCONSOLE.print("[9] Current Sensor Deadband in mV: ");
              SERIALCONSOLE.println(settings.CurDead);  
              SERIALCONSOLE.println("[10] Calibrate");
            break;             
            default: SERIALCONSOLE.println("Undefined");
          }
          SERIALCONSOLE.println("[d] Debug");    
          SERIALCONSOLE.println("[q] Quit");         
      }
    break;  
    case Menu_Charger:
      switch (menu_option){
        case 1: settings.ChargeHys = float(menu_option_val) / 1000; Menu(); break;
        case 2: settings.ChargerChargeCurrentMax = menu_option_val * 10; Menu(); break;
        case 3: settings.chargecurrentend = menu_option_val * 10; Menu(); break;
        case 4:
          settings.chargertype++;
          if (settings.chargertype > 6){ settings.chargertype = 0; }
          Menu();
        break;
        case 5:
          if (settings.ChargerDirect){settings.ChargerDirect = 0;}
          else {settings.ChargerDirect = 1;}
          Menu();
        break;
        case 6:
          settings.nchargers = menu_option_val; Menu();
        break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.println("Charger");
          SERIALCONSOLE.println("--------------------");
          SERIALCONSOLE.print("[1] Charge Hysteresis in mV: ");
          SERIALCONSOLE.println(settings.ChargeHys * 1000, 0 );
          if (settings.chargertype > 0){
            SERIALCONSOLE.print("[2] Max Charge Current per Charger in A: ");
            SERIALCONSOLE.println(settings.ChargerChargeCurrentMax / 10);
            SERIALCONSOLE.print("[3] Pack End of Charge Current in A: ");
            SERIALCONSOLE.println(settings.chargecurrentend / 10);
          }
          SERIALCONSOLE.print("[4] Charger Type: ");
          switch (settings.chargertype){
            case 0: SERIALCONSOLE.println("Relay Control"); break;
            case 1: SERIALCONSOLE.println("Brusa NLG5xx");  break;
            case 2: SERIALCONSOLE.println("Volt Charger");  break;
            case 3: SERIALCONSOLE.println("Eltek Charger"); break;
            case 4: SERIALCONSOLE.println("Elcon Charger"); break;
            case 5: SERIALCONSOLE.println("Victron/SMA");   break;
            case 6: SERIALCONSOLE.println("Coda"); break;
          }        
          SERIALCONSOLE.print("[5] Charger HV Connection: ");
          switch (settings.ChargerDirect){
            case 0: SERIALCONSOLE.println("Behind Contactors"); break;
            case 1: SERIALCONSOLE.println("Direct To Battery HV"); break;
            default: SERIALCONSOLE.println(settings.ChargerDirect); break;
          }       
          SERIALCONSOLE.print("[6] Number of Chargers in parallel: ");
          SERIALCONSOLE.println(settings.nchargers);
          SERIALCONSOLE.println("[q] Quit"); 
      }
    break; 
    case Menu_MC:
      switch (menu_option){
        case 1:
          settings.mctype++;
          if (settings.mctype > 1){ settings.mctype = 0; }
          Menu();
          break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
        Serial_clear();
        SERIALCONSOLE.println("Motor Controller");
        SERIALCONSOLE.println("--------------------");
        SERIALCONSOLE.print("[1] Controller Type: ");
        switch (settings.mctype){
          case 0: SERIALCONSOLE.println("not configured"); break;
          case 1: SERIALCONSOLE.println("Curtis"); break;
          default: SERIALCONSOLE.println("undefined");
        }
        SERIALCONSOLE.println("[q] Quit"); 
      }
    break;
    case Menu_Outputs:
      switch (menu_option){
        case 1: cycle_OUT_Mapping(1); Menu(); break;
        case 2: cycle_OUT_Mapping(2); Menu(); break;
        case 3: cycle_OUT_Mapping(3); Menu(); break;
        case 4: cycle_OUT_Mapping(4); Menu(); break;
        case 5: cycle_OUT_Mapping(5); Menu(); break;
        case 6: set_OUT_Mapping(5,get_OUT_Mapping(5,0),menu_option_val); Menu(); break;
        case 7: cycle_OUT_Mapping(6); Menu(); break;
        case 8: set_OUT_Mapping(6,get_OUT_Mapping(6,0),menu_option_val); Menu(); break;
        case 9: cycle_OUT_Mapping(7); Menu(); break;
        case 10: set_OUT_Mapping(7,get_OUT_Mapping(7,0),menu_option_val); Menu(); break;
        case 11: cycle_OUT_Mapping(8); Menu(); break;
        case 12: set_OUT_Mapping(8,get_OUT_Mapping(8,0),menu_option_val); Menu(); break;
        case 20: settings.Pretime = menu_option_val; Menu(); break;
        case 21: settings.Precurrent = menu_option_val; Menu(); break;
        case 22: settings.conthold = menu_option_val; Menu(); break;
        case 23: settings.gaugelow = menu_option_val; Menu(); break;
        case 24: settings.gaugehigh = menu_option_val; Menu(); break;
        case 25:
          debug_Gauge_val = menu_option_val;
          debug_Gauge = 1;
          Menu();
          break;        
        case 100:
          debug_Output = !debug_Output;
          output_debug_counter = 0;
          Menu();
          break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.println("Outputs");
          SERIALCONSOLE.println("--------------------");
          SERIALCONSOLE.println("12V Outputs:");
          for (byte i = 1; i < 5; i++){
            byte tmp_out = i;
            SERIALCONSOLE.print("["+String(tmp_out)+"]");   
            SERIALCONSOLE.print(" Out"); 
            SERIALCONSOLE.print(tmp_out);   
            SERIALCONSOLE.print(": ");
            SERIALCONSOLE.println(Out_Functions[settings.Out_Map[0][i]]);                   
          }
          SERIALCONSOLE.println();
          SERIALCONSOLE.println("PWM/GND Outputs:");
          for (size_t i = 5; i < 9; i++){
            byte tmp_out = i;
            SERIALCONSOLE.print("["+String(2*i-5)+"]");   
            SERIALCONSOLE.print(" Out"); 
            SERIALCONSOLE.print(tmp_out);
            SERIALCONSOLE.print(": ");
            SERIALCONSOLE.println(Out_Functions[settings.Out_Map[0][i]]);
            SERIALCONSOLE.print("["+String(2*i-4)+"]");
            SERIALCONSOLE.print(" Out"); 
            SERIALCONSOLE.print(tmp_out);
            SERIALCONSOLE.print(" PWM: ");
            SERIALCONSOLE.println(settings.Out_Map[1][i]);
          }      
          SERIALCONSOLE.println();
          SERIALCONSOLE.println("Additional Settings:");                                              
          SERIALCONSOLE.print("[20] Precharge Timer in ms: ");
          SERIALCONSOLE.println(settings.Pretime);
          SERIALCONSOLE.print("[21] Precharge finish Current in mA: ");
          SERIALCONSOLE.println(settings.Precurrent);
          SERIALCONSOLE.print("[22] PWM Contactor hold [0-255]: ");
          SERIALCONSOLE.println(settings.conthold);
          SERIALCONSOLE.print("[23] PWM Gauge low [0-255]: ");
          SERIALCONSOLE.println(settings.gaugelow);
          SERIALCONSOLE.print("[24] PWM Gauge high [0-255]: ");
          SERIALCONSOLE.println(settings.gaugehigh);
          SERIALCONSOLE.print("[25] Gauge Test [0-255] for 30s: ");
          SERIALCONSOLE.println(debug_Gauge_val);
          SERIALCONSOLE.print("[d] Debug Output ");         
          if(debug_Output){ SERIALCONSOLE.println("ON"); } 
          else {SERIALCONSOLE.println("OFF");}
          SERIALCONSOLE.println("[q] Quit");          
      }
    break; 
    case Menu_Alarms:
      switch (menu_option){
        case 1: settings.WarnOff = float(menu_option_val) / 1000; Menu(); break;
        case 2: settings.CellGap = float(menu_option_val) / 1000; Menu(); break;
        case 3: settings.WarnToff = menu_option_val; Menu(); break;                        
        case 4: settings.error_delay = menu_option_val; Menu(); break;             
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.println("Alarms");
          SERIALCONSOLE.println("--------------------");
          SERIALCONSOLE.print("[1] Voltage Warning Offset in mV: ");
          SERIALCONSOLE.println(settings.WarnOff * 1000, 0);
          SERIALCONSOLE.print("[2] Cell Delta Voltage Alarm in mV: ");
          SERIALCONSOLE.println(settings.CellGap * 1000, 0);
          SERIALCONSOLE.print("[3] Temp Warning offset in °C: ");
          SERIALCONSOLE.println(settings.WarnToff);
          SERIALCONSOLE.print("[4] Error Delay: ");
          SERIALCONSOLE.println(settings.error_delay);
          SERIALCONSOLE.println("[q] Quit");          
      }
    break;   
    case Menu_IgnVal:
      switch (menu_option){
        case 1:
          settings.IgnoreTemp++;
          if (settings.IgnoreTemp > 2){settings.IgnoreTemp = 0;}
          BMS_revive(); // reset BMS with new Settings
          Menu();
        break; 
        case 2: 
          settings.IgnoreVolt = float(menu_option_val) / 1000; 
          BMS_revive(); // reset BMS with new Settings
          Menu(); break;       
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.println("Ignore Values");
          SERIALCONSOLE.println("--------------------");
          SERIALCONSOLE.print("[1] Use Temp Sensors: ");
          if (!settings.IgnoreTemp){SERIALCONSOLE.println("both");}
          else{SERIALCONSOLE.println(settings.IgnoreTemp);}
          SERIALCONSOLE.print("[2] Ignore Cells under mV: ");
          SERIALCONSOLE.println(settings.IgnoreVolt * 1000, 0);
          SERIALCONSOLE.println("[q] Quit"); 
      }
    break;  
    case Menu_CAN:
      switch (menu_option){
        case 1: settings.CAN1_Interval = menu_option_val; 
          can1_start();
          Menu(); 
        break;
        case 2: 
          settings.CAN1_Speed = menu_option_val * 1000;
          can1_start();
          Menu(); 
        break;  
        case 3: settings.CAN2_Interval = menu_option_val; 
          can2_start();
          Menu(); 
        break;
        case 4: 
          settings.CAN2_Speed = menu_option_val * 1000;
          can2_start();
          Menu(); 
        break;  
        case 5:
          if(menu_option_val >= 0 && menu_option_val < 4){settings.CAN_Map[0] = menu_option_val;}
          Menu(); 
        break;
        case 6:
          if(menu_option_val >= 0 && menu_option_val < 4){settings.CAN_Map[1] = menu_option_val;}
          Menu(); 
        break;
        case 7:
          if(menu_option_val >= 0 && menu_option_val < 4){settings.CAN_Map[2] = menu_option_val;}
          Menu(); 
        break;
        case 8:
          if(menu_option_val >= 0 && menu_option_val < 4){settings.CAN_Map[3] = menu_option_val;}
          Menu(); 
        break;
        case 9:
          if(menu_option_val >= 0 && menu_option_val < 4){settings.CAN_Map[4] = menu_option_val;}
          Menu(); 
        break;
        case 111: CO_Send_SDO(0x26,2,0,0x1800,0x02,0); break; // "o" for SDO
        case 112: CO_SDO_send_test(0x26); break; // "p" for PDO
        case 100: debug_CAN1 = !debug_CAN1; debug_CAN2 = !debug_CAN2; Menu(); break;    
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.println("CAN-Bus settings");
          SERIALCONSOLE.println("--------------------");
          SERIALCONSOLE.println("CAN1:");
          SERIALCONSOLE.print("[1] CAN Msg Speed in ms: ");
          SERIALCONSOLE.println(settings.CAN1_Interval);
          SERIALCONSOLE.print("[2] Can Baudrate in kbps: ");
          SERIALCONSOLE.println(settings.CAN1_Speed / 1000);
          SERIALCONSOLE.println();
          SERIALCONSOLE.println("CAN2:");
          SERIALCONSOLE.print("[3] CAN Msg Speed in ms: ");
          SERIALCONSOLE.println(settings.CAN2_Interval);
          SERIALCONSOLE.print("[4] Can Baudrate in kbps: ");
          SERIALCONSOLE.println(settings.CAN2_Speed / 1000);
          SERIALCONSOLE.println();
          SERIALCONSOLE.print("[d] Debug CAN1&2: ");
          if(debug_CAN2){ SERIALCONSOLE.println("ON"); } 
          else {SERIALCONSOLE.println("OFF");}
          SERIALCONSOLE.println();
          SERIALCONSOLE.println("Functions:");
          for (size_t i = 0; i < 5; i++){
            SERIALCONSOLE.print("[");
            SERIALCONSOLE.print(5+i);
            SERIALCONSOLE.print("] ");
            switch (i){
              case 0: SERIALCONSOLE.print("BMC Data Output: "); break;
              case 1: SERIALCONSOLE.print("BMS: "); break;
              case 2: SERIALCONSOLE.print("Charger: "); break;
              case 3: SERIALCONSOLE.print("Current Sensor: "); break;
              case 4: SERIALCONSOLE.print("Motor Controller: "); break;
            }
            switch (settings.CAN_Map[i]){
              case 0: SERIALCONSOLE.println("not set"); break;
              case 1: SERIALCONSOLE.println("CAN1"); break;
              case 2: SERIALCONSOLE.println("CAN2"); break;
              case 3: SERIALCONSOLE.println("CAN1 & CAN2"); break;
            }
          }
          
          SERIALCONSOLE.println("[q] Quit"); 
      }
    break;     
    case Menu_Exp:
      switch (menu_option){
        //case 1: settings.ExpMess = !settings.ExpMess; Menu(); break;
        case 2: settings.SerialCan++; if(settings.SerialCan > 2){settings.SerialCan = 0;} Menu(); break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.println("Experimental");
          SERIALCONSOLE.println("--------------------");
          /*
          SERIALCONSOLE.print("1 Send Experimental Victron CAN: ");
          SERIALCONSOLE.println(settings.ExpMess);
          */
          SERIALCONSOLE.print("[2] Second Serial Port: ");
          switch (settings.SerialCan){
            case 0: SERIALCONSOLE.println("Serial Display"); break;
            case 1: SERIALCONSOLE.println("Can Bus Expansion"); break;
            case 2: SERIALCONSOLE.println("Bluetooth App"); break;
          }
          SERIALCONSOLE.println("[q] Quit");      
      }
    break; 
    case Menu_Debug:
      switch (menu_option){
        case 1: debug_CAN1 = !debug_CAN1; Menu(); break;
        case 2: debug_Cur = !debug_Cur; Menu(); break;
        case 3:
          debug_Output = !debug_Output;
          output_debug_counter = 0;
          Menu();
        break;
        case 4: debug_Input = !debug_Input; Menu(); break;
        case 5: BMS_revive(); Menu(); break;
        case 6: debug_CSV = !debug_CSV; Menu(); break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.println("Debug");
          SERIALCONSOLE.println("--------------------");
          SERIALCONSOLE.println("[1] CAN");
          SERIALCONSOLE.println("[2] Current");
          SERIALCONSOLE.print("[3] Output Check: ");
          if (debug_Output){SERIALCONSOLE.println("ON");} 
          else{SERIALCONSOLE.println("OFF");}
          SERIALCONSOLE.println("[4] Input Check");
          SERIALCONSOLE.println("[5] BMS revive");
          SERIALCONSOLE.println("[6] CSV Output");
          SERIALCONSOLE.println("[q] Quit");  
      }
    break;          
  }
}

void Serial_clear(){
  for (byte i = 0; i < 40; i++){SERIALCONSOLE.println();}
}

int pgnFromCANId(int canId){ //Parameter Group Number
  if ((canId & 0x10000000) == 0x10000000)
  {return (canId & 0x03FFFF00) >> 8;}
  else
  {return canId;}
}

void ChargeCurrentLimit(){
  //[debug]
  /*
  float CellV = 4.13;
  float PackV = CellV * settings.Scells;
  */

  ///Start at no derating///
  chargecurrent = settings.ChargerChargeCurrentMax;
  u_int16_t EndCurrent = settings.chargecurrentend / settings.nchargers;
  u_int16_t tmp_chargecurrent = 0;

  if (bms.getHighCellVolt() > settings.OverVSetpoint){ chargecurrent = 0; }

  //Modifying Charge current
  if (chargecurrent > 0){
    //Temperature based
    if (bms.getLowTemperature() < settings.UnderTDerateSetpoint){
      chargecurrent = map(bms.getLowTemperature()*1000, settings.UnderTSetpoint*1000, settings.UnderTDerateSetpoint*1000, 0, settings.ChargerChargeCurrentMax);
    }
    if (bms.getHighTemperature() > settings.OverTDerateSetpoint){
      chargecurrent = map(bms.getHighTemperature()*1000, settings.OverTDerateSetpoint*1000, settings.OverTSetpoint*1000, settings.ChargerChargeCurrentMax, 0);
    }    
    //Voltage based
    if (storagemode){
      float upperStoreVLimit = settings.StoreVsetpoint - settings.ChargeHys/2;
      if (bms.getHighCellVolt() > upperStoreVLimit){
        tmp_chargecurrent = map(bms.getHighCellVolt()*1000, upperStoreVLimit*1000, settings.StoreVsetpoint*1000, settings.ChargerChargeCurrentMax, EndCurrent);
        if(tmp_chargecurrent < chargecurrent){
          chargecurrent = tmp_chargecurrent;
          constrain(chargecurrent,settings.chargecurrentend,settings.ChargerChargeCurrentMax);
        }
      }
    } else { 
      float upperVLimit = settings.ChargeVSetpoint - settings.ChargeHys/2;
      if (bms.getHighCellVolt() /*CellV*/ > upperVLimit){
        tmp_chargecurrent = map(bms.getHighCellVolt()*1000 /*CellV*/, upperVLimit*1000, settings.ChargeVSetpoint*1000, settings.ChargerChargeCurrentMax, EndCurrent);
        if(tmp_chargecurrent < chargecurrent){
          chargecurrent = tmp_chargecurrent;
          constrain(chargecurrent,settings.chargecurrentend,settings.ChargerChargeCurrentMax);
        }
      }
    }
    //16A Limit
    tmp_chargecurrent = 3600 / bms.getPackVoltage() /*PackV*/ * 95 / 10; //3,6kW max, 95% Eff. , factor 10 [ToDo] make conf. + CAN
    if(tmp_chargecurrent < chargecurrent){
      chargecurrent = tmp_chargecurrent;
      constrain(chargecurrent,settings.chargecurrentend,settings.ChargerChargeCurrentMax);
    }
  }   

  //compensate for consumers if Chargecurrent is 0 [ToTest]
  //[ToDO] besser glätten, schwankt sehr stark
  if(chargecurrent == 0 && currentact < 0){
    chargecurrent = chargecurrentlast + (currentact / 100 / settings.nchargers) * -1;
  }

  //[ToDo] implement feedback loop
  //multiply with calculated current
  chargecurrentFactor = chargecurrent / (currentact / 100);

  constrain(chargecurrent,0,settings.ChargerChargeCurrentMax); //[ToTest]
  constrain(chargecurrent,0,settings.PackChargeCurrentMax / settings.nchargers); //[ToTest]
  chargecurrentlast = chargecurrent;
}

void DischargeCurrentLimit(){
  ///Start at no derating///
  discurrent = settings.PackDisCurrentMax;
  u_int16_t tmp_discurrent = 0;

  //Modifying discharge current//
  if (discurrent > 0){
    //Temperature based//
    if (bms.getHighTemperature() > settings.OverTDerateSetpoint){
      discurrent = map(bms.getHighTemperature(), settings.OverTDerateSetpoint, settings.OverTSetpoint, settings.PackDisCurrentMax, 0);
    }
    //Voltage based//
    if (bms.getLowCellVolt() < (settings.UnderVDerateSetpoint + settings.DisTaper)){
      tmp_discurrent = map(bms.getLowCellVolt(), (settings.UnderVDerateSetpoint + settings.DisTaper), settings.UnderVDerateSetpoint, settings.PackDisCurrentMax, 0);
      if(discurrent>tmp_discurrent){discurrent = tmp_discurrent;}//don't override temperature based modification
    }
  }
  constrain(discurrent,0,settings.PackDisCurrentMax);
}


void Input_Debug(){
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Input: ");
  if (digitalRead(IN1_Key) == HIGH){SERIALCONSOLE.print("1 ON  ");}
  else {SERIALCONSOLE.print("1 OFF ");}
  if (digitalRead(IN2_Gen) == HIGH){ SERIALCONSOLE.print("2 ON  ");}
  else {SERIALCONSOLE.print("2 OFF ");}
  if (digitalRead(IN3_AC) == HIGH){SERIALCONSOLE.print("3 ON  ");}
  else {SERIALCONSOLE.print("3 OFF ");}
  if (digitalRead(IN4) == HIGH){SERIALCONSOLE.print("4 ON  ");}
  else {SERIALCONSOLE.print("4 OFF ");}
  SERIALCONSOLE.println();
}

void Output_debug(){
  if (output_debug_counter < 10){
    digitalWrite(OUT1, HIGH);
    digitalWrite(OUT2, HIGH);
    digitalWrite(OUT3, HIGH);
    digitalWrite(OUT4, HIGH);
    analogWrite(OUT5, 255);
    analogWrite(OUT6, 255);
    analogWrite(OUT7, 255);
    analogWrite(OUT8, 255);
  }else{
    digitalWrite(OUT1, LOW);
    digitalWrite(OUT2, LOW);
    digitalWrite(OUT3, LOW);
    digitalWrite(OUT4, LOW);
    analogWrite(OUT5, 0);
    analogWrite(OUT6, 0);
    analogWrite(OUT7, 0);
    analogWrite(OUT8, 0);
  }
  output_debug_counter ++;
  if (output_debug_counter > 20){output_debug_counter = 0;}
}

/*
void WDOG_reset(){
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}
*/

void Dash_update(){
  //power gauge
  int dashpower = 0; //in W
  dashpower = round(currentact/1000*bms.getPackVoltage());

  //temp gauge
  int dashtemp = 0;
  dashtemp = round(bms.getAvgTemperature());

  //voltage gauge
  int dashvolt = 0;
  dashvolt = round(map(bms.getPackVoltage(), settings.Scells * settings.UnderVSetpoint, settings.Scells * settings.OverVSetpoint, 5, 95));
  dashvolt = constrain(dashvolt, 0, 100);

  String eta = "";
  if (BMC_Stat == Stat_Drive || BMC_Stat == Stat_Charge){
    eta = "\"" + String(ETA() / 60) + "h " + String(ETA() % 60) + "m" + "\"";
  } else {
    eta = "\"-\"";
  }
  
  switch (BMC_Stat){
      case (Stat_Boot): Nextion_send("stat.txt=", "\" Boot \""); break;
      case (Stat_Ready): Nextion_send("stat.txt=", "\" Ready \""); break;
      case (Stat_Precharge): Nextion_send("stat.txt=", "\" Precharge \""); break;
      case (Stat_Drive): Nextion_send("stat.txt=", "\" Drive \""); break;
      case (Stat_Charge): Nextion_send("stat.txt=", "\" Charge \""); break;
      case (Stat_Charged): Nextion_send("stat.txt=", "\" Charged \""); break;
      case (Stat_Error): Nextion_send("stat.txt=", "\" Error \""); break;
      case (Stat_Healthy): Nextion_send("stat.txt=", "\" Healthy \""); break;
  }
  
  Nextion_send("soc.val=", SOC);
  Nextion_send("soc1_gauge.val=", SOC);
  Nextion_send("ah.val=", int(mampsecond / 3600000));
  Nextion_send("current.val=", int(currentact / 100));
  Nextion_send("power.val=", dashpower);
  Nextion_send("temp.val=", int(bms.getAvgTemperature()));
  Nextion_send("temp1.val=", dashtemp);
  Nextion_send("templow.val=", int(bms.getLowTemperature()));
  Nextion_send("temphigh.val=", int(bms.getHighTemperature()));
  Nextion_send("volt.val=", int(bms.getPackVoltage() * 10));
  Nextion_send("volt1_gauge.val=", dashvolt);
  Nextion_send("lowcell.val=", int(bms.getLowCellVolt() * 1000));
  Nextion_send("highcell.val=", int(bms.getHighCellVolt() * 1000));
  Nextion_send("firm.val=", firmver);
  Nextion_send("celldelta.val=", int((bms.getHighCellVolt() - bms.getLowCellVolt()) * 1000));
  Nextion_send("cellbal.val=", bms.getBalancing());
  Nextion_send("debug.val=", uint32_t(mWs / 3600000));
  Nextion_send("eta.txt=", eta);
  Nextion_send("etad.txt=", eta);
  Nextion_send("click ", "refresh,1"); //use Refresh-Button to Update Values on the Display
  Nextion_send("click ", "refresh,0");

  //Err_Warn-LED
  int Err_Warn = 0;
  if (BMC_Stat == Stat_Error){
    Err_Warn = 63488; // red
  } else if (Warn_handle()){
    Err_Warn = 65504; // yellow
  }
  Nextion_send("Err_Warn.bco=", Err_Warn);
  Nextion_send("Err_Warn.pco=", Err_Warn);
}

void Nextion_send(String var, String val){
  DisplaySerial.print(var);
  DisplaySerial.print(val);
  DisplaySerial.write(0xff);
  DisplaySerial.write(0xff);
  DisplaySerial.write(0xff);  
}

void can1_start(){
  can1.begin();
  can1.setBaudRate(settings.CAN1_Speed);
}

void can2_start(){
  can2.begin();
  can2.setBaudRate(settings.CAN2_Speed);
}

void CAN_read(){
  CAN_message_t MSG;

  while (can1.read(MSG)){
    if (settings.cursens == Sen_Canbus && settings.CAN_Map[CAN_Curr_Sen] & 1){currentact =  CAN_SEN_read(MSG);}
    if (settings.mctype && settings.CAN_Map[CAN_MC] & 1){CAN_MC_read(MSG);}
    if (debug_CAN1){CAN_Debug_IN(MSG, 1);}
  }
  while (can2.read(MSG)){
    if (settings.cursens == Sen_Canbus && settings.CAN_Map[CAN_Curr_Sen] & 2){currentact =  CAN_SEN_read(MSG);}
    if (settings.mctype && settings.CAN_Map[CAN_MC] & 2){CAN_MC_read(MSG);}
    if (debug_CAN2){CAN_Debug_IN(MSG, 2);}
  }
}

void CAN_BMC_send(byte CAN_Nr) //BMC CAN Messages
{
  CAN_message_t MSG;
  MSG.id  = 0x351;
  MSG.len = 8;
  if (storagemode){
    MSG.buf[0] = lowByte(uint16_t((settings.StoreVsetpoint * settings.Scells ) * 10));
    MSG.buf[1] = highByte(uint16_t((settings.StoreVsetpoint * settings.Scells ) * 10));    
  }else{
    MSG.buf[0] = lowByte(uint16_t((settings.ChargeVSetpoint * settings.Scells ) * 10));
    MSG.buf[1] = highByte(uint16_t((settings.ChargeVSetpoint * settings.Scells ) * 10));
  }
  MSG.buf[2] = lowByte(chargecurrent);
  MSG.buf[3] = highByte(chargecurrent);
  MSG.buf[4] = lowByte(discurrent);
  MSG.buf[5] = highByte(discurrent);
  MSG.buf[6] = lowByte(uint16_t((settings.UnderVDerateSetpoint * settings.Scells) * 10));
  MSG.buf[7] = highByte(uint16_t((settings.UnderVDerateSetpoint * settings.Scells) * 10));
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}

  MSG.id  = 0x355;
  MSG.len = 8;
  MSG.buf[0] = lowByte(SOC);
  MSG.buf[1] = highByte(SOC);
  MSG.buf[2] = lowByte(SOH_calc());
  MSG.buf[3] = highByte(SOH_calc());
  MSG.buf[4] = lowByte(SOC * 10);
  MSG.buf[5] = highByte(SOC * 10);
  MSG.buf[6] = 0;
  MSG.buf[7] = 0;
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}

  MSG.id  = 0x356;
  MSG.len = 8;
  MSG.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
  MSG.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  MSG.buf[2] = lowByte(long(currentact / 100));
  MSG.buf[3] = highByte(long(currentact / 100));
  MSG.buf[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
  MSG.buf[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
  MSG.buf[6] = 0;
  MSG.buf[7] = 0;
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}

  //[ToDo] Pylontech Errors & Warnings
  //MSG.id = 0x359

  //SMA Alarms & Warnings
  delay(2);
  MSG.id  = 0x35A;
  MSG.len = 8;
  MSG.buf[0] = alarm[0];
  MSG.buf[1] = alarm[1];
  MSG.buf[2] = alarm[2];
  MSG.buf[3] = alarm[3];
  MSG.buf[4] = warning[0];
  MSG.buf[5] = warning[1];
  MSG.buf[6] = warning[2];
  MSG.buf[7] = warning[3];
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}

  MSG.id  = 0x35E;
  MSG.len = 8;
  MSG.buf[0] = bmcname[0];
  MSG.buf[1] = bmcname[1];
  MSG.buf[2] = bmcname[2];
  MSG.buf[3] = bmcname[3];
  MSG.buf[4] = bmcname[4];
  MSG.buf[5] = bmcname[5];
  MSG.buf[6] = bmcname[6];
  MSG.buf[7] = bmcname[7];
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}

  delay(2);
  MSG.id  = 0x370;
  MSG.len = 8;
  MSG.buf[0] = bmcmanu[0];
  MSG.buf[1] = bmcmanu[1];
  MSG.buf[2] = bmcmanu[2];
  MSG.buf[3] = bmcmanu[3];
  MSG.buf[4] = bmcmanu[4];
  MSG.buf[5] = bmcmanu[5];
  MSG.buf[6] = bmcmanu[6];
  MSG.buf[7] = bmcmanu[7];
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}

/* [ToDo] what for?
  if (balancecells == 1){
    MSG.id = 0x3c3;
    MSG.len = 8;
    if (bms.getLowCellVolt() < settings.balanceVoltage){
      MSG.buf[0] = highByte(uint16_t(settings.balanceVoltage * 1000));
      MSG.buf[1] = lowByte(uint16_t(settings.balanceVoltage * 1000));
    }
    else{
      MSG.buf[0] = highByte(uint16_t(bms.getLowCellVolt() * 1000));
      MSG.buf[1] = lowByte(uint16_t(bms.getLowCellVolt() * 1000));
    }
    MSG.buf[2] = 0x01;
    MSG.buf[3] = 0x04;
    MSG.buf[4] = 0x03;
    MSG.buf[5] = 0x00;
    MSG.buf[6] = 0x00;
    MSG.buf[7] = 0x00;
   if(CAN_Nr & 1){can1.write(MSG);}
   if(CAN_Nr & 2){can2.write(MSG);}
  }
*/
 delay(2);
  MSG.id  = 0x372;
  MSG.len = 8;
  MSG.buf[0] = lowByte(bms.getNumModules());
  MSG.buf[1] = highByte(bms.getNumModules());
  MSG.buf[2] = 0x00;
  MSG.buf[3] = 0x00;
  MSG.buf[4] = 0x00;
  MSG.buf[5] = 0x00;
  MSG.buf[6] = 0x00;
  MSG.buf[7] = 0x00;
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}

  delay(2);
  MSG.id  = 0x373;
  MSG.len = 8;
  MSG.buf[0] = lowByte(uint16_t(bms.getLowCellVolt() * 1000));
  MSG.buf[1] = highByte(uint16_t(bms.getLowCellVolt() * 1000));
  MSG.buf[2] = lowByte(uint16_t(bms.getHighCellVolt() * 1000));
  MSG.buf[3] = highByte(uint16_t(bms.getHighCellVolt() * 1000));
  MSG.buf[4] = lowByte(uint16_t(bms.getLowTemperature() + 273.15));
  MSG.buf[5] = highByte(uint16_t(bms.getLowTemperature() + 273.15));
  MSG.buf[6] = lowByte(uint16_t(bms.getHighTemperature() + 273.15));
  MSG.buf[7] = highByte(uint16_t(bms.getHighTemperature() + 273.15));
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}

  delay(2);
  MSG.id  = 0x379; //Installed capacity
  MSG.len = 2;
  MSG.buf[0] = lowByte(uint16_t(settings.Pstrings * settings.CAP));
  MSG.buf[1] = highByte(uint16_t(settings.Pstrings * settings.CAP));
  MSG.buf[2] = 0x00;
  MSG.buf[3] = 0x00;
  MSG.buf[4] = 0x00;
  MSG.buf[5] = 0x00;
  MSG.buf[6] = 0x00;
  MSG.buf[7] = 0x00;
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
}

void CAN_Charger_Send(byte CAN_Nr){
  CAN_message_t MSG;
  // CAN intervall [ToDo] Intervall for CAN2?
  if (millis() - looptime1 < settings.CAN1_Interval){return;}
  looptime1 = millis();

  switch (settings.chargertype){
    case Charger_Elcon:
      MSG.id  =  0x1806E5F4; //broadcast to all Elcons
      MSG.len = 8;
      MSG.flags.extended = 1;
      MSG.buf[0] = highByte(uint16_t(settings.ChargeVSetpoint * settings.Scells * 10));
      MSG.buf[1] = lowByte(uint16_t(settings.ChargeVSetpoint * settings.Scells * 10));
      MSG.buf[2] = highByte(chargecurrent);
      MSG.buf[3] = lowByte(chargecurrent);
      MSG.buf[4] = 0x00;
      MSG.buf[5] = 0x00;
      MSG.buf[6] = 0x00;
      MSG.buf[7] = 0x00;

      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
      MSG.flags.extended = 0;
    break;

    case Charger_Eltek:
      MSG.id  = 0x2FF; //broadcast to all Elteks
      MSG.len = 7;
      MSG.buf[0] = 0x01;
      MSG.buf[1] = lowByte(1000);
      MSG.buf[2] = highByte(1000);
      MSG.buf[3] = lowByte(uint16_t(settings.ChargeVSetpoint * settings.Scells * 10));
      MSG.buf[4] = highByte(uint16_t(settings.ChargeVSetpoint * settings.Scells * 10));
      MSG.buf[5] = lowByte(chargecurrent);
      MSG.buf[6] = highByte(chargecurrent);

      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
    break;

    case Charger_BrusaNLG5:
      MSG.id  = chargerid1;
      MSG.len = 7;
      MSG.buf[0] = 0x80;

      if (digitalRead(IN2_Gen) == LOW){ //Gen OFF
        MSG.buf[1] = highByte(maxac1 * 10);
        MSG.buf[2] = lowByte(maxac1 * 10);
      }else{
        MSG.buf[1] = highByte(maxac2 * 10);
        MSG.buf[2] = lowByte(maxac2 * 10);
      }
      MSG.buf[5] = highByte(chargecurrent);
      MSG.buf[6] = lowByte(chargecurrent);
      MSG.buf[3] = highByte(uint16_t(((settings.ChargeVSetpoint * settings.Scells ) - chargerendbulk) * 10));
      MSG.buf[4] = lowByte(uint16_t(((settings.ChargeVSetpoint * settings.Scells ) - chargerendbulk)  * 10));
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}

      delay(2);

      MSG.id  = chargerid2;
      MSG.len = 7;
      MSG.buf[0] = 0x80;
      if (digitalRead(IN2_Gen) == LOW){ //Gen OFF
        MSG.buf[1] = highByte(maxac1 * 10);
        MSG.buf[2] = lowByte(maxac1 * 10);
      }else{
        MSG.buf[1] = highByte(maxac2 * 10);
        MSG.buf[2] = lowByte(maxac2 * 10);
      }
      MSG.buf[3] = highByte(uint16_t(((settings.ChargeVSetpoint * settings.Scells ) - chargerend) * 10));
      MSG.buf[4] = lowByte(uint16_t(((settings.ChargeVSetpoint * settings.Scells ) - chargerend) * 10));
      MSG.buf[5] = highByte(chargecurrent);
      MSG.buf[6] = lowByte(chargecurrent);
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
    break;

    case Charger_ChevyVolt:
      MSG.id  = 0x30E;
      MSG.len = 1;
      MSG.buf[0] = 0x02; //only HV charging , 0x03 hv and 12V charging
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}

      MSG.id  = 0x304;
      MSG.len = 4;
      MSG.buf[0] = 0x40; //fixed
      if ((chargecurrent * 2) > 255){MSG.buf[1] = 255;}
      else{MSG.buf[1] = (chargecurrent * 2);}
      if ((settings.ChargeVSetpoint * settings.Scells ) > 200){
        MSG.buf[2] = highByte(uint16_t((settings.ChargeVSetpoint * settings.Scells ) * 2));
        MSG.buf[3] = lowByte(uint16_t((settings.ChargeVSetpoint * settings.Scells ) * 2));
      }else{
        MSG.buf[2] = highByte( 400);
        MSG.buf[3] = lowByte( 400);
      }
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
    break;

    case Charger_Coda:
      MSG.id  = 0x050;
      MSG.len = 8;
      MSG.buf[0] = 0x00;
      MSG.buf[1] = 0xDC;
      if ((settings.ChargeVSetpoint * settings.Scells ) > 200){
        MSG.buf[2] = highByte(uint16_t((settings.ChargeVSetpoint * settings.Scells ) * 10));
        MSG.buf[3] = lowByte(uint16_t((settings.ChargeVSetpoint * settings.Scells ) * 10));
      }else{
        MSG.buf[2] = highByte( 400);
        MSG.buf[3] = lowByte( 400);
      }
      MSG.buf[4] = 0x00;
      if ((settings.ChargeVSetpoint * settings.Scells)*chargecurrent < 3300){
        MSG.buf[5] = highByte(uint16_t(((settings.ChargeVSetpoint * settings.Scells) * chargecurrent) / 240));
        MSG.buf[6] = highByte(uint16_t(((settings.ChargeVSetpoint * settings.Scells) * chargecurrent) / 240));
      }else{  //15 A AC limit
        MSG.buf[5] = 0x00;
        MSG.buf[6] = 0x96;
      }
      MSG.buf[7] = 0x01; //HV charging
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
    break;
  }
}

void CAN_MC_read(CAN_message_t MSG){
  uint32_t tmp_id = 0;
  if(settings.mctype == Curtis){
    tmp_id = CO_Handle(MSG);
    if (tmp_id){
        //CO_Send_SDO(tmp_id,2,1,0x1005,0x00);
        // 1001 Errors
        // 1003 more Errors
        // 1005 SYNC
        // 1006 Cycle period
        // 1007 Sync time windows
        //CO_PDO1_send(tmp_id);
        //CO_PDO2_send(tmp_id);
    }
  }
}

void CAN_Debug_OUT(){
  SERIALCONSOLE.print("OUT: ");
  SERIALCONSOLE.print(outMsg.id, HEX); //debug
  if (outMsg.len){
    for (int i = 0; i < (outMsg.len + 1); i++){
      SERIALCONSOLE.print(" ");
      SERIALCONSOLE.print(outMsg.buf[i], HEX); //debug
    }
  }
  SERIALCONSOLE.println();
}

//
//CANOpen implementation
//

// NMT messages to control CANOpen nodes.
bool CO_NMT(uint32_t CO_Target_ID, uint8_t CO_Target_State){
  outMsg.id = 0x000;
  outMsg.len = 2; //1Byte requested State, 1Byte addressed Node
  if (CO_Target_State & 0x83){ //check for valid State values
    //0x01 Start Node, 0x02 Stop Node, 0x80 Pre-Operational, 0x81 reset application, 0x82 reset communication
    outMsg.buf[0] = CO_Target_State;
    outMsg.buf[1] = CO_Target_ID;
    can1.write(outMsg);
    return 1;
  }else{return 0;}
}

// SYNC message to sync devices & trigger PDOs from servers
void CO_SYNC(){
  CAN_message_t SyncMSG;
  if(settings.mctype == Curtis){
    SyncMSG.id = 0x80;
    SyncMSG.len = 0;
    can1.write(SyncMSG);
  //  CO_PDO1_send(0x26);
  //  CO_PDO2_send(0x26);
    //if(debug_CAN1){CAN_Debug_OUT();}
  }
}

void CO_PDO1_send(uint32_t CO_Target_ID){
  outMsg.id = 0x200 + CO_Target_ID;
  outMsg.len = 8;
  //msg.rtr = 1;
  outMsg.buf[0] = 0x00;
  outMsg.buf[1] = 0x00;
  outMsg.buf[2] = 0x00;
  outMsg.buf[3] = 0x00;
  outMsg.buf[4] = 0x00;
  outMsg.buf[5] = 0x00;
  outMsg.buf[6] = 0x00;
  outMsg.buf[7] = 0x00;
  can1.write(outMsg);
  if(debug_CAN1){CAN_Debug_OUT();}
}

void CO_PDO2_send(uint32_t CO_Target_ID){
  outMsg.id = 0x300 + CO_Target_ID;
  outMsg.len = 8;
  //msg.rtr = 1;
  outMsg.buf[0] = 0x00;
  outMsg.buf[1] = 0x00;
  outMsg.buf[2] = 0x00;
  outMsg.buf[3] = 0x00;
  outMsg.buf[4] = 0x00;
  outMsg.buf[5] = 0x00;
  outMsg.buf[6] = 0x00;
  outMsg.buf[7] = 0x00;
  can1.write(outMsg);
  if(debug_CAN1){CAN_Debug_OUT();}

}

void CO_SDO_send_test(uint32_t CO_Target_ID){
  
  outMsg.buf[4] = 0x00;
  outMsg.buf[5] = 0x00;
  outMsg.buf[6] = 0x00;
  outMsg.buf[7] = 0x00;
  /*
  CO_Send_SDO(0x26,1,1,0x1800,0x02,1);
  */
  CO_Send_SDO(CO_Target_ID,1,0,0x1800,0x02,1);
 
  //CO_PDO1_send(CO_Target_ID);
  //CO_PDO2_send(CO_Target_ID);
  //CO_Send_SDO(CO_Target_ID,2,1,0x1400,0x00);
}

/*
Send SDOs to request or write single Data from / to the Object Dictionary
CO_Target_ID: NodeId (0x00-0xFF)
CCS: 1 - write, 2 - read
expedited: 1 - send/receive in one Frame, 0 - segmented
index: OD index
subindex: OD subindex
datasize: how many Bytes to write/send (0 - 4)
*/
void CO_Send_SDO(uint32_t CO_Target_ID, uint8_t CCS, uint8_t expedited, uint16_t index, uint8_t subindex, uint8_t datasize){
  //[ToDo] Funktionsaufruf um Übergabe für Byte 4-7 erweitern?
  uint8_t command_byte = 0;
  outMsg.id = 0x600 + CO_Target_ID; //COB-ID
  outMsg.len = 8;

  if(datasize > 0 && datasize < 5){
    //Datasize: 1Byte = 11b, 2Byte = 10b, 3Byte = 01b, 4Byte = 00b
    datasize = 4 - datasize;
    command_byte |= datasize << 2; //set datasize
    command_byte |= 1; //set datasize shown
  }else{datasize = 0;}

  if (CCS == 1 || CCS == 2){
    //1 write,Download 2 read/upload
    command_byte |= CCS << 5;
  } else {return;}
  
  if (expedited){
    //activate single-frame transer
    command_byte |= 2;
  }
  outMsg.buf[0] = command_byte;
  outMsg.buf[1] = lowByte(index);
  outMsg.buf[2] = highByte(index);
  outMsg.buf[3] = subindex;
  outMsg.buf[4] = 0x00; // lowest Data, first Byte
  outMsg.buf[5] = 0x00;
  outMsg.buf[6] = 0x00;
  outMsg.buf[7] = 0x00; // highest Data, Last byte
  can1.write(outMsg);
  if(debug_CAN1){CAN_Debug_OUT();}
}

// Handle incomming CANOpen messages
// returns sender CAN-ID
uint32_t CO_Handle(CAN_message_t MSG){
  // Tx Read Data Server -> Client
  // Rx write Data Client -> Server

  if (MSG.id & 0x580){
    //SDO Tx: eg. reply to SDO Rx
  }
  if (MSG.id & 0x600){
    //SDO Rx: see CO_Send_SDO
  }
  if (MSG.id & 0x180){
    //PDO1 Tx
  }
  if (MSG.id & 0x200){
    //PDO1 Rx
  }
  if (MSG.id & 0x280){
    //PDO2 Tx
  }  
  if (MSG.id & 0x300){
    //PDO2 Rx
  }
  if (MSG.id & 0x380){
    //PDO3 Tx
  }
  if (MSG.id & 0x400){
    //PDO3 Rx
  }      
  if (MSG.id & 0x480){
    //PDO4 Tx
  }
  if (MSG.id & 0x500){
    //PDO4 Rx
  }    
  if (MSG.id & 0x700){
    //NMT Heartbeats / Status
    if(MSG.buf[0] == 0x00){}//boot: just wait
    if(MSG.buf[0] == 0x04){CO_NMT(0x00, co_state_preop);}//stopped: bring into pre-operational
    if(MSG.buf[0] == 0x05){return (MSG.id & ~0x700);}//operational: [ToDo] Do nothing? Send SYNC/rPDO in interval?
    if(MSG.buf[0] == 0x7f){CO_NMT(0x00, co_NMT_operational);}//pre-operational: bring into operational
  }  
  
  return 0;
}
