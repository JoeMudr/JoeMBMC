/*
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
#include <BMSManager.h>
#include <Arduino.h>
#include <config.h>
#include <EEPROM.h>
#include <ADC.h>          //https://github.com/pedvide/ADC
#include <FlexCAN_T4.h>   //https://github.com/tonton81/FlexCAN_T4
#include <Filters.h>      //https://github.com/JonHub/Filters
#include <Watchdog_t4.h>  //https://github.com/tonton81/WDT_T4

/////Version Identifier/////////
uint32_t firmver = 240327;

//Tesla_BMSModuleManager bms;
BMSManager bms;
EEPROMSettings settings;

//Curent filter//
byte filterFrequency = 5 ;
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );

//BMC wiring//
//#define doesn't work here because Pins are used in Arrays below.
const byte ACUR2 = A0;      // analogue current sensor 1
const byte ACUR1 = A1;      // analogue current sensor 2
const byte IN1_Key = 19;    // input 1 - high active
const byte IN2_Gen = 18;    // input 2 - high active
const byte IN3_AC = 20;     // input 3 - high active
const byte IN4 = 21;        // input 4 - high active
const byte OUT1 = 6;        // output 1 - high active
const byte OUT2 = 5;        // output 2 - high active
const byte OUT3 = 4;        // output 3 - high active
const byte OUT4 = 3;        // output 4 - high active
const byte OUT5 = 11;       // output 5 - Low active / PWM
const byte OUT6 = 12;       // output 6 - Low active / PWM
const byte OUT7 = 10;       // output 7 - Low active / PWM
const byte OUT8 = 9;        // output 8 - Low active / PWM
const byte LED = 13;

uint32_t LED_Timer = 0;

byte BMC_Stat = 0;

//Serial Menu
bool menu_load = 0;
byte menu_option = 0;
byte menu_current = 0;

//Errors
uint32_t error_timer = 0;

byte Settings_unsaved= 0;

//variables for output control
uint32_t Pretimer;
uint16_t pwmfreq = 18000;//pwm frequency

uint16_t chargecurrent = 0; // in 0,1A
uint16_t chargecurrentlast = 0; // in 0,1A
//float chargecurrentFactor = 0.0f; //correction factor, when using multiple chargers
uint16_t discurrent = 0;

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

uint32_t warning_timer = 0;

unsigned char bmcname[8] = {'J', 'O', 'E', 'M', ' ', 'B', 'M', 'C'};
unsigned char bmcmanu[8] = {'J', 'O', 'E', 'M', ' ', 'T', 'E', 'C'};

int32_t ISAVoltage1, ISAVoltage2, ISAVoltage3 = 0; //mV only with ISAscale sensor

uint32_t BMSLastRead = 0;

//variables for current/capacity calulation
uint16_t Sen_AnalogueRawValue;
int32_t currentact = 0; // mA
int32_t currentlast = 0; // mA
int32_t currentavg = 0; // mA
int32_t currentavg_array[60];
byte currentavg_counter = 0;
int32_t RawCur;
int32_t mampsecond = 0; // Range 0 = full to settings.cap * -1 = empty
volatile int32_t mampsecondTimer = 0;
uint32_t lastTime;
uint32_t looptime, looptime1, UnderTime, cleartime, baltimer = 0; //ms
byte Sen_Analogue_Num = 1; // 1 = Sensor 1; 2 = Sensor 2
int16_t TCAP = 0; //Temperature corrected Capacity in Ah including settings.Pstrings!
int16_t TCAP_Wh = 0;
//Variables for SOC calc
byte SOC = 100; //State of Charge
byte SOCset = 0;

//charger variables
byte maxac1 = 16; //Shore power 16A per charger
byte maxac2 = 10; //Generator Charging
const int16_t chargerid1 = 0x618; //bulk chargers (brusa)
const int16_t chargerid2 = 0x638; //finishing charger (brusa)
uint16_t chargerendbulk = 0; //mV before Charge Voltage to turn off the bulk charger/s
uint16_t chargerend = 0; //mV before Charge Voltage to turn off the finishing charger/s

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
const byte *Outputs[9] = {0,&OUT1,&OUT2,&OUT3,&OUT4,&OUT5,&OUT6,&OUT7,&OUT8};

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
  "Gauge"
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

uint32_t cont_timer = 0;
uint32_t Out_PWM_fullpull_Timer = 0; // PWM Timer

// CAN
FlexCAN_T4<CAN1> can1;
FlexCAN_T4<CAN2> can2;

uint32_t CAN_BMC_Std_send_Timer = 0;

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

ADC *adc = new ADC(); // adc object

// TCU Variables
byte TCU_Pump = 0;
byte TCU_Cool = 0;
byte TCU_Heat = 0;
byte TCU_REL4 = 0;

IntervalTimer Timer_mAmpSec;

void loadSettings(){
  settings.version = EEPROM_VERSION;
  settings.checksum = 2;
  settings.ReadTimeout = 2000;
  settings.batteryID = 0x01; // in the future should be 0xFF to force it to ask for an address
  settings.BMSType = BMS_Dummy;
  settings.OverVSetpoint = 4200;
  settings.ChargeVSetpoint = 4150;
  settings.UnderVSetpoint = 3000;
  settings.UnderVDerateSetpoint = 3200;
  settings.ChargeHys = 100; // mV drop required for charger to kick back on
  settings.WarnVoltageOffset = 100; // mV offset to raise a warning
  settings.DischHys = 200; // Discharge voltage offset
  settings.CellGap = 500; // max delta between high and low cell
  settings.OverTSetpoint = 550; // 0.1°C
  settings.OverTDerateSetpoint = 450; // 0.1°C
  settings.UnderTSetpoint = 0; // 0.1°C
  settings.UnderTDerateSetpoint = 50; // 0.1°C
  settings.WarnTempOffset = 50; // 0.1°C, temp offset before raising warning
  settings.useTempSensor = 0; // 0 - use both sensors, 1 or 2 only use that sensor
  settings.IgnoreVolt = 500; //
  settings.balanceVoltage = 3700;
  settings.balanceHyst = 10;
  settings.balanceDuty = 50;
  settings.CAP = 230; // battery size in Ah
  settings.designCAP = 230; // battery design capazity in Ah
  settings.CAP_Wh = 25000;
  settings.Pstrings = 1; // strings in parallel
  settings.Scells = 30;// Cells in series
  settings.StoreVsetpoint = 3800; // mV storage mode charge max
  settings.PackDisCurrentMax = 5500; // max discharge current in 0.1A
  settings.DisTaper = 300; //mV offset to bring in discharge taper to Zero Amps at settings.UnderVDerateSetpoint
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
  settings.Precurrent = 1000; //ma before closing main contator
  settings.convhigh = 312; // mV/A current sensor high range channel (*100)
  settings.convlow = 1250; // mV/A current sensor low range channel (*100)
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
  settings.Temp_Cap_Map[0][0] = -200;
  settings.Temp_Cap_Map[0][1] = -100;
  settings.Temp_Cap_Map[0][2] = 0;
  settings.Temp_Cap_Map[0][3] = 250;
  settings.Temp_Cap_Map[0][4] = 400;
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
  settings.CAN_Map[0][i] = 0;
  settings.CAN_Map[1][i] = 0;
  }
}

WDT_T4<WDT1> watchdog;
WDT_timings_t watchdog_config;

uint32_t lastResetCause;

void setup(){

  // Save copy of Reset Status Register
  lastResetCause = SRC_SRSR;
  // Clear all Reset Status Register bits
  SRC_SRSR = (uint32_t)0x1FF;

  pinMode(IN1_Key, INPUT);
  pinMode(IN2_Gen, INPUT);
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
  pinMode(LED, OUTPUT);

  analogWriteFrequency(OUT5, pwmfreq);
  analogWriteFrequency(OUT6, pwmfreq);
  analogWriteFrequency(OUT7, pwmfreq);
  analogWriteFrequency(OUT8, pwmfreq);

  EEPROM.get(0, settings);
  if (settings.version != EEPROM_VERSION){ loadSettings(); }
  
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
  SERIALCONSOLE.printf("Starting up...\r\n");
  SERIALCONSOLE.printf("JoeM BMC\r\n");

  //display and can adpater canbus
  DisplaySerial.begin(115200); 

  // enable WDT
  watchdog.begin(watchdog_config);

  //Tesla serial bus
  SERIALBMS.begin(612500);
  SERIALCONSOLE.printf("Started serial interface to Tesla-BMS.\r\n");

  BMSInit();

  Timer_mAmpSec.begin(mAmpsec_calc, 500000);
}

//called by Timer
void mAmpsec_calc(){ 
  uint32_t nowTime = millis();
  float tmpTime = 0;
  int32_t tmpmampsecond = 0;
  tmpTime = nowTime - lastTime;
  if((nowTime - lastTime) < 0){tmpTime = 4294967295 - lastTime + nowTime;} // Catch millis() overflow [ToTest]
  tmpmampsecond = ((currentact+currentlast)/2 * tmpTime / 1000);
  mampsecondTimer += tmpmampsecond;
  currentlast = currentact;
  lastTime = millis();  
}

void loop(){
  BMC_Status_LED();

  //everything CAN related happens here
  CAN_read();

  if ( settings.cursens == Sen_Analoguedual || settings.cursens == Sen_Analoguesing){
    currentact = SEN_AnalogueRead(currentlast);
  }

  // make serial console available after 3s
  if (millis() > 3000 && SERIALCONSOLE.available()){Menu();} 

  WarnAlarm_Check();

  if (!debug_Output){
    if (settings.ESSmode){BMC_Stat = ESS_CondCheck(BMC_Stat);}
    else {BMC_Stat = Vehicle_CondCheck(BMC_Stat);}
    set_BMC_Status(BMC_Stat);
    set_OUTs();
  }

  if(debug_Cur){Current_debug();}

  // Poll the BMS/CMUs
  // Interval restrictions are managed in BMSManager.cpp
  CAN_Struct CAN_Msg;
  CAN_Msg = bms.poll();
  for (byte CANMsgNr = 0; CANMsgNr < CAN_Struct_size; CANMsgNr++){
    if (CAN_Msg.Frame[CANMsgNr].id){ // bms.poll() returns a previously cleaned CAN_Struct. We only send actually filled Frames.
      if (settings.CAN_Map[0][CAN_BMS] & 1){can1.write(CAN_Msg.Frame[CANMsgNr]);}
      if (settings.CAN_Map[0][CAN_BMS] & 2){can2.write(CAN_Msg.Frame[CANMsgNr]);}
      delay(1);
    }
  }

  if (millis() - looptime > 500){ // 0.5s loop
    looptime = millis();

    // TCAP is negative. 0 = full, settings.CAP * -1 is empty.
    TCAP = settings.Pstrings * CAP_Temp_alteration() * -1;
    TCAP_Wh = TCAP * round(double(bms.getPackVoltage()) / 1000) * -1; // [ToDo] value will fluctuate

    // copy Ampseconds from Timer Funktion
    noInterrupts();
    mampsecond += mampsecondTimer;
    if(mampsecond > 0){mampsecond = 0;}
    mampsecondTimer = 0;
    interrupts();

    if (!menu_load){ 
      // main debug output
      SERIALCONSOLEprint(); 
      // OUT functions and states
      OUT_Debug();
      // Display reason the Teensy was last reset
      Reset_Cause(lastResetCause);
    } //"debug" output on serial console
    //if (debug_CSV){ bms.printAllCSV(millis(), currentact, SOC); }
    if (debug_Input){ Input_Debug(); }
    if (debug_Output){ Output_debug(); }
    else{ Gauge_update(); }
    
    SOC_update();
    //if(settings.CAN_Map[0][CAN_BMC_std]){CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);} // send unconditionally
    
    Currentavg_Calc();

    if (settings.SerialCan == 0){ Dash_update(); }//Info on serial bus 2
    //if (settings.SerialCan == 2){ BT_update(); }// --> Bluetooth App
    
    //WDT reset;
    watchdog.feed();
  }

}

/* ==================================================================== */
// i.MX RT1060 Processor Reference Manual, 21.8.3 SRC Reset Status Register
void Reset_Cause(uint32_t resetStatusReg) {
  bool info = false;
  SERIALCONSOLE.printf("\r\nReason for last Reset: ");
  if (resetStatusReg & SRC_SRSR_TEMPSENSE_RST_B) {
      Serial.printf("Temperature Sensor Software Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_WDOG3_RST_B) {
      Serial.printf("IC Watchdog3 Timeout Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_JTAG_SW_RST) {
      Serial.printf("JTAG Software Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_JTAG_RST_B) {
      Serial.printf("High-Z JTAG Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_WDOG_RST_B) {
      Serial.printf("IC Watchdog Timeout Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_IPP_USER_RESET_B) {
      Serial.printf("Power-up Sequence (Cold Reset Event)\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_CSU_RESET_B) {
      Serial.printf("Central Security Unit Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_LOCKUP_SYSRESETREQ) {
      Serial.printf("CPU Lockup or Software Reset\r\n");
      info = true;
      /* Per datasheet: "SW needs to write a value to SRC_GPR5
       * before writing the SYSRESETREQ bit and use the SRC_GPR5
       * value to distinguish if the reset is caused by SYSRESETREQ
       * or CPU lockup."
       */
  }
  if (resetStatusReg & SRC_SRSR_IPP_RESET_B) {
      Serial.printf("Power-up Sequence\r\n");
      info = true;
  }
  if (!info) {
      Serial.printf("No status bits set in SRC Reset Status Register\r\n");
  }
}

void BMSInit(){
  bms.initBMS(settings.BMSType,settings.IgnoreVolt,settings.useTempSensor,settings.ReadTimeout);
}

void WarnAlarm_Check(){
  //warnings
  for(byte i = 0; i<4; i++){warning[i] = 0b10101010;} // reset to all OK

  if (bms.getHighCellVolt() > (settings.OverVSetpoint - settings.WarnVoltageOffset)){ warning[0] ^= 0b00001100;}
  if (bms.getLowCellVolt() < (settings.UnderVSetpoint + settings.WarnVoltageOffset)){
    warning[0] ^= 0b00110000;
    if (!warning_timer){warning_timer = millis() + 10000;}
    if (millis() > warning_timer){CAP_recalc();warning_timer = 0;} // recalculate capacity when lowest cell is under warning setpoint; reset Timer to prevent EEPROM Kill
  } else {warning_timer = 0;}

  if (bms.getHighTemperature() > (settings.OverTSetpoint - settings.WarnTempOffset)){warning[0] ^= 0b11000000;}
  if (bms.getLowTemperature() < (settings.UnderTSetpoint + settings.WarnTempOffset)){warning[1] ^= 0b00000011;}
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap){warning[3] ^= 0b00000011;}
  if (bms.getSeriesCells() != settings.Scells * settings.Pstrings){warning[2] ^= 0b11000000;}
 
  //Errors
  for(byte i = 0; i<4; i++){alarm[i] = 0b10101010;} // reset to all OK

  if (bms.getHighCellVolt() > settings.OverVSetpoint){alarm[0] ^= 0b00001100;}
  if (bms.getLowCellVolt() < settings.UnderVSetpoint){alarm[0] ^= 0b00110000;}
  if (bms.getHighTemperature() > settings.OverTSetpoint){alarm[0] ^= 0b11000000;}
  if (bms.getLowTemperature() < settings.UnderTSetpoint){alarm[1] ^= 0b00000011;}
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap){alarm[3] ^= 0b00000011;} // [ToDO] bmsinit?
  //if (bms.getSeriesCells() != settings.Scells * settings.Pstrings){alarm[2] ^= 0b11000000;}
}



byte Vehicle_CondCheck(byte tmp_status){ 
  // start with no Errors
  tmp_status = Stat_Ready;

  // Reset to Ready if all Inputs are LOW ?
  // if (digitalRead(IN1_Key) == LOW && digitalRead(IN2_Gen) == LOW && digitalRead(IN3_AC) == LOW && digitalRead(IN4) == LOW){tmp_status = Stat_Ready;}

  // detect KEY ON & AC OFF -> Drive
  if (digitalRead(IN1_Key) == HIGH && ChargeActive() == false){
  //if (ChargeActive() == false){
    if (precharged) {tmp_status = Stat_Drive;}
    else {tmp_status = Stat_Precharge;}
  } 

  // detect Undervoltage before Charging / let Charging override this Error
  if (alarm[0] & 0b00010000) {tmp_status = Stat_Error;}

  //detect AC present & Check charging conditions
  if (ChargeActive() == true){
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

  // Set Error depending on Error conditions, except Undervoltage to let Charging recover from Undervoltage
  // Undertemp is no Error in Drive & Charge (--> derate)
  //[ToDO] Fix Tlow --> "Separate temperaturer thresholds for charge / discharge #18 "
  if (alarm[0] & 0b01000101 || alarm[3] & 0b00000001){tmp_status = Stat_Error;}


  // reset Error-Timer
  if (tmp_status != Stat_Error){error_timer = 0;}
  return tmp_status; 
}

byte ESS_CondCheck(byte tmp_status){
  // Precharge first
  if (precharged){tmp_status = Stat_Healthy;}
  else {tmp_status = Stat_Precharge;}  
  //tmp_status = Stat_Healthy;
  if (digitalRead(IN1_Key) == HIGH) { 
    //[ToDo] storagemode
  }

  // detect Undervoltage before Charging / let Charging override this Error
  if (alarm[0] & 0b00010000) {tmp_status = Stat_Error;}
  
  if (bms.getHighCellVolt() > (settings.ChargeVSetpoint)){
    SOC_charged();
  }

  // Set Error depending on Error conditions, except Undervoltage to let Charging recover from Undervoltage
  if (alarm[0] & 0b01000101 || (alarm[1] & 0b00000001)  || alarm[2] & 0b01010100){tmp_status = Stat_Error;}

  if (tmp_status != Stat_Error){error_timer = 0;}
  
  return tmp_status;
}

/*
  Detect charging via external inputs like IN3_AC, CAN-bus...
*/
bool ChargeActive(){
  bool retVal = false;
  // Add more charge detections like CAN
  if (digitalRead(IN3_AC) == HIGH){retVal = true;}
  return retVal;
}

byte Warn_Out_handle(){
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
      Warn_Out_handle();
    break;
    case Stat_Drive:
      set_OUT_States(); // all off
      set_OUT_States(Out_Cont_Pos);
      set_OUT_States(Out_Cont_Neg);
      set_OUT_States(Out_Gauge);
      DischargeCurrentLimit();
      Warn_Out_handle();
      Balancing(0);
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;
    case Stat_Precharge:
      set_OUT_States(); // all off
      Prechargecon();
      Balancing(0);
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;    
    case Stat_Charge:
      set_OUT_States(); // all off
      CAP_recalc();
      if (Settings_unsaved){EEPROM.put(0, settings); Settings_unsaved = 0; SERIALCONSOLE.printf("--Saved--\r\n");}
      set_OUT_States(Out_Charge); // enable charger
      if (digitalRead(IN1_Key) == HIGH){set_OUT_States(Out_Gauge);} // enable gauge if key is on
      Balancing();
      CAN_Charger_Send(settings.CAN_Map[0][CAN_Charger]);
      ChargeCurrentLimit();
      Warn_Out_handle();
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;
    case Stat_Charged:
      set_OUT_States(); // all off
      if (digitalRead(IN1_Key) == HIGH){set_OUT_States(Out_Gauge);} // enable gauge if key is on    
      Balancing();
      chargecurrentlast = 0;
      Warn_Out_handle();
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;
    case Stat_Error:
      if (!error_timer){ error_timer = millis() + settings.error_delay; }//10s delay before turning everything off
      if (millis() > error_timer){
        set_OUT_States(); // all off
        set_OUT_States(Out_Error);
        set_OUT_States(Out_Err_Warn);
        if (digitalRead(IN1_Key) == HIGH){set_OUT_States(Out_Gauge);} // enable gauge if key is on    
        if (digitalRead(IN1_Key) == HIGH && ChargeActive()){CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);}
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
      Warn_Out_handle();
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;
  } 
}

void BMC_Status_LED(){
  if(BMC_Stat == Stat_Error){
    if(!LED_Timer){LED_Timer = millis();}
    if(millis() - LED_Timer > 500){
      digitalWrite(LED,!digitalRead(LED));
      LED_Timer = millis();
    }
  } else {
    digitalWrite(LED, HIGH);
  }
}

byte Balancing(byte active){
  if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst && active){
    bms.Balancing(settings.balanceHyst, true);
    return 1;
  } else {
    bms.Balancing(settings.balanceHyst, false);
    return 0;
  }
}

void SERIALCONSOLEprint(){
  Serial_clear();
  SERIALCONSOLE.printf("Firmware:   %i\r\n", firmver);
  SERIALCONSOLE.printf("BMC Status: ");
  switch (BMC_Stat){
    case (Stat_Boot): SERIALCONSOLE.printf("Boot"); break;
    case (Stat_Ready): SERIALCONSOLE.printf("Ready"); break;
    case (Stat_Precharge): SERIALCONSOLE.printf("Precharge"); break;
    case (Stat_Drive): SERIALCONSOLE.printf("Drive"); break;
    case (Stat_Charge): SERIALCONSOLE.printf("Charge"); break;
    case (Stat_Charged): SERIALCONSOLE.printf("Charged"); break;
    case (Stat_Error): SERIALCONSOLE.printf("Error"); break;
    case (Stat_Healthy): SERIALCONSOLE.printf("Healthy"); break;
  }
  if (Warn_Out_handle()){SERIALCONSOLE.printf(" (Warning!)\r\n");} else {SERIALCONSOLE.printf("\r\n");}

  SERIALCONSOLE.printf("\r\nCharge Current Limit: %5.2f (%5.2f) A | DisCharge Current Limit: %5.2fA \r\n",float(chargecurrent) * settings.nchargers / 10, float(chargecurrent) / 10, float(discurrent) / 10);
  SERIALCONSOLE.printf("Vpack: %5.2fV | Vlow: %4umV | Vhigh: %4umV | DeltaV: %4umV | Tlow: %3.1f°C | Thigh: %3.1f°C\r\n",double(bms.getPackVoltage()) / 1000,bms.getLowCellVolt(),bms.getHighCellVolt(),bms.getHighCellVolt() - bms.getLowCellVolt(),float(bms.getLowTemperature()) / 10,float(bms.getHighTemperature()) / 10);
  SERIALCONSOLE.printf("Cells: %u/%u",bms.getSeriesCells(),settings.Scells * settings.Pstrings);

  if (Balancing()){ SERIALCONSOLE.printf(" | Balancing Active\r\n\n"); } else { SERIALCONSOLE.printf("\r\n\n"); }

  bms.printPackDetails();
  if (warning[2] & 0b01000000){
    SERIALCONSOLE.printf("!!! Internal Error / Series Cells Fault !!!\r\n\n");
  } else { SERIALCONSOLE.printf("\r\n");}
  
  if (settings.cursens == Sen_Analoguedual){
    if (Sen_Analogue_Num == 1)
      { SERIALCONSOLE.printf("Analogue Low:  "); }
    else if (Sen_Analogue_Num == 2)
      { SERIALCONSOLE.printf("Analogue High: "); }
  }
  if (settings.cursens == Sen_Analoguesing)
    {SERIALCONSOLE.printf("Analogue Single: ");}
  if (settings.cursens == Sen_Canbus)
    {SERIALCONSOLE.printf("CANbus: ");}
  SERIALCONSOLE.printf("%6imA\r\n",currentact);
  SERIALCONSOLE.printf("SOC:       %3u%%  (%.2fmAh)\r\n",SOC,double(mampsecond) / 3600);
  SERIALCONSOLE.printf("SOH:       %3u%%  (%u/%uAh)\r\n",SOH_calc(),settings.designCAP,settings.CAP);
  SERIALCONSOLE.printf("TCap:      %3uAh (%uWh)\r\n",abs(TCAP),abs(TCAP_Wh));
  SERIALCONSOLE.printf("Warning:   ");
  for (byte i = 0; i < 4; i++){
    SERIALCONSOLE.printf("%08i ",ConvertToBin(warning[i])); // [ToDo] in Binary
  }
  SERIALCONSOLE.printf("\r\n");
  SERIALCONSOLE.printf("Error:     ");
  for (byte i = 0; i < 4; i++){
    SERIALCONSOLE.printf("%08i ",ConvertToBin(alarm[i])); // [ToDo] in Binary
  }
  SERIALCONSOLE.printf("\r\n");
  SERIALCONSOLE.printf("ETA:       %2ih %2im\r\n",ETA() / 60,ETA() % 60);
  SERIALCONSOLE.printf("\r\n");

  SERIALCONSOLE.printf("IN %d%d%d%d %s %s\r\n",digitalRead(IN1_Key),digitalRead(IN2_Gen),digitalRead(IN3_AC),digitalRead(IN4),ChargeActive()?"| Charger plugged":"",digitalRead(IN1_Key)?"| Key ON":"");
  if(TCU_Pump || TCU_Cool || TCU_Heat || TCU_REL4){
    SERIALCONSOLE.printf("TCU: %s%s%s%s\r\n",TCU_Pump-1?"Pumping ":"",TCU_Cool-1?"Cooling ":"",TCU_Heat-1?"Heating ":"",TCU_REL4-1?"REL4":"");
  }
  SERIALCONSOLE.printf("\r\n");
}

uint32_t ConvertToBin(byte Input){
  byte exponent = 0;
  uint32_t base = 10;
  uint32_t retVal = 0;

  while (Input > 0){
    if(Input & 0b1){
      uint32_t product = 1;
      for(byte i = 0; i < exponent; i++){
        product *= base;
      }
      retVal += product;
    }
    exponent++;
    Input>>=1;
  }
  return retVal;
}

uint32_t SEN_AnalogueRead(int32_t tmp_currrentlast){
  int32_t tmp_current = 0;
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
      RawCur = round(((float(Sen_AnalogueRawValue) * 3300 / adc->adc0->getMaxValue()) - settings.offset1) / (float(settings.convlow) / 100 / 1000 / (5 / 3.3)));
      if (labs((round((float(Sen_AnalogueRawValue) * 3300 / adc->adc0->getMaxValue()) - settings.offset1))) <  settings.CurDead) { RawCur = 0; }
    break;
    case 2:
      RawCur = round(((float(Sen_AnalogueRawValue) * 3300 / adc->adc0->getMaxValue()) - settings.offset2) / (float(settings.convhigh) / 100 / 1000 / (5 / 3.3)));
    break;
    }      

  lowpassFilter.input(RawCur);
  tmp_current = lowpassFilter.output();
  tmp_current = settings.ncur * tmp_current;
  if (settings.invertcur){ tmp_current *= -1; }
  return tmp_current;

} 

void CAN_SEN_read(CAN_message_t MSG, int32_t& CANmilliamps){
  //int32_t CANmilliamps = 0;
  bool newVal = true;
  switch (MSG.id){
    //LEM CAB
    // [ToTest] CAB1500 has same ID. ALso same Data?
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
    default: newVal = false; break;
  }

  //Victron Lynx  
  if (pgnFromCANId(MSG.id) == 0x1F214 && MSG.buf[0] == 0) // Check PGN and only use the first packet of each sequence
  {CANmilliamps = CAN_SEN_VictronLynx(MSG);}
  else {newVal = false;}

  //SERIALCONSOLE.println(CANmilliamps);

  if (settings.invertcur && newVal){ CANmilliamps *= -1; }
  //return CANmilliamps;
}

int32_t CAN_SEN_LEMCAB(CAN_message_t MSG){
  /*
  80000000H = 0 mA,
  7FFFFFFFH = −1 mA,
  80000001H = 1 mA
  Byte 0 = MSB
  Byte 3 = LSB
  */
 
  if(MSG.buf[0]==0xff && MSG.buf[1]==0xff && MSG.buf[2]==0xff && MSG.buf[3]==0xff && MSG.buf[4] & 8){return 0;} // check error bit 32. 1 = Error [ToTest]
  int32_t LEM_milliamps = 0;
  for (byte i = 0; i < 4; i++){LEM_milliamps = (LEM_milliamps << 8) | MSG.buf[i];}
  LEM_milliamps -= 0x80000000;
  return LEM_milliamps;
}

int32_t CAN_SEN_VictronLynx(CAN_message_t MSG){
  int32_t Victron_milliamps = 0;
  if (MSG.buf[4] == 0xff && MSG.buf[3] == 0xff) return 0;
  int16_t current = (int)MSG.buf[4] << 8; // in 0.1A increments
  current |= MSG.buf[3];
  Victron_milliamps = current * 100;
  return Victron_milliamps;
}

void CAN_Debug_IN(byte CanNr, CAN_message_t MSG){
  int16_t pgn = 0;
  
  switch (CanNr){
    case 1: SERIALCONSOLE.printf("CAN1 "); break;
    case 2: SERIALCONSOLE.printf("CAN2 "); break;
  }
  SERIALCONSOLE.printf("IN ");
  if (MSG.flags.extended){
    SERIALCONSOLE.printf("ext:%x  ",MSG.id);
    pgn = pgnFromCANId(MSG.id);
  } else {
    SERIALCONSOLE.printf("std:%x  ",MSG.id);
  }
  for (byte i = 0; i < MSG.len; i++) { // print the data
    SERIALCONSOLE.printf("%x ",MSG.buf[i]);
  }
  if(pgn){
    SERIALCONSOLE.printf("PGN: %i",pgn);
  }
  SERIALCONSOLE.printf("\r\n");
}

void Currentavg_Calc(){
  //[ToDo] überarbeiten! (glätten, reset bei Statuswechsel)
  currentavg_array[currentavg_counter] = currentact;
  currentavg_counter++;
  if(currentavg_counter > 59){currentavg_counter = 0;}
  uint32_t current_temp = 0;
  uint16_t max_size = sizeof(currentavg_array)/sizeof(currentavg_array[0]);
  for (byte i=0; i< max_size; i++){
    current_temp += currentavg_array[i];
  }
  currentavg = current_temp / max_size;
}

void Current_debug(){
  if ( settings.cursens == Sen_Analoguedual || settings.cursens == Sen_Analoguesing){
    if (Sen_Analogue_Num == 1){
      SERIALCONSOLE.printf("\r\n");
      if (settings.cursens == Sen_Analoguedual)
      {SERIALCONSOLE.printf("Low Range: ");}
      else
      {SERIALCONSOLE.printf("Single In: ");}
      SERIALCONSOLE.printf("Value ADC0: %i  %i  %i  %imA  ",Sen_AnalogueRawValue * 3300 / adc->adc0->getMaxValue(),settings.offset1,Sen_AnalogueRawValue * 3300 / adc->adc0->getMaxValue() - settings.offset1,RawCur);
    }else if (Sen_Analogue_Num == 2){
      SERIALCONSOLE.printf("\r\n");
      SERIALCONSOLE.printf("High Range: Value ADC0: %i  %i  %i  %imA  ",Sen_AnalogueRawValue * 3300 / adc->adc0->getMaxValue(),settings.offset2,Sen_AnalogueRawValue * 3300 / adc->adc0->getMaxValue() - settings.offset2,RawCur);
    }
    SERIALCONSOLE.printf("%.2f | %i | %imA",lowpassFilter.output(),settings.changecur,currentact); // [ToDo] eliminate float?
  }
  if ( settings.cursens == Sen_Canbus ) {
    SERIALCONSOLE.printf("\r\n%imA",currentact);
  }
}


int16_t ETA(){ // return minutes
  //[ToDo] weiter glätten! mögl. vordef./errechneten Durchschnittswert einbeziehen?
  if(BMC_Stat == Stat_Charge){
    return labs(mampsecond / 1000 / 60 / labs(currentavg/1000));
  }else{
    if (currentavg >= 0){ return 0;}
    else { return abs((TCAP * 60 - mampsecond / 1000 / 60) / labs(currentavg/1000)); }
  }
}

uint16_t SOH_calc(){
  return round(float(settings.CAP) / float(settings.designCAP) * 100);
}

void SOC_update(){
  int16_t SOC_tmp = 0;
  if (millis() > 10000 && bms.getAvgCellVolt() > 0){
    
    if(!SOCset || settings.voltsoc){      
      SOC_tmp = map(bms.getAvgCellVolt(), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);
      SOC = constrain(SOC_tmp, 0, 100);// keep SOC bettween 0 and 100
      mampsecond = (100 - SOC) * TCAP * (3600000/100);
      if(!SOCset){
        SOCset = 1;
        if (!menu_load){
          SERIALCONSOLE.printf("\r\n--------SOC SET: %i%%, %imAs--------",SOC,mampsecond);
        }
      }
    } else {
        SOC_tmp = round((TCAP - double(mampsecond) / 3600000) * 100 / TCAP);
        SOC = constrain(SOC_tmp, 0, 100);// keep SOC bettween 0 and 100
    }
  }
  //SERIALCONSOLE.println(mampsecond);
}

void SOC_charged(){
    SOC = 100;
    mampsecond = 0;
}

/*
  Returns temperature corrected capacity in Ah.
  Uses:
    bms.getLowTemperature()
    settings.Temp_Cap_Map[x][x]
*/
uint32_t CAP_Temp_alteration(){
  byte factor_map = 0;

  //temp below lowest Setpoint
  if (bms.getLowTemperature() < settings.Temp_Cap_Map[0][0]){ 
    return settings.CAP * (settings.Temp_Cap_Map[1][0] / 100);
  }

  //temp above highest setpoint.
  if(bms.getLowTemperature() >= settings.Temp_Cap_Map[0][4]) {
    return settings.CAP * (settings.Temp_Cap_Map[1][4] / 100);
  }
  
  //everything in between
  for (byte i = 1; i < 4; i++){
    if (bms.getLowTemperature() >= settings.Temp_Cap_Map[0][i] && bms.getLowTemperature() < settings.Temp_Cap_Map[0][i+1]){
      factor_map = map(bms.getLowTemperature(), settings.Temp_Cap_Map[0][i], settings.Temp_Cap_Map[0][i+1],settings.Temp_Cap_Map[1][i],settings.Temp_Cap_Map[1][i+1]);
      return round(settings.CAP * float(factor_map) / 100);
    }  
  }
  return settings.CAP; // failsafe
}

void CAP_recalc(){
  if ((BMC_Stat == Stat_Charge && labs(mampsecond) > (settings.CAP * 3600 * 1000)) /*|| ((warning[0] & 0x10) && bms.getAvgTemperature() > 20.00)*/){ //[todo]
    settings.CAP = round((labs(mampsecond) / 3600) / 1000 / settings.Pstrings);
    Settings_unsaved = 1;
  }
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
  for(byte i = 1; i < 9; i++){
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
  SERIALCONSOLE.printf("Function         State Timer\r\n");
  SERIALCONSOLE.printf("----------------------------\r\n");
  for(byte i = 1; i < 12; i++){ //[ToDo] us array size for loop
    SERIALCONSOLE.printf("%-16s %5i %5i\r\n",Out_Functions[i].c_str(),Out_States[0][i],Out_States[1][i]);
  } 
  SERIALCONSOLE.printf("\r\n");
  SERIALCONSOLE.printf("Output        Function  PWM\r\n");
  SERIALCONSOLE.printf("---------------------------\r\n");
  for(byte i = 1; i < 9; i++){ //[ToDo] us array size for loop
    SERIALCONSOLE.printf("OUT%i   %16s ",i,Out_Functions[settings.Out_Map[0][i]].c_str());
    if(i>4){
      if(settings.Out_Map[0][i] == Out_Gauge){SERIALCONSOLE.printf("%3i",Gauge_update());} // get Gauge PWM directly from function.
      else {SERIALCONSOLE.printf("%3i",settings.Out_Map[1][i]);}
    }
    SERIALCONSOLE.printf("\r\n");
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

  adc->adc0->startContinuous(ACUR1);
  Sen_Analogue_Num = 1;
  SERIALCONSOLE.printf(" Calibrating Current Offset ");
  for (byte i = 0; i < 20; i++){
    settings.offset1 = settings.offset1 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
    SERIALCONSOLE.printf(".");
    delay(100);
  }
  settings.offset1 = settings.offset1 / 20;
  SERIALCONSOLE.printf("\r\n Current offset 1 calibrated: %i \r\n",settings.offset1);

  if(settings.cursens == Sen_Analoguedual){
    adc->adc0->startContinuous(ACUR2);
    Sen_Analogue_Num = 2;
    SERIALCONSOLE.printf(" Calibrating Current Offset ");
    for (byte i = 0; i < 20; i++){
      settings.offset2 = settings.offset2 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
      SERIALCONSOLE.printf(".");
      delay(100);
    }
    settings.offset2 = settings.offset2 / 20;
    SERIALCONSOLE.printf("\r\n Current offset 2 calibrated: %i",settings.offset2);
  }
}

void Menu(){
  String menu_option_string;
  char menu_option_char;
  int32_t menu_option_val = 0;
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
  
  switch (menu_current) {
    case Menu_Start:
      menu_current = Menu_Main;
      Serial_clear();
      SERIALCONSOLE.printf("Main\r\n");
      SERIALCONSOLE.printf("--------------------\r\n");
      SERIALCONSOLE.printf("[1]  Battery\r\n");
      SERIALCONSOLE.printf("[2]  Current Sensor\r\n");
      SERIALCONSOLE.printf("[3]  Charger\r\n");
      SERIALCONSOLE.printf("[4]  Motor Controller\r\n");
      SERIALCONSOLE.printf("[5]  Outputs\r\n");
      SERIALCONSOLE.printf("[6]  Alarms\r\n");
      SERIALCONSOLE.printf("[7]  Ignore Values\r\n");
      SERIALCONSOLE.printf("[8]  CAN-Bus\r\n");
      SERIALCONSOLE.printf("[9]  Experiental\r\n");
      SERIALCONSOLE.printf("[10] Debug\r\n");
      SERIALCONSOLE.printf("[11] Reboot\r\n");
      SERIALCONSOLE.printf("[12] Load Defaults\r\n");
      SERIALCONSOLE.printf("[13] Mode: %s\r\n",settings.ESSmode?"ESS":"Vehicle");
      SERIALCONSOLE.printf("[14] BMS-Type: ");
      switch (settings.BMSType){
        case BMS_Dummy: SERIALCONSOLE.printf("NONE\r\n"); break;
        case BMS_Tesla: SERIALCONSOLE.printf("Tesla Model S/X\r\n"); break;
        case BMS_VW_MEB:SERIALCONSOLE.printf("VW MEB\r\n");          break;
        case BMS_VW_eGolf: SERIALCONSOLE.printf("VW eGolf/GTE\r\n"); break;
        case BMS_BMW_I3: SERIALCONSOLE.printf("BMW i3\r\n"); break;
        case BMS_BMW_MiniE: SERIALCONSOLE.printf("BMW MiniE\r\n"); break;
        default: break;
      }
      SERIALCONSOLE.printf("\r\n");
      SERIALCONSOLE.printf("[q]  Save & Quit\r\n");
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
        case 11: SCB_AIRCR = 0x05FA0004; break;
        case 12: loadSettings(); Menu(); SERIALCONSOLE.printf("::::::Defaults loaded::::::\r\n"); break;    
        case 13: settings.ESSmode = !settings.ESSmode; menu_current = Menu_Start; Menu(); break;       
        case 14: 
          if(settings.BMSType == BMS_Type_MAX){settings.BMSType = BMS_Dummy;} 
          else {settings.BMSType = static_cast<BMS_t>(settings.BMSType + 1);} 
          BMSInit(); // reset BMS with new Settings
          menu_current = Menu_Start; Menu(); 
        break;
        case 120: (_reboot_Teensyduino_()); //-> hidden Program Mode (x)
        case Menu_Quit:
          EEPROM.put(0, settings); //save all change to eeprom
          menu_current = Menu_Start;
          menu_load = 0;
          SERIALCONSOLE.printf("::::::Settings saved::::::\r\n");
        break;
      }
    break;
    case Menu_Battery:
      switch (menu_option){
        case 1: settings.OverVSetpoint = menu_option_val; Menu(); break;
        case 2: settings.ChargeVSetpoint = menu_option_val; Menu(); break;
        case 3: settings.UnderVDerateSetpoint = menu_option_val; Menu(); break;
        case 4: settings.UnderVSetpoint = menu_option_val; Menu(); break;
        case 5: settings.OverTSetpoint = menu_option_val * 10; Menu(); break;
        case 6: settings.OverTDerateSetpoint = menu_option_val * 10; Menu(); break;
        case 7: settings.UnderTDerateSetpoint = menu_option_val * 10; Menu(); break;    
        case 8: settings.UnderTSetpoint = menu_option_val * 10; Menu(); break;    
        case 9: settings.balanceVoltage = menu_option_val; Menu(); break;   
        case 10: settings.balanceHyst = menu_option_val; Menu(); break;   
        case 11: settings.designCAP = menu_option_val; Menu(); break;
        case 12: settings.CAP = menu_option_val; Menu(); break;
        case 13: settings.Pstrings = menu_option_val; Menu(); break;     
        case 14: settings.Scells = menu_option_val; Menu(); break;  
        case 15: settings.PackDisCurrentMax = menu_option_val * 10; Menu(); break;
        case 16: settings.socvolt[0] = menu_option_val; Menu(); break;  
        case 17: settings.socvolt[1] = menu_option_val; Menu(); break; 
        case 18: settings.socvolt[2] = menu_option_val; Menu(); break;                                                               
        case 19: settings.socvolt[3] = menu_option_val; Menu(); break;  
        case 20: settings.StoreVsetpoint = menu_option_val; Menu(); break;  
        case 21: settings.Temp_Cap_Map[0][0] = menu_option_val * 10; Menu(); break;
        case 22: settings.Temp_Cap_Map[1][0] = menu_option_val; Menu(); break;
        case 23: settings.Temp_Cap_Map[0][1] = menu_option_val * 10; Menu(); break;
        case 24: settings.Temp_Cap_Map[1][1] = menu_option_val; Menu(); break;
        case 25: settings.Temp_Cap_Map[0][2] = menu_option_val * 10; Menu(); break;
        case 26: settings.Temp_Cap_Map[1][2] = menu_option_val; Menu(); break;
        case 27: settings.Temp_Cap_Map[0][3] = menu_option_val * 10; Menu(); break;
        case 28: settings.Temp_Cap_Map[1][3] = menu_option_val; Menu(); break;
        case 29: settings.Temp_Cap_Map[0][4] = menu_option_val * 10; Menu(); break;
        case 30: settings.Temp_Cap_Map[1][4] = menu_option_val; Menu(); break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.printf("Battery\r\n");
          SERIALCONSOLE.printf("--------------------\r\n\r\n");
          SERIALCONSOLE.printf("Cell limits:\r\n");    
          SERIALCONSOLE.printf("[1] Cell Overvoltage (mV):           %4i\r\n",settings.OverVSetpoint);
          SERIALCONSOLE.printf("[2] Charge Voltage Setpoint (mV):    %4i\r\n",settings.ChargeVSetpoint);
          SERIALCONSOLE.printf("[3] Derate at Undervoltage (mV):     %4i\r\n",settings.UnderVDerateSetpoint);     
          SERIALCONSOLE.printf("[4] Cell Undervoltage (mV):          %4i\r\n",settings.UnderVSetpoint);
          SERIALCONSOLE.printf("[5] Over Temperature (°C):           %4i\r\n",settings.OverTSetpoint / 10);
          SERIALCONSOLE.printf("[6] Derate at high Temperature (°C): %4i\r\n",settings.OverTDerateSetpoint / 10);
          SERIALCONSOLE.printf("[7] Derate at low Temperature (°C):  %4i\r\n",settings.UnderTDerateSetpoint / 10);
          SERIALCONSOLE.printf("[8] Under Temperature (°C):          %4i\r\n",settings.UnderTSetpoint / 10);
          SERIALCONSOLE.printf("[9] Cell Balance Voltage (mV):       %4i\r\n",settings.balanceVoltage);
          SERIALCONSOLE.printf("[10] Balance Hysteresis (mV):        %4i\r\n",settings.balanceHyst);
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("Pack configuration:\r\n");         
          SERIALCONSOLE.printf("[11] Pack Design Capacity (Ah):  %4i\r\n",settings.designCAP);
          SERIALCONSOLE.printf("[12] Pack current Capacity (Ah): %4i\r\n",settings.CAP); 
          SERIALCONSOLE.printf("[13] Cells in Parallel:          %4i\r\n",settings.Pstrings);
          SERIALCONSOLE.printf("[14] Cells in Series:            %4i\r\n",settings.Scells );
          SERIALCONSOLE.printf("[15] Pack Max Discharge (A):     %4i\r\n",settings.PackDisCurrentMax / 10);
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("Voltage based SOC:\r\n");
          SERIALCONSOLE.printf("[16] Setpoint1 (mV):        %4i\r\n",settings.socvolt[0]);
          SERIALCONSOLE.printf("[17] SOC @ Setpoint1 (%%):   %4i\r\n",settings.socvolt[1]);
          SERIALCONSOLE.printf("[18] Setpoint2 (mV):        %4i\r\n",settings.socvolt[2]);
          SERIALCONSOLE.printf("[19] SOC @ Setpoint2 (%%):   %4i\r\n",settings.socvolt[3]);
          SERIALCONSOLE.printf("[20] Storage Setpoint (mV): %4i\r\n",settings.StoreVsetpoint);
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("Temperature based SOC Adjustment:\r\n");
          for (byte i = 0; i < 5; i++){
            byte y = i*2;
            SERIALCONSOLE.printf("[%i] T%i(°C): %3i\r\n",21+y,i+1,settings.Temp_Cap_Map[0][i]/10);
            SERIALCONSOLE.printf("[%i] C%i (%%): %3i\r\n",22+y,i+1,settings.Temp_Cap_Map[1][i]);
          }
          SERIALCONSOLE.printf("\r\n");                    
          SERIALCONSOLE.printf("[q] Quit\r\n");          
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
        case 5: settings.convlow = menu_option_val; Menu(); break;
        case 6: settings.convhigh = menu_option_val; Menu(); break;
        case 7: settings.changecur = menu_option_val * 1000; Menu(); break;
        case 8: settings.CurDead = menu_option_val; Menu(); break;                 
        case 9: CurrentOffsetCalc(); Menu();break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        case 100: debug_Cur = !debug_Cur; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.printf("Current Sensor\r\n");
          SERIALCONSOLE.printf("--------------------\r\n");
          SERIALCONSOLE.printf("[1] Invert Current:     %i\r\n",settings.invertcur);
          SERIALCONSOLE.printf("[2] Voltage based SOC:  %i\r\n",settings.voltsoc);
          SERIALCONSOLE.printf("[3] Current Multiplier: %i\r\n",settings.ncur);
          SERIALCONSOLE.printf("[4] Sensor Type: ");  
          switch (settings.cursens) {
            case Sen_Canbus:
              SERIALCONSOLE.printf(" Canbus Current Sensor\r\n");
            break;
            case Sen_Analoguesing:
              SERIALCONSOLE.printf(" Analogue Single\r\n");
              SERIALCONSOLE.printf(" Enter value below without decimal point. E.g. 1250 for 12.50mV/A\r\n");
              SERIALCONSOLE.printf("[7] Analog Sensor 1 (low) mV/A:    %5.2f\r\n",float(settings.convlow) / 100);
              SERIALCONSOLE.printf("[8] Current Sensor Deadband in mV: %5i\r\n",settings.CurDead); 
              SERIALCONSOLE.printf("[9] Calibrate\r\n");
            break;
            case Sen_Analoguedual:
              SERIALCONSOLE.printf(" Analogue Dual\r\n");
              SERIALCONSOLE.printf(" Enter values below without decimal point. E.g. 1250 for 12.50mV/A\r\n");
              SERIALCONSOLE.printf("[5] Analog Sensor 1 (low) mV/A:      %5.2f\r\n",float(settings.convlow) / 100);
              SERIALCONSOLE.printf("[6] Analog Sensor 2 (high) mV/A:     %5.2f\r\n",float(settings.convhigh) / 100);  
              SERIALCONSOLE.printf("[7] Current Sensor switch over in A: %5i\r\n",settings.changecur / 1000);                          
              SERIALCONSOLE.printf("[8] Current Sensor Deadband in mV:   %5i\r\n",settings.CurDead);  
              SERIALCONSOLE.printf("[9] Calibrate\r\n");
            break;             
            default: SERIALCONSOLE.printf("Undefined\r\n");
          }
          SERIALCONSOLE.printf("[d] Debug\r\n");    
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("[q] Quit\r\n");         
      }
    break;  
    case Menu_Charger:
      switch (menu_option){
        case 1: settings.ChargeHys = menu_option_val; Menu(); break;
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
          SERIALCONSOLE.printf("Charger\r\n");
          SERIALCONSOLE.printf("--------------------\r\n");
          SERIALCONSOLE.printf("[1] Charge Hysteresis (mV):                %4i\r\n",settings.ChargeHys);
          if (settings.chargertype > 0){
            SERIALCONSOLE.printf("[2] Max Charge Current per Charger (A):    %4i\r\n",settings.ChargerChargeCurrentMax / 10);
            SERIALCONSOLE.printf("[3] Pack End of Charge Current in (A):     %4i\r\n",settings.chargecurrentend / 10);
          }
          SERIALCONSOLE.printf("[4] Charger Type:                 ");
          switch (settings.chargertype){
            case 0: SERIALCONSOLE.printf("%13s\r\n","undefined"); break;
            case 1: SERIALCONSOLE.printf("%13s\r\n","Brusa NLG5xx");  break;
            case 2: SERIALCONSOLE.printf("%13s\r\n","Volt Charger");  break;
            case 3: SERIALCONSOLE.printf("%13s\r\n","Eltek Charger"); break;
            case 4: SERIALCONSOLE.printf("%13s\r\n","Elcon Charger"); break;
            case 5: SERIALCONSOLE.printf("%13s\r\n","Victron/SMA");   break;
            case 6: SERIALCONSOLE.printf("%13s\r\n","Coda"); break;
          }        
          SERIALCONSOLE.printf("[5] Charger HV Connection: ");
          switch (settings.ChargerDirect){
            case 0: SERIALCONSOLE.printf("%20s\r\n","Behind Contactors"); break;
            case 1: SERIALCONSOLE.printf("%20s\r\n","Direct to Battery HV"); break;
          }       
          SERIALCONSOLE.printf("[6] Number of Chargers in Parallel:        %4i\r\n",settings.nchargers);
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("[q] Quit\r\n"); 
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
        SERIALCONSOLE.printf("Motor Controller\r\n");
        SERIALCONSOLE.printf("--------------------\r\n");
        SERIALCONSOLE.printf("[1] Controller Type: ");
        switch (settings.mctype){
          case 0: SERIALCONSOLE.printf("not configured\r\n"); break;
          case 1: SERIALCONSOLE.printf("Curtis\r\n”"); break;
          default: SERIALCONSOLE.printf("undefined\r\n");
        }
        SERIALCONSOLE.printf("\r\n");
        SERIALCONSOLE.printf("[q] Quit\r\n"); 
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
        case 22: settings.gaugelow = menu_option_val; Menu(); break;
        case 23: settings.gaugehigh = menu_option_val; Menu(); break;
        case 24:
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
          SERIALCONSOLE.printf("Outputs\r\n");
          SERIALCONSOLE.printf("--------------------\r\n");
          SERIALCONSOLE.printf("12V Outputs:\r\n");
          for (byte i = 1; i < 5; i++){ // [ToDo] use array size
            SERIALCONSOLE.printf("[%i] Out%i: %s\r\n",i,i,Out_Functions[settings.Out_Map[0][i]].c_str());                    
          }
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("PWM/GND Outputs:\r\n");
          for (byte i = 5; i < 9; i++){ // [ToDo] use array size
            byte pos = 2*i-5;
            SERIALCONSOLE.printf("[%i] %sOut%i:             %16s\r\n",pos,pos<10?" ":"",i,Out_Functions[settings.Out_Map[0][i]].c_str());   
            pos = 2*i-4;
            SERIALCONSOLE.printf("[%i] %sOut%i PWM [0-255]: %13s%3i\r\n",pos,pos<10?" ":"",i,"",settings.Out_Map[1][i]);   //%13s just for alignment, whitespaces
          }      
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("Additional Settings:\r\n");
          SERIALCONSOLE.printf("[20] Precharge Timer in ms:          %4i\r\n",settings.Pretime);
          SERIALCONSOLE.printf("[21] Precharge finish Current in mA: %4i\r\n",settings.Precurrent);
          SERIALCONSOLE.printf("[22] PWM Gauge low [0-255]:          %4i\r\n",settings.gaugelow);
          SERIALCONSOLE.printf("[23] PWM Gauge high [0-255]:         %4i\r\n",settings.gaugehigh);
          SERIALCONSOLE.printf("[24] Gauge Test [0-255] for 30s:     %4i\r\n",debug_Gauge_val);
          SERIALCONSOLE.printf("[d]  Debug Output ");         
          if(debug_Output){ SERIALCONSOLE.printf("ON\r\n"); } 
          else {SERIALCONSOLE.printf("OFF\r\n");}
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("[q] Quit\r\n");
      }
    break; 
    case Menu_Alarms:
      switch (menu_option){
        case 1: settings.WarnVoltageOffset = menu_option_val; Menu(); break;
        case 2: settings.CellGap = menu_option_val; Menu(); break;
        case 3: settings.WarnTempOffset = menu_option_val * 10; Menu(); break;                        
        case 4: settings.error_delay = menu_option_val; Menu(); break;             
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.printf("Alarms\r\n");
          SERIALCONSOLE.printf("--------------------\r\n");
          SERIALCONSOLE.printf("[1] Voltage Warning Offset (mV):   %4i\r\n",settings.WarnVoltageOffset);
          SERIALCONSOLE.printf("[2] Cell Delta Voltage Alarm (mV): %4i\r\n",settings.CellGap);
          SERIALCONSOLE.printf("[3] Temp Warning offset (°C):      %4i\r\n",settings.WarnTempOffset / 10);
          SERIALCONSOLE.printf("[4] Error Delay (ms):             %5i\r\n",settings.error_delay);
          SERIALCONSOLE.printf("\r\n");        
          SERIALCONSOLE.printf("[q] Quit\r\n");          
      }
    break;   
    case Menu_IgnVal:
      switch (menu_option){
        case 1:
          settings.useTempSensor++;
          if (settings.useTempSensor > 2){settings.useTempSensor = 0;}
          BMSInit(); // reset BMS with new Settings
          Menu();
        break; 
        case 2: 
          settings.IgnoreVolt = menu_option_val; 
          BMSInit(); // reset BMS with new Settings
          Menu(); break;       
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.printf("Ignore Values\r\n");
          SERIALCONSOLE.printf("--------------------\r\n");
          SERIALCONSOLE.printf("[1] Use Temp. Sensor:       ");
          if (!settings.useTempSensor){SERIALCONSOLE.printf("both\r\n");}
          else{SERIALCONSOLE.printf("%4i\r\n",settings.useTempSensor);}
          SERIALCONSOLE.printf("[2] Ignore Cells under mV: %4i\r\n",settings.IgnoreVolt);
          SERIALCONSOLE.printf("\r\n"); 
          SERIALCONSOLE.printf("[q] Quit\r\n"); 
      }
    break;  
    case Menu_CAN:
      switch (menu_option){
        case 1: settings.CAN1_Speed = menu_option_val * 1000; can1_start(); Menu(); break;  
        case 2: debug_CAN1 = !debug_CAN1; Menu(); break;
        case 3: settings.CAN2_Speed = menu_option_val * 1000; can2_start(); Menu(); break;  
        case 4: debug_CAN2 = !debug_CAN2; Menu(); break;
        case 5: if(menu_option_val >= 0 && menu_option_val < 4){settings.CAN_Map[0][CAN_BMC_std] = menu_option_val;} Menu(); break;
        case 6: if(menu_option_val >= 0 && menu_option_val < 4){settings.CAN_Map[0][CAN_BMC_HV] = menu_option_val;} Menu(); break;
        //case 7: settings.CAN_Map[1][0] = constrain(menu_option_val,0,255); Menu(); break;
        case 7: if(menu_option_val >= 0 && menu_option_val < 3){settings.CAN_Map[0][CAN_BMS] = menu_option_val;} // 3 not allowed. Only CAN1 or CAN2. Not both.
          Menu(); break;
        //case 9: settings.CAN_Map[1][1] = constrain(menu_option_val,0,255); Menu(); break;        
        case 8: if(menu_option_val >= 0 && menu_option_val < 4){settings.CAN_Map[0][CAN_Charger] = menu_option_val;} Menu(); break;
        //case 11: settings.CAN_Map[1][2] = constrain(menu_option_val,0,255); Menu(); break;        
        case 9: if(menu_option_val >= 0 && menu_option_val < 4){settings.CAN_Map[0][CAN_Curr_Sen] = menu_option_val;} Menu(); break;
        case 10: if(menu_option_val >= 0 && menu_option_val < 4){settings.CAN_Map[0][CAN_MC] = menu_option_val;} Menu(); break;
        case 111: //CO_Send_SDO(0x26,2,0,0x1800,0x02,0); // "o" for SDO
        break;
        case 112: //CO_SDO_send_test(0x26); // "p" for PDO
        break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.printf("CAN-Bus settings\r\n");
          SERIALCONSOLE.printf("--------------------\r\n");
          SERIALCONSOLE.printf("[1] Can1 Baudrate in kbps: %i\r\n",settings.CAN1_Speed / 1000);
          SERIALCONSOLE.printf("[2] Debug CAN1: %s\r\n",debug_CAN1?"ON":"OFF");
          SERIALCONSOLE.printf("[3] Can2 Baudrate in kbps: %i\r\n",settings.CAN2_Speed / 1000);
          SERIALCONSOLE.printf("[4] Debug CAN2: %s\r\n",debug_CAN2?"ON":"OFF");
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("Functions:\r\n");
          for (byte i = 0; i < CAN_MAP_MAX; i++){
            //byte pos = (i+5)*2-5;
            byte pos = i+5;
            SERIALCONSOLE.printf("[%i] %s",pos,pos<10?" ":"");
            switch (i){
              case 0: SERIALCONSOLE.printf("BMC Std:          "); break;
              case 1: SERIALCONSOLE.printf("BMC HV:           "); break;
              case 2: SERIALCONSOLE.printf("BMS:              "); break;
              case 3: SERIALCONSOLE.printf("Charger:          "); break;
              case 4: SERIALCONSOLE.printf("Current Sensor:   "); break;
              case 5: SERIALCONSOLE.printf("Motor Controller: "); break;
            }
            switch (settings.CAN_Map[0][i]){
              case 0: SERIALCONSOLE.printf("%11s\r\n","undefined"); break;
              case 1: SERIALCONSOLE.printf("%11s\r\n","CAN1"); break;
              case 2: SERIALCONSOLE.printf("%11s\r\n","CAN2"); break;
              case 3: SERIALCONSOLE.printf("%11s\r\n","CAN1 & CAN2"); break;
            }
            if (i < 3){
              pos = (i+5)*2-4;
              //SERIALCONSOLE.printf("[%i] %sInterval:         %9ims\r\n",pos,pos<10?" ":"",settings.CAN_Map[1][i]);
            }
          }
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("[q] Quit\r\n");
      }
    break;     
    case Menu_Exp:
      switch (menu_option){
        //case 1: settings.ExpMess = !settings.ExpMess; Menu(); break;
        case 2: settings.SerialCan++; if(settings.SerialCan > 2){settings.SerialCan = 0;} Menu(); break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.printf("Experimental\r\n");
          SERIALCONSOLE.printf("--------------------\r\n");
          SERIALCONSOLE.printf("[2] Second Serial Port: ");
          switch (settings.SerialCan){
            case 0: SERIALCONSOLE.printf("%20s\r\n","Serial Display"); break;
            case 1: SERIALCONSOLE.printf("%20s\r\n","Can Bus Expansion"); break;
            case 2: SERIALCONSOLE.printf("%20s\r\n","Bluetooth App"); break;
          
          }
          SERIALCONSOLE.printf("\r\n");
          SERIALCONSOLE.printf("[q] Quit\r\n");
      }
    break; 
    case Menu_Debug:
      switch (menu_option){
        case 1: debug_CAN1 = !debug_CAN1; Menu(); break;
        case 2: debug_CAN2 = !debug_CAN2; Menu(); break;
        case 3: debug_Cur = !debug_Cur; Menu(); break;
        case 4: debug_Output = !debug_Output; output_debug_counter = 0; Menu(); break;
        case 5: debug_Input = !debug_Input; Menu(); break;
        case 6: Menu(); break;
        case 7: debug_CSV = !debug_CSV; Menu(); break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          SERIALCONSOLE.printf("Debug\r\n");
          SERIALCONSOLE.printf("--------------------\r\n");
          SERIALCONSOLE.printf("[1] CAN1\r\n");
          SERIALCONSOLE.printf("[2] CAN2\r\n");
          SERIALCONSOLE.printf("[3] Current\r\n");
          SERIALCONSOLE.printf("[4] Output Check: %s\r\n",debug_Output?"ON":"OFF");
          SERIALCONSOLE.printf("[5] Input Check\r\n");
          SERIALCONSOLE.printf("[6] ---\r\n");
          SERIALCONSOLE.printf("[7] CSV Output\r\n");
          SERIALCONSOLE.printf("\r\n");  
          SERIALCONSOLE.printf("[q] Quit\r\n");  
      }
    break;          
  }
}

void Serial_clear(){
  // Form feed
  SERIALCONSOLE.write(12);
  
  // ESC + Clear
  //SERIALCONSOLE.write(27);
  //SERIALCONSOLE.printf("[2J");
  
  // bunch of new lines
  //for (byte i = 0; i < 40; i++){SERIALCONSOLE.printf("\r\n");}
}

int16_t pgnFromCANId(int16_t canId){ //Parameter Group Number
  if ((canId & 0x10000000) == 0x10000000){return (canId & 0x03FFFF00) >> 8;}
  else{return canId;}
}

void ChargeCurrentLimit(){
  ///Start at no derating///
  chargecurrent = settings.ChargerChargeCurrentMax;
  u_int16_t EndCurrent = settings.chargecurrentend / settings.nchargers;
  u_int16_t tmp_chargecurrent = 0;

  if (bms.getHighCellVolt() > settings.OverVSetpoint){ chargecurrent = 0; }

  //Modifying Charge current
  if (chargecurrent > 0){
    //Temperature based
    if (bms.getLowTemperature() < settings.UnderTDerateSetpoint){
      chargecurrent = map(bms.getLowTemperature()*10, settings.UnderTSetpoint*10, settings.UnderTDerateSetpoint*10, 0, settings.ChargerChargeCurrentMax);
    }
    if (bms.getHighTemperature() > settings.OverTDerateSetpoint){
      chargecurrent = map(bms.getHighTemperature()*10, settings.OverTDerateSetpoint*10, settings.OverTSetpoint*10, settings.ChargerChargeCurrentMax, 0);
    }    
    //Voltage based
    if (storagemode){
      uint16_t upperStoreVLimit = settings.StoreVsetpoint - settings.ChargeHys/2;
      if (bms.getHighCellVolt() > upperStoreVLimit){
        tmp_chargecurrent = map(bms.getHighCellVolt(), upperStoreVLimit, settings.StoreVsetpoint, settings.ChargerChargeCurrentMax, EndCurrent);
        if(tmp_chargecurrent < chargecurrent){
          chargecurrent = constrain(tmp_chargecurrent,settings.chargecurrentend,settings.ChargerChargeCurrentMax);   
        }
      }
    } else { 
      uint16_t upperVLimit = settings.ChargeVSetpoint - settings.ChargeHys/2;
      if (bms.getHighCellVolt() /*CellV*/ > upperVLimit){
        tmp_chargecurrent = map(bms.getHighCellVolt() /*CellV*/, upperVLimit, settings.ChargeVSetpoint, settings.ChargerChargeCurrentMax, EndCurrent);
        if(tmp_chargecurrent < chargecurrent){
          chargecurrent = constrain(tmp_chargecurrent,settings.chargecurrentend,settings.ChargerChargeCurrentMax);
        }
      }
    }
    //16A Limit
    tmp_chargecurrent = 3600 / (round(double(bms.getPackVoltage()) / 1000)) /*PackV*/ * 95 / 10; //3,6kW max, 95% Eff. , factor 10 [ToDo] make conf. + CAN
    if(tmp_chargecurrent < chargecurrent){
      chargecurrent = constrain(tmp_chargecurrent,settings.chargecurrentend,settings.ChargerChargeCurrentMax);
    }
  }   

  //compensate for consumers if Chargecurrent is 0 [ToTest]
  //[ToDO] besser glätten, schwankt sehr stark
  if(chargecurrent == 0 && currentact < 0){
    chargecurrent = chargecurrentlast + (currentact / 100 / settings.nchargers) * -1;
  }

  //[ToDo] implement feedback loop
  //multiply with calculated current
  //chargecurrentFactor = chargecurrent / (currentact / 100);

  chargecurrent = constrain(chargecurrent,0,settings.ChargerChargeCurrentMax); //[ToTest]
  chargecurrent = constrain(chargecurrent,0,settings.PackChargeCurrentMax / settings.nchargers); //[ToTest]
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
  discurrent = constrain(discurrent,0,settings.PackDisCurrentMax);
}


void Input_Debug(){
  Menu();
  SERIALCONSOLE.printf("\r\nInput: ");
  if (digitalRead(IN1_Key) == HIGH){SERIALCONSOLE.printf("1 ON  ");}
  else {SERIALCONSOLE.printf("1 OFF ");}
  if (digitalRead(IN2_Gen) == HIGH){ SERIALCONSOLE.printf("2 ON  ");}
  else {SERIALCONSOLE.printf("2 OFF ");}
  if (digitalRead(IN3_AC) == HIGH){SERIALCONSOLE.printf("3 ON  ");}
  else {SERIALCONSOLE.printf("3 OFF ");}
  if (digitalRead(IN4) == HIGH){SERIALCONSOLE.printf("4 ON  ");}
  else {SERIALCONSOLE.printf("4 OFF ");}
  SERIALCONSOLE.printf("\r\n");
}

void Output_debug(){
  // 5s each when looped every 500ms
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

void Dash_update(){
  //power gauge
  int32_t dashpower = 0; //in W
  dashpower = round((double(currentact)/1000)*(double(bms.getPackVoltage())/1000));

  //temp gauge
  int32_t dashtemp = 0;
  dashtemp = round(bms.getAvgTemperature());

  //voltage gauge
  int32_t dashvolt = 0;
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
  Nextion_send("ah.val=", int(round(double(mampsecond) / 36000)));
  Nextion_send("current.val=", int(currentact));
  Nextion_send("power.val=", dashpower);
  Nextion_send("temp.val=", bms.getAvgTemperature());
  Nextion_send("temp1.val=", dashtemp);
  Nextion_send("templow.val=", bms.getLowTemperature());
  Nextion_send("temphigh.val=", bms.getHighTemperature());
  Nextion_send("volt.val=", int(round(double(bms.getPackVoltage()) / 100)));
  Nextion_send("volt1_gauge.val=", dashvolt);
  Nextion_send("lowcell.val=", bms.getLowCellVolt());
  Nextion_send("highcell.val=", bms.getHighCellVolt());
  Nextion_send("firm.val=", firmver);
  Nextion_send("celldelta.val=", bms.getHighCellVolt() - bms.getLowCellVolt());
  Nextion_send("cellbal.val=", bms.getBalancing());
  Nextion_send("debug.val=", abs(TCAP_Wh));
  Nextion_send("eta.txt=", eta);
  Nextion_send("etad.txt=", eta);
  Nextion_send("click ", "refresh,1"); //use Refresh-Button to Update Values on the Display
  Nextion_send("click ", "refresh,0");

  //Err_Warn-LED
  uint16_t Err_Warn = 0;
  if (BMC_Stat == Stat_Error){
    Err_Warn = 63488; // red
  } else if (Warn_Out_handle()){
    Err_Warn = 65504; // yellow
  }
  Nextion_send("Err_Warn.bco=", Err_Warn);
  Nextion_send("Err_Warn.pco=", Err_Warn);
}

void Nextion_send(String var, String val){
  DisplaySerial.printf("%s%s",var.c_str(),val.c_str());
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
    if (settings.cursens == Sen_Canbus && settings.CAN_Map[0][CAN_Curr_Sen] & 1){CAN_SEN_read(MSG, currentact);}
    if (settings.mctype && settings.CAN_Map[0][CAN_MC] & 1){CAN_MC_read(MSG);}
    if (settings.BMSType != BMS_Tesla && settings.CAN_Map[0][CAN_BMS] & 1){bms.readModulesValues(MSG); BMSLastRead = millis();}
    if (settings.CAN_Map[0][CAN_BMC_HV] & 1){CAN_BMC_HV_send(1, MSG);} // Send HV CAN triggered by Inverter
    if (debug_CAN1){CAN_Debug_IN(1, MSG);}
    if (settings.CAN_Map[0][CAN_BMC_std]) {CAN_TCU_read(MSG);}
  }
  while (can2.read(MSG)){
    if (settings.cursens == Sen_Canbus && settings.CAN_Map[0][CAN_Curr_Sen] & 2){CAN_SEN_read(MSG, currentact);}
    if (settings.mctype && settings.CAN_Map[0][CAN_MC] & 2){CAN_MC_read(MSG);}
    if (settings.BMSType != BMS_Tesla && settings.CAN_Map[0][CAN_BMS] & 2){bms.readModulesValues(MSG); BMSLastRead = millis();}
    if (settings.CAN_Map[0][CAN_BMC_HV] & 2){CAN_BMC_HV_send(2, MSG);} // Send HV CAN triggered by Inverter
    if (debug_CAN2){CAN_Debug_IN(2, MSG);}
    if (settings.CAN_Map[0][CAN_BMC_std]) {CAN_TCU_read(MSG);}
  }
}

void CAN_BMC_Std_send(byte CAN_Nr){ //BMC standard CAN Messages
  
  if(CAN_BMC_Std_send_Timer + 1000 >= millis()){return;} // Timer 1s;
  if(!CAN_Nr){return;} // No CanNr. set.

  CAN_message_t MSG;

  MSG.id  = 0x351;
  MSG.len = 8;
  if (storagemode){
    MSG.buf[0] = lowByte(uint16_t(round(float(settings.StoreVsetpoint)/100 * settings.Scells)));
    MSG.buf[1] = highByte(uint16_t(round(float(settings.StoreVsetpoint)/100 * settings.Scells)));    
  }else{
    MSG.buf[0] = lowByte(uint16_t(round(float(settings.ChargeVSetpoint)/100 * settings.Scells)));
    MSG.buf[1] = highByte(uint16_t(round(float(settings.ChargeVSetpoint)/100 * settings.Scells)));
  }
  MSG.buf[2] = lowByte(chargecurrent);
  MSG.buf[3] = highByte(chargecurrent);
  MSG.buf[4] = lowByte(discurrent);
  MSG.buf[5] = highByte(discurrent);
  MSG.buf[6] = lowByte(uint16_t(round(float(settings.UnderVDerateSetpoint)/100 * settings.Scells)));
  MSG.buf[7] = highByte(uint16_t(round(float(settings.UnderVDerateSetpoint)/100 * settings.Scells)));
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}

  delay(2);
  MSG.id  = 0x355;
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

  delay(2);
  MSG.id  = 0x356;
  MSG.buf[0] = lowByte(uint16_t(round(double(bms.getPackVoltage()) / 10)));
  MSG.buf[1] = highByte(uint16_t(round(double(bms.getPackVoltage()) / 10)));
  MSG.buf[2] = lowByte(int32_t(round(double(currentact) / 100))); // [ToDo] values > ~6500A will overflow!
  MSG.buf[3] = highByte(int32_t(round(double(currentact) / 100)));
  MSG.buf[4] = lowByte(bms.getAvgTemperature());
  MSG.buf[5] = highByte(bms.getAvgTemperature());
  MSG.buf[6] = 0;
  MSG.buf[7] = 0;
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}

  //[ToDo] Pylontech Errors & Warnings / Protection
  //MSG.id = 0x359

  //SMA Alarms & Warnings
  delay(2);
  MSG.id  = 0x35A;
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

  delay(2);
  MSG.id  = 0x35E;
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
      MSG.buf[0] = highByte(uint16_t(settings.balanceVoltage));
      MSG.buf[1] = lowByte(uint16_t(settings.balanceVoltage));
    }
    else{
      MSG.buf[0] = highByte(uint16_t(bms.getLowCellVolt()));
      MSG.buf[1] = lowByte(uint16_t(bms.getLowCellVolt()));
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
  MSG.id  = 0x373; // non standard
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
  MSG.id  = 0x379; // installed capacity, non standard
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

  CAN_BMC_Std_send_Timer = millis();
}

void CAN_BMC_HV_send(byte CAN_Nr, CAN_message_t inMSG){ //BMC CAN HV Messages
  byte BMC_Addr = 1; //[ToDo] Initial testing. Assign address globally -> settings.
  
  if(inMSG.id == 0x4200 && inMSG.buf[0] == 0){ // Broadcast from Inverter asking for System Data
    
    CAN_message_t MSG;
    MSG.len = 8;
    MSG.flags.extended = true;

    MSG.id  = 0x4210 + BMC_Addr;
    MSG.buf[0] = lowByte(uint16_t(round(double(bms.getPackVoltage()) / 100)));
    MSG.buf[1] = highByte(uint16_t(round(double(bms.getPackVoltage()) / 100)));
    MSG.buf[2] = lowByte(int32_t(round(double(currentact) / 100))); // [ToDo] values > ~6500A will overflow!
    MSG.buf[3] = highByte(int32_t(round(double(currentact) / 100)));
    MSG.buf[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
    MSG.buf[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
    MSG.buf[6] = SOC;
    MSG.buf[7] = SOH_calc();
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}

    delay(2);
    MSG.id  = 0x4220 + BMC_Addr;
    if (storagemode){
      MSG.buf[0] = lowByte(uint16_t(round(float(settings.StoreVsetpoint) / 100 * settings.Scells)));
      MSG.buf[1] = highByte(uint16_t(round(float(settings.StoreVsetpoint) / 100 * settings.Scells)));    
    }else{
      MSG.buf[0] = lowByte(uint16_t(round(float(settings.ChargeVSetpoint) / 100 * settings.Scells)));
      MSG.buf[1] = highByte(uint16_t(round(float(settings.ChargeVSetpoint) / 100 * settings.Scells)));
    }
    MSG.buf[2] = lowByte(uint16_t(round(float(settings.UnderVDerateSetpoint) / 100 * settings.Scells)));
    MSG.buf[3] = highByte(uint16_t(round(float(settings.UnderVDerateSetpoint) / 100 * settings.Scells)));
    MSG.buf[4] = lowByte(chargecurrent);
    MSG.buf[5] = highByte(chargecurrent);
    MSG.buf[6] = lowByte(discurrent);
    MSG.buf[7] = highByte(discurrent);
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}

    delay(2);
    MSG.id  = 0x4230 + BMC_Addr;
    MSG.buf[0] = lowByte(bms.getLowCellVolt());
    MSG.buf[1] = highByte(bms.getLowCellVolt());
    MSG.buf[2] = lowByte(bms.getHighCellVolt());
    MSG.buf[3] = highByte(bms.getHighCellVolt());
    MSG.buf[4] = 0; // MAX Single Battery Cell Voltage Number [ToDo]
    MSG.buf[5] = 0; // MAX Single Battery Cell Voltage Number [ToDo]
    MSG.buf[6] = 0; // MIN Single Battery Cell Voltage Number [ToDo]
    MSG.buf[7] = 0; // MIN Single Battery Cell Voltage Number [ToDo]
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}

    delay(2);
    MSG.id  = 0x4230 + BMC_Addr;
    MSG.buf[0] = lowByte(uint16_t(bms.getHighTemperature() + 100));
    MSG.buf[1] = highByte(uint16_t(bms.getHighTemperature() + 100));
    MSG.buf[2] = lowByte(uint16_t(bms.getLowTemperature() + 100));
    MSG.buf[3] = highByte(uint16_t(bms.getLowTemperature() + 100));
    MSG.buf[4] = 0; // MAX Single Battery Cell Temperature Number [ToDo]
    MSG.buf[5] = 0; // MAX Single Battery Cell Temperature Number [ToDo]
    MSG.buf[6] = 0; // MIN Single Battery Cell Temperature Number [ToDo]
    MSG.buf[7] = 0; // MIN Single Battery Cell VolTemperaturetage Number [ToDo]
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}

    // Errors & Alarms [ToDo]
    delay(2);
    uint16_t tmpAlarm = 0;
    uint16_t tmp = 0;
    tmp = (alarm[0] & 0b00010000) >> 4; // Bit0: Cell low Volt
    tmpAlarm |= tmp;
    tmp = (alarm[0] & 0b00000100) >> 1; // Bit1: Cell high Volt
    tmpAlarm |= tmp;
    tmp = (alarm[0] & 0b00010000) >> 2; // Bit2: Discharge low Volt
    tmpAlarm |= tmp;
    tmp = (alarm[0] & 0b00000100);      // Bit3: Charge high Volt
    tmpAlarm |= tmp;
    tmp = (alarm[1] & 0b00000001) << 3; // Bit4: Charge Cell low Temp
    tmpAlarm |= tmp;
    tmp = (alarm[0] & 0b01000000) >> 2; // Bit5: Charge Cell high Temp
    tmpAlarm |= tmp;
    tmp = (alarm[1] & 0b00000001) << 5; // Bit6: Discharge Cell low Temp
    tmpAlarm |= tmp;
    tmp = (alarm[0] & 0b01000000);      // Bit7: Discharge Cell high Temp
    tmpAlarm |= tmp;
    tmp = 0b00000000; // Bit8: Charge Overcurrent [ToDo]
    tmpAlarm |= tmp;
    tmp = 0b01000000; // Bit9: Discharge Overcurrent [ToDo]
    tmpAlarm |= tmp;

    MSG.id  = 0x4250 + BMC_Addr;
    MSG.buf[0] = 0b00000011; // bit0-2: 0 = sleep, 1 = Charge, 2 = Discharge, 3 = Idle; Bit3: Force Charge; Bit4: Force Balance Charge; Bit5-7: Reserved; [ToDo]
    MSG.buf[1] = 0; // Cycle Period?
    MSG.buf[2] = 0; // Cycle Period?
    MSG.buf[3] = 0; // Error
    MSG.buf[4] = lowByte(tmpAlarm); // Alarm
    MSG.buf[5] = highByte(tmpAlarm); // Alarm
    MSG.buf[6] = 0; // Protection same as Alarm?
    MSG.buf[7] = 0; // Protection same as Alarm?
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}

    MSG.id  = 0x4260 + BMC_Addr; // Module Voltage MIN / MAX & Nr.
    MSG.id  = 0x4270 + BMC_Addr; // Module Temp. MIN / MAX & Nr.
    MSG.id  = 0x4280 + BMC_Addr; // Charge / Dischage foorbidden ?
  }

  if(inMSG.id == 0x4200 && inMSG.buf[0] == 2){ // Broadcast from Inverter asking for System Information
    CAN_message_t MSG;
    MSG.len = 8;
    MSG.flags.extended = true;

    MSG.id = 0x7310 + BMC_Addr;
    MSG.buf[0] = 0; // Version 1=A, 2=B
    MSG.buf[1] = 0; // reserved
    MSG.buf[2] = 3; // Hardware Version e.g. 3.2
    MSG.buf[3] = 2; // Hardware Version e.g. 3.2
    MSG.buf[4] = 3; // Software Version e.g. 3.4 (03/2024)
    MSG.buf[5] = 4; // Software Version e.g. 3.4 (03/2024)
    MSG.buf[6] = 0; // Software Version ??
    MSG.buf[7] = 0; // Software Version ??
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}

    delay(2);
    MSG.id = 0x7320 + BMC_Addr;
    MSG.buf[0] = lowByte(bms.getNumModules());
    MSG.buf[0] = highByte(bms.getNumModules());
    MSG.buf[2] = bms.getNumModules();
    MSG.buf[3] = 0; // [ToDo] Cells per module
    MSG.buf[4] = 0; // Volage Level?
    MSG.buf[5] = 0; // Volage Level?
    MSG.buf[6] = 0; // AH Number?
    MSG.buf[7] = 0; // AH NUmber?
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}

    delay(2);
    MSG.id = 0x7330 + BMC_Addr;
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
    MSG.id = 0x7340 + BMC_Addr;
    MSG.buf[0] = 0;
    MSG.buf[1] = 0;
    MSG.buf[2] = 0;
    MSG.buf[3] = 0;
    MSG.buf[4] = 0;
    MSG.buf[5] = 0;
    MSG.buf[6] = 0;
    MSG.buf[7] = 0;
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}
  }
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
      MSG.buf[0] = highByte(uint16_t(round(float(settings.ChargeVSetpoint) / 1000 * settings.Scells * 10)));
      MSG.buf[1] = lowByte(uint16_t(round(float(settings.ChargeVSetpoint) / 1000 * settings.Scells  * 10)));
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
      MSG.buf[3] = lowByte(uint16_t(round(float(settings.ChargeVSetpoint) / 1000 * settings.Scells * 10)));
      MSG.buf[4] = highByte(uint16_t(round(float(settings.ChargeVSetpoint) / 1000 * settings.Scells * 10)));
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
      MSG.buf[3] = highByte(uint16_t(round(((float(settings.ChargeVSetpoint) / 1000 * settings.Scells) - float(chargerendbulk) / 1000) * 10)));
      MSG.buf[4] = lowByte(uint16_t(round(((float(settings.ChargeVSetpoint) / 1000 * settings.Scells) - float(chargerendbulk) / 1000) * 10)));
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
      MSG.buf[3] = highByte(uint16_t(round(((float(settings.ChargeVSetpoint) / 1000 * settings.Scells) - float(chargerend) / 1000) * 10)));
      MSG.buf[4] = lowByte(uint16_t(round(((float(settings.ChargeVSetpoint) / 1000 * settings.Scells) - float(chargerend) / 1000) * 10)));
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
      if ((float(settings.ChargeVSetpoint)/1000 * settings.Scells) > 200){
        MSG.buf[2] = highByte(uint16_t(round((float(settings.ChargeVSetpoint) / 1000 * settings.Scells ) * 2)));
        MSG.buf[3] = lowByte(uint16_t(round((float(settings.ChargeVSetpoint) / 1000 * settings.Scells ) * 2)));
      }else{
        MSG.buf[2] = highByte(400);
        MSG.buf[3] = lowByte(400);
      }
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
    break;

    case Charger_Coda:
      MSG.id  = 0x050;
      MSG.len = 8;
      MSG.buf[0] = 0x00;
      MSG.buf[1] = 0xDC;
      if ((float(settings.ChargeVSetpoint) / 1000 * settings.Scells) > 200){
        MSG.buf[2] = highByte(uint16_t(round((float(settings.ChargeVSetpoint) / 1000 * settings.Scells ) * 10)));
        MSG.buf[3] = lowByte(uint16_t((float(settings.ChargeVSetpoint) / 1000 * settings.Scells ) * 10));
      }else{
        MSG.buf[2] = highByte( 400);
        MSG.buf[3] = lowByte( 400);
      }
      MSG.buf[4] = 0x00;
      if ((float(settings.ChargeVSetpoint/1000) * settings.Scells) * chargecurrent < 3300){
        MSG.buf[5] = highByte(uint16_t(round(((float(settings.ChargeVSetpoint) / 1000 * settings.Scells) * chargecurrent) / 240)));
        MSG.buf[6] = highByte(uint16_t(round(((float(settings.ChargeVSetpoint) / 1000 * settings.Scells) * chargecurrent) / 240)));
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
    //tmp_id = CO_Handle(MSG);
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

void CAN_TCU_read(CAN_message_t MSG){
  TCU_Pump = TCU_Cool = TCU_Heat = TCU_REL4 = 0;
  if (MSG.id == 0xBA){
    TCU_Pump = MSG.buf[0];
    TCU_Cool = MSG.buf[1];
    TCU_Heat = MSG.buf[2];
    TCU_REL4 = MSG.buf[3];
  }
}