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

#include "JoeMBMC.h"
#include <Arduino.h>
#include <EEPROM.h>
#include "Watchdog_t4.h"  //https://github.com/tonton81/WDT_T4
#include "Globals.h"

//CAN - Don't outsource to globals! Breaks the FlexCAN_T4 library. It needs to be in the main file.
FlexCAN_T4<CAN1> can1;
FlexCAN_T4<CAN2> can2;


void loadDefaultSettings(){
  settings.version = EEPROM_VERSION;
  settings.checksum = 2;
  settings.ReadTimeout = 2000;
  settings.batteryID = 0x01;
  settings.BMSType = BMS_Dummy;
  settings.OverVAlarm = 4200;
  settings.OverVWarn = 4100;
  settings.ChargeVSetpoint = 4150;
  settings.UnderVWarn = 3200;
  settings.UnderVAlarm = 3000;
  settings.ChargeHys = 100; 
  settings.UnderVAlarm_DischHys = 200; 
  settings.CellGap = 500; 
  settings.OverTAlarm = 550; // 0.1°C
  settings.OverTWarn = 450; // 0.1°C
  settings.UnderTWarn = 0; // 0.1°C
  settings.UnderTAlarm = -200; // 0.1°C
  settings.ChargeOverTAlarm = 550; // 0.1°C
  settings.ChargeOverTWarn = 450; // 0.1°C
  settings.ChargeUnderTWarn = 50; // 0.1°C
  settings.ChargeUnderTAlarm = 0; // 0.1°C
  settings.useTempSensor = 0; 
  settings.IgnoreVolt = 500;
  settings.balanceVoltage = 3700;
  settings.balanceHyst = 10;
  settings.balanceDuty = 50;
  settings.CAP = 230; 
  settings.designCAP = 230; 
  settings.CAP_Wh = 25000;
  settings.Pstrings = 1;
  settings.Scells = 30;
  settings.StoreVsetpoint = 3800; 
  settings.OverCurrAlarm = 5500; 
  settings.OverCurrWarn = 5000; 
  settings.ChargerChargeCurrentMax = 320; 
  settings.ChargeOverCurrAlarm = 1000; 
  settings.ChargeOverCurrWarn = 950; 
  settings.ChargeCurrentEnd = 50; 
  settings.socvolt[0] = 3100; 
  settings.socvolt[1] = 10; 
  settings.socvolt[2] = 4100;
  settings.socvolt[3] = 90;
  settings.CurSenInvert = 0; 
  settings.CurSenType = Sen_Canbus;
  settings.voltsoc = 0; 
  settings.PreTime = 5000; 
  settings.PreCurrent = 1000; 
  settings.analogueSen2_convhigh = 312; 
  settings.analogueSen1_convlow = 1250; 
  settings.analogueSen1_offset = 1650; 
  settings.analogueSen2_offset = 1650;
  settings.analogueSen_ChangeCur = 140000;
  settings.GaugeLow = 0; 
  settings.GaugeHigh = 255; 
  settings.analogueSen_CurDead = 5;
  settings.ESSmode = 0; 
  settings.CurSenMultiplier = 1; 
  settings.nChargers = 1; 
  settings.ChargerType = Charger_Elcon; 
  settings.ChargerDirect = 1; 
  settings.mctype = 0; 
  settings.secondarySerial = 0; 
  settings.error_delay = 10000; 
  settings.Temp_CAP_Map[0][0] = -200;
  settings.Temp_CAP_Map[0][1] = -100;
  settings.Temp_CAP_Map[0][2] = 0;
  settings.Temp_CAP_Map[0][3] = 250;
  settings.Temp_CAP_Map[0][4] = 400;
  settings.Temp_CAP_Map[1][0] = 38;
  settings.Temp_CAP_Map[1][1] = 69;
  settings.Temp_CAP_Map[1][2] = 78;
  settings.Temp_CAP_Map[1][3] = 90;
  settings.Temp_CAP_Map[1][4] = 100;
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
  settings.SOC = 0;
  settings.Warning_Blink_Hz = 0;
  settings.Error_Blink_Hz = 0;
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
  pinMode(IN4_HVIL, INPUT);
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
  // manage EEPROM versions
  if (settings.version != EEPROM_VERSION){ loadDefaultSettings(); }
  
  
  //if using enable pins on a transceiver they need to be set on

  adc->adc0->setAveraging(16); // set number of averages
  adc->adc0->setResolution(16); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc->adc0->startContinuous(IN_ACUR1);

  //delay(2000);  //just for easy debugging. It takes a few seconds for USB to come up properly on most OS's  
  Serial_USB.begin(115200);
  Serial_USB.setTimeout(20);
  Serial_BT.begin(115200);
  activeSerial->printf("Starting up...\r\n");
  activeSerial->printf("JoeM BMC\r\n");
  //display serial
  Serial_Display.begin(115200); 
  //Tesla serial bus
  Serial_BMS.begin(612500);
  activeSerial->printf("Started serial interface to Tesla-BMS.\r\n");

  // enable WDT
  watchdog.begin(watchdog_config);

  BMSInit();

  Timer_mAmpSec.begin(mAmpsec_calc, 500000);

  can1_start();
  can2_start();
}

//called by Timer
void mAmpsec_calc(){ 
  uint32_t nowTime = millis();
  static uint32_t lastTime = 0;
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
  
  static byte Modules_Print_ON = 1;
  static byte Out_Print_ON = 0;

  BMC_Status_LED();
  //BOD(analogRead(IN_BOD)); // [ToDo] detect voltage drop on 12V side
  
  //everything CAN related happens here
  CAN_read();

  if ( settings.CurSenType == Sen_AnalogueDual || settings.CurSenType == Sen_AnalogueSing){
    currentact = SEN_AnalogueRead(currentlast);
  }

  // make serial console available after 3s
  if (millis() > 3000 && activeSerial->available()){
    String tmp_String = activeSerial->readString();
    tmp_String.trim();
    if(menu_load){Menu(tmp_String);}
    else{
      if(tmp_String == 'o'){Out_Print_ON = !Out_Print_ON;}
      if(tmp_String == 'b'){Modules_Print_ON = !Modules_Print_ON;}
      if(tmp_String == "menu"){menu_load = 1; Menu();}
    }
  }

  WarnAlarm_Check();
  if (!debug_Output){
    if (settings.ESSmode){BMC_Stat = ESS_CondCheck(BMC_Stat);}
    else {BMC_Stat = Vehicle_CondCheck(BMC_Stat);}
    BMC_Statemachine(BMC_Stat);
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

  // 1s loop
  static uint32_t looptime_1s = 0;
  if (millis() - looptime_1s > 1000){
    looptime_1s = millis();
    Uptime++;
  }

    
  // 0.5s loop
  static uint32_t looptime = 0;
  if (millis() - looptime > 500){ 
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
      Serial_Print(Modules_Print_ON); 
      // OUT functions and states
      OUT_print(Out_Print_ON);
      // Display reason the Teensy was last reset
      Reset_Cause(lastResetCause);
    } //"debug" output on serial console
    //if (debug_CSV){ bms.printAllCSV(millis(), currentact, SOC); }
    
    if (debug_Input){ Input_Debug(); }
    if (debug_Output){ Output_debug(); }
    else{ Gauge_update(); }
    SOC_update();
    Currentavg_Calc();
    if (settings.secondarySerial == 0){ Dash_update(); }//Info on serial bus 2
    

    //if (settings.secondarySerial == 2){ BT_update(); }// --> Bluetooth App
    //WDT reset;

    watchdog.feed();
  }   
}

/* ==================================================================== */
// i.MX RT1060 Processor Reference Manual, 21.8.3 SRC Reset Status Register
void Reset_Cause(uint32_t resetStatusReg) {
  bool info = false;
  activeSerial->printf("\r\nReason for last Reset: ");
  if (resetStatusReg & SRC_SRSR_TEMPSENSE_RST_B) {
      activeSerial->printf("Temperature Sensor Software Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_WDOG3_RST_B) {
      activeSerial->printf("IC Watchdog3 Timeout Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_JTAG_SW_RST) {
      activeSerial->printf("JTAG Software Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_JTAG_RST_B) {
      activeSerial->printf("High-Z JTAG Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_WDOG_RST_B) {
      activeSerial->printf("IC Watchdog Timeout Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_IPP_USER_RESET_B) {
      activeSerial->printf("Power-up Sequence (Cold Reset Event)\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_CSU_RESET_B) {
      activeSerial->printf("Central Security Unit Reset\r\n");
      info = true;
  }
  if (resetStatusReg & SRC_SRSR_LOCKUP_SYSRESETREQ) {
      activeSerial->printf("CPU Lockup or Software Reset\r\n");
      info = true;
      /* Per datasheet: "SW needs to write a value to SRC_GPR5
       * before writing the SYSRESETREQ bit and use the SRC_GPR5
       * value to distinguish if the reset is caused by SYSRESETREQ
       * or CPU lockup."
       */
  }
  if (resetStatusReg & SRC_SRSR_IPP_RESET_B) {
      activeSerial->printf("Power-up Sequence\r\n");
      info = true;
  }
  if (!info) {
      activeSerial->printf("No status bits set in SRC Reset Status Register\r\n");
  }
}

void BOD(int16_t ADC_read){
  if (ADC_read <= 4000){
    activeSerial->printf("::::::::::::::::::::::::BOD triggered::::::::::::::::::::::::\r\n");
    settings.SOC = SOC;
    EEPROM.put(0,settings);
  }
}

void BMSInit(){
  bms.initBMS(settings.BMSType,settings.IgnoreVolt,settings.useTempSensor,settings.ReadTimeout);
}

// Returns 1 if requested Warning or Alarm is raised. Type 0 = ignored, 1 = Warning, 2 = Error
bool WarnAlarm_Check(byte Type, byte WarnAlarm){
  static uint32_t warning_timer = 0;
  byte hyst = 5; // 0.5°C hysteresis [ToDO] make config.

  // reset to all OK
  for(byte i = 0; i<4; i++){
    warning[i] = 0b10101010;
    alarm[i] = 0b10101010;
  } 

  // Overvoltage
  if (bms.getHighCellVolt() > settings.OverVWarn){warning[0] ^= 0b00001100;}  
  if (bms.getHighCellVolt() > settings.OverVAlarm){alarm[0] ^= 0b00001100;}

  // Undervoltage
  if (bms.getLowCellVolt() < (settings.UnderVWarn)){
    warning[0] ^= 0b00110000;
    if (!warning_timer){warning_timer = millis() + 10000;}
    //if (millis() > warning_timer){CAP_recalc();warning_timer = 0;} // [ToDo] recalculate capacity when lowest cell is under warning setpoint; reset Timer to prevent EEPROM Kill
  } else {warning_timer = 0;}    
  if (bms.getLowCellVolt() < settings.UnderVAlarm){alarm[0] ^= 0b00110000;}

  // Over Temperature 
  if (bms.getHighTemperature() > settings.OverTWarn){warning[0] ^= 0b11000000;}  
  if (bms.getHighTemperature() > settings.OverTAlarm){alarm[0] ^= 0b11000000;}

  // Under Temperature
  if (bms.getLowTemperature() < settings.UnderTWarn){warning[1] ^= 0b00000011;}  
  if (bms.getLowTemperature() < settings.UnderTAlarm){alarm[1] ^= 0b00000011;}

  // Charge Over Temperature
  if (bms.getHighTemperature() > settings.ChargeOverTWarn){warning[1] ^= 0b00001100;}
  if (bms.getHighTemperature() > settings.ChargeOverTAlarm){alarm[1] ^= 0b00001100;}

  // Charge Under Temperature
  if (bms.getLowTemperature() < settings.ChargeUnderTWarn){warning[1] ^= 0b00110000;}
  if (bms.getLowTemperature() < settings.ChargeUnderTAlarm){alarm[1] ^= 0b00110000;}

  // Over Current
  if (currentact / 100 * -1 > settings.OverCurrWarn){warning[1] ^= 0b11000000;}
  if (currentact / 100 * -1 > settings.OverCurrAlarm){alarm[1] ^= 0b11000000;}

  // Charge Over Current
  if (currentact / 100 > settings.ChargeOverCurrWarn){warning[2] ^= 0b00000011;} 
  if (currentact / 100 > settings.ChargeOverCurrAlarm){alarm[2] ^= 0b00000011;}

  // External Alarm / Series Cells [ToDo] include BMB/CMU Alerts
  if (bms.getSeriesCells() != settings.Scells * settings.Pstrings){warning[2] ^= 0b11000000;}
//if (bms.getSeriesCells() != settings.Scells * settings.Pstrings){alarm[2] ^= 0b11000000;} // [ToDo] elaborate

  // Cell Gap
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap){warning[3] ^= 0b00000011;}
//if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap){alarm[3] ^= 0b00000011;} // [ToDo] elaborate


  switch (WarnAlarm){
    case WarnAlarm_VLow:
      if(Type == 1){return warning[0] & 0b00010000;}
      if(Type == 2){return alarm[0] & 0b00010000;}
    break;
    case WarnAlarm_Vhigh:
      if(Type == 1){return warning[0] & 0b00000100;}
      if(Type == 2){return alarm[0] & 0b00000100;}
    break;
    case WarnAlarm_ChargeTLow:
      if(Type == 1){return warning[1] & 0b00010000;}
      if(Type == 2){return alarm[1] & 0b00010000;}
    break;
    case WarnAlarm_ChargeTHigh:
      if(Type == 1){return warning[1] & 0b00000100;}
      if(Type == 2){return alarm[1] & 0b00000100;}
    break;
    case WarnAlarm_TLow:
      if(Type == 1){return warning[1] & 0b00000001;}
      if(Type == 2){return alarm[1] & 0b00000001;}
    break;
    case WarnAlarm_THigh:
      if(Type == 1){return warning[0] & 0b01000000;}
      if(Type == 2){return alarm[0] & 0b01000000;}
    break;
    case WarnAlarm_CurHigh:
      if(Type == 1){return warning[1] & 0b01000000;}
      if(Type == 2){return alarm[1] & 0b01000000;}
    break;
    case WarnAlarm_ChargeCurHigh:
      if(Type == 1){return warning[2] & 0b00000001;}
      if(Type == 2){return alarm[2] & 0b00000001;}
    break;
    case WarnAlarm_External:
      if(Type == 1){return warning[2] & 0b01000000;}
      if(Type == 2){return alarm[2] & 0b01000000;}
    break;
    case WarnAlarm_CellGap:
      if(Type == 1){return warning[3] & 0b00000001;}
      if(Type == 2){return alarm[3] & 0b00000001;}
    break;
    
    default: // return 1 if any Warning or Alarm is raised. No specific WarnAlarm passed (=0=Dummy).
      if(Type == 1){if (warning[0] & 0b01010101 || warning[1] & 0b01010101 || warning[2] & 0b01010101 || warning[3] & 0b01010101) {return 1;}}
      if(Type == 2){if (alarm[0] & 0b01010101 || alarm[1] & 0b01010101 || alarm[2] & 0b01010101 || alarm[3] & 0b01010101) {return 1;}} 
    break;
  }
  return 0;
}



byte Vehicle_CondCheck(byte tmp_status){ 
  // start with no Errors
  if(!Charge_Active()){
    tmp_status = Stat_Ready;
    // detect KEY ON
    if (digitalRead(IN1_Key)){
      if (precharged) {tmp_status = Stat_Drive;}
      else {tmp_status = Stat_Precharge;}
    } 
  }

  // detect Undervoltage before Charging / let Charging override this Error
  if (WarnAlarm_Check(WarnAlarm_Alarm,WarnAlarm_VLow)) {tmp_status = Stat_Error;}

  //detect AC present & Check charging conditions
  if (Charge_Active()){
    //start charging when Voltage is below Charge Voltage - ChargeHyst.
    if (bms.getHighCellVolt() < (settings.ChargeVSetpoint - settings.ChargeHys)){ 
      if (precharged || settings.ChargerDirect){tmp_status = Stat_Charge;}
      else {tmp_status = Stat_Precharge;}      
    }
    //Set SOC 100% / charged when Voltage gets above Charge Voltage 
    if (bms.getHighCellVolt() > (settings.ChargeVSetpoint)){
      SOC_charged();
      tmp_status = Stat_Charged;
    }
    // Set charged Voltage is above Charge Voltage - ChargeHyst.
    if (bms.getHighCellVolt() > (settings.ChargeVSetpoint - settings.ChargeHys) && tmp_status != Stat_Charge){
      tmp_status = Stat_Charged;
    }    

    if(WarnAlarm_Check(WarnAlarm_Alarm,WarnAlarm_ChargeCurHigh) || 
       WarnAlarm_Check(WarnAlarm_Alarm,WarnAlarm_ChargeTHigh) || 
       WarnAlarm_Check(WarnAlarm_Alarm,WarnAlarm_ChargeTLow))
       {tmp_status = Stat_Error;}
  }

  // Set Error depending on Error conditions, except Undervoltage to let Charging recover from Undervoltage
  
  if (WarnAlarm_Check(WarnAlarm_Alarm,WarnAlarm_Vhigh) || 
      WarnAlarm_Check(WarnAlarm_Alarm,WarnAlarm_THigh) || 
      WarnAlarm_Check(WarnAlarm_Alarm,WarnAlarm_TLow) ||
      WarnAlarm_Check(WarnAlarm_Alarm,WarnAlarm_CurHigh))
      {tmp_status = Stat_Error;}


  // reset Error-Timer
  if (tmp_status != Stat_Error){error_timer = 0;}
  return tmp_status; 
}

byte ESS_CondCheck(byte tmp_status){
  //byte tmp_status = 0;
  static byte hyst_cleared = true;
  // Precharge first
  if (precharged){
    tmp_status = Stat_Idle;

    if(currentact < 0 && hyst_cleared){tmp_status = Stat_Discharge;}

    //start charging when Voltage is below Charge Voltage - ChargeHyst.
    if (bms.getHighCellVolt() < settings.ChargeVSetpoint && currentact > 0 && hyst_cleared){ 
      tmp_status = Stat_Charge;
    }

    //Set SOC 100% / charged when Voltage gets above Charge Voltage 
    if (bms.getHighCellVolt() > (settings.ChargeVSetpoint)){
      SOC_charged();
      tmp_status = Stat_Charged;
      hyst_cleared = false;
    }  

    // clear hysterese when voltage is below Charge Voltage - ChargeHyst.
     if (bms.getHighCellVolt() < (settings.ChargeVSetpoint - settings.ChargeHys) && !hyst_cleared){
      hyst_cleared = true;
    }  

    if(!hyst_cleared){
      tmp_status = Stat_Charged;
    }
  }
  else {tmp_status = Stat_Precharge;}  

//[ToDo] storagemode?
/*
  if (digitalRead(IN1_Key) == HIGH) {}
*/


  // Set Error depending on Error conditions
  if (WarnAlarm_Check(WarnAlarm_Alarm, WarnAlarm_Vhigh) || 
      WarnAlarm_Check(WarnAlarm_Alarm, WarnAlarm_VLow) || 
      WarnAlarm_Check(WarnAlarm_Alarm, WarnAlarm_THigh) || 
      WarnAlarm_Check(WarnAlarm_Alarm, WarnAlarm_TLow) || 
      WarnAlarm_Check(WarnAlarm_Alarm, WarnAlarm_ChargeTHigh) || 
      WarnAlarm_Check(WarnAlarm_Alarm, WarnAlarm_ChargeTLow) || 
      WarnAlarm_Check(WarnAlarm_Alarm, WarnAlarm_ChargeCurHigh) || 
      WarnAlarm_Check(WarnAlarm_Alarm, WarnAlarm_CellGap))
      {tmp_status = Stat_Error;}

  if (tmp_status != Stat_Error) {error_timer = 0;}
  return tmp_status;
}

/*
  Detect charging via external inputs like IN3_AC, CAN-bus...
*/
bool Charge_Active(){
  bool retVal = false;
  if (digitalRead(IN3_AC) == HIGH){retVal = true;} // read Input3 for AC presence
  if (v_CAN_Charger_Active == true) {
    if(v_CAN_Charger_Active_Timer + 5000 <= millis()){ // Reset CAN_Charger_Active after 5s without CAN message.
      v_CAN_Charger_Active = false; 
    } else {
      retVal = true;
    }
  }
  return retVal;
}

void CAN_Charge_Check(CAN_message_t MSG){
  if (MSG.id == 0x18ff50e5){ // Elcon / TC Chargers
    v_CAN_Charger_Active = true;
    v_CAN_Charger_Active_Timer = millis();
  }
}

byte Warn_Out_handle(){
  if(WarnAlarm_Check(WarnAlarm_Warning,WarnAlarm_Dummy)){
    set_OUT_States(Out_Err_Warn);
    set_OUT_States(Out_Warning);
    // high temp -> activate cooling
    if(WarnAlarm_Check(WarnAlarm_Warning,WarnAlarm_THigh) || WarnAlarm_Check(WarnAlarm_Warning,WarnAlarm_ChargeTHigh)){
      if(bms.getLowCellVolt() > settings.UnderVAlarm + settings.UnderVAlarm_DischHys){
        set_OUT_States(Out_Cooling);
      }
    }
    // low temp -> activate heating
    if(WarnAlarm_Check(WarnAlarm_Warning,WarnAlarm_TLow) || WarnAlarm_Check(WarnAlarm_Warning,WarnAlarm_ChargeTLow)){
      if(bms.getLowCellVolt() > settings.UnderVAlarm + settings.UnderVAlarm_DischHys){
        set_OUT_States(Out_Heating);
      }
    }
    return 1;
  }
  return 0;
}

void BMC_Statemachine(byte status){
  switch (status){
    case Stat_Boot:  //[ToDO] not used. Change to maintenance mode?
      set_OUT_States(); // all off
      Balancing(false);
    break;
    case Stat_Ready:
      precharged = 0;
      set_OUT_States(); // all off
      Balancing(true);
      Warn_Out_handle();
      //CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]); [ToDo] why send in ready?
    break;
    case Stat_Drive:
      set_OUT_States(); // all off
      set_OUT_States(Out_Cont_Pos);
      set_OUT_States(Out_Cont_Neg);
      set_OUT_States(Out_Gauge);
      DischargeCurrentLimit();
      ChargeCurrentLimit();
      Warn_Out_handle();
      Balancing(false);
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;
    case Stat_Precharge:
      set_OUT_States(); // all off
      Prechargecon();
      Balancing(false);
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;    
    case Stat_Charge:
      set_OUT_States(); // all off
      set_OUT_States(Out_Charge);
      set_OUT_States(Out_Cont_Neg);
      set_OUT_States(Out_Cont_Pos);      
      //CAP_recalc(); [ToDo]
      //if (Settings_unsaved){EEPROM.put(0, settings); Settings_unsaved = 0; activeSerial->printf("--Saved--\r\n");} [ToDo]
      if (digitalRead(IN1_Key) == HIGH){set_OUT_States(Out_Gauge);} // enable gauge if key is on
      Balancing(true);
      DischargeCurrentLimit();
      ChargeCurrentLimit();
      Warn_Out_handle();
      CAN_Charger_Send(settings.CAN_Map[0][CAN_Charger]);
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;
    case Stat_Charged:
      set_OUT_States(); // all off
      // enable gauge if key is on  
      if (digitalRead(IN1_Key) == HIGH){
        set_OUT_States(Out_Gauge);
        }
      // leave contactors on in ESS mode
      if (settings.ESSmode){
        set_OUT_States(Out_Cont_Neg);
        set_OUT_States(Out_Cont_Pos);   
      }
      Balancing(true);
      DischargeCurrentLimit();
      ChargeCurrentLimit(0);
      //chargecurrent = 0;
      Warn_Out_handle();
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;
    case Stat_Error:
      if (!error_timer){ error_timer = millis() + settings.error_delay; }// delay before turning everything off
      if (millis() > error_timer){
        set_OUT_States(); // all off
        set_OUT_States(Out_Error);
        set_OUT_States(Out_Err_Warn);
        if (digitalRead(IN1_Key) == HIGH){set_OUT_States(Out_Gauge);} // enable gauge if key is on    
        if (Charge_Active()){CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);}
        if (settings.ESSmode){CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);}
        Balancing(false);
        discurrent = 0;
        chargecurrent = 0;
      }
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;
    case Stat_Idle:
      set_OUT_States(); // all off
      set_OUT_States(Out_Discharge);
      set_OUT_States(Out_Cont_Pos);
      set_OUT_States(Out_Cont_Neg);
      set_OUT_States(Out_Gauge);
      Balancing(true);
      DischargeCurrentLimit();
      ChargeCurrentLimit();
      Warn_Out_handle();
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;
    case Stat_Discharge:
      set_OUT_States(); // all off
      set_OUT_States(Out_Discharge);
      set_OUT_States(Out_Cont_Pos);
      set_OUT_States(Out_Cont_Neg);
      set_OUT_States(Out_Gauge);
      DischargeCurrentLimit();
      ChargeCurrentLimit();
      Warn_Out_handle();
      CAN_BMC_Std_send(settings.CAN_Map[0][CAN_BMC_std]);
    break;
  } 
}

void BMC_Status_LED(){
  static uint32_t LED_Timer = 0;
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

// triggered by statemachine
void Balancing(bool active){
  CAN_Struct BalanceMatrix;
  BalanceMatrix = clearCANStruct();
  if (active && bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst){
    BalanceMatrix = bms.Balancing(settings.balanceHyst, true, activeSerial);
    for (byte CANMsgNr = 0; CANMsgNr < CAN_Struct_size; CANMsgNr++){
      if (BalanceMatrix.Frame[CANMsgNr].id){ // We only send actually filled Frames.
      //[ToDo] restrict to interval? will otherwise flood the CAN bus.
      /*
        if (settings.CAN_Map[0][CAN_BMS] & 1){can1.write(BalanceMatrix.Frame[CANMsgNr]);}
        if (settings.CAN_Map[0][CAN_BMS] & 2){can2.write(BalanceMatrix.Frame[CANMsgNr]);}
        delay(1);
        */
      }
    }
  } else {
    bms.Balancing(settings.balanceHyst, false, activeSerial);
  }
}



uint32_t ConvertToBin(uint32_t Input){
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
  switch (settings.CurSenType){
    case Sen_AnalogueSing:
      Sen_Analogue_active_Nr = 1;
      adc->adc0->startContinuous(IN_ACUR1);   
    break;
    case Sen_AnalogueDual:
      if (tmp_currrentlast < settings.analogueSen_ChangeCur && tmp_currrentlast > (settings.analogueSen_ChangeCur * -1)){
        if (Sen_Analogue_active_Nr == 2){adc->adc0->stopContinuous();}
        Sen_Analogue_active_Nr = 1;
        adc->adc0->startContinuous(IN_ACUR1);
      }else{
        if (Sen_Analogue_active_Nr == 1){adc->adc0->stopContinuous();}
        Sen_Analogue_active_Nr = 2;
        adc->adc0->startContinuous(IN_ACUR2);
      }    
    break;
  }

  Sen_AnalogueRawValue = (uint16_t)adc->adc0->analogReadContinuous(); 

  switch (Sen_Analogue_active_Nr){
    case 1:
      RawCur = round(((float(Sen_AnalogueRawValue) * 3300 / adc->adc0->getMaxValue()) - settings.analogueSen1_offset) / (float(settings.analogueSen1_convlow) / 100 / 1000 / (5 / 3.3)));
      if (labs((round((float(Sen_AnalogueRawValue) * 3300 / adc->adc0->getMaxValue()) - settings.analogueSen1_offset))) <  settings.analogueSen_CurDead) { RawCur = 0; }
    break;
    case 2:
      RawCur = round(((float(Sen_AnalogueRawValue) * 3300 / adc->adc0->getMaxValue()) - settings.analogueSen2_offset) / (float(settings.analogueSen2_convhigh) / 100 / 1000 / (5 / 3.3)));
    break;
    }      

  lowpassFilter.input(RawCur);
  tmp_current = lowpassFilter.output();
  tmp_current = settings.CurSenMultiplier * tmp_current;
  if (settings.CurSenInvert){ tmp_current *= -1; }
  return tmp_current;
} 

void CAN_SEN_read(CAN_message_t MSG, int32_t& CANmilliamps){
  //int32_t CANmilliamps = 0;
  bool newVal = false;
  switch (MSG.id){
    //LEM CAB [ToTest] CAB1500 has same ID. ALso same Data?
    case 0x3c0: //CAB 300-C/SP3-000
    case 0x3c1: //CAB 300-C/SP3-001
    case 0x3c2: //CAB 300-C/SP3-002 & CAB 300-C/SP3-010 & CAB-500 (all Versions)
    case 0x3c3: //CAB 300-C/SP3-003
    case 0x3c4: //CAB 300-C/SP3-004
    case 0x3c5: //CAB 300-C/SP3-005
    case 0x3c6: //CAB 300-C/SP3-006
    case 0x3c7: //CAB 300-C/SP3-007
    case 0x3c8: //CAB 300-C/SP3-008
    case 0x3c9: //CAB 300-C/SP3-009
      CANmilliamps = CAN_SEN_LEMCAB(MSG); newVal = true; break; 
    //IsaScale
    case 0x521: 
      CANmilliamps = MSG.buf[5] + (MSG.buf[4] << 8) + (MSG.buf[3] << 16) + (MSG.buf[2] << 24); newVal = true; break;
    //case 0x522: ISAVoltage1 = MSG.buf[5] + (MSG.buf[4] << 8) + (MSG.buf[3] << 16) + (MSG.buf[2] << 24); break;
    //case 0x523: ISAVoltage2 = MSG.buf[5] + (MSG.buf[4] << 8) + (MSG.buf[3] << 16) + (MSG.buf[2] << 24); break;    
    //default: newVal = false; activeSerial->println("false"); break;
  }

  //Victron Lynx  
  if (pgnFromCANId(MSG.id) == 0x1F214 && MSG.buf[0] == 0) // Check PGN and only use the first packet of each sequence
  {CANmilliamps = CAN_SEN_VictronLynx(MSG); newVal = true; }
  //else {newVal = false;activeSerial->println("false2");}

  if (settings.CurSenInvert && newVal){ CANmilliamps *= -1;}
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
    case 1: activeSerial->printf("CAN1 "); break;
    case 2: activeSerial->printf("CAN2 "); break;
  }
  activeSerial->printf("IN ");
  if (MSG.flags.extended){
    activeSerial->printf("ext:%x  ",MSG.id);
    pgn = pgnFromCANId(MSG.id);
  } else {
    activeSerial->printf("std:%x  ",MSG.id);
  }
  for (byte i = 0; i < MSG.len; i++) { // print the data
    activeSerial->printf("%x ",MSG.buf[i]);
  }
  if(pgn){
    activeSerial->printf("PGN: %i",pgn);
  }
  activeSerial->printf("\r\n");
}

void CAN_Debug_OUT(byte CanNr, CAN_message_t MSG){
  int16_t pgn = 0;
  
  switch (CanNr){
    case 1: activeSerial->printf("CAN1 "); break;
    case 2: activeSerial->printf("CAN2 "); break;
  }
  activeSerial->printf("OUT ");
  if (MSG.flags.extended){
    activeSerial->printf("ext:%x  ",MSG.id);
    pgn = pgnFromCANId(MSG.id);
  } else {
    activeSerial->printf("std:%x  ",MSG.id);
  }
  for (byte i = 0; i < MSG.len; i++) { // print the data
    activeSerial->printf("%x ",MSG.buf[i]);
  }
  if(pgn){
    activeSerial->printf("PGN: %i",pgn);
  }
  activeSerial->printf("\r\n");
}

int32_t Currentavg_Calc(){
  //[ToDo] überarbeiten! (glätten)
  static byte BMC_last_Stat = 0;
  static byte currentavg_counter = 0;
  static uint32_t looptime = 0;
  static int32_t retval = 0;
  static int32_t currentavg_array[60];

  // run once per 1s max
  if (millis() < looptime + 500) {return retval;}

  // reset on state machine change
  if (BMC_Stat != BMC_last_Stat) {
    for (byte i = 0; i < 60; i++){
      currentavg_array[i] = currentact;
    }
  }

  currentavg_array[currentavg_counter] = currentact;
  currentavg_counter++;
  if(currentavg_counter > 59){currentavg_counter = 0;}
  int32_t current_temp = 0;
  uint16_t max_size = sizeof(currentavg_array)/sizeof(currentavg_array[0]);
  for (byte i=0; i< max_size; i++){
    current_temp += currentavg_array[i];
  }
  retval = current_temp / max_size;
  looptime = millis();
  BMC_last_Stat = BMC_Stat;

  return retval;
}

void Current_debug(){
  if ( settings.CurSenType == Sen_AnalogueDual || settings.CurSenType == Sen_AnalogueSing){
    if (Sen_Analogue_active_Nr == 1){
      activeSerial->printf("\r\n");
      if (settings.CurSenType == Sen_AnalogueDual)
      {activeSerial->printf("Low Range: ");}
      else
      {activeSerial->printf("Single In: ");}
      activeSerial->printf("Value ADC0: %i  %i  %i  %imA  ",Sen_AnalogueRawValue * 3300 / adc->adc0->getMaxValue(),settings.analogueSen1_offset,Sen_AnalogueRawValue * 3300 / adc->adc0->getMaxValue() - settings.analogueSen1_offset,RawCur);
    }else if (Sen_Analogue_active_Nr == 2){
      activeSerial->printf("\r\n");
      activeSerial->printf("High Range: Value ADC0: %i  %i  %i  %imA  ",Sen_AnalogueRawValue * 3300 / adc->adc0->getMaxValue(),settings.analogueSen2_offset,Sen_AnalogueRawValue * 3300 / adc->adc0->getMaxValue() - settings.analogueSen2_offset,RawCur);
    }
    activeSerial->printf("%.2f | %i | %imA",lowpassFilter.output(),settings.analogueSen_ChangeCur,currentact); // [ToDo] eliminate float?
  }
  if ( settings.CurSenType == Sen_Canbus ) {
    activeSerial->printf("\r\n%imA",currentact);
  }
}


int32_t ETA(){ // return minutes
  //[ToDo] weiter glätten! mögl. vordef./errechneten Durchschnittswert einbeziehen?
  int32_t currentavg = Currentavg_Calc();
  if(BMC_Stat == Stat_Charge){
    return labs(mampsecond / 60 / currentavg);
  }else{
    if (currentavg >= 0){return 0;}
    else { return abs((TCAP * 1000 * 60 - mampsecond / 60) / currentavg); }
  }
}

uint16_t SOH_calc(){
  return round(float(settings.CAP) / float(settings.designCAP) * 100);
}

void SOC_update(){
  static byte SOCset = 0;
  int16_t SOC_tmp = 0;
  bool SOC_restored = false;

  if (millis() > 10000 && bms.getAvgCellVolt() > 0){
    
    if(!SOCset || settings.voltsoc){      
      if(settings.SOC == 0) { // check if there was an SOC value in EEPROM
        activeSerial->println("SOC was 0");
        SOC_tmp = map(bms.getAvgCellVolt(), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);
      } else {
        activeSerial->println("SOC was not 0");
        SOC_tmp = settings.SOC; // get SOC from EEPROM
        SOC_restored = true;
      }
      mampsecond = (100 - SOC_tmp) * TCAP * (3600000/100);
    } else {
        SOC_tmp = round((TCAP - double(mampsecond) / 3600000) * 100 / TCAP);
    }
    
    SOC = constrain(SOC_tmp, 0, 100);// keep SOC bettween 0 and 100
    
    if(!SOCset){
      SOCset = 1;
      if (!menu_load){
        activeSerial->printf("\r\n--------SOC %s: %i%%, %imAs--------",SOC_restored?"restored":"set",SOC,mampsecond);
      }
    }
  }
}

void SOC_charged(){
    SOC = 100;
    mampsecond = 0;
}

/*
  Returns temperature corrected capacity in Ah.
  Uses:
    bms.getLowTemperature()
    settings.Temp_CAP_Map[x][x]
*/
uint32_t CAP_Temp_alteration(){
  byte factor_map = 0;

  //temp below lowest Setpoint
  if (bms.getLowTemperature() < settings.Temp_CAP_Map[0][0]){ 
    return settings.CAP * (settings.Temp_CAP_Map[1][0] / 100);
  }

  //temp above highest setpoint.
  if(bms.getLowTemperature() >= settings.Temp_CAP_Map[0][4]) {
    return settings.CAP * (settings.Temp_CAP_Map[1][4] / 100);
  }
  
  //everything in between
  for (byte i = 1; i < 4; i++){
    if (bms.getLowTemperature() >= settings.Temp_CAP_Map[0][i] && bms.getLowTemperature() < settings.Temp_CAP_Map[0][i+1]){
      factor_map = map(bms.getLowTemperature(), settings.Temp_CAP_Map[0][i], settings.Temp_CAP_Map[0][i+1],settings.Temp_CAP_Map[1][i],settings.Temp_CAP_Map[1][i+1]);
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
  for(byte Output = 1; Output < 9; Output++){
    if(settings.Out_Map[0][Output] == Function){
      return Output;
    }
  } 
  return 0;
}
/*
Change the function of the given output
OUT1 = 1 ...
*/
void cycle_OUT_Mapping(byte OUT){
  byte Function = get_OUT_Mapping(OUT,0) + 1;
  if(OUT < 5 && Function == Out_Gauge){Function++;} //Gauge not possible on 12V outputs
  while(find_OUT_Mapping(Function)){Function++;} //doubles not possible
  if(Function > 11){Function = 0;} //max number of different Output Types
  set_OUT_Mapping(OUT,Function,get_OUT_Mapping(OUT,1));
}

/*
no parameter = all functions off
Use parameter (function) to activate corresponding function.
*/
void set_OUT_States(byte Function){
  if (Function == 254){for (byte i = 0; i < Out_States_Max; i++){Out_States[0][i] = 0;}} // all off
  else {Out_States[0][Function] = 1;}
}

// Set output pins according to Out_States.
void set_OUTs(){
  static uint32_t lastrun = 0;
  uint32_t tmp_timer = millis() - lastrun;
  static uint32_t Warn_Blink_Timer = 0;
  static uint32_t Error_Blink_Timer = 0;
  static bool Warn_Blink = 1;
  static bool Error_Blink = 1;

  // Handle "blinking" for warnings and errors
  if(settings.Warning_Blink_Hz && (millis() - Warn_Blink_Timer >= 1000/settings.Warning_Blink_Hz)){
    Warn_Blink = !Warn_Blink;
    Warn_Blink_Timer = millis();
  }
  if(settings.Error_Blink_Hz && (millis() - Error_Blink_Timer >= 1000/settings.Error_Blink_Hz)){
    Error_Blink = !Error_Blink;
    Error_Blink_Timer = millis();
  }

  for (byte Out_Cnt = 1; Out_Cnt < 9;Out_Cnt++){
    byte State = Out_States[0][settings.Out_Map[0][Out_Cnt]];
    byte Timer = Out_States[1][settings.Out_Map[0][Out_Cnt]];
    // [ToTest]
    // find warning output and override State with "blinking"
    if(Out_Cnt == find_OUT_Mapping(Out_Warning)){
      State = State * Warn_Blink;
    }

    // find error output and override State with "blinking"
    if(Out_Cnt == find_OUT_Mapping(Out_Error)){
      State = State * Error_Blink;
    }    

    // find warning/error combined output and override State with "blinking"
    if(Out_Cnt == find_OUT_Mapping(Out_Err_Warn)){
      if(Out_States[0][Out_Warning]){
        State = State * Warn_Blink;
      }
      // Error overrules warning
      if(Out_States[0][Out_Error]){
        State = State * Error_Blink;
      }
    }

    if(Out_Cnt < 5){digitalWrite(*Outputs[Out_Cnt], State);} // digitalWrite(Outn, HIGH/LOW)
    if(Out_Cnt >= 5){
      if(Out_Cnt != find_OUT_Mapping(Out_Gauge)){
        if( // find contactors & check if ON and TIMER left for full pull
          (Out_Cnt == find_OUT_Mapping(Out_Cont_Neg) || Out_Cnt == find_OUT_Mapping(Out_Cont_Pos) || Out_Cnt == find_OUT_Mapping(Out_Cont_Precharge)) && 
          (State && Timer)
          ){
          analogWrite(*Outputs[Out_Cnt], 255); // analogWrite(OUTn, HIGH)
          if(Timer > tmp_timer){Timer -= tmp_timer;} //reduce Timer
          else{Timer = 0;} // timer to 0
          Out_States[1][settings.Out_Map[0][Out_Cnt]] = Timer; // write back Timer
        } else if ( // find contactors & check if OFF to reset TIMER
          (Out_Cnt == find_OUT_Mapping(Out_Cont_Neg) || Out_Cnt == find_OUT_Mapping(Out_Cont_Pos) || Out_Cnt == find_OUT_Mapping(Out_Cont_Precharge)) && 
          (!State && Timer != cont_pulltime)
          ) {
            Out_States[1][settings.Out_Map[0][Out_Cnt]] = cont_pulltime; // reset Timer
            analogWrite(*Outputs[Out_Cnt], 0); // analogWrite(OUTn, LOW) [toTest] remove?
        } else {
          analogWrite(*Outputs[Out_Cnt], State * settings.Out_Map[1][Out_Cnt]); // analogWrite(OUTn, HIGH/LOW * PWM)
        }
      }
    } 

  }

  lastrun = millis();
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
    Gauge_PWM = map(SOC, 0, 100, settings.GaugeLow, settings.GaugeHigh) * Out_States[0][Out_Gauge];
  }
  analogWrite(*Outputs[find_OUT_Mapping(Out_Gauge)], Gauge_PWM);
  return Gauge_PWM;
}

void Prechargecon(){
  static uint32_t Precharge_Timer;
  if (!Precharge_Timer){Precharge_Timer = millis();}
  Out_States[0][Out_Cont_Neg] = 1;    
  if (Precharge_Timer + settings.PreTime > millis()){ // [ToDo] Add later (|| currentact < settings.PreCurrent)
    Out_States[0][Out_Cont_Precharge] = 1; 
    precharged = 0;
  }else{ 
    Out_States[0][Out_Cont_Pos] = 1;
    Out_States[0][Out_Cont_Precharge] = 0;
    Precharge_Timer = 0;
    precharged = 1;
  }
}

void CurrentOffsetCalc(){

  adc->adc0->startContinuous(IN_ACUR1);
  Sen_Analogue_active_Nr = 1;
  activeSerial->printf(" Calibrating Current Offset ");
  for (byte i = 0; i < 20; i++){
    settings.analogueSen1_offset = settings.analogueSen1_offset + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
    activeSerial->printf(".");
    delay(100);
  }
  settings.analogueSen1_offset = settings.analogueSen1_offset / 20;
  activeSerial->printf("\r\n Current offset 1 calibrated: %i \r\n",settings.analogueSen1_offset);

  if(settings.CurSenType == Sen_AnalogueDual){
    adc->adc0->startContinuous(IN_ACUR2);
    Sen_Analogue_active_Nr = 2;
    activeSerial->printf(" Calibrating Current Offset ");
    for (byte i = 0; i < 20; i++){
      settings.analogueSen2_offset = settings.analogueSen2_offset + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
      activeSerial->printf(".");
      delay(100);
    }
    settings.analogueSen2_offset = settings.analogueSen2_offset / 20;
    activeSerial->printf("\r\n Current offset 2 calibrated: %i",settings.analogueSen2_offset);
  }
}

void Serial_clear(){
  // Form feed
  activeSerial->write(12);
  
  // ESC + Clear
  //activeSerial->write(27);
  //activeSerial->printf("[2J");
  
  // bunch of new lines
  //for (byte i = 0; i < 40; i++){activeSerial->printf("\r\n");}
}

int16_t pgnFromCANId(int16_t canId){ //Parameter Group Number
  if ((canId & 0x10000000) == 0x10000000){return (canId & 0x03FFFF00) >> 8;}
  else{return canId;}
}
/* 
  This function will allways select the lowest charge current depending on environmental conditions.
*/
void ChargeCurrentLimit(uint16_t _tmp_chargecurrent){
  uint16_t EndCurrent = settings.ChargeCurrentEnd / settings.nChargers;
  int16_t tmp_chargecurrent = 0; // can get negative by mapping
  uint16_t ChargeMaxCurrent = 0;
  static uint16_t chargecurrentlast = 0;   // in 0,1A
  //static uint32_t chargecurrent_Timer = 0;
  //static bool chargecurrent_Taper = false;
  //uint16_t Taper_Time = 30000; // 10s
  ///Start at no derating///

  //select smaller value as max charge current
  ChargeMaxCurrent = chargecurrent = settings.ChargeOverCurrAlarm < settings.ChargerChargeCurrentMax ? settings.ChargeOverCurrAlarm : settings.ChargerChargeCurrentMax;

  if (WarnAlarm_Check(WarnAlarm_Alarm,WarnAlarm_Vhigh)){ chargecurrent = 0; }

  //Modifying Charge current
  if (chargecurrent > 0){
    //Temperature based
    if (WarnAlarm_Check(WarnAlarm_Warning,WarnAlarm_ChargeTLow)){
      tmp_chargecurrent = map(bms.getLowTemperature(),settings.ChargeUnderTWarn, settings.ChargeUnderTAlarm, ChargeMaxCurrent, 0);
      tmp_chargecurrent = constrain(tmp_chargecurrent,0,ChargeMaxCurrent);
      chargecurrent = tmp_chargecurrent < chargecurrent ? tmp_chargecurrent : chargecurrent;
    }
    if (WarnAlarm_Check(WarnAlarm_Warning, WarnAlarm_ChargeTHigh)){
      tmp_chargecurrent = map(bms.getHighTemperature(), settings.ChargeOverTWarn, settings.ChargeOverTAlarm, ChargeMaxCurrent, 0);
      tmp_chargecurrent = constrain(tmp_chargecurrent,0,ChargeMaxCurrent);
      chargecurrent = tmp_chargecurrent < chargecurrent ? tmp_chargecurrent : chargecurrent;
    }    
    //Voltage based
    if (storagemode){
      uint16_t upperStoreVLimit = settings.StoreVsetpoint - settings.ChargeHys;
      if (bms.getHighCellVolt() > upperStoreVLimit){
        uint16_t tmp_Vdiff = 0;
        if((settings.StoreVsetpoint - bms.getHighCellVolt() < 0)){chargecurrent = 0;} // 0A if highest cell is above Store setpoint or charged
        else {tmp_Vdiff = settings.StoreVsetpoint - bms.getHighCellVolt();}
        tmp_chargecurrent = (ChargeMaxCurrent * (tmp_Vdiff*tmp_Vdiff)/((settings.ChargeHys)*(settings.ChargeHys))) + EndCurrent;
        // Old linear derating
        //tmp_chargecurrent = map(bms.getHighCellVolt(), upperStoreVLimit, settings.StoreVsetpoint, ChargeMaxCurrent, EndCurrent);
        tmp_chargecurrent = constrain(tmp_chargecurrent,0,ChargeMaxCurrent);
        chargecurrent = tmp_chargecurrent < chargecurrent ? tmp_chargecurrent : chargecurrent;
      }
    } else { 
      uint16_t upperVLimit = settings.ChargeVSetpoint - settings.ChargeHys; //[ToDo] make derate setpoint configurable?
      if (bms.getHighCellVolt() > upperVLimit){
        // Old linear derating
        //tmp_chargecurrent = map(bms.getHighCellVolt(), upperVLimit, settings.ChargeVSetpoint, ChargeMaxCurrent, EndCurrent);
        uint16_t tmp_Vdiff = 0;
        if(settings.ChargeVSetpoint - bms.getHighCellVolt() < 0){chargecurrent = 0;} // 0A if highest cell is above Charge setpoint or charged
        else {tmp_Vdiff = settings.ChargeVSetpoint - bms.getHighCellVolt();}
        tmp_chargecurrent = (ChargeMaxCurrent * (tmp_Vdiff*tmp_Vdiff)/((settings.ChargeHys)*(settings.ChargeHys))) + EndCurrent;
        tmp_chargecurrent = constrain(tmp_chargecurrent,0,ChargeMaxCurrent);
        chargecurrent = tmp_chargecurrent < chargecurrent ? tmp_chargecurrent : chargecurrent;
       }      
    }
    // 16A Limit
    //tmp_chargecurrent = 3600 / (round(double(bms.getPackVoltage()) / 1000)) * 95 / 10; //3,6kW max, 95% Eff. , factor 10 [ToDo] make conf. + CAN
    //chargecurrent = tmp_chargecurrent < chargecurrent ? tmp_chargecurrent : chargecurrent;
  }   

  // compensate for consumers if Chargecurrent is 0
  // [ToDO] smoothing with P/I/D Controller?
  // [ToTest] Creates weird behavior when chargecurrent is zero
  if(chargecurrent == 0 && currentact < 0){
    chargecurrent = chargecurrentlast + (currentact / 100 / settings.nChargers) * -1;
  }

  // overwrite with passed charge current if lower
  chargecurrent = _tmp_chargecurrent < chargecurrent ? _tmp_chargecurrent : chargecurrent;

  // limit to XA above actual current when near hysteresis
  // replace "30" with settings in struct
  // Limit charge current increase to X above actual current. For Slow Inverters
  if (((bms.getHighCellVolt() > (settings.ChargeVSetpoint - settings.ChargeHys * 1.5))) && (chargecurrent > (currentact / 100) + 30)){
    chargecurrent = currentact < 0 ? 30 : (currentact / 100) + 30;
  }

  /*
  // prepare for tapering after 0A
  if(chargecurrent == 0) {
    chargecurrent_Timer = millis();
    chargecurrent_Taper = true;
  }
  
  // slowly increase chargecurrent after 0A
  if(chargecurrent_Taper && millis() - chargecurrent_Timer < Taper_Time){
    tmp_chargecurrent = map(millis() - chargecurrent_Timer, 0, Taper_Time, 0, ChargeMaxCurrent);
    tmp_chargecurrent = constrain(tmp_chargecurrent,0,ChargeMaxCurrent);
    chargecurrent = tmp_chargecurrent < chargecurrent ? tmp_chargecurrent : chargecurrent;
  }

  // disable tapering after Taper_Time
  if(chargecurrent_Taper && millis() - chargecurrent_Timer >= Taper_Time){
    chargecurrent_Taper = false;
  }
  */

  // [ToDo] implement feedback loop
  // multiply with calculated current
  chargecurrentFactor = currentact / 100 / chargecurrent;
  chargecurrentlast = chargecurrent;
}

void DischargeCurrentLimit(){
  ///Start at no derating///
  discurrent = settings.OverCurrAlarm;
  int16_t tmp_discurrent = 0; // can get nagative bei mapping

  //Modifying discharge current//
  if (discurrent > 0){
    //Temperature based//
    if (WarnAlarm_Check(WarnAlarm_Warning,WarnAlarm_THigh)){
      tmp_discurrent = map(bms.getHighTemperature(), settings.OverTWarn, settings.OverTAlarm, settings.OverCurrAlarm, 0);
      tmp_discurrent = constrain(tmp_discurrent,0,settings.OverCurrAlarm);
      discurrent = tmp_discurrent < discurrent ? tmp_discurrent : discurrent;
    }
    if (WarnAlarm_Check(WarnAlarm_Warning,WarnAlarm_TLow)){
      tmp_discurrent = map(bms.getLowTemperature(), settings.UnderTWarn, settings.UnderTAlarm, settings.OverCurrAlarm, 0);
      tmp_discurrent = constrain(tmp_discurrent,0,settings.OverCurrAlarm);
      discurrent = tmp_discurrent < discurrent ? tmp_discurrent : discurrent;
    }    
    //Voltage based//
    //if (bms.getLowCellVolt() < (settings.UnderVWarn + settings.DisTaper)){
    if (WarnAlarm_Check(WarnAlarm_Warning, WarnAlarm_VLow)){
      tmp_discurrent = map(bms.getLowCellVolt(), settings.UnderVWarn, settings.UnderVAlarm, settings.OverCurrAlarm, 0);
      tmp_discurrent = constrain(tmp_discurrent,0,settings.OverCurrAlarm);
      discurrent = tmp_discurrent < discurrent ? tmp_discurrent : discurrent;
    }
  }
  discurrent = constrain(discurrent,0,settings.OverCurrAlarm);
}


void Input_Debug(){
  Menu();
  activeSerial->printf("\r\nInput: ");
  if (digitalRead(IN1_Key) == HIGH){activeSerial->printf("1 ON  ");}
  else {activeSerial->printf("1 OFF ");}
  if (digitalRead(IN2_Gen) == HIGH){ activeSerial->printf("2 ON  ");}
  else {activeSerial->printf("2 OFF ");}
  if (digitalRead(IN3_AC) == HIGH){activeSerial->printf("3 ON  ");}
  else {activeSerial->printf("3 OFF ");}
  if (digitalRead(IN4_HVIL) == HIGH){activeSerial->printf("4 ON  ");}
  else {activeSerial->printf("4 OFF ");}
  activeSerial->printf("\r\n");
}

void Output_debug(){
  // 5s each when looped every 500ms
  if (debug_output_counter < 10){
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
  debug_output_counter ++;
  if (debug_output_counter > 20){debug_output_counter = 0;}
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
  dashvolt = round(map(bms.getPackVoltage(), settings.Scells * settings.UnderVAlarm, settings.Scells * settings.OverVAlarm, 5, 95));
  dashvolt = constrain(dashvolt, 0, 100);

  String eta = "";
  int32_t minutes = ETA();
  if (BMC_Stat == Stat_Drive || BMC_Stat == Stat_Charge){
    eta = "\"" + String(minutes / 60) + "h " + String(minutes % 60) + "m" + "\"";
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
      case (Stat_Idle): Nextion_send("stat.txt=", "\" Idle \""); break;
      case (Stat_Discharge): Nextion_send("stat.txt=", "\" Discharge \""); break;
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
  Nextion_send("debug.val=", ConvertToBin(warning[0]));
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
  Serial_Display.printf("%s%s",var.c_str(),val.c_str());
  Serial_Display.write(0xff);
  Serial_Display.write(0xff);
  Serial_Display.write(0xff);  
}

void can1_start(){
  can1.begin();
  can1.setBaudRate(settings.CAN1_Speed);
  //NVIC_DISABLE_IRQ(IRQ_CAN1);
  NVIC_SET_PRIORITY(IRQ_CAN1, 192);
}

void can2_start(){
  can2.begin();
  can2.setBaudRate(settings.CAN2_Speed);
}

void CAN_read(){
  CAN_message_t MSG;

  while (can1.read(MSG)){
    if (settings.CurSenType == Sen_Canbus && settings.CAN_Map[0][CAN_Curr_Sen] & 1){CAN_SEN_read(MSG, currentact);}
    if (settings.mctype && settings.CAN_Map[0][CAN_MC] & 1){CAN_MC_read(MSG);}
    if (settings.BMSType != BMS_Tesla && settings.CAN_Map[0][CAN_BMS] & 1){bms.readModulesValues(MSG); BMSLastRead = millis();}
    if (settings.CAN_Map[0][CAN_BMC_HV] & 1){CAN_BMC_HV_send(1, MSG);} // Send HV CAN triggered by Inverter
    if (debug_CAN1){CAN_Debug_IN(1, MSG);}
    if (settings.CAN_Map[0][CAN_BMC_std] & 1){CAN_TCU_read(MSG);}
    if (settings.CAN_Map[0][CAN_Charger] & 1){CAN_Charge_Check(MSG);}
  }
  while (can2.read(MSG)){
    if (settings.CurSenType == Sen_Canbus && settings.CAN_Map[0][CAN_Curr_Sen] & 2){CAN_SEN_read(MSG, currentact);}
    if (settings.mctype && settings.CAN_Map[0][CAN_MC] & 2){CAN_MC_read(MSG);}
    if (settings.BMSType != BMS_Tesla && settings.CAN_Map[0][CAN_BMS] & 2){bms.readModulesValues(MSG); BMSLastRead = millis();}
    if (settings.CAN_Map[0][CAN_BMC_HV] & 2){CAN_BMC_HV_send(2, MSG);} // Send HV CAN triggered by Inverter
    if (debug_CAN2){CAN_Debug_IN(2, MSG);}
    if (settings.CAN_Map[0][CAN_BMC_std] & 2){CAN_TCU_read(MSG);}
    if (settings.CAN_Map[0][CAN_Charger] & 2){CAN_Charge_Check(MSG);}
  }
}

void CAN_send(byte CAN_Nr, CAN_message_t MSG){
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
  if(debug_CAN1 || debug_CAN2){CAN_Debug_OUT(CAN_Nr, MSG);}
}

void CAN_BMC_Std_send(byte CAN_Nr){ //BMC standard CAN Messages
  //activeSerial->print("HIT");
  static uint32_t CAN_BMC_Std_send_Timer = 0;
  if(millis() - CAN_BMC_Std_send_Timer <= 1000){return;} // Timer 1s;
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
  MSG.buf[6] = lowByte(uint16_t(round(float(settings.UnderVWarn)/100 * settings.Scells)));
  MSG.buf[7] = highByte(uint16_t(round(float(settings.UnderVWarn)/100 * settings.Scells)));
  CAN_send(CAN_Nr, MSG);
  /*
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
  */

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
  CAN_send(CAN_Nr, MSG);
  /*
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
  */

  delay(2);
  MSG.id  = 0x356;
  MSG.buf[0] = lowByte(uint16_t(round(double(bms.getPackVoltage()) / 10)));
  MSG.buf[1] = highByte(uint16_t(round(double(bms.getPackVoltage()) / 10)));
  MSG.buf[2] = lowByte(int16_t(round(double(currentact) / 100))); // [ToDo] values > ~6500A will overflow!
  MSG.buf[3] = highByte(int16_t(round(double(currentact) / 100)));
  MSG.buf[4] = lowByte(bms.getAvgTemperature());
  MSG.buf[5] = highByte(bms.getAvgTemperature());
  MSG.buf[6] = 0;
  MSG.buf[7] = 0;
  CAN_send(CAN_Nr, MSG);
  /*
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
  */

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
  CAN_send(CAN_Nr, MSG);
  /*  
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
  */
  /*
  0x35C request Flag
  Byte 0: Bit7 - Charge Enable, Bit6 - Discharge Enable, Bit5 - Request force Charge, Bit4 - Request force Charge 2, Bit3 - Request full Charge
  */

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
  CAN_send(CAN_Nr, MSG);
  /*
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
  */

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
  CAN_send(CAN_Nr, MSG);
  /*
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
  */

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
  CAN_send(CAN_Nr, MSG);
  /*
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
  */

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
  CAN_send(CAN_Nr, MSG);
  /*
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
  */

  delay(2);
  MSG.id  = 0x379; // non standard, installed capacity, Power in kW, [ToDo] add Status?
  MSG.buf[0] = lowByte(uint16_t(settings.Pstrings * settings.CAP));
  MSG.buf[1] = highByte(uint16_t(settings.Pstrings * settings.CAP));
  MSG.buf[2] = lowByte((int16_t((double(currentact)/1000)*(double(bms.getPackVoltage())/1000)/10)));
  MSG.buf[3] = highByte((int16_t((double(currentact)/1000)*(double(bms.getPackVoltage())/1000)/10)));
  MSG.buf[4] = 0x00;
  MSG.buf[5] = 0x00;
  MSG.buf[6] = 0x00;
  MSG.buf[7] = 0x00;
  CAN_send(CAN_Nr, MSG);
  /*
  if(CAN_Nr & 1){can1.write(MSG);}
  if(CAN_Nr & 2){can2.write(MSG);}
  */

  CAN_BMC_Std_send_Timer = millis();
}

void CAN_BMC_HV_send(byte CAN_Nr, CAN_message_t inMSG){ //BMC CAN HV Messages
  
  if(inMSG.id == 0x4200 && inMSG.buf[0] == 0){ // Broadcast from Inverter asking for System Data
    CAN_message_t MSG;
    MSG.len = 8;
    MSG.flags.extended = true;

    MSG.id  = 0x4210 + settings.batteryID;
    MSG.buf[0] = lowByte(uint16_t(round(double(bms.getPackVoltage()) / 100)));
    MSG.buf[1] = highByte(uint16_t(round(double(bms.getPackVoltage()) / 100)));
    MSG.buf[2] = lowByte(int32_t(round(double(currentact) / 100) + 30000)); // [ToDo] values > ~6500A will overflow!
    MSG.buf[3] = highByte(int32_t(round(double(currentact) / 100) + 30000));
    MSG.buf[4] = lowByte(int16_t(bms.getAvgTemperature() + 1000));
    MSG.buf[5] = highByte(int16_t(bms.getAvgTemperature() + 1000));
    MSG.buf[6] = SOC;
    MSG.buf[7] = SOH_calc();
    CAN_send(CAN_Nr, MSG);
    /*
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}
    */
    delay(2);
    MSG.id  = 0x4220 + settings.batteryID;
    if (storagemode){
      MSG.buf[0] = lowByte(uint16_t(round(float(settings.StoreVsetpoint) / 100 * settings.Scells)));
      MSG.buf[1] = highByte(uint16_t(round(float(settings.StoreVsetpoint) / 100 * settings.Scells)));    
    }else{
      MSG.buf[0] = lowByte(uint16_t(round(float(settings.ChargeVSetpoint) / 100 * settings.Scells)));
      MSG.buf[1] = highByte(uint16_t(round(float(settings.ChargeVSetpoint) / 100 * settings.Scells)));
    }
    MSG.buf[2] = lowByte(uint16_t(round(float(settings.UnderVWarn) / 100 * settings.Scells)));
    MSG.buf[3] = highByte(uint16_t(round(float(settings.UnderVWarn) / 100 * settings.Scells)));
    MSG.buf[4] = lowByte(chargecurrent + 30000);
    MSG.buf[5] = highByte(chargecurrent + 30000);
    MSG.buf[6] = lowByte(discurrent + 30000);
    MSG.buf[7] = highByte(discurrent + 30000);
    CAN_send(CAN_Nr, MSG);
    /*
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}
    */

    delay(2);
    MSG.id  = 0x4230 + settings.batteryID;
    MSG.buf[0] = lowByte(bms.getLowCellVolt());
    MSG.buf[1] = highByte(bms.getLowCellVolt());
    MSG.buf[2] = lowByte(bms.getHighCellVolt());
    MSG.buf[3] = highByte(bms.getHighCellVolt());
    MSG.buf[4] = 0; // MAX Single Battery Cell Voltage Number [ToDo]
    MSG.buf[5] = 0; // MAX Single Battery Cell Voltage Number [ToDo]
    MSG.buf[6] = 0; // MIN Single Battery Cell Voltage Number [ToDo]
    MSG.buf[7] = 0; // MIN Single Battery Cell Voltage Number [ToDo]
    CAN_send(CAN_Nr, MSG);
    /*
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}
    */

    delay(2);
    MSG.id  = 0x4240 + settings.batteryID;
    MSG.buf[0] = lowByte(uint16_t(bms.getHighTemperature() + 1000));
    MSG.buf[1] = highByte(uint16_t(bms.getHighTemperature() + 1000));
    MSG.buf[2] = lowByte(uint16_t(bms.getLowTemperature() + 1000));
    MSG.buf[3] = highByte(uint16_t(bms.getLowTemperature() + 1000));
    MSG.buf[4] = 0; // MAX Single Battery Cell Temperature Number [ToDo]
    MSG.buf[5] = 0; // MAX Single Battery Cell Temperature Number [ToDo]
    MSG.buf[6] = 0; // MIN Single Battery Cell Temperature Number [ToDo]
    MSG.buf[7] = 0; // MIN Single Battery Cell VolTemperaturetage Number [ToDo]
    CAN_send(CAN_Nr, MSG);
    /*
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}
    */

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
    //tmp = 0b00000000; // Bit8: Charge Overcurrent [ToDo]
    //tmpAlarm |= tmp;
    //tmp = 0b01000000; // Bit9: Discharge Overcurrent [ToDo]
    //tmpAlarm |= tmp;

    MSG.id  = 0x4250 + settings.batteryID;
    MSG.buf[0] = 0b00000011; // bit0-2: 0 = sleep, 1 = Charge, 2 = Discharge, 3 = Idle; Bit3: Force Charge; Bit4: Force Balance Charge; Bit5-7: Reserved; [ToDo]
    MSG.buf[1] = 0; // Cycle Period? [ToDo]
    MSG.buf[2] = 0; // Cycle Period? [ToDo]
    MSG.buf[3] = 0; // Error / Faults [ToDo] not implemented yet
    MSG.buf[4] = lowByte(tmpAlarm); // Alarm
    MSG.buf[5] = highByte(tmpAlarm); // Alarm
    MSG.buf[6] = 0; // Protection same as Alarm? [ToDo]
    MSG.buf[7] = 0; // Protection same as Alarm? [ToDo]
    CAN_send(CAN_Nr, MSG);
    /*
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}
    */

    MSG.id  = 0x4260 + settings.batteryID; // Module Voltage MIN / MAX & Nr.
    MSG.id  = 0x4270 + settings.batteryID; // Module Temp. MIN / MAX & Nr.
    MSG.id  = 0x4280 + settings.batteryID; // Charge / Dischage foorbidden ?
  }

  if(inMSG.id == 0x4200 && inMSG.buf[0] == 2){ // Broadcast from Inverter asking for System Information
    CAN_message_t MSG;
    MSG.len = 8;
    MSG.flags.extended = true;

    MSG.id = 0x7310 + settings.batteryID;
    MSG.buf[0] = 0; // Version 1=A, 2=B
    MSG.buf[1] = 0; // reserved
    MSG.buf[2] = 3; // Hardware Version e.g. 3.2
    MSG.buf[3] = 2; // Hardware Version e.g. 3.2
    MSG.buf[4] = 3; // Software Version e.g. 3.4 (03/2024)
    MSG.buf[5] = 4; // Software Version e.g. 3.4 (03/2024)
    MSG.buf[6] = 0; // Software Version ??
    MSG.buf[7] = 0; // Software Version ??
    CAN_send(CAN_Nr, MSG);
    /*
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}
    */
    delay(2);
    MSG.id = 0x7320 + settings.batteryID;
    MSG.buf[0] = lowByte(bms.getNumModules());
    MSG.buf[0] = highByte(bms.getNumModules());
    MSG.buf[2] = bms.getNumModules();
    MSG.buf[3] = 0; // [ToDo] Cells per module
    MSG.buf[4] = 0; // Volage Level?
    MSG.buf[5] = 0; // Volage Level?
    MSG.buf[6] = 0; // AH Number?
    MSG.buf[7] = 0; // AH NUmber?
    CAN_send(CAN_Nr, MSG);
    /*
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}
    */

    delay(2);
    MSG.id = 0x7330 + settings.batteryID;
    MSG.buf[0] = bmcname[0];
    MSG.buf[1] = bmcname[1];
    MSG.buf[2] = bmcname[2];
    MSG.buf[3] = bmcname[3];
    MSG.buf[4] = bmcname[4];
    MSG.buf[5] = bmcname[5];
    MSG.buf[6] = bmcname[6];
    MSG.buf[7] = bmcname[7];
    CAN_send(CAN_Nr, MSG);
    /*
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}
    */

    delay(2);
    MSG.id = 0x7340 + settings.batteryID;
    MSG.buf[0] = 0;
    MSG.buf[1] = 0;
    MSG.buf[2] = 0;
    MSG.buf[3] = 0;
    MSG.buf[4] = 0;
    MSG.buf[5] = 0;
    MSG.buf[6] = 0;
    MSG.buf[7] = 0;
    CAN_send(CAN_Nr, MSG);
    /*
    if(CAN_Nr & 1){can1.write(MSG);}
    if(CAN_Nr & 2){can2.write(MSG);}
    */
  }
}

void CAN_Charger_Send(byte CAN_Nr){
  CAN_message_t MSG;
  static uint32_t looptime1 = 0;
  // CAN intervall [ToDo] Intervall for CAN2?
  if (millis() - looptime1 < settings.CAN1_Interval){return;}
  looptime1 = millis();

  switch (settings.ChargerType){
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

      CAN_send(CAN_Nr, MSG);
      /*
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
      */
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

      CAN_send(CAN_Nr, MSG);
      /*
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
      */
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
      MSG.buf[3] = highByte(uint16_t(round(((float(settings.ChargeVSetpoint) / 1000 * settings.Scells) - float(chargerendbulk) / 1000) * 10)));
      MSG.buf[4] = lowByte(uint16_t(round(((float(settings.ChargeVSetpoint) / 1000 * settings.Scells) - float(chargerendbulk) / 1000) * 10)));
      MSG.buf[5] = highByte(chargecurrent);
      MSG.buf[6] = lowByte(chargecurrent);
      CAN_send(CAN_Nr, MSG);
      /*
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
      */

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
      CAN_send(CAN_Nr, MSG);
      /*
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
      */
    break;

    case Charger_ChevyVolt:
      MSG.id  = 0x30E;
      MSG.len = 1;
      MSG.buf[0] = 0x02; //only HV charging , 0x03 hv and 12V charging
      CAN_send(CAN_Nr, MSG);
      /*
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
      */

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
      CAN_send(CAN_Nr, MSG);
      /*
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
      */
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
      CAN_send(CAN_Nr, MSG);
      /*
      if(CAN_Nr & 1){can1.write(MSG);}
      if(CAN_Nr & 2){can2.write(MSG);}
      */
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
  if (MSG.id == 0x7FF){
    TCU_Pump = MSG.buf[0];
    TCU_Cool = MSG.buf[1];
    TCU_Heat = MSG.buf[2];
    TCU_REL4 = MSG.buf[3];
  }
}