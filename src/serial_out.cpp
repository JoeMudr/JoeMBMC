#include <Arduino.h> 
#include "config.h"
#include "JoeMBMC.h"
#include "Globals.h"

extern byte Warn_Out_handle();

void Serial_Print(bool Modules_Print){
  Serial_clear();
  activeSerial->printf("Firmware:   %i\r\n", firmver);
  activeSerial->printf("BMC Status: ");
  switch (BMC_Stat){
    case (Stat_Boot): activeSerial->printf("Boot"); break;
    case (Stat_Ready): activeSerial->printf("Ready"); break;
    case (Stat_Precharge): activeSerial->printf("Precharge"); break;
    case (Stat_Drive): activeSerial->printf("Drive"); break;
    case (Stat_Charge): activeSerial->printf("Charge"); break;
    case (Stat_Charged): activeSerial->printf("Charged"); break;
    case (Stat_Error): activeSerial->printf("Error"); break;
    case (Stat_Idle): activeSerial->printf("Idle"); break;
    case (Stat_Discharge): activeSerial->printf("Discharge"); break; 
  }
  if (Warn_Out_handle()){activeSerial->printf(" (Warning!)\r\n");} else {activeSerial->printf("\r\n");}
  activeSerial->printf("Uptime: %02d:%02d:%02d\r\n",Uptime/3600,(Uptime%3600)/60,Uptime%60);
  
  activeSerial->printf("\r\nCurrent Limit: Charge %5.2fA (%5.2fA), Factor: %3u% | Discharge %5.2fA \r\n",float(chargecurrent) * settings.nChargers / 10, float(chargecurrent) / 10, chargecurrentFactor, float(discurrent) / 10);
  activeSerial->printf("Vpack: %5.2fV | Vlow: %4umV | Vhigh: %4umV | DeltaV: %4umV | Tlow: %3.1f°C | Thigh: %3.1f°C\r\n",double(bms.getPackVoltage()) / 1000,bms.getLowCellVolt(),bms.getHighCellVolt(),bms.getHighCellVolt() - bms.getLowCellVolt(),float(bms.getLowTemperature()) / 10,float(bms.getHighTemperature()) / 10);
  activeSerial->printf("Cells: %u/%u",bms.getSeriesCells(),settings.Scells * settings.Pstrings);

  if (bms.getBalancing()){ activeSerial->printf(" | Balancing Active\r\n\n"); } else { activeSerial->printf("\r\n\n"); }
  if(Modules_Print){bms.printPackDetails(activeSerial);}
  else{activeSerial->printf("Modules / Cells hidden\r\n");}
  if(WarnAlarm_Check(WarnAlarm_Warning,WarnAlarm_External)){
    activeSerial->printf("!!! Internal Error / Series Cells Fault !!!\r\n\n");
  } else { activeSerial->printf("\r\n");}
  
  activeSerial->printf("Current ");
  if (settings.CurSenType == Sen_AnalogueDual){
    if (Sen_Analogue_active_Nr == 1)
      { activeSerial->printf("(A Low):  "); }
    else if (Sen_Analogue_active_Nr == 2)
      { activeSerial->printf("(A High): "); }
  }
  if (settings.CurSenType == Sen_AnalogueSing)
    {activeSerial->printf("(A Single): ");}
  if (settings.CurSenType == Sen_Canbus)
    {activeSerial->printf("(CAN): ");}
  activeSerial->printf("%6imA, %10.2fmW\r\n",currentact, currentact * double(bms.getPackVoltage()) / 1000);
  activeSerial->printf("SOC:       %3u%%  (%.2fmAh)\r\n",SOC,double(mampsecond) / 3600);
  activeSerial->printf("SOH:       %3u%%  (%u/%uAh)\r\n",SOH_calc(),settings.CAP,settings.designCAP);
  activeSerial->printf("TCap:      %3uAh (%uWh)\r\n",abs(TCAP),abs(TCAP_Wh));
  int32_t minutes = ETA();
  activeSerial->printf("ETA:       %2ih %2im\r\n",minutes / 60,minutes % 60);
  activeSerial->printf("Warning:   ");
  for (byte i = 0; i < 4; i++){
    activeSerial->printf("%08i ",ConvertToBin(warning[i]));
  }
  activeSerial->printf("\r\n");
  activeSerial->printf("Error:     ");
  for (byte i = 0; i < 4; i++){
    activeSerial->printf("%08i ",ConvertToBin(alarm[i]));
  }
  activeSerial->printf("\r\n");
  activeSerial->printf("\r\n");

  activeSerial->printf("IN %d%d%d%d %s %s\r\n",digitalRead(IN1_Key),digitalRead(IN2_Gen),digitalRead(IN3_AC),digitalRead(IN4_HVIL),Charge_Active()?"| Charger plugged":"",digitalRead(IN1_Key)?"| Key ON":"");
  if(TCU_Pump || TCU_Cool || TCU_Heat || TCU_REL4){
    activeSerial->printf("TCU: %s%s%s%s\r\n",TCU_Pump-1?"Pumping ":"",TCU_Cool-1?"Cooling ":"",TCU_Heat-1?"Heating ":"",TCU_REL4-1?"REL4":"");
  }
  activeSerial->printf("\r\n");
}

void OUT_print(bool Out_Print){
  if(!Out_Print){return;}

  activeSerial->printf("Function         State Timer\r\n");
  activeSerial->printf("----------------------------\r\n");
  for(byte i = 1; i < Out_States_Max; i++){
    activeSerial->printf("%-16s %5i %5i\r\n",Out_Functions[i].c_str(),Out_States[0][i],Out_States[1][i]);
  } 
  activeSerial->printf("\r\n");
  activeSerial->printf("Output        Function  PWM\r\n");
  activeSerial->printf("---------------------------\r\n");
  for(byte i = 1; i < 9; i++){ //[ToDo] use array size for loop?
    activeSerial->printf("OUT%i   %16s ",i,Out_Functions[settings.Out_Map[0][i]].c_str());
    if(i>4){
      if(settings.Out_Map[0][i] == Out_Gauge){activeSerial->printf("%3i",Gauge_update());} // get Gauge PWM directly from function.
      else {activeSerial->printf("%3i",settings.Out_Map[1][i]);}
    }
    activeSerial->printf("\r\n");
  }  
}