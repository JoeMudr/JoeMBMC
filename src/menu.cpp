#include <Arduino.h> 
#include "config.h"
#include "JoeMBMC.h"
#include <EEPROM.h>
#include "Globals.h"

extern void loadDefaultSettings();
extern void CurrentOffsetCalc();

void Menu(String menu_option_string){
  static byte menu_current = 0;
  static byte menu_option = 0;
  //String menu_option_string;
  char menu_option_char;
  int32_t menu_option_val = 0;
  float menu_option_val_Float = 0.0;
  String menu_option_val_String = "0";  
  
  //  menu_option = 0;
  if (menu_option_string != ""){
  //if (Serial_USB.available()){
    //menu_option_string = Serial_USB.readString(); 
    //menu_option_string = activeSerial->readString(); 
    menu_option_string.trim();
    menu_option_char = menu_option_string.charAt(0);
    menu_option = menu_option_string.toInt();
    if (menu_option == 0){ menu_option = menu_option_char; }
    //menu_option_val = split(menu_option_string,'=',1);
    for (uint16_t i = 0; i < menu_option_string.length(); i++) {
      if (menu_option_string.substring(i, i+1) == "=") { 
        menu_option_val = menu_option_string.substring(i+1).toInt();
        menu_option_val_Float = menu_option_string.substring(i+1).toFloat();
        menu_option_val_String = menu_option_string.substring(i+1);        
        break;
      }
    }
  } else { menu_option = 0; }
  
  /*
  if (menu_option != 109 && menu_load == 0){return;}
  else {menu_load = 1;}
*/

  switch (menu_current) {
    case Menu_Start:
      menu_current = Menu_Main;
      Serial_clear();
      activeSerial->printf("Main\r\n");
      activeSerial->printf("--------------------\r\n");
      activeSerial->printf("[1]  Battery\r\n");
      activeSerial->printf("[2]  Current Sensor\r\n");
      activeSerial->printf("[3]  Charger\r\n");
      activeSerial->printf("[4]  Motor Controller\r\n");
      activeSerial->printf("[5]  Outputs\r\n");
      activeSerial->printf("[6]  Alarms\r\n");
      activeSerial->printf("[7]  Ignore Values\r\n");
      activeSerial->printf("[8]  CAN-Bus\r\n");
      activeSerial->printf("[9]  Misc.\r\n");
      activeSerial->printf("[10] Debug\r\n");
      activeSerial->printf("[11] Reboot\r\n");
      activeSerial->printf("[12] Load Defaults\r\n");
      activeSerial->printf("[13] Mode: %s\r\n",settings.ESSmode?"ESS":"Vehicle");
      activeSerial->printf("[14] BMS-Type: ");
      switch (settings.BMSType){
        case BMS_Dummy: activeSerial->printf("NONE\r\n"); break;
        case BMS_Tesla: activeSerial->printf("Tesla Model S/X\r\n"); break;
        case BMS_VW_MEB:activeSerial->printf("VW MEB\r\n");          break;
        case BMS_VW_eGolf: activeSerial->printf("VW eGolf/GTE\r\n"); break;
        case BMS_BMW_I3: activeSerial->printf("BMW i3\r\n"); break;
        case BMS_BMW_MiniE: activeSerial->printf("BMW MiniE\r\n"); break;
        default: break;
      }
      activeSerial->printf("\r\n");
      activeSerial->printf("[q]  Save & Quit\r\n");
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
        case 9: menu_current = Menu_Misc; Menu(); break;   
        case 10: menu_current = Menu_Debug; Menu(); break; 
        case 11: SCB_AIRCR = 0x05FA0004; break;
        case 12: loadDefaultSettings(); Menu(); activeSerial->printf("::::::Defaults loaded::::::\r\n"); break;    
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
          activeSerial->printf("::::::Settings saved::::::\r\n");
        break;
      }
    break;
    case Menu_Battery:
      switch (menu_option){
        case 1: settings.OverVWarn = menu_option_val; Menu(); break;
        case 2: settings.OverVAlarm = menu_option_val; Menu(); break;
        case 3: settings.UnderVWarn = menu_option_val; Menu(); break;
        case 4: settings.UnderVAlarm = menu_option_val; Menu(); break;
        case 5: settings.OverTWarn = menu_option_val * 10; Menu(); break;
        case 6: settings.OverTAlarm = menu_option_val * 10; Menu(); break;
        case 7: settings.UnderTWarn = menu_option_val * 10; Menu(); break;    
        case 8: settings.UnderTAlarm = menu_option_val * 10; Menu(); break;
        case 9: settings.ChargeOverTWarn = menu_option_val * 10; Menu(); break;
        case 10: settings.ChargeOverTAlarm = menu_option_val * 10; Menu(); break;
        case 11: settings.ChargeUnderTWarn = menu_option_val * 10; Menu(); break;    
        case 12: settings.ChargeUnderTAlarm = menu_option_val * 10; Menu(); break;            
        case 13: settings.OverCurrWarn = menu_option_val * 10; Menu(); break;
        case 14: settings.OverCurrAlarm = menu_option_val * 10; Menu(); break;
        case 15: settings.ChargeOverCurrWarn = menu_option_val * 10; Menu(); break;
        case 16: settings.ChargeOverCurrAlarm = menu_option_val * 10; Menu(); break;
        case 17: settings.CellGap = menu_option_val; Menu(); break;  
        case 18: settings.ChargeVSetpoint = menu_option_val; Menu(); break;
        case 19: settings.balanceVoltage = menu_option_val; Menu(); break;   
        case 20: settings.balanceHyst = menu_option_val; Menu(); break; 
        case 21: settings.designCAP = menu_option_val; Menu(); break;
        case 22: settings.CAP = menu_option_val; Menu(); break;
        case 23: settings.Pstrings = menu_option_val; Menu(); break;     
        case 24: settings.Scells = menu_option_val; Menu(); break;  
        case 25: settings.socvolt[0] = menu_option_val; Menu(); break;  
        case 26: settings.socvolt[1] = menu_option_val; Menu(); break; 
        case 27: settings.socvolt[2] = menu_option_val; Menu(); break;                                                               
        case 28: settings.socvolt[3] = menu_option_val; Menu(); break;  
        case 29: settings.StoreVsetpoint = menu_option_val; Menu(); break;  
        case 30: settings.Temp_CAP_Map[0][0] = menu_option_val * 10; Menu(); break;
        case 31: settings.Temp_CAP_Map[1][0] = menu_option_val; Menu(); break;
        case 32: settings.Temp_CAP_Map[0][1] = menu_option_val * 10; Menu(); break;
        case 33: settings.Temp_CAP_Map[1][1] = menu_option_val; Menu(); break;
        case 34: settings.Temp_CAP_Map[0][2] = menu_option_val * 10; Menu(); break;
        case 35: settings.Temp_CAP_Map[1][2] = menu_option_val; Menu(); break;
        case 36: settings.Temp_CAP_Map[0][3] = menu_option_val * 10; Menu(); break;
        case 37: settings.Temp_CAP_Map[1][3] = menu_option_val; Menu(); break;
        case 38: settings.Temp_CAP_Map[0][4] = menu_option_val * 10; Menu(); break;
        case 39: settings.Temp_CAP_Map[1][4] = menu_option_val; Menu(); break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          activeSerial->printf("Battery\r\n");
          activeSerial->printf("--------------------\r\n\r\n");
          activeSerial->printf("Cell limits:                        Warning         Alarm\r\n");    
          activeSerial->printf("Cell Overvoltage (mV):           [1]   %4i    [2]   %4i\r\n",settings.OverVWarn, settings.OverVAlarm);
          activeSerial->printf("Cell Undervoltage (mV):          [3]   %4i    [4]   %4i\r\n",settings.UnderVWarn, settings.UnderVAlarm);  //                     
          activeSerial->printf("Over Temperature (°C):           [5]   %4i    [6]   %4i\r\n",settings.OverTWarn / 10, settings.OverTAlarm / 10);
          activeSerial->printf("Under Temperature (°C):          [7]   %4i    [8]   %4i\r\n",settings.UnderTWarn / 10, settings.UnderTAlarm / 10);
          activeSerial->printf("Charge Over Temperature (°C):    [9]   %4i   [10]   %4i\r\n",settings.ChargeOverTWarn / 10, settings.ChargeOverTAlarm / 10);
          activeSerial->printf("Charge Under Temperature (°C):  [11]   %4i   [12]   %4i\r\n",settings.ChargeUnderTWarn / 10, settings.ChargeUnderTAlarm / 10);
          activeSerial->printf("Discharge Current (A):          [13]   %4i   [14]   %4i\r\n",settings.OverCurrWarn / 10, settings.OverCurrAlarm / 10);
          activeSerial->printf("Charge Current (A):             [15]   %4i   [16]   %4i\r\n",settings.ChargeOverCurrWarn / 10, settings.ChargeOverCurrAlarm / 10);
          activeSerial->printf("Cell Delta Voltage (mV):                      [17]   %4i\r\n",settings.CellGap);
          activeSerial->printf("\r\n");
          activeSerial->printf("[18] Charge Voltage Setpoint (mV):          %4i\r\n",settings.ChargeVSetpoint); //
          activeSerial->printf("[19] Cell Balance Voltage (mV):             %4i\r\n",settings.balanceVoltage);
          activeSerial->printf("[20] Balance Hysteresis (mV):               %4i\r\n",settings.balanceHyst);
          
          activeSerial->printf("\r\n");
          activeSerial->printf("Pack configuration:\r\n");         
          activeSerial->printf("[21] Design Capacity (Ah):          %4i\r\n",settings.designCAP);
          activeSerial->printf("[22] Current Capacity (Ah):         %4i\r\n",settings.CAP); 
          activeSerial->printf("[23] Cells in Parallel:             %4i\r\n",settings.Pstrings);
          activeSerial->printf("[24] Cells in Series:               %4i\r\n",settings.Scells );
          activeSerial->printf("\r\n");
          activeSerial->printf("Voltage based SOC:\r\n");
          activeSerial->printf("[25] Setpoint1 (mV):        %4i\r\n",settings.socvolt[0]);
          activeSerial->printf("[26] SOC @ Setpoint1 (%%):   %4i\r\n",settings.socvolt[1]);
          activeSerial->printf("[27] Setpoint2 (mV):        %4i\r\n",settings.socvolt[2]);
          activeSerial->printf("[28] SOC @ Setpoint2 (%%):   %4i\r\n",settings.socvolt[3]);
          activeSerial->printf("[29] Storage Setpoint (mV): %4i\r\n",settings.StoreVsetpoint);
          activeSerial->printf("\r\n");
          activeSerial->printf("Temperature based SOC Adjustment:\r\n");
          for (byte i = 0; i < 5; i++){
            byte y = i*2;
            activeSerial->printf("[%i] %3i°C [%i] %3i%%\r\n",30+y,settings.Temp_CAP_Map[0][i]/10,31+y,settings.Temp_CAP_Map[1][i]);
          }
          activeSerial->printf("\r\n");                    
          activeSerial->printf("[q] Quit\r\n");          
      }
    break; 
    case Menu_CurSen:
      switch (menu_option){
        case 1: settings.CurSenInvert = !settings.CurSenInvert; Menu(); break;
        case 2: settings.voltsoc = !settings.voltsoc;  Menu(); break;
        case 3: settings.CurSenMultiplier = menu_option_val; Menu(); break;
        case 4: 
          settings.CurSenType ++; 
          if (settings.CurSenType > 3){settings.CurSenType = 0;}
          Menu();
        break;
        case 5: settings.analogueSen1_convlow = menu_option_val; Menu(); break;
        case 6: settings.analogueSen2_convhigh = menu_option_val; Menu(); break;
        case 7: settings.analogueSen_ChangeCur = menu_option_val * 1000; Menu(); break;
        case 8: settings.analogueSen_CurDead = menu_option_val; Menu(); break;                 
        case 9: CurrentOffsetCalc(); Menu();break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        case 100: debug_Cur = !debug_Cur; Menu(); break;
        default:
          Serial_clear();
          activeSerial->printf("Current Sensor\r\n");
          activeSerial->printf("--------------------\r\n");
          activeSerial->printf("[1] Invert Current:     %i\r\n",settings.CurSenInvert);
          activeSerial->printf("[2] Voltage based SOC:  %i\r\n",settings.voltsoc);
          activeSerial->printf("[3] Current Multiplier: %i\r\n",settings.CurSenMultiplier);
          activeSerial->printf("[4] Sensor Type: ");  
          switch (settings.CurSenType) {
            case Sen_Canbus:
              activeSerial->printf(" Canbus Current Sensor\r\n");
            break;
            case Sen_AnalogueSing:
              activeSerial->printf(" Analogue Single\r\n");
              activeSerial->printf(" Enter value below without decimal point. E.g. 1250 for 12.50mV/A\r\n");
              activeSerial->printf("[7] Analog Sensor 1 (low) mV/A:    %5.2f\r\n",float(settings.analogueSen1_convlow) / 100);
              activeSerial->printf("[8] Current Sensor Deadband in mV: %5i\r\n",settings.analogueSen_CurDead); 
              activeSerial->printf("[9] Calibrate\r\n");
            break;
            case Sen_AnalogueDual:
              activeSerial->printf(" Analogue Dual\r\n");
              activeSerial->printf(" Enter values below without decimal point. E.g. 1250 for 12.50mV/A\r\n");
              activeSerial->printf("[5] Analog Sensor 1 (low) mV/A:      %5.2f\r\n",float(settings.analogueSen1_convlow) / 100);
              activeSerial->printf("[6] Analog Sensor 2 (high) mV/A:     %5.2f\r\n",float(settings.analogueSen2_convhigh) / 100);  
              activeSerial->printf("[7] Current Sensor switch over in A: %5i\r\n",settings.analogueSen_ChangeCur / 1000);                          
              activeSerial->printf("[8] Current Sensor Deadband in mV:   %5i\r\n",settings.analogueSen_CurDead);  
              activeSerial->printf("[9] Calibrate\r\n");
            break;             
            default: activeSerial->printf("Undefined\r\n");
          }
          activeSerial->printf("[d] Debug\r\n");    
          activeSerial->printf("\r\n");
          activeSerial->printf("[q] Quit\r\n");         
      }
    break;  
    case Menu_Charger:
      switch (menu_option){
        case 1: settings.ChargeHys = menu_option_val; Menu(); break;
        case 2: settings.ChargerChargeCurrentMax = menu_option_val * 10; Menu(); break;
        case 3: settings.ChargeCurrentEnd = menu_option_val * 10; Menu(); break;
        case 4:
          settings.ChargerType++;
          if (settings.ChargerType > 6){ settings.ChargerType = 0; }
          Menu();
        break;
        case 5:
          if (settings.ChargerDirect){settings.ChargerDirect = 0;}
          else {settings.ChargerDirect = 1;}
          Menu();
        break;
        case 6:
          settings.nChargers = menu_option_val; Menu();
        break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          activeSerial->printf("Charger\r\n");
          activeSerial->printf("--------------------\r\n");
          activeSerial->printf("[1] Charge Hysteresis (mV):                %4i\r\n",settings.ChargeHys);
          if (settings.ChargerType > 0){
            activeSerial->printf("[2] Max Charge Current per Charger (A):    %4i\r\n",settings.ChargerChargeCurrentMax / 10);
            activeSerial->printf("[3] Pack End of Charge Current in (A):     %4i\r\n",settings.ChargeCurrentEnd / 10);
          }
          activeSerial->printf("[4] Charger Type:                 ");
          switch (settings.ChargerType){
            case 0: activeSerial->printf("%13s\r\n","undefined"); break;
            case 1: activeSerial->printf("%13s\r\n","Brusa NLG5xx");  break;
            case 2: activeSerial->printf("%13s\r\n","Volt Charger");  break;
            case 3: activeSerial->printf("%13s\r\n","Eltek Charger"); break;
            case 4: activeSerial->printf("%13s\r\n","Elcon Charger"); break;
            case 5: activeSerial->printf("%13s\r\n","Victron/SMA");   break;
            case 6: activeSerial->printf("%13s\r\n","Coda"); break;
          }        
          activeSerial->printf("[5] Charger HV Connection: ");
          switch (settings.ChargerDirect){
            case 0: activeSerial->printf("%20s\r\n","Behind Contactors"); break;
            case 1: activeSerial->printf("%20s\r\n","Direct to Battery HV"); break;
          }       
          activeSerial->printf("[6] Number of Chargers in Parallel:        %4i\r\n",settings.nChargers);
          activeSerial->printf("\r\n");
          activeSerial->printf("[q] Quit\r\n"); 
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
        activeSerial->printf("Motor Controller\r\n");
        activeSerial->printf("--------------------\r\n");
        activeSerial->printf("[1] Controller Type: ");
        switch (settings.mctype){
          case 0: activeSerial->printf("not configured\r\n"); break;
          case 1: activeSerial->printf("Curtis\r\n"); break;
          default: activeSerial->printf("undefined\r\n");
        }
        activeSerial->printf("\r\n");
        activeSerial->printf("[q] Quit\r\n"); 
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
        case 20: settings.PreTime = menu_option_val; Menu(); break;
        case 21: settings.PreCurrent = menu_option_val; Menu(); break;
        case 22: settings.GaugeLow = menu_option_val; Menu(); break;
        case 23: settings.GaugeHigh = menu_option_val; Menu(); break;
        case 24:
          debug_Gauge_val = menu_option_val;
          debug_Gauge = 1;
          Menu();
          break;        
        case 100:
          debug_Output = !debug_Output;
          debug_output_counter = 0;
          Menu();
          break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          activeSerial->printf("Outputs\r\n");
          activeSerial->printf("--------------------\r\n");
          activeSerial->printf("12V Outputs:\r\n");
          for (byte i = 1; i < 5; i++){ // [ToDo] use array size
            activeSerial->printf("[%i] Out%i: %s\r\n",i,i,Out_Functions[settings.Out_Map[0][i]].c_str());                    
          }
          activeSerial->printf("\r\n");
          activeSerial->printf("PWM/GND Outputs:\r\n");
          for (byte i = 5; i < 9; i++){ // [ToDo] use array size
            byte pos = 2*i-5;
            activeSerial->printf("[%i] %sOut%i:             %16s\r\n",pos,pos<10?" ":"",i,Out_Functions[settings.Out_Map[0][i]].c_str());   
            pos = 2*i-4;
            activeSerial->printf("[%i] %sOut%i PWM [0-255]: %13s%3i\r\n",pos,pos<10?" ":"",i,"",settings.Out_Map[1][i]);   //%13s just for alignment, whitespaces
          }      
          activeSerial->printf("\r\n");
          activeSerial->printf("Additional Settings:\r\n");
          activeSerial->printf("[20] Precharge Timer in ms:          %4i\r\n",settings.PreTime);
          activeSerial->printf("[21] Precharge finish Current in mA: %4i\r\n",settings.PreCurrent);
          activeSerial->printf("[22] PWM Gauge low [0-255]:          %4i\r\n",settings.GaugeLow);
          activeSerial->printf("[23] PWM Gauge high [0-255]:         %4i\r\n",settings.GaugeHigh);
          activeSerial->printf("[24] Gauge Test [0-255] for 30s:     %4i\r\n",debug_Gauge_val);
          activeSerial->printf("[d]  Debug Output ");         
          if(debug_Output){ activeSerial->printf("ON\r\n"); } 
          else {activeSerial->printf("OFF\r\n");}
          activeSerial->printf("\r\n");
          activeSerial->printf("[q] Quit\r\n");
      }
    break; 
    case Menu_Alarms:
      switch (menu_option){
        case 1: settings.error_delay = menu_option_val; Menu(); break;
        case 2: settings.Warning_Blink_Hz = menu_option_val; Menu(); break;
        case 3: settings.Error_Blink_Hz = menu_option_val; Menu(); break;
        case 4: settings.UnderVAlarm_DischHys = menu_option_val; Menu(); break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          activeSerial->printf("Alarms\r\n");
          activeSerial->printf("--------------------\r\n");
          activeSerial->printf("[1] Error Delay (ms):             %5i\r\n",settings.error_delay);
          activeSerial->printf("[2] Warning blink (Hz):           %5i\r\n",settings.Warning_Blink_Hz);
          activeSerial->printf("[3] Error blink (Hz):             %5i\r\n",settings.Error_Blink_Hz);
          activeSerial->printf("[4] UnderV Discharge Hysteresis   %5i\r\n",settings.UnderVAlarm_DischHys);
          activeSerial->printf("\r\n");        
          activeSerial->printf("[q] Quit\r\n");          
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
          Menu(); 
        break;       
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          activeSerial->printf("Ignore Values\r\n");
          activeSerial->printf("--------------------\r\n");
          activeSerial->printf("[1] Use Temp. Sensor:       ");
          if (!settings.useTempSensor){activeSerial->printf("both\r\n");}
          else{activeSerial->printf("%4i\r\n",settings.useTempSensor);}
          activeSerial->printf("[2] Ignore Cells under mV: %4i\r\n",settings.IgnoreVolt);
          activeSerial->printf("\r\n"); 
          activeSerial->printf("[q] Quit\r\n"); 
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
          activeSerial->printf("CAN-Bus settings\r\n");
          activeSerial->printf("--------------------\r\n");
          activeSerial->printf("[1] Can1 Baudrate in kbps: %i\r\n",settings.CAN1_Speed / 1000);
          activeSerial->printf("[2] Debug CAN1: %s\r\n",debug_CAN1?"ON":"OFF");
          activeSerial->printf("[3] Can2 Baudrate in kbps: %i\r\n",settings.CAN2_Speed / 1000);
          activeSerial->printf("[4] Debug CAN2: %s\r\n",debug_CAN2?"ON":"OFF");
          activeSerial->printf("\r\n");
          activeSerial->printf("Functions:\r\n");
          for (byte i = 0; i < CAN_MAP_MAX; i++){
            //byte pos = (i+5)*2-5;
            byte pos = i+5;
            activeSerial->printf("[%i] %s",pos,pos<10?" ":"");
            switch (i){
              case 0: activeSerial->printf("BMC Std:          "); break;
              case 1: activeSerial->printf("BMC HV:           "); break;
              case 2: activeSerial->printf("BMS:              "); break;
              case 3: activeSerial->printf("Charger:          "); break;
              case 4: activeSerial->printf("Current Sensor:   "); break;
              case 5: activeSerial->printf("Motor Controller: "); break;
            }
            switch (settings.CAN_Map[0][i]){
              case 0: activeSerial->printf("%11s\r\n","undefined"); break;
              case 1: activeSerial->printf("%11s\r\n","CAN1"); break;
              case 2: activeSerial->printf("%11s\r\n","CAN2"); break;
              case 3: activeSerial->printf("%11s\r\n","CAN1 & CAN2"); break;
            }
            if (i < 3){
              pos = (i+5)*2-4;
              //activeSerial->printf("[%i] %sInterval:         %9ims\r\n",pos,pos<10?" ":"",settings.CAN_Map[1][i]);
            }
          }
          activeSerial->printf("\r\n");
          activeSerial->printf("[q] Quit\r\n");
      }
    break;     
    case Menu_Misc:
      switch (menu_option){
        case 1: settings.secondarySerial++; if(settings.secondarySerial > 2){settings.secondarySerial = 0;} Menu(); break;
        case 2: settings.batteryID = strtol(menu_option_val_String.c_str(),NULL,16); Menu(); break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          activeSerial->printf("Experimental\r\n");
          activeSerial->printf("--------------------\r\n");
          activeSerial->printf("[1] Second Serial Port: ");
          switch (settings.secondarySerial){
            case 0: activeSerial->printf("%s\r\n","Serial Display"); break;
            case 1: activeSerial->printf("%s\r\n","undefined"); break;
            case 2: activeSerial->printf("%s\r\n","Bluetooth App (not used)"); break;
          
          }
          activeSerial->printf("[2] Battery ID (HEX):   %#x\r\n",settings.batteryID);
          activeSerial->printf("\r\n");
          activeSerial->printf("[q] Quit\r\n");
      }
    break; 
    case Menu_Debug:
      switch (menu_option){
        case 1: debug_CAN1 = !debug_CAN1; Menu(); break;
        case 2: debug_CAN2 = !debug_CAN2; Menu(); break;
        case 3: debug_Cur = !debug_Cur; Menu(); break;
        case 4: debug_Output = !debug_Output; debug_output_counter = 0; Menu(); break;
        case 5: debug_Input = !debug_Input; Menu(); break;
        case 6: Menu(); break;
        case 7: debug_CSV = !debug_CSV; Menu(); break;
        case Menu_Quit: menu_current = Menu_Start; Menu(); break;
        default:
          Serial_clear();
          activeSerial->printf("Debug\r\n");
          activeSerial->printf("--------------------\r\n");
          activeSerial->printf("[1] CAN1\r\n");
          activeSerial->printf("[2] CAN2\r\n");
          activeSerial->printf("[3] Current\r\n");
          activeSerial->printf("[4] Output Check: %s\r\n",debug_Output?"ON":"OFF");
          activeSerial->printf("[5] Input Check\r\n");
          activeSerial->printf("[6] ---\r\n");
          activeSerial->printf("[7] CSV Output\r\n");
          activeSerial->printf("\r\n");  
          activeSerial->printf("[q] Quit\r\n");  
      }
    break;          
  }
}