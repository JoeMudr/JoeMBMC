#include <Arduino.h> 
#include "BMSManager.h"
#include <Filters.h>
#include <ADC.h>  
#include "config.h"

extern uint32_t firmver; 
extern uint32_t Uptime;
extern byte filterFrequency;
extern FilterOnePole lowpassFilter;
extern byte BMC_Stat;
extern bool menu_load;
extern uint32_t error_timer;
extern byte Settings_unsaved;
extern uint16_t pwmfreq;
extern uint16_t chargecurrent;
extern uint16_t chargecurrentFactor;
extern uint16_t discurrent;
extern byte alarm[4],warning[4];
extern unsigned char bmcname[8];
extern unsigned char bmcmanu[8];
extern uint32_t BMSLastRead;
extern uint16_t Sen_AnalogueRawValue;
extern int32_t currentact;
extern int32_t currentlast; // mA
extern int32_t RawCur;
extern int32_t mampsecond; // Range 0 = full to settings.cap * -1 = empty
extern volatile int32_t mampsecondTimer;
extern byte Sen_Analogue_active_Nr; // active analogue sensor: 1 = Sensor 1; 2 = Sensor 2
extern int16_t TCAP; //Temperature corrected Capacity in Ah including settings.Pstrings!
extern int16_t TCAP_Wh;
extern byte SOC; //State of Charge
extern byte maxac1; //Shore power 16A per charger; only used for prusa charger
extern byte maxac2; //Generator Charging; only used for prusa charger
extern const int16_t chargerid1; //bulk chargers (brusa)
extern const int16_t chargerid2; //finishing charger (brusa)
extern uint16_t chargerendbulk; //mV before Charge Voltage to turn off the bulk charger/s
extern uint16_t chargerend; //mV before Charge Voltage to turn off the finishing charger/s
extern bool v_CAN_Charger_Active; // global variable for CAN-Based chargers
extern u_int32_t v_CAN_Charger_Active_Timer; // Reset-timer for v_CAN_Charger_Active
extern byte storagemode;
extern byte precharged;
extern byte debug_output_counter;
extern byte cont_pulltime;
extern const byte *Outputs[9];
extern byte Out_States[2][Out_States_Max]; 
extern String Out_Functions[];
extern bool debug;
extern bool debug_Input;   // read digital inputs
extern bool debug_Output;  // check outputs
extern bool debug_CAN1;    // view can frames
extern bool debug_CAN2;    // view can frames
extern bool debug_Gauge;
extern byte  debug_Gauge_val;
extern uint32_t debug_Gauge_timer;
extern bool debug_Cur;
extern bool debug_CSV;
extern byte debug_Digits; // amount of digits behind decimal for voltage reading
extern byte TCU_Pump;
extern byte TCU_Cool;
extern byte TCU_Heat;
extern byte TCU_REL4;
extern IntervalTimer Timer_mAmpSec;
extern ADC *adc;
extern Stream* activeSerial;
extern BMSManager bms;
extern EEPROMSettings settings;
extern FlexCAN_T4<CAN1> can1;
extern FlexCAN_T4<CAN2> can2;
extern Stream* activeSerial;
