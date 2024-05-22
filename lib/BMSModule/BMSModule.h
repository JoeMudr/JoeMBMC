#pragma once
#include "config.h"
#include <FlexCAN_T4.h>

class BMSModule {
public:
    BMSModule();
    void initModule(BMS_t BMS_Type, uint16_t IgnoreV, byte tempsensor);
    bool readModule(); // Serial, return value used to confirm presence of module.
    bool readModule(CAN_message_t &msg, int Id); // CAN-Bus, return value used to confirm presence of module.
    void clearModule();
    byte balanceModule(byte balance_Bitmask);
    byte getscells();
    uint16_t getCellVoltage(int cell);
    uint16_t getLowCellV();
    uint16_t getHighCellV();
    uint16_t getAverageV();
    int16_t getLowTemp();
    int16_t getHighTemp();
    int16_t getAvgTemp();
    uint16_t getModuleVoltage();
    int16_t getTemperature(byte tempsensor);
    uint8_t getFaults();
    uint8_t getAlerts();
    uint8_t getCOVCells();
    uint8_t getCUVCells();
    uint32_t getError();
    void setModuleAddress(byte newAddr);
    int getModuleAddress();
    int getBalStat();
    bool isExisting();
    bool isReset();
    void setAddress(byte Addr);
    void setReset(bool ex);
    void setExists(bool ex);
    void setTempsensor(byte tempsensor);
    void setIgnoreCellV(uint16_t IgnoreV);
    void setDelta(uint16_t ex);    
    void readStatus();
    void stopBalance();

 
private:
    BMS_t BMSType;
    uint16_t cellVolt[MAX_CELL_No];             // mV, factor 1000
    uint16_t moduleVolt;                        // mV, factor 1000
    int16_t temperatures[MAX_Temp_Sens];        // 0,1°C, factor 10
    uint16_t IgnoreCellV;                       // mV, factor 1000
    uint16_t VoltDelta;                         // mV, factor 1000
    bool exists;
    bool reset;
    int alerts;
    int faults;
    int COVFaults;
    int CUVFaults;
    byte TSensor;
    uint8_t moduleAddress;     //1 to MAX_MODULE_ADDRESS
    uint32_t balstat; // Bitmask
    uint32_t lasterror;
    uint32_t CMUerror;
    uint32_t timeout;
    uint16_t retmoduleVolt;  
    bool VW_decodeV(CAN_message_t &msg, byte Id);
    bool VW_decodeT_Bal(CAN_message_t &msg);
    bool BMW_decodeV_Bal(CAN_message_t &msg, byte Id);
    bool BMW_decodeT(CAN_message_t &msg);
    bool Tesla_readModule();
};