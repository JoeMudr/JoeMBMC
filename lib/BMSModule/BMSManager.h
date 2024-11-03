/*
    Usage:
    1. initBMS() - set all neccesary values and init modules
    2. poll() - sends trigger to modules to get their data. This also disables balancing. Make sure not to enable balancing before data is read (3.).
    3. readModulesValues() - read and processes the answer
    4. Balancing() - Activate / deactivate balancing, when needed. Also cheks, if all module values have been read before activating balancing.

*/

#pragma once
#include "BMSModule.h"
#include "CAN_Struct.h"
#include "CRC8.h"

class BMSManager{
public:
    BMSManager();
    ~BMSManager();
    void initBMS(BMS_t BMS_Type,uint16_t IgnoreV,byte sensor, uint32_t  Read_Timeout);
    CAN_Struct poll(); // call this to poll modules
    void readModulesValues(); // Serial
    void readModulesValues(CAN_message_t &msg); // CAN
    void printAllCSV(unsigned long timestamp, float current, int SOC);
    void printPackDetails();
    uint16_t getHighCellVolt(); // in mV
    uint16_t getHighModuleVolt(); // in mV
    uint16_t getLowCellVolt(); // in mV
    uint16_t getLowModuleVolt(); // in mV
    int16_t getHighTemperature();
    int16_t getLowTemperature();
    uint32_t getPackVoltage(); // in mV
    uint16_t getAvgCellVolt();
    int16_t getAvgTemperature();
    void clearModules();
    byte getBalancingCells(byte module_id); //move to module?
    uint16_t getBalancing(); //returns number of currently balancing cells
    byte getNumModules(); //byte ausreichend?
    uint16_t getSeriesCells();
    void clearFaults();
    CAN_Struct Balancing(uint16_t balhys, bool active); 

private:
    BMS_t BMSType;
    BMSModule modules[MAX_MODULE_ADDR+1];
    uint32_t packVolt;                         // All modules added together
    uint16_t LowCellVolt;
    uint16_t HighCellVolt;
    uint16_t AvgCellVolt;
    uint16_t IgnoreCellV;
    int16_t AvgTemp;
    int16_t highTemp;
    int16_t lowTemp;
    byte TSensor;
    byte numFoundModules;
    byte moduleReadCnt; // count how many modules have been read
    byte balancingCells[MAX_MODULE_ADDR+1];
    void VW_get_CMU_ID(CAN_message_t &msg,byte &CMU,byte &Id);
    void BMW_get_CMU_ID(CAN_message_t &msg,byte &CMU,byte &Id);
    uint8_t BMW_CRC(CAN_message_t &msg, byte Id);
    CAN_Struct VW_Balancing(uint16_t BalHys,bool active);
    CAN_Struct BMW_Balancing(uint16_t BalHys,bool active);
    void Tesla_Balancing(uint16_t BalHys,bool active);
    void Tesla_renumberModulesIDs(); // reset and re enumerate all Modules
    void Tesla_setupModules(); // init Modules
    void Tesla_sleepModules();
    void Tesla_wakeModules();
    void Tesla_findModules();
    void Tesla_clearFaults();
    void Tesla_resetModules();
    void UpdateValues();
    CRC8 crc8;
    uint32_t ReadTimeout;
    uint32_t LastRead;
    uint32_t polltime; // keep track of poll intervall
    byte MsgCnt;
};
