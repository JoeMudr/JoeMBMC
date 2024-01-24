/*
    Usage:
    1. initBMS() - set all neccesary values and init modules
    2. poll() - sends trigger to modules to get their data. This also disables balancing. Make sure not to enable balancing before data is read (3.).
    3. readModulesValues() - read and processes the answer
    4. Balancing() - activate / deactivate balancing, when needed 

*/

#pragma once
#include "config.h"
#include "BMSModule.h"
#include <FlexCAN_T4.h>
#include "CAN_Struct.h"

class BMSManager{
public:
    BMSManager();
    ~BMSManager();
    void initBMS(BMS_t BMS_Type,uint16_t IgnoreV,byte sensor);
    CAN_Struct poll(); // call this to poll modules
    void readModulesValues(); // Serial
    void readModulesValues(CAN_message_t &msg); // CAN
    void printAllCSV(unsigned long timestamp, float current, int SOC);
    void printPackDetails();
    uint16_t getHighCellVolt();
    uint16_t getLowCellVolt();
    uint16_t getHighTemperature();
    uint16_t getLowTemperature();
    uint32_t getPackVoltage();
    uint16_t getAvgCellVolt();
    uint16_t getAvgTemperature();
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
    uint16_t BalHys;
    byte TSensor;
    byte numFoundModules;
    byte moduleReadCnt; // count how many modules have been read
    byte balancingCells[MAX_MODULE_ADDR+1];
    bool balancingActive;
    void VW_get_CMU_ID(CAN_message_t &msg,byte &CMU,byte &Id);
    CAN_Struct VW_Balancing();
    void Tesla_Balancing();
    void Tesla_renumberModulesIDs(); // reset and re enumerate all Modules
    void Tesla_setupModules(); // init Modules
    void Tesla_sleepModules();
    void Tesla_wakeModules();
    void Tesla_findModules();
    void Tesla_clearFaults();
    void Tesla_resetModules();
    CAN_Struct clearCANStruct(); // returns an empty initialized CAN_Struct
    void UpdateValues();
};
