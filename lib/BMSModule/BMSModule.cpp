#include <config.h>
#include "BMSModule.h"
#include "Tesla_BMSUtil.h"

BMSModule::BMSModule(){
    clearModule();
    IgnoreCellV = 0; // mandatory
    VoltDelta = 0;
    exists = false;
    reset = false;
    alerts = 0;
    faults = 0;
    COVFaults = 0;
    CUVFaults = 0;
    TSensor = 0; // mandatory
    moduleAddress = 0;
    balstat = 0;
    lasterror = 0;
    CMUerror = 0;
    timeout = 30000; //milliseconds before comms timeout;
    BMSType = BMS_Dummy; // mandatory
    ModuleAddress = 0;
}

void BMSModule::initModule(BMS_t BMS_Type, uint16_t IgnoreV, byte tempsensor){
    BMSType = BMS_Type;
    setTempsensor(tempsensor);
    setIgnoreCellV(IgnoreV);
}
/*
    read Serial
*/
bool BMSModule::readModule(){
    bool retVal = false;
    switch (BMSType){
        case BMS_Tesla:
            retVal = Tesla_readModule();
            break;
        
        default:
            break;
    }
    return retVal; 
}

/*
    read CAN
*/

bool BMSModule::readModule(CAN_message_t &msg, int Id = 0){
    bool retVal = false;
    switch (BMSType){
        case BMS_VW_eGolf:
        case BMS_VW_MEB:
        {
            if(msg.id < 0x300){retVal = VW_decodeV(msg, Id);} // VW Voltage-IDs
            uint32_t tmpID = msg.id;
            tmpID &= 0x1FFFFFFF;
            if((0x1A555400 < tmpID && tmpID < 0x1A555440)  || (0x1A5555EF < tmpID && tmpID < 0x1A5555FF)) // VW Temp.-IDs
                {VW_decodeT_Bal(msg);} // don't set retVal. Return value used to confirm presence of module when reading Voltage!
        }
        break;
        case BMS_BMW_I3:
        {
            if(0x99 < msg.id && msg.id < 0x160){retVal = BMW_decodeV_Bal(msg,Id);}
            if((msg.id & 0xFF0) == 0x170){BMW_decodeT(msg);} // don't set retVal. Return value used to confirm presence of module when reading Voltage!
        }
        break;
        default:
            break;
    }
    return retVal;
}

bool BMSModule::VW_decodeV(CAN_message_t &msg, byte Id){
    bool retVal = false;
    switch (Id){
        case 0:
            if (msg.buf[2] != 0xFF && msg.buf[5] != 0xFF && msg.buf[7] != 0xFF){ //Check module is not initializing OR a "spoof module" 
                CMUerror = 0;
                cellVolt[0] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000);
                cellVolt[2] = (uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000);
                cellVolt[1] = (msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000);
                cellVolt[3] = (msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000);
                retVal = true;
            }
        break;
        case 1:
            if (msg.buf[2] != 0xFF && msg.buf[5] != 0xFF && msg.buf[7] != 0xFF){ //Check module is not initializing OR a "spoof module" 
                CMUerror = 0;
                cellVolt[4] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000);
                cellVolt[6] = (uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000);
                cellVolt[5] = (msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000);
                cellVolt[7] = (msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000);
                retVal = true;
            }
        break;
        case 2:
            if (msg.buf[2] != 0xFF){ //Check module is not initializing OR a "spoof module" 
                CMUerror = 0;
                cellVolt[8] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000);
                cellVolt[10] = (uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000);
                cellVolt[9] = (msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000);
                cellVolt[11] = (msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000);
                retVal = true;
            }
        break;
        case 3:
            if (msg.buf[2] != 0xFF){ //Check module is not initializing OR a "spoof module" 
                CMUerror = 0;
                cellVolt[12] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000);
                retVal = true;    
            }
        break;

        default: break;    
    } 
    return retVal;
}

bool BMSModule::VW_decodeT_Bal(CAN_message_t &msg){
    bool retVal = false;
    switch (BMSType){
    case BMS_VW_eGolf:
        if (msg.buf[7] == 0xFD && msg.buf[2] != 0xFD){temperatures[0] = ((msg.buf[2] >> 1) - 40) * 10;}
        else {
            //[ToTest]
            // Sensor 1
            if (msg.buf[0] < 0xDF){
                temperatures[0] = ((msg.buf[0] >> 1) - 43) * 10;
                balstat = msg.buf[2] + (msg.buf[3] << 8); //[ToDo] reset balstat prior?
                retVal = true;
            }
            else{temperatures[0] = msg.buf[3] - 43;retVal = true;}
            
            // Sensor 2
            if (msg.buf[4] < 0xF0){temperatures[1] = ((msg.buf[4] >> 1) - 43) * 10;}
            else{temperatures[1] = 0;retVal = true;}
            
            // Sensor 3
            if (msg.buf[5] < 0xF0){temperatures[2] = ((msg.buf[5] >> 1) - 43) * 10;}
            else{temperatures[2] = 0;retVal = true;}
        }
    break;
    case BMS_VW_MEB:
        //Check module is not initializing OR a "spoof module" 
        if (msg.buf[5] != 0xFF){
            temperatures[0] = ((uint16_t(((msg.buf[5] & 0x0F) << 4) | ((msg.buf[4] & 0xF0) >> 4)) * 0.5) - 40) * 10; //MEB Bits 36-44
            retVal = true;
        }
    break;
    default: break;
    }  
    return retVal; 
}

bool BMSModule::BMW_decodeV_Bal(CAN_message_t &msg, byte Id){
    bool retVal = false;
    switch (Id){
        case 0:
        CMUerror = msg.buf[0] + (msg.buf[1] << 8) + (msg.buf[2] << 16) + (msg.buf[3] << 24);
        balstat = ((msg.buf[5] & 0x0F ) << 8) + msg.buf[4];
        break;

        case 1:
            if (msg.buf[1] < 0x40){ cellVolt[0] = msg.buf[0] + ((msg.buf[1] & 0x3F) << 8); }
            if (msg.buf[3] < 0x40){ cellVolt[1] = msg.buf[2] + ((msg.buf[3] & 0x3F) << 8); }
            if (msg.buf[5] < 0x40){ cellVolt[2] = msg.buf[4] + ((msg.buf[5] & 0x3F) << 8); }
            retVal = true;
        break;

        case 2:
            if (msg.buf[1] < 0x40){ cellVolt[3] = msg.buf[0] + ((msg.buf[1] & 0x3F) << 8); }
            if (msg.buf[3] < 0x40){ cellVolt[4] = msg.buf[2] + ((msg.buf[3] & 0x3F) << 8); }
            if (msg.buf[5] < 0x40){ cellVolt[5] = msg.buf[4] + ((msg.buf[5] & 0x3F) << 8); }
            retVal = true;
        break;

        case 3:
            if (msg.buf[1] < 0x40){ cellVolt[6] = msg.buf[0] + ((msg.buf[1] & 0x3F) << 8); }
            if (msg.buf[3] < 0x40){ cellVolt[7] = msg.buf[2] + ((msg.buf[3] & 0x3F) << 8); }
            if (msg.buf[5] < 0x40){ cellVolt[8] = msg.buf[4] + ((msg.buf[5] & 0x3F) << 8); }
            retVal = true;
        break;

        case 4:
            if (msg.buf[1] < 0x40){ cellVolt[9] = msg.buf[0] + ((msg.buf[1] & 0x3F) << 8); }
            if (msg.buf[3] < 0x40){ cellVolt[10] = msg.buf[2] + ((msg.buf[3] & 0x3F) << 8); }
            if (msg.buf[5] < 0x40){ cellVolt[11] = msg.buf[4] + ((msg.buf[5] & 0x3F) << 8); }
            retVal = true;
        break;

        default: break;
    }
    return retVal;
}
bool BMSModule::BMW_decodeT(CAN_message_t &msg){
    bool retVal = false;
    for (int Tsens = 0; Tsens < MAX_Temp_Sens; Tsens++){
        temperatures[Tsens] = (msg.buf[Tsens] - 40) * 10;
        retVal = false;
        // if (temperatures[Tsens] > -40){temperatures[Tsens] = temperatures[Tsens] + TempOff;} // [Todo] WTF?!
    }
    return retVal;
}


bool BMSModule::Tesla_readModule(){
        bool retVal = false;
        uint8_t payload[4];
        uint8_t buff[50];
        uint8_t calcCRC;
        int retLen;
        float tempCalc;
        float tempTemp;

        payload[0] = moduleAddress << 1;
        delay(2); // [Todo]???
        payload[0] = moduleAddress << 1;

        readStatus();

        payload[1] = REG_ADC_CTRL;
        payload[2] = 0b00111101; //ADC Auto mode, read every ADC input we can (Both Temps, Pack, 6 cells)
        BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

        payload[1] = REG_IO_CTRL;
        payload[2] = 0b00000011; //enable temperature measurement VSS pins
        BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

        payload[1] = REG_ADC_CONV; //start all ADC conversions
        payload[2] = 1;
        BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

        payload[1] = REG_GPAI; //start reading registers at the module voltage registers
        payload[2] = 0x12; //read 18 bytes (Each value takes 2 - ModuleV, CellV1-6, Temp1, Temp2)
        retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 22);

        calcCRC = BMSUtil::genCRC(buff, retLen - 1);

        //18 data bytes, address, command, length, and CRC = 22 bytes returned
        //Also validate CRC to ensure we didn't get garbage data.
        if ( (retLen == 22) && (buff[21] == calcCRC) ){
            if (buff[0] == (moduleAddress << 1) && buff[1] == REG_GPAI && buff[2] == 0x12) //Also ensure this is actually the reply to our intended query
            {
            //payload is 2 bytes gpai, 2 bytes for each of 6 cell voltages, 2 bytes for each of two temperatures (18 bytes of data)
            //retmoduleVolt = ((buff[3] << 8 + buff[4]) * 2.034609f; // Value * 33333 / 16383 //not used?
            for (int i = 0; i < 6; i++){
                cellVolt[i] = ((buff[5 + (i * 2)] << 8) + buff[6 + (i * 2)]) * 0.381493f; // Value * 625 / 16383
            }

            ////use added up cells and not reported module voltage////////
            moduleVolt = 0;
            for (int i = 0; i < 6; i++){moduleVolt = moduleVolt + cellVolt[i];}

            //Now using steinhart-hart equation for temperatures. We'll see if it is better than old code.
            tempTemp = (1.78f / (((buff[17] << 8) + buff[18] + 2) / 33046.0f) - 3.57f);
            tempTemp *= 1000.0f;
            //      Factor 10 / (       a0        +     a1           * ln(R)           +  ln³(R)                  * a3)
            tempCalc =  10.0f / (0.0007610373573f + (0.0002728524832 * logf(tempTemp)) + (powf(logf(tempTemp), 3) * 0.0000001022822735f));
            temperatures[0] = tempCalc - 2731.5f; // result of Steinhart-Hart is in K

            tempTemp = 1.78f / (((buff[19] << 8) + buff[20] + 9) / 33068.0f) - 3.57f;
            tempTemp *= 1000.0f;
            tempCalc = 10.0f / (0.0007610373573f + (0.0002728524832 * logf(tempTemp)) + (powf(logf(tempTemp), 3) * 0.0000001022822735f));
            temperatures[1] = tempCalc - 2731.5f; // result of Steinhart-Hart is in K

            //if (getLowTemp() < lowestTemperature) lowestTemperature = getLowTemp();
            //if (getHighTemp() > highestTemperature) highestTemperature = getHighTemp();

            //Logger::debug("Got voltage and temperature readings");
            retVal = true;
            }
        } else { 
            // [todo]error here
            retVal = false;
        }

        //turning the temperature wires off here seems to cause weird temperature glitches
        // payload[1] = REG_IO_CTRL;
        // payload[2] = 0b00000000; //turn off temperature measurement pins
        // BMSUtil::sendData(payload, 3, true);
        // delay(3);
        // BMSUtil::getReply(buff, 50);    //TODO: we're not validating the reply here. Perhaps check to see if a valid reply came back       
        return retVal;
}

void BMSModule::clearModule(){
    for (byte i = 0; i < MAX_CELL_No; i++){cellVolt[i] = 0;}
    for (byte i = 0; i < MAX_Temp_Sens; i++){temperatures[i] = -999;}
    moduleVolt = 0;
    setExists(false);
}

byte BMSModule::balanceModule(byte balance_Bitmask){

    switch (BMSType){
        case BMS_Tesla:{
            byte payload[4];
            byte buff[30];
            byte duration = 50;

            if (balance_Bitmask != 0){ //only send balance command when needed
                payload[0] = moduleAddress << 1;
                payload[1] = REG_BAL_TIME;
                payload[2] = duration; //5 second balance limit, if not triggered to balance it will stop after 5 seconds
                BMSUtil::sendData(payload, 3, true);
                delay(2);
                BMSUtil::getReply(buff, 30);
                payload[0] = moduleAddress << 1;
                payload[1] = REG_BAL_CTRL;
                payload[2] = balance_Bitmask; //write balance state to register
                BMSUtil::sendData(payload, 3, true);
                delay(2);
                BMSUtil::getReply(buff, 30);
            }
        }
        break;
    
        default:
            break;
    }
    return balance_Bitmask;
}

byte BMSModule::getscells(){
    byte scells = 0;
    for (int i = 0; i < MAX_CELL_No; i++){
        if (cellVolt[i] > IgnoreCellV && cellVolt[i] < 5000){scells++;}
    }
    return scells;
}

uint16_t BMSModule::getCellVoltage(int cell){
    uint16_t retval = 0;
    if (0 <= cell  && cell < MAX_CELL_No){
        if (IgnoreCellV < cellVolt[cell] && cellVolt[cell] < 5000){retval = cellVolt[cell];}
    }
    return retval;
}

uint16_t BMSModule::getLowCellV(){
    uint16_t lowVal = 10000;
    for (int i = 0; i < MAX_CELL_No; i++) if(cellVolt[i] < lowVal && cellVolt[i] > IgnoreCellV) lowVal = cellVolt[i];
    return lowVal;
}

uint16_t BMSModule::getHighCellV(){
    uint16_t hiVal = 0;
    for (int i = 0; i < MAX_CELL_No; i++)
        if (cellVolt[i] > IgnoreCellV && cellVolt[i] < 5000)
        {if (cellVolt[i] > hiVal) hiVal = cellVolt[i];}
    return hiVal;
}

uint16_t BMSModule::getAverageV(){
    int CellCount = 0;
    uint16_t avgVolt = 0;

    for (int i = 0; i < MAX_CELL_No; i++){
        if (cellVolt[i] > IgnoreCellV && cellVolt[i] < 5000){
            CellCount++;
            avgVolt += cellVolt[i];
        }
    }

    if (CellCount == 0){avgVolt = 0;}
    else {avgVolt /= CellCount;}

    return avgVolt;
}

int16_t BMSModule::getLowTemp(){
    int16_t TLow = 999;
    if (TSensor == 0){
        for (byte i = 0; i < MAX_Temp_Sens; i++){
            if (temperatures[i] > -999 && temperatures[i] < TLow){TLow = temperatures[i];}
        } 
    }
    else {TLow = temperatures[TSensor - 1];}
    return TLow;
}

int16_t BMSModule::getHighTemp(){
    int16_t THigh = -999;
    if (TSensor == 0){
        for (byte i = 0; i < MAX_Temp_Sens; i++){
            if (temperatures[i] > -999 && temperatures[i] > THigh){THigh = temperatures[i];}        
        } 
    } else {THigh = temperatures[TSensor - 1];}
    return THigh;
}

int16_t BMSModule::getAvgTemp(){
    int16_t TAvg = 0;
    byte TSenCount = 0;
    if (TSensor == 0){
        for (byte i = 0; i < MAX_Temp_Sens; i++){
            if (temperatures[i] > -999){TAvg += temperatures[i];}        
            TSenCount++;
        } 
    } else {TAvg = temperatures[TSensor - 1];}
    if (TSenCount > 0){TAvg /= TSenCount;}
    
    return TAvg;
}

uint16_t BMSModule::getModuleVoltage(){
    moduleVolt = 0;
    for (byte i = 0; i < MAX_CELL_No; i++){
        if (cellVolt[i] > IgnoreCellV && cellVolt[i] < 5000)
            {moduleVolt = moduleVolt + cellVolt[i];}
        }
    return moduleVolt;
}

int16_t BMSModule::getTemperature(byte tempsensor){return temperatures[tempsensor];}
uint8_t BMSModule::getFaults(){return faults;}
uint8_t BMSModule::getAlerts(){return alerts;}
uint8_t BMSModule::getCOVCells(){return COVFaults;}
uint8_t BMSModule::getCUVCells(){return CUVFaults;}
void BMSModule::setModuleAddress(byte newAddr){
    if (0 < newAddr && newAddr > MAX_MODULE_ADDR) {moduleAddress = newAddr;}
}

int BMSModule::getModuleAddress(){return moduleAddress;}
int BMSModule::getBalStat(){return balstat;}
bool BMSModule::isExisting(){return exists;}
bool BMSModule::isReset(){return reset;} // [ToDo] actually reset the module?!
void BMSModule::setAddress(byte Addr){moduleAddress = Addr;}
void BMSModule::setReset(bool ex){reset = ex;}
void BMSModule::setExists(bool ex){exists = ex;}
void BMSModule::setTempsensor(byte tempsensor){TSensor = tempsensor;}
void BMSModule::setIgnoreCellV(uint16_t IgnoreV){IgnoreCellV = IgnoreV;}
void BMSModule::setDelta(uint16_t ex){VoltDelta = ex;}
void BMSModule::readStatus(){
    switch (BMSType){
    case BMS_Tesla:
        uint8_t payload[3];
        uint8_t buff[8];

        payload[0] = moduleAddress << 1; //adresss
        payload[1] = REG_DEV_STATUS;//Alert Status start
        payload[2] = 0x01;
        BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);


        payload[0] = moduleAddress << 1; //adresss
        payload[1] = REG_ALERT_STATUS;//Alert Status start
        payload[2] = 0x04;
        BMSUtil::sendDataWithReply(payload, 3, false, buff, 7);
        alerts = buff[3];
        faults = buff[4];
        COVFaults = buff[5];
        CUVFaults = buff[6];
        payload[0] = moduleAddress << 1; //adresss
        payload[1] = REG_BAL_TIME;//Alert Status start
        payload[2] = 0x01;
        BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
        break;
    
    default:
        break;
    }
}
void BMSModule::stopBalance(){
    switch (BMSType)
    {
    case BMS_Tesla:
        uint8_t buff[8];
        uint8_t payload[4];
        payload[0] = moduleAddress << 1;
        payload[1] = REG_BAL_CTRL;
        payload[2] = 0; //write balance state to register
        BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);
        delay(2);
    break;
    
    default: break;
    }
}

uint32_t BMSModule::getError(){ return CMUerror;}