#include "BMSManager.h"
#include "CAN_Struct.h"
#include "Tesla_BMSUtil.h"

BMSManager::BMSManager(){
    BMSType = BMS_Dummy;
    AvgCellVolt = 5000;
    AvgTemp = -999;
    HighCellVolt = 0;
    LowCellVolt = 5000;
    highTemp = -999;
    lowTemp = 999;
    TSensor = 0; // default use all Sensors
    packVolt = 0;
    for (int moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
        modules[moduleNr].setExists(false);
        modules[moduleNr].setAddress(moduleNr);
        balancingCells[moduleNr] = 0;
    }
    numFoundModules = 0;
    moduleReadCnt = 0;
    ReadTimeout = 0;
    polltime = 0;
    MsgCnt = 0;
}

BMSManager::~BMSManager(){}

void BMSManager::initBMS(BMS_t BMS_Type,uint16_t IgnoreV,byte sensor, uint32_t  Read_Timeout){
    BMSType = BMS_Type;
    IgnoreCellV = IgnoreV;
    TSensor = sensor;
    ReadTimeout = Read_Timeout;
    // reset all values again in case this method is called to reset the BMS
    AvgCellVolt = 5000;
    AvgTemp = -999;
    HighCellVolt = 0;
    LowCellVolt = 5000;
    highTemp = -999;
    lowTemp = 999;
    packVolt = 0;
    for (byte moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
        modules[moduleNr].clearModule();
        modules[moduleNr].initModule(BMSType,IgnoreCellV,TSensor);
    }
    switch (BMSType){
        case BMS_Tesla:
            Tesla_renumberModulesIDs();
            Tesla_findModules();
        break;

        case BMS_BMW_I3:
        case BMS_BMW_MiniE:

        break;
        
        default: break;
    }
    clearFaults();
}

/*
    Call this to make Modules send their values on CAN-Bus.
    Returns CAN-Struct to loop through and send via CAN.
*/

CAN_Struct BMSManager::poll(){
    // reset BMS if last module read took too long.
    CAN_Struct msg;
    msg = clearCANStruct();
    switch (BMSType){
        case BMS_VW_eGolf: 
        case BMS_VW_MEB: 
        {        
            if(millis() - polltime < 500) break; // poll erevy 500ms
            moduleReadCnt = 0;
            polltime = millis();

            msg = Balancing(0,false); // deactivate balancing for measurement first
            
            //Attention: CAN_Struct has to be big enough to hold all messages!
            byte msgNr;
            for(msgNr = 0; msgNr < CAN_Struct_size; msgNr++){
                if(!msg.Frame[msgNr].id) break; //find first unused ID.
            }
            uint16_t controlid = 0xBA;
            msg.Frame[msgNr].id = controlid;
            msg.Frame[msgNr].len = 8;
            msg.Frame[msgNr].buf[0] = 0x00;
            msg.Frame[msgNr].buf[1] = 0x00;
            msg.Frame[msgNr].buf[2] = 0x00;
            msg.Frame[msgNr].buf[3] = 0x00;
            msg.Frame[msgNr].buf[4] = 0x00;
            msg.Frame[msgNr].buf[5] = 0x00;
            msg.Frame[msgNr].buf[6] = 0x00;
            msg.Frame[msgNr].buf[7] = 0x00;
            
            msg.Frame[msgNr+1].id = controlid;
            msg.Frame[msgNr+1].len = 8;
            msg.Frame[msgNr+1].buf[0] = 0x45;
            msg.Frame[msgNr+1].buf[1] = 0x01;
            msg.Frame[msgNr+1].buf[2] = 0x28;
            msg.Frame[msgNr+1].buf[3] = 0x00;
            msg.Frame[msgNr+1].buf[4] = 0x00;
            msg.Frame[msgNr+1].buf[5] = 0x00;
            msg.Frame[msgNr+1].buf[6] = 0x00;
            msg.Frame[msgNr+1].buf[7] = 0x30;
        }
        break;

        case BMS_BMW_I3:
        case BMS_BMW_MiniE:
        {
            if(millis() - polltime < 50) break; // poll erevy 50ms
            moduleReadCnt = 0;
            polltime = millis();

            msg = Balancing(0,false); // deactivate balancing for measurement first

            byte msgNr;
            for(msgNr = 0; msgNr < CAN_Struct_size; msgNr++){
                if(!msg.Frame[msgNr].id) break; //find first unused msgNr.
            }

            byte MAX_Module_No = (BMSType == BMS_BMW_I3)? 7 : 11; // i3: 8 Modules; MiniE: 12 Modules
            
            for (byte ModuleNr = 0; ModuleNr <= MAX_Module_No; ModuleNr++){
                
                if(MsgCnt == 0x0F){MsgCnt = 0;}
                
                msg.Frame[msgNr].id = 0x80 + ModuleNr;
                msg.Frame[msgNr].len = 8;
                msg.Frame[msgNr].buf[0] = 0x68;
                msg.Frame[msgNr].buf[1] = 0x10;
                msg.Frame[msgNr].buf[2] = 0x00;
                msg.Frame[msgNr].buf[3] = 0x50;
                msg.Frame[msgNr].buf[4] = 0x00;
                msg.Frame[msgNr].buf[5] = 0x00;
                msg.Frame[msgNr].buf[6] = MsgCnt << 4;
                msg.Frame[msgNr].buf[7] = BMW_CRC(msg.Frame[msgNr], ModuleNr);

                MsgCnt++;
            }
        }
        break;

        case BMS_Tesla:
            if(millis() - polltime < 500) break; // poll every 500ms
            moduleReadCnt = 0;
            polltime = millis();

            Balancing(0, false);
            delay(200);
            readModulesValues();   
        break;

        default: break;
    }
    if(millis() > LastRead + ReadTimeout){
        initBMS(BMSType,IgnoreCellV,TSensor,ReadTimeout);
    }
    return msg;
}

/*
    read values on Serial
    First call Poll() to initiate the module messages.
*/
void BMSManager::readModulesValues(){
    
    bool chkRead = false;
    for (byte moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
        if(modules[moduleNr].isExisting()){
            switch (BMSType){
                case BMS_Tesla:
                    chkRead = modules[moduleNr].readModule();
                    if (chkRead){moduleReadCnt++;}
                break;
                
                default: break;
            }
        }
        // only update values if moduleNr has actually been read
        if(chkRead){ UpdateValues(); LastRead = millis();}
    }
}

/*
    read values on CAN-Bus
    First call Poll() to initiate the module messages.
*/
void BMSManager::readModulesValues(CAN_message_t &msg){
    bool chkRead = false;
    byte CMU, Id = 0;
    switch (BMSType){
        case BMS_VW_eGolf:
        case BMS_VW_MEB:
        {
            VW_get_CMU_ID(msg, CMU, Id);
            if(CMU){
                chkRead = modules[CMU].readModule(msg, Id);
                if (chkRead){
                    modules[CMU].setExists(true);
                    moduleReadCnt++;
                };
            }
        }
        break;
        case BMS_BMW_I3:
        BMW_get_CMU_ID(msg, CMU, Id);
            if(CMU)
            {
                chkRead = modules[CMU].readModule(msg, Id);
                if (chkRead){
                    modules[CMU].setExists(true);
                    moduleReadCnt++;
                };
            }
        break;
        
        default: break;
    }
    // only update values if moduleNr has actually been read
    if(chkRead){ UpdateValues(); LastRead = millis();}
}

void BMSManager::printPackDetails(){
    for (byte moduleNr = 0; moduleNr < MAX_MODULE_ADDR; moduleNr++){
        if(modules[moduleNr].isExisting()){
            activeSerial->printf("Module #%i%s %imV",moduleNr,moduleNr<10?" ":"",modules[moduleNr].getModuleVoltage());
            
            for (byte cellNr = 0; cellNr < MAX_CELL_No; cellNr++){
                uint16_t tmpV = modules[moduleNr].getCellVoltage(cellNr);
                if(tmpV){
                    activeSerial->printf(" C%i: %4imV%s",cellNr+1,tmpV,balancingCells[moduleNr] & (1 << cellNr)?"*":" ");
                }
            }
            for (byte senNr = 0; senNr < MAX_Temp_Sens; senNr++){
                if (modules[moduleNr].getTemperature(senNr) > -999){
                    activeSerial->printf(" T%i: %.1fC",senNr+1,float(modules[moduleNr].getTemperature(senNr)) / 10);
                }
            }
            activeSerial->printf("\r\n");
        }
    }
    
}

void BMSManager::UpdateValues(){
    packVolt = 0;
    HighCellVolt = 0;
    LowCellVolt = 5000;
    AvgCellVolt = 5000;
    highTemp = -999;
    lowTemp = 999;
    AvgTemp = 0;

    uint16_t cellCnt = 0;
    uint16_t moduleCnt = 0;

    for (byte moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
        
        if(modules[moduleNr].isExisting()){
            moduleCnt++;
            if (modules[moduleNr].getHighCellV() >  HighCellVolt)  HighCellVolt = modules[moduleNr].getHighCellV();
            if (modules[moduleNr].getLowCellV() <  LowCellVolt)  LowCellVolt = modules[moduleNr].getLowCellV();
            if (modules[moduleNr].getAvgTemp() > -70){
                if (modules[moduleNr].getHighTemp() > highTemp){highTemp = modules[moduleNr].getHighTemp();} 
                if (modules[moduleNr].getLowTemp() < lowTemp){lowTemp = modules[moduleNr].getLowTemp();}
                AvgTemp += modules[moduleNr].getAvgTemp();
            }
            packVolt += modules[moduleNr].getModuleVoltage();
            for (byte cellNr = 0; cellNr < MAX_CELL_No; cellNr++){
                if(modules[moduleNr].getCellVoltage(cellNr)){cellCnt++;}
            }
        }
    } 
   
    AvgTemp = AvgTemp / moduleCnt;
    AvgCellVolt = round(packVolt / cellCnt);
}

void BMSManager::VW_get_CMU_ID(CAN_message_t &msg, byte &CMU, byte &Id){
    //Voltages
    switch (msg.id){
      /////////standard ids////////////////
      case (0x1B0): CMU = 1; Id = 0; break;
      case (0x1B1): CMU = 1; Id = 1; break;
      case (0x1B2): CMU = 1; Id = 2; break;
      case (0x1B3): CMU = 1; Id = 3; break;
      case (0x1B4): CMU = 2; Id = 0; break;
      case (0x1B5): CMU = 2; Id = 1; break;
      case (0x1B6): CMU = 2; Id = 2; break;
      case (0x1B7): CMU = 2; Id = 3; break;
      case (0x1B8): CMU = 3; Id = 0; break;
      case (0x1B9): CMU = 3; Id = 1; break;
      case (0x1BA): CMU = 3; Id = 2; break;
      case (0x1BB): CMU = 3; Id = 3; break;
      case (0x1BC): CMU = 4; Id = 0; break;
      case (0x1BD): CMU = 4; Id = 1; break;
      case (0x1BE): CMU = 4; Id = 2; break;
      case (0x1BF): CMU = 4; Id = 3; break;
      case (0x1C0): CMU = 5; Id = 0; break;
      case (0x1C1): CMU = 5; Id = 1; break;
      case (0x1C2): CMU = 5; Id = 2; break;
      case (0x1C3): CMU = 5; Id = 3; break;
      case (0x1C4): CMU = 6; Id = 0; break;
      case (0x1C5): CMU = 6; Id = 1; break;
      case (0x1C6): CMU = 6; Id = 2; break;
      case (0x1C7): CMU = 6; Id = 3; break;
      case (0x1C8): CMU = 7; Id = 0; break;
      case (0x1C9): CMU = 7; Id = 1; break;
      case (0x1CA): CMU = 7; Id = 2; break;
      case (0x1CB): CMU = 7; Id = 3; break;
      case (0x1CC): CMU = 8; Id = 0; break;
      case (0x1CD): CMU = 8; Id = 1; break;
      case (0x1CE): CMU = 8; Id = 2; break;
      case (0x1CF): CMU = 8; Id = 3; break; 
      //// one extender increment//////////
      case (0x1D0): CMU = 9; Id = 0; break;
      case (0x1D1): CMU = 9; Id = 1; break;
      case (0x1D2): CMU = 9; Id = 2; break;
      case (0x1D3): CMU = 9; Id = 3; break;
      case (0x1D4): CMU = 10;Id = 0; break;
      case (0x1D5): CMU = 10;Id = 1; break;
      case (0x1D6): CMU = 10;Id = 2; break;
      case (0x1D8): CMU = 11;Id = 0; break;
      case (0x1D9): CMU = 11;Id = 1; break;
      case (0x1DA): CMU = 11;Id = 2; break;
      case (0x1DC): CMU = 12;Id = 0; break;
      case (0x1DD): CMU = 12;Id = 1; break;
      case (0x1DE): CMU = 12;Id = 2; break;
      case (0x1E0): CMU = 13;Id = 0; break;
      case (0x1E1): CMU = 13;Id = 1; break;
      case (0x1E2): CMU = 13;Id = 2; break;
      case (0x1E4): CMU = 14;Id = 0; break;
      case (0x1E5): CMU = 14;Id = 1; break;
      case (0x1E6): CMU = 14;Id = 2; break;
      case (0x1E8): CMU = 15;Id = 0; break;
      case (0x1E9): CMU = 15;Id = 1; break;
      case (0x1EA): CMU = 15;Id = 2; break;
      case (0x1EC): CMU = 16;Id = 0; break;
      case (0x1ED): CMU = 16;Id = 1; break;
      case (0x1EE): CMU = 16;Id = 2; break;

      default: break;
    }

    //Temperatures
    uint32_t tmpID = msg.id;
    tmpID &= 0x1FFFFFFF;
    if((0x1A555400 < tmpID && tmpID < 0x1A555440)  || (0x1A5555EF < tmpID && tmpID < 0x1A5555FF)){
        switch (BMSType){
        case BMS_VW_eGolf:
            CMU = (msg.id & 0xFF);
            if (10 < CMU && CMU < 60){ CMU = ((CMU & 0x0F) >> 1) + 1; } //[ToDo] Toms Code! Makes no sense. lowest CMU-ID would be 6!          
        break;
        case BMS_VW_MEB:
            CMU = ((msg.id & 0x0F) + 1);
        break;
        default:
            break;
        }
    }
}

void BMSManager::BMW_get_CMU_ID(CAN_message_t &msg, byte &CMU, byte &Id){
    // Voltages
    if(0x99 < msg.id && msg.id < 0x160){
        CMU = (msg.id & 0x00F) + 1;
        Id = (msg.id & 0x0F0) >> 4;
        switch (Id){
            // [ToDo] What's up with 0x1?
            case 0x0: Id = 0; break;
            case 0x2: Id = 1; break;
            case 0x3: Id = 2; break;
            case 0x4: Id = 3; break;
            case 0x5: Id = 4; break;
        }
    }
    // Temperatures
    if((msg.id & 0xFF0) == 0x170){
        CMU = (msg.id & 0x00F) + 1;
    }
}

uint8_t BMSManager::BMW_CRC(CAN_message_t &msg, byte Id){
    const uint8_t finalxor[12] = {0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C};
    unsigned char canmes [11];
    int meslen = msg.len + 1; //remove one for crc and add two for id bytes
    canmes [1] = msg.id;
    canmes [0] = msg.id >> 8;

    for (int i = 0; i < (msg.len - 1); i++){canmes[i + 2] = msg.buf[i];}
    return (crc8.get_crc8(canmes, meslen, finalxor[Id]));
}

//void printAllCSV(unsigned long timestamp, float current, int SOC){}
uint16_t BMSManager::getHighCellVolt(){return HighCellVolt;}
uint16_t BMSManager::getLowCellVolt(){return LowCellVolt;}
int16_t BMSManager::getHighTemperature(){return highTemp;}
int16_t BMSManager::getLowTemperature(){return lowTemp;}
uint32_t BMSManager::getPackVoltage(){return packVolt;}
uint16_t BMSManager::getAvgCellVolt(){return AvgCellVolt;}
int16_t BMSManager::getAvgTemperature(){return AvgTemp;}

void BMSManager::clearModules(){
    for (int moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
        if (modules[moduleNr].isExisting()){
            modules[moduleNr].clearModule();
            modules[moduleNr].setExists(false);
        }
    }    
}

byte BMSManager::getBalancingCells(byte moduleNr){
  if (modules[moduleNr].isExisting()){return balancingCells[moduleNr];}
  else{return 254;}    
}

uint16_t BMSManager::getBalancing(){
    uint16_t CellsNo = 0;
    for (byte moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++)
    {
        if(modules[moduleNr].isExisting()){
            // count number of bits set in the balancing mask for each module
            byte mask = balancingCells[moduleNr];
            while(mask)
            {
                CellsNo += mask & 1;
                mask >>= 1;
            }
        }
        
    }
    return CellsNo;
}

byte BMSManager::getNumModules(){
    numFoundModules = 0;
    for (byte moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
        if (modules[moduleNr].isExisting()){numFoundModules++;}
    }
    return numFoundModules;
}

uint16_t BMSManager::getSeriesCells(){
    uint16_t spack = 0;
    for (int moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
        if (modules[moduleNr].isExisting()){spack += modules[moduleNr].getscells();}
    }
    return spack;
}

void BMSManager::clearFaults(){
    switch (BMSType){
        case BMS_Tesla:
            Tesla_clearFaults();
        break;
        
        default: break;
    }
}

CAN_Struct BMSManager::Balancing(uint16_t BalHys, bool active){
    CAN_Struct BalanceMatrix;
    BalanceMatrix = clearCANStruct();

    // check if all messages for module values have been received. If not, do not balance!
    if(active && moduleReadCnt != getNumModules()){
        activeSerial->printf(" not all messages recieved! %i/%i",moduleReadCnt,numFoundModules);
        return BalanceMatrix;
    }

    for (byte moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){balancingCells[moduleNr] = 0;}
    
    switch (BMSType){
        case BMS_VW_eGolf:
        case BMS_VW_MEB:
            BalanceMatrix = VW_Balancing(BalHys, active);
        break;
        case BMS_BMW_MiniE:
        case BMS_BMW_I3:
        break;
        case BMS_Tesla:
            Tesla_Balancing(BalHys, active);
        break;

        default: break;
    }    
    return BalanceMatrix;
}

void BMSManager::Tesla_Balancing(uint16_t BalHys,bool active){
    
    uint8_t balance_Bitmask = 0; //bit 0 - 5 are to activate cell balancing 1-6 etc.
    for (byte moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){ 
        if(modules[moduleNr].isExisting()){
            if (active){
                balance_Bitmask = 0;
                for (byte cellNr = 0; cellNr < MAX_CELL_No_Tesla; cellNr++)
                {if (getLowCellVolt() + BalHys < modules[moduleNr].getCellVoltage(cellNr)){balance_Bitmask = balance_Bitmask | (1 << cellNr);}}
                if (balance_Bitmask){
                    modules[moduleNr].balanceModule(balance_Bitmask);
                    balancingCells[moduleNr] = balance_Bitmask;
                }
            } else {
                modules[moduleNr].stopBalance();
                balancingCells[moduleNr] = 0;
            }
        }
    }  
}

CAN_Struct BMSManager::VW_Balancing(uint16_t BalHys,bool active){
    CAN_Struct BalanceMatrix;
    BalanceMatrix = clearCANStruct();
    uint8_t balance_Bitmask = 0; //bit 0 - 5 are to activate cell balancing 1-6 etc.
    // set balance IDs
    uint32_t BalanceIDs[24];
    BalanceIDs[0] = 0x1A55540A;
    BalanceIDs[1] = 0x1A55540B;
    BalanceIDs[2] = 0x1A55540C;
    BalanceIDs[3] = 0x1A55540D;
    BalanceIDs[4] = 0x1A55540E;
    BalanceIDs[5] = 0x1A55540F;
    BalanceIDs[6] = 0x1A555410;
    BalanceIDs[7] = 0x1A555411;
    BalanceIDs[8] = 0x1A555412;
    BalanceIDs[9] = 0x1A555413;
    BalanceIDs[10] = 0x1A555414;
    BalanceIDs[11] = 0x1A555415;
    BalanceIDs[12] = 0x1A555416;
    BalanceIDs[13] = 0x1A555417;
    BalanceIDs[14] = 0x1A555418;
    BalanceIDs[15] = 0x1A555419;
    BalanceIDs[16] = 0x1A55541A;
    BalanceIDs[17] = 0x1A55541B;
    BalanceIDs[18] = 0x1A5554AB;
    BalanceIDs[19] = 0x1A5554AC;
    BalanceIDs[20] = 0x1A5554AD;
    BalanceIDs[21] = 0x1A5554AE;
    BalanceIDs[22] = 0x1A5554AF;
    BalanceIDs[23] = 0x1A5554B0;

    // fill matrix
    for (byte moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
        if (modules[moduleNr].isExisting()){
            balance_Bitmask = 0;
            byte MessageNr = 0;

            for (byte i = 0; i < 12; i++){
                if ((LowCellVolt + BalHys) < modules[moduleNr].getCellVoltage(i)){balance_Bitmask = balance_Bitmask | (1 << i);}
            }
            
            if (active)  {
                // look for first empty message in BelanceMatrix
                byte MessageNr;
                for (MessageNr = 0; MessageNr < CAN_Struct_size; MessageNr++){
                    if(BalanceMatrix.Frame[MessageNr].id == 0) break;
                }

                // Frame 1: Balancing for first 8 Cells ON (0x08) or OFF (0x00)
                // even IDs [2 * y - 2] i.e. 2*1-2 = 0, 2*2-2 = 2 ...
                BalanceMatrix.Frame[MessageNr].id = BalanceIDs[2 * moduleNr - 2]; 
                for (byte i = 0; i < 8; i++){
                    if (bitRead(balance_Bitmask, i) == 1){
                        BalanceMatrix.Frame[MessageNr].buf[i] = 0x08;
                    }
                }

                // Frame 2: Balancing for last 4 Cells ON (0x08) or OFF (0x00)
                // odd IDs [2 * y - 1] i.e. 2*1-2 = 1, 2*2-2 = 3 ...
                BalanceMatrix.Frame[MessageNr+1].id = BalanceIDs[2 * moduleNr - 1]; 
                for (byte i = 0; i < 4; i++){
                    if (bitRead(balance_Bitmask, i) == 1){
                        BalanceMatrix.Frame[MessageNr+1].buf[i] = 0x08;
                    }
                }
            }
            // unused Bytes
            BalanceMatrix.Frame[MessageNr].buf[4] = 0xFE;
            BalanceMatrix.Frame[MessageNr].buf[5] = 0xFE;
            BalanceMatrix.Frame[MessageNr].buf[6] = 0xFE;
            BalanceMatrix.Frame[MessageNr].buf[7] = 0xFE;
        }
    }        
    return BalanceMatrix;
}

CAN_Struct BMSManager::BMW_Balancing(uint16_t BalHys,bool active){
    CAN_Struct BalanceMatrix;
    BalanceMatrix = clearCANStruct();
    uint8_t balance_Bitmask = 0; //bit 0 - 5 are to activate cell balancing 1-6 etc.
}

void BMSManager::Tesla_renumberModulesIDs(){
    Tesla_resetModules();
    Tesla_setupModules();
}

void BMSManager::Tesla_setupModules(){
    uint8_t payload[3];
    uint8_t buff[10];
    int retLen;
    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 1;

    while (1 == 1){
        payload[0] = 0;
        payload[1] = 0;
        payload[2] = 1;
        retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
        if (retLen == 4){
            if (buff[0] == 0x80 && buff[1] == 0 && buff[2] == 1){
                //look for a free address to use
                for (int moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
                    if (!modules[moduleNr].isExisting()){
                        payload[0] = 0;
                        payload[1] = REG_ADDR_CTRL;
                        payload[2] = moduleNr | 0x80;
                        BMSUtil::sendData(payload, 3, true);
                        delay(3);
                        if (BMSUtil::getReply(buff, 10) > 2){
                            if (buff[0] == (0x81) && buff[1] == REG_ADDR_CTRL && buff[2] == (moduleNr + 0x80)){ modules[moduleNr].setExists(true); }
                        }
                        break; //quit the for loop
                    }
                }
            } else break; //nobody responded properly to the zero address so our work here is done.
        } else break;
    }
}

/*
  Puts all boards on the bus into a Sleep state, very good to use when the vehicle is a rest state.
  Pulling the boards out of sleep only to check voltage decay and temperature when the contactors are open.
*/
void BMSManager::Tesla_sleepModules(){
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_IO_CTRL;//IO ctrl start
    payload[2] = 0x04;//write sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

/*
  Wakes all the boards up and clears thier SLEEP state bit in the Alert Status Registery
*/
void BMSManager::Tesla_wakeModules(){
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_IO_CTRL;//IO ctrl start
    payload[2] = 0x00;//write sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);

    payload[0] = 0x7F; //broadcast
    payload[1] = REG_ALERT_STATUS;//Fault Status
    payload[2] = 0x04;//data to cause a reset
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

void BMSManager::Tesla_findModules(){
    uint8_t payload[3];
    uint8_t buff[8];

    payload[0] = 0;
    payload[1] = 0; //read registers starting at 0
    payload[2] = 1; //read one byte
    for (int moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
        modules[moduleNr].setExists(false);
        payload[0] = moduleNr << 1;
        BMSUtil::sendData(payload, 3, false);
        delay(20);
        if (BMSUtil::getReply(buff, 8) > 4){
            if (buff[0] == (moduleNr << 1) && buff[1] == 0 && buff[2] == 1 && buff[4] > 0) {modules[moduleNr].setExists(true);}
        }
        delay(5);
    }
}

void BMSManager::Tesla_clearFaults(){
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_ALERT_STATUS;//Alert Status
    payload[2] = 0xFF;//data to cause a reset
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[1] = REG_FAULT_STATUS;//Fault Status
    payload[2] = 0xFF;//data to cause a reset
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    //isFaulted = false;
}

void BMSManager::Tesla_resetModules(){
    uint8_t payload[3];
    uint8_t buff[8];
    int attempts = 1;

    for (byte moduleNr = 1; moduleNr <= MAX_MODULE_ADDR; moduleNr++){
        modules[moduleNr].setExists(false);
        numFoundModules = 0;
    }

    while (attempts < 3){
        payload[0] = 0x3F << 1; //broadcast the reset command
        payload[1] = 0x3C; //reset
        payload[2] = 0xA5; //data to cause a reset
        BMSUtil::sendData(payload, 3, true);
        delay(100);
        BMSUtil::getReply(buff, 8);
        if (buff[0] == 0x7F && buff[1] == 0x3C && buff[2] == 0xA5 && buff[3] == 0x57) break;
        attempts++;
    }
}

CAN_Struct BMSManager::clearCANStruct(){
    CAN_Struct msg;
    for(byte i = 0; i < CAN_Struct_size; i++){
        msg.Frame[i].flags.extended = 0;
        msg.Frame[i].len = 0;
        msg.Frame[i].id = 0;
        for (byte x = 0; x < 8; x++){
            msg.Frame[i].buf[x] = 0;
        }
    }
    return msg;
}