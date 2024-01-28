//
//CANOpen implementation
//

#include "CANOpen.h"

// IntervalTimer Timer_CO_SYNC;
// Timer_CO_SYNC.begin(CO_SYNC,100000);

// NMT messages to control CANOpen nodes.
bool CO_NMT(uint32_t CO_Target_ID, uint8_t CO_Target_State){
  outMsg.id = 0x000;
  outMsg.len = 2; //1Byte requested State, 1Byte addressed Node
  if (CO_Target_State & 0x83){ //check for valid State values
    //0x01 Start Node, 0x02 Stop Node, 0x80 Pre-Operational, 0x81 reset application, 0x82 reset communication
    outMsg.buf[0] = CO_Target_State;
    outMsg.buf[1] = CO_Target_ID;
    can1.write(outMsg);
    return 1;
  }else{return 0;}
}

// SYNC message to sync devices & trigger PDOs from servers
void CO_SYNC(){
  CAN_message_t SyncMSG;
  if(settings.mctype == Curtis){
    SyncMSG.id = 0x80;
    SyncMSG.len = 0;
    can1.write(SyncMSG);
  //  CO_PDO1_send(0x26);
  //  CO_PDO2_send(0x26);
    //if(debug_CAN1){CAN_Debug_OUT();}
  }
}

void CO_PDO1_send(uint32_t CO_Target_ID){
  outMsg.id = 0x200 + CO_Target_ID;
  outMsg.len = 8;
  //msg.rtr = 1;
  outMsg.buf[0] = 0x00;
  outMsg.buf[1] = 0x00;
  outMsg.buf[2] = 0x00;
  outMsg.buf[3] = 0x00;
  outMsg.buf[4] = 0x00;
  outMsg.buf[5] = 0x00;
  outMsg.buf[6] = 0x00;
  outMsg.buf[7] = 0x00;
  can1.write(outMsg);
  if(debug_CAN1){CAN_Debug_OUT();}
}

void CO_PDO2_send(uint32_t CO_Target_ID){
  outMsg.id = 0x300 + CO_Target_ID;
  outMsg.len = 8;
  //msg.rtr = 1;
  outMsg.buf[0] = 0x00;
  outMsg.buf[1] = 0x00;
  outMsg.buf[2] = 0x00;
  outMsg.buf[3] = 0x00;
  outMsg.buf[4] = 0x00;
  outMsg.buf[5] = 0x00;
  outMsg.buf[6] = 0x00;
  outMsg.buf[7] = 0x00;
  can1.write(outMsg);
  if(debug_CAN1){CAN_Debug_OUT();}

}

void CO_SDO_send_test(uint32_t CO_Target_ID){
  
  outMsg.buf[4] = 0x00;
  outMsg.buf[5] = 0x00;
  outMsg.buf[6] = 0x00;
  outMsg.buf[7] = 0x00;
  /*
  CO_Send_SDO(0x26,1,1,0x1800,0x02,1);
  */
  CO_Send_SDO(CO_Target_ID,1,0,0x1800,0x02,1);
 
  //CO_PDO1_send(CO_Target_ID);
  //CO_PDO2_send(CO_Target_ID);
  //CO_Send_SDO(CO_Target_ID,2,1,0x1400,0x00);
}

/*
Send SDOs to request or write single Data from / to the Object Dictionary
CO_Target_ID: NodeId (0x00-0xFF)
CCS: 1 - write, 2 - read
expedited: 1 - send/receive in one Frame, 0 - segmented
index: OD index
subindex: OD subindex
datasize: how many Bytes to write/send (0 - 4)
*/
void CO_Send_SDO(uint32_t CO_Target_ID, uint8_t CCS, uint8_t expedited, uint16_t index, uint8_t subindex, uint8_t datasize){
  //[ToDo] Funktionsaufruf um Übergabe für Byte 4-7 erweitern?
  uint8_t command_byte = 0;
  outMsg.id = 0x600 + CO_Target_ID; //COB-ID
  outMsg.len = 8;

  if(datasize > 0 && datasize < 5){
    //Datasize: 1Byte = 11b, 2Byte = 10b, 3Byte = 01b, 4Byte = 00b
    datasize = 4 - datasize;
    command_byte |= datasize << 2; //set datasize
    command_byte |= 1; //set datasize shown
  }else{datasize = 0;}

  if (CCS == 1 || CCS == 2){
    //1 write,Download 2 read/upload
    command_byte |= CCS << 5;
  } else {return;}
  
  if (expedited){
    //activate single-frame transer
    command_byte |= 2;
  }
  outMsg.buf[0] = command_byte;
  outMsg.buf[1] = lowByte(index);
  outMsg.buf[2] = highByte(index);
  outMsg.buf[3] = subindex;
  outMsg.buf[4] = 0x00; // lowest Data, first Byte
  outMsg.buf[5] = 0x00;
  outMsg.buf[6] = 0x00;
  outMsg.buf[7] = 0x00; // highest Data, Last byte
  can1.write(outMsg);
  if(debug_CAN1){CAN_Debug_OUT();}
}

// Handle incomming CANOpen messages
// returns sender CAN-ID
uint32_t CO_Handle(CAN_message_t MSG){
  // Tx Read Data Server -> Client
  // Rx write Data Client -> Server

  if (MSG.id & 0x580){
    //SDO Tx: eg. reply to SDO Rx
  }
  if (MSG.id & 0x600){
    //SDO Rx: see CO_Send_SDO
  }
  if (MSG.id & 0x180){
    //PDO1 Tx
  }
  if (MSG.id & 0x200){
    //PDO1 Rx
  }
  if (MSG.id & 0x280){
    //PDO2 Tx
  }  
  if (MSG.id & 0x300){
    //PDO2 Rx
  }
  if (MSG.id & 0x380){
    //PDO3 Tx
  }
  if (MSG.id & 0x400){
    //PDO3 Rx
  }      
  if (MSG.id & 0x480){
    //PDO4 Tx
  }
  if (MSG.id & 0x500){
    //PDO4 Rx
  }    
  if (MSG.id & 0x700){
    //NMT Heartbeats / Status
    if(MSG.buf[0] == 0x00){}//boot: just wait
    if(MSG.buf[0] == 0x04){CO_NMT(0x00, co_state_preop);}//stopped: bring into pre-operational
    if(MSG.buf[0] == 0x05){return (MSG.id & ~0x700);}//operational: [ToDo] Do nothing? Send SYNC/rPDO in interval?
    if(MSG.buf[0] == 0x7f){CO_NMT(0x00, co_NMT_operational);}//pre-operational: bring into operational
  }  
  
  return 0;
}
