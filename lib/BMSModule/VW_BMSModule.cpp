#include "config.h"
#include "VW_BMSModule.h"


VW_BMSModule::VW_BMSModule()
{
  for (int i = 0; i < 13; i++){
    cellVolt[i] = 0.0f;
    lowestCellVolt[i] = 5.0f;
    highestCellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  temperatures[2] = 0.0f;
  lowestTemperature = 200.0f;
  highestTemperature = -100.0f;
  lowestModuleVolt = 200.0f;
  highestModuleVolt = 0.0f;
  balstat = 0;
  exists = false;
  reset = false;
  moduleAddress = 0;
  timeout = 30000; //milliseconds before comms timeout;
}

void VW_BMSModule::clearmodule(){
  for (int i = 0; i < 13; i++){
    cellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  temperatures[2] = 0.0f;
  balstat = 0;
  exists = false;
  reset = false;
  moduleAddress = 0;
}

void VW_BMSModule::decodetemp(CAN_message_t &msg)
{
  if (msg.buf[7] == 0xFD){
    if (msg.buf[2] != 0xFD){
      temperatures[0] = (msg.buf[2] * 0.5) - 40;
    }
  }
  else{
    if (msg.buf[0] < 0xDF){
      temperatures[0] = (msg.buf[0] * 0.5) - 43;
      balstat = msg.buf[2] + (msg.buf[3] << 8);
    }
    else{
      temperatures[0] = (msg.buf[3] * 0.5) - 43;
    }
    if (msg.buf[4] < 0xF0){
      temperatures[1] = (msg.buf[4] * 0.5) - 43;
    }
    else{
      temperatures[1] = 0;
    }
    if (msg.buf[5] < 0xF0){
      temperatures[2] = (msg.buf[5] * 0.5) - 43;
    }
    else{
      temperatures[2] = 0;
    }
  }

}

void VW_BMSModule::decodecan(int Id, CAN_message_t &msg)
{
  switch (Id)
  {
    case 0:
      cmuerror = 0;
      cellVolt[0] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001;
      cellVolt[2] = (uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001;
      cellVolt[1] = (msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001;
      cellVolt[3] = (msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001;

      /*
        if (float((uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001) > 0 && float((uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001) < cellVolt[0] + VoltDelta && float((uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001) > cellVolt[0] - VoltDelta || cellVolt[0] == 0)
        {
          cellVolt[0] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001;
        }
        else
        {
          cmuerror = 1;
        }
        if (float((uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001) > 0 && float((uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001) <  cellVolt[2] + VoltDelta && float((uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001) > cellVolt[2] - VoltDelta || cellVolt[2] == 0)
        {
          cellVolt[2] = (uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001;
        }
        else
        {
          cmuerror = 1;
        }
        if (float((msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001) > 0 && float((msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001) < cellVolt[1] + VoltDelta && float((msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001) > cellVolt[1] - VoltDelta || cellVolt[1] == 0)
        {
          cellVolt[1] = (msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001;
        }
        else
        {
          cmuerror = 1;
        }
        if (float((msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001) > 0 && float((msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001) < cellVolt[3] + VoltDelta && float((msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001) > cellVolt[3] - VoltDelta || cellVolt[3] == 0)
        {
          cellVolt[3] = (msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001;
        }
        else
        {
          cmuerror = 1;
        }
      */
      break;
    case 1:
      cmuerror = 0;
      cellVolt[4] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001;
      cellVolt[6] = (uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001;
      cellVolt[5] = (msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001;
      cellVolt[7] = (msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001;
      /*
        if (float((uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001) > 0 && float((uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001) < cellVolt[4] + VoltDelta && float((uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001) > cellVolt[4] - VoltDelta || cellVolt[4] == 0)
        {
        cellVolt[4] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001;
        }
        else
        {
        cmuerror = 1;
        }
        if (float((uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001) > 0 && float((uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001) <  cellVolt[6] + VoltDelta && float((uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001) > cellVolt[6] - VoltDelta || cellVolt[6] == 0)
        {
        cellVolt[6] = (uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001;
        }
        else
        {
        cmuerror = 1;
        }
        if (float((msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001) > 0 && float((msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001) < cellVolt[5] + VoltDelta && float((msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001) > cellVolt[5] - VoltDelta || cellVolt[5] == 0)
        {
        cellVolt[5] = (msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001;
        }
        else
        {
        cmuerror = 1;
        }
        if (float((msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001) > 0 && float((msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001) < cellVolt[7] + VoltDelta && float((msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001) > cellVolt[7] - VoltDelta || cellVolt[7] == 0)
        {
        cellVolt[7] = (msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001;
        }
        else
        {
        cmuerror = 1;
        }
      */
      break;

    case 2:
      cmuerror = 0;
      cellVolt[8] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001;
      cellVolt[10] = (uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001;
      cellVolt[9] = (msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001;
      cellVolt[11] = (msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001;
      /*
        if (float((uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001) > 0 && float((uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001) < cellVolt[8] + VoltDelta && float((uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001) > cellVolt[8] - VoltDelta || cellVolt[8] == 0)
        {
        cellVolt[8] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001;
        }
        else
        {
        cmuerror = 1;
        }

        if (float((uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001) > 0 && float((uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001) <  cellVolt[10] + VoltDelta && float((uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001) > cellVolt[10] - VoltDelta || cellVolt[10] == 0)
        {
        cellVolt[10] = (uint16_t(msg.buf[5] << 4) + uint16_t(msg.buf[4] >> 4) + 1000) * 0.001;
        }
        else
        {
        cmuerror = 1;
        }

        if (float((msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001) > 0 && float((msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001) < cellVolt[9] + VoltDelta && float((msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001) > cellVolt[9] - VoltDelta || cellVolt[9] == 0)
        {
        cellVolt[9] = (msg.buf[3] + uint16_t((msg.buf[4] & 0x0F) << 8) + 1000) * 0.001;
        }
        else
        {
        cmuerror = 1;
        }
        if (float((msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001) > 0 && float((msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001) < cellVolt[11] + VoltDelta && float((msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001) > cellVolt[11] - VoltDelta || cellVolt[11] == 0)
        {
        cellVolt[11] = (msg.buf[6] + uint16_t((msg.buf[7] & 0x0F) << 8) + 1000) * 0.001;
        }
        else
        {
        cmuerror = 1;
        }
      */
      break;

    case 3:
      cmuerror = 0;
      cellVolt[12] = (uint16_t(msg.buf[1] >> 4) + uint16_t(msg.buf[2] << 4) + 1000) * 0.001;
      break;

    default:
      break;

  }
  if (getLowTemp() < lowestTemperature) lowestTemperature = getLowTemp();
  if (getHighTemp() > highestTemperature) highestTemperature = getHighTemp();

  for (int i = 0; i < 13; i++)
  {
    if (lowestCellVolt[i] > cellVolt[i] && cellVolt[i] >= IgnoreCell) lowestCellVolt[i] = cellVolt[i];
    if (highestCellVolt[i] < cellVolt[i] && cellVolt[i] > 5.0) highestCellVolt[i] = cellVolt[i];
  }

  if (cmuerror == 0)
  {
    lasterror = millis();
  }
  else
  {
    if (millis() - lasterror < timeout)
    {
      if (lasterror + timeout - millis() < 5000)
      {
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("Module");
        SERIALCONSOLE.print(moduleAddress);
        SERIALCONSOLE.print("Counter Till Can Error : ");
        SERIALCONSOLE.println(lasterror + timeout - millis() );
      }
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        cellVolt[i] = 0.0f;
      }
      moduleVolt = 0.0f;
      temperatures[0] = 0.0f;
      temperatures[1] = 0.0f;
      temperatures[2] = 0.0f;
    }
  }
}


/*
  Reading the status of the board to identify any flags, will be more useful when implementing a sleep cycle
*/

uint8_t VW_BMSModule::getFaults()
{
  return faults;
}

uint8_t VW_BMSModule::getAlerts()
{
  return alerts;
}

uint8_t VW_BMSModule::getCOVCells()
{
  return COVFaults;
}

uint8_t VW_BMSModule::getCUVCells()
{
  return CUVFaults;
}

float VW_BMSModule::getCellVoltage(int cell)
{
  if (cell < 0 || cell > 13) return 0.0f;
  return cellVolt[cell];
}

float VW_BMSModule::getLowCellV()
{
  float lowVal = 10.0f;
  for (int i = 0; i < 13; i++) if (cellVolt[i] < lowVal && cellVolt[i] > IgnoreCell) lowVal = cellVolt[i];
  return lowVal;
}

float VW_BMSModule::getHighCellV()
{
  float hiVal = 0.0f;
  for (int i = 0; i < 13; i++)
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 5.0)
    {
      if (cellVolt[i] > hiVal) hiVal = cellVolt[i];
    }
  return hiVal;
}

float VW_BMSModule::getAverageV()
{
  int x = 0;
  float avgVal = 0.0f;
  for (int i = 0; i < 13; i++)
  {
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 5.0)
    {
      x++;
      avgVal += cellVolt[i];
    }
  }

  scells = x;
  avgVal /= x;

  if (scells == 0)
  {
    avgVal = 0;
  }

  return avgVal;
}

int VW_BMSModule::getscells(){
  return scells;
}

float VW_BMSModule::getHighestModuleVolt(){
  return highestModuleVolt;
}

float VW_BMSModule::getLowestModuleVolt(){
  return lowestModuleVolt;
}

float VW_BMSModule::getHighestCellVolt(int cell){
  if (cell < 0 || cell > 13) return 0.0f;
  return highestCellVolt[cell];
}

float VW_BMSModule::getLowestCellVolt(int cell){
  if (cell < 0 || cell > 13) return 0.0f;
  return lowestCellVolt[cell];
}

float VW_BMSModule::getHighestTemp(){
  return highestTemperature;
}

float VW_BMSModule::getLowestTemp(){
  return lowestTemperature;
}

float VW_BMSModule::getLowTemp(){
  if (sensor == 0){
    if (getAvgTemp() > 0.5){
      if (temperatures[0] > 0.5){
        if (temperatures[0] < temperatures[1] && temperatures[0] < temperatures[2]){
          return (temperatures[0]);
        }
      }
      if (temperatures[1] > 0.5){
        if (temperatures[1] < temperatures[0] && temperatures[1] < temperatures[2]){
          return (temperatures[1]);
        }
      }
      if (temperatures[2] > 0.5){
        if (temperatures[2] < temperatures[1] && temperatures[2] < temperatures[0]){
          return (temperatures[2]);
        }
      }
    }
  }
  else{
    return temperatures[sensor - 1];
  }
}

float VW_BMSModule::getHighTemp(){
  if (sensor == 0){
    return (temperatures[0] < temperatures[1]) ? temperatures[1] : temperatures[0];
  } else {
    return temperatures[sensor - 1];
  }
}

float VW_BMSModule::getAvgTemp(){
  if (sensor == 0){
    if ((temperatures[0] + temperatures[1] + temperatures[2]) / 3.0f > 0.5){
      if (temperatures[0] > 0.5 && temperatures[1] > 0.5 && temperatures[2] > 0.5){
        return (temperatures[0] + temperatures[1] + temperatures[2]) / 3.0f;
      }
      if (temperatures[0] < 0.5 && temperatures[1] > 0.5 && temperatures[2] > 0.5){
        return (temperatures[1] + temperatures[2]) / 2.0f;
      }
      if (temperatures[0] > 0.5 && temperatures[1] < 0.5 && temperatures[2] > 0.5){
        return (temperatures[0] + temperatures[2]) / 2.0f;
      }
      if (temperatures[0] > 0.5 && temperatures[1] > 0.5 && temperatures[2] < 0.5){
        return (temperatures[0] + temperatures[1]) / 2.0f;
      }
      if (temperatures[0] > 0.5 && temperatures[1] < 0.5 && temperatures[2] < 0.5){
        return (temperatures[0]);
      }
      if (temperatures[0] < 0.5 && temperatures[1] > 0.5 && temperatures[2] < 0.5){
        return (temperatures[1]);
      }
      if (temperatures[0] < 0.5 && temperatures[1] < 0.5 && temperatures[2] > 0.5){
        return (temperatures[2]);
      }
      if (temperatures[0] < 0.5 && temperatures[1] < 0.5 && temperatures[2] < 0.5){
        return (-80);
      }
    }
  } else {
    return temperatures[sensor - 1];
  }
}

float VW_BMSModule::getModuleVoltage(){
  moduleVolt = 0;
  for (int I = 0; I < 13; I++){
    if (cellVolt[I] > IgnoreCell && cellVolt[I] < 5.0){
      moduleVolt = moduleVolt + cellVolt[I];
    }
  }
  return moduleVolt;
}

float VW_BMSModule::getTemperature(int temp){
  if (temp < 0 || temp > 2) return 0.0f;
  return temperatures[temp];
}

void VW_BMSModule::setAddress(int newAddr){
  if (newAddr < 0 || newAddr > MAX_MODULE_ADDR) return;
  moduleAddress = newAddr;
}

int VW_BMSModule::getAddress(){
  return moduleAddress;
}

int VW_BMSModule::getBalStat(){
  return balstat;
}

bool VW_BMSModule::isExisting(){
  return exists;
}

bool VW_BMSModule::isReset(){
  return reset;
}

void VW_BMSModule::settempsensor(int tempsensor){
  sensor = tempsensor;
}

void VW_BMSModule::setExists(bool ex){
  exists = ex;
}

void VW_BMSModule::setDelta(float ex){
  VoltDelta = ex;
}

void VW_BMSModule::setReset(bool ex){
  reset = ex;
}

void VW_BMSModule::setIgnoreCell(float Ignore){
  IgnoreCell = Ignore;
  Serial.println();
  Serial.println();
  Serial.println(Ignore);
  Serial.println();
}
