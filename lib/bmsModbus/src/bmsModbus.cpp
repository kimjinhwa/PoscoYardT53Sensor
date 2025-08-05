#include "bmsModbus.h"
#include "max14921.h"
#include "EEPROM.h"
#include "../../src/main.h"
#include "../../version.h"
extern max14921 _max14921;
ModbusServerRTU MBserver(2000,GPIO_NUM_33);
uint16_t sendBuffer[100];
void getVersion(int16_t *desc){
  String ver = String(VERSION);
  int16_t firstDot = ver.indexOf(".");
  int16_t secondDot = ver.indexOf(".",firstDot+1);
  int16_t underScore= ver.indexOf("_");
  desc[0] = ver.substring(underScore+1,firstDot).toInt();
  desc[1] = ver.substring(firstDot+1,secondDot).toInt();
  desc[2] = ver.substring(secondDot+1).toInt();


}
extern MD_AK35_INSTANCE_T   MD_AK35_obj;
ModbusMessage BmsModbus::FC03(ModbusMessage request)
{
  uint16_t address;           // requested register address
  uint16_t words;             // requested number of registers
  ModbusMessage response;     // response message to be sent back
  int16_t firmVersion[3];
  getVersion(firmVersion);
  EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM

  String ver = String(VERSION);

  memset(sendBuffer, 0, sizeof(sendBuffer));
  EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
  sendBuffer[0] = nvmSet.ModbusAddress; // modbus address
  sendBuffer[1] = nvmSet.InstalledCells; // installed cells
  sendBuffer[2] = nvmSet.Max1161_RefVolt;  // reference voltage
  sendBuffer[3] = nvmSet.Max1161_CellGain;  // gain
  sendBuffer[4] = nvmSet.Max1161_CellOffset;  // offset
  sendBuffer[5] = firmVersion[0];  // major version
  sendBuffer[6] = firmVersion[1];  // minor version
  sendBuffer[7] = firmVersion[2];  // patch version
  sendBuffer[8] = _max14921.openWireStatus;
  sendBuffer[9] = nvmSet.UseHoleCT;
  sendBuffer[10] = nvmSet.TempOffset;
  sendBuffer[11] = nvmSet.AmpereOffset;
  sendBuffer[12] = nvmSet.AmpereGain;
  sendBuffer[13] = nvmSet.TotalVoltageOffset;
  sendBuffer[14] = nvmSet.TotalVoltageGain;
  sendBuffer[15] = MD_AK35_obj.modbusBalanceC01_C08;
  sendBuffer[16] = MD_AK35_obj.modbusBalanceC09_C16;
  sendBuffer[17] = MD_AK35_obj.modbusCanIBalance;
  sendBuffer[18] = MD_AK35_obj.balanceTargetVoltage;
  // get request values
  request.get(2, address);
  request.get(4, words);

  // Address and words valid? We assume 10 registers here for demo
  if (words && (address + words) < 100) {
    // Looks okay. Set up message with serverID, FC and length of data
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
    // Fill response with requested data
    for (uint16_t i = address; i < address + words; ++i) {
      response.add(sendBuffer[i]);
    }
  } else {
    // No, either address or words are outside the limits. Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}
ModbusMessage BmsModbus::FC04(ModbusMessage request)
{
  uint16_t address;           // requested register address
  uint16_t words;             // requested number of registers
  ModbusMessage response;     // response message to be sent back

  if (xSemaphoreTake(max14921::dataMutex, portMAX_DELAY) == pdPASS)
  {
    for (int i = 0; i < nvmSet.InstalledCells; i++)
    {
      sendBuffer[i] = (1000.0 * _max14921.cellVoltage[i] * _max14921.VREF / 65536.0);
    }
    sendBuffer[16] = _max14921.T123[0];
    sendBuffer[17] = _max14921.T123[1];
    sendBuffer[18] = _max14921.T123[2]; // current
    sendBuffer[19] = float(nvmSet.InstalledCells) * 10.0 * _max14921.totalVoltage * _max14921.VREF / 65536.0;
    sendBuffer[20] = (uint16_t)(temperature34.average_temperature_3 * 10);
    sendBuffer[21] = (uint16_t)(temperature34.average_temperature_4 * 10);
    sendBuffer[22] = (uint16_t)(temperature34.average_temperature_in_th * 10);
    xSemaphoreGive(max14921::dataMutex);
  }
  // get request values
  request.get(2, address);
  request.get(4, words);

  // Address and words valid? We assume 10 registers here for demo
  if (words && (address + words) < 100) {
    // Looks okay. Set up message with serverID, FC and length of data
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
    // Fill response with requested data
    for (uint16_t i = address; i < address + words; ++i) {
      response.add(sendBuffer[i]);
    }
  } else {
    // No, either address or words are outside the limits. Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}
ModbusMessage BmsModbus::FC06(ModbusMessage request)
{
  uint16_t address;           // requested register address
  uint16_t words;             // requested number of registers
  ModbusMessage response;     // response message to be sent back
  // get request values
  request.get(2, address);
  request.get(4, words);

  if (address  < 19) {
    // Looks okay. Set up message with serverID, FC and length of data
    switch(address){
      case 0:
        nvmSet.ModbusAddress = words; // modbus address
        updateModbusAddress((uint8_t)words);
        break;
      case 1:
        nvmSet.InstalledCells = words; // installed cells
        break;
      case 2:
        nvmSet.Max1161_RefVolt = words;  // reference voltage
        _max14921.VREF = float(nvmSet.Max1161_RefVolt)/1000.0;
        break;
      case 3:
        nvmSet.Max1161_CellGain = words;  // gain
        break;
      case 4:
        nvmSet.Max1161_CellOffset = words;  // offset
        break;
      case 9:
        nvmSet.UseHoleCT = words;
        break;
      case 10:
        nvmSet.TempOffset = words;
        break;
      case 11:
        nvmSet.AmpereOffset = words;
        break;
      case 12:
        nvmSet.AmpereGain = words;
        break;
      case 13:
        nvmSet.TotalVoltageOffset = words;
        break;
      case 14:
        nvmSet.TotalVoltageGain = words;
        break;
      case 15:
        MD_AK35_obj.modbusBalanceC01_C08 = words;
        break;
      case 16:
        MD_AK35_obj.modbusBalanceC09_C16 = words;
        break;
      case 17:
        MD_AK35_obj.modbusCanIBalance = words;
        break;
      case 18:
        MD_AK35_obj.balanceTargetVoltage = words;
        break;
      default:
        break;
    }
    EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
    EEPROM.commit(); //Commit EEPROM
    EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
    response.add(request.getServerID(), request.getFunctionCode(), address, words);
    // Fill response with requested data
  } else {
    // No, either address or words are outside the limits. Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}
void BmsModbus::FC06_Broadcast(ModbusMessage request)
{
  uint16_t address;           // requested register address
  uint16_t words;             // requested number of registers
  // get request values
  request.get(2, address);
  request.get(4, words);
  if(request.getServerID() == ANY_SERVER){
    Serial.printf("BROADCAST\n");
  }
  if( request.getFunctionCode() != WRITE_HOLD_REGISTER){
    Serial.printf("Not WRITE_HOLD_REGISTER\n");
    return;
  }

  if (address  < 19) {
    // Looks okay. Set up message with serverID, FC and length of data
    switch(address){
      case 0:
        nvmSet.ModbusAddress = words; // modbus address
        updateModbusAddress((uint8_t)words);
        break;
      case 1:
        nvmSet.InstalledCells = words; // installed cells
        break;
      case 2:
        nvmSet.Max1161_RefVolt = words;  // reference voltage
        _max14921.VREF = float(nvmSet.Max1161_RefVolt)/1000.0;
        break;
      case 3:
        nvmSet.Max1161_CellGain = words;  // gain
        break;
      case 4:
        nvmSet.Max1161_CellOffset = words;  // offset
        break;
      case 9:
        nvmSet.UseHoleCT = words;
        break;
      case 10:
        nvmSet.TempOffset = words;
        break;
      case 11:
        nvmSet.AmpereOffset = words;
        break;
      case 12:
        nvmSet.AmpereGain = words;
        break;
      case 13:
        nvmSet.TotalVoltageOffset = words;
        break;
      case 14:
        nvmSet.TotalVoltageGain = words;
        break;
      case 15:
        MD_AK35_obj.modbusBalanceC01_C08 = words;
        break;
      case 16:
        MD_AK35_obj.modbusBalanceC09_C16 = words;
        break;
      case 17:
        MD_AK35_obj.modbusCanIBalance = words;
        break;
      case 18:
        MD_AK35_obj.balanceTargetVoltage = words;
        break;
      default:
        break;
    }
    EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
    EEPROM.commit(); //Commit EEPROM
    EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
  } 
}

void BmsModbus::updateModbusAddress(uint8_t newAddress) {
    // 기존 서비스 중지
    Serial.printf("\nupdateModbusAddress %d",newAddress);
    EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet));
    nvmSet.ModbusAddress = newAddress;
    EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet));
    Serial.println("\nSystem Restart");
    ESP.restart();
    
    // // 새 주소로 서비스 재시작
    // Serial1.begin(nvmSet.BAUDRATE, SERIAL_8N1, GPIO_NUM_26, GPIO_NUM_22);
    
    // // 새 주소로 워커 재등록
    // MBserver.registerWorker(nvmSet.ModbusAddress, READ_HOLD_REGISTER, &FC03);
    // MBserver.registerWorker(nvmSet.ModbusAddress, READ_INPUT_REGISTER, &FC04);
    // MBserver.registerWorker(nvmSet.ModbusAddress, WRITE_HOLD_REGISTER, &FC06);
    
    // // ModbusRTU 다시 시작
    // MBserver.begin(Serial1);
}
void BmsModbus::setup()
{

    Serial1.begin(nvmSet.BAUDRATE, SERIAL_8N1, GPIO_NUM_26, GPIO_NUM_22);

    MBserver.registerBroadcastWorker( &FC06_Broadcast);
    // Register served function code worker for server 1, FC 0x03
    MBserver.registerWorker(nvmSet.ModbusAddress, READ_HOLD_REGISTER, &FC03);
    MBserver.registerWorker(nvmSet.ModbusAddress, READ_INPUT_REGISTER, &FC04);
    MBserver.registerWorker(nvmSet.ModbusAddress, WRITE_HOLD_REGISTER, &FC06);

    // Start ModbusRTU background task
    MBserver.begin(Serial1);
}
