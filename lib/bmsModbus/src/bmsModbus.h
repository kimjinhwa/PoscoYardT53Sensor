#ifndef BMS_MODBUS_H
#define BMS_MODBUS_H

#include <ModbusServerRTU.h>
#include <HardwareSerial.h>
#include <driver/uart.h>


#endif // BMS_MODBUS_H
class BmsModbus
{
    public:
    BmsModbus()
    {
    };
    ~BmsModbus(){
    };
    static ModbusMessage FC03(ModbusMessage request);
    static ModbusMessage FC04(ModbusMessage request);
    static ModbusMessage FC06(ModbusMessage request);
    static void FC06_Broadcast(ModbusMessage request);
    static void updateModbusAddress(uint8_t newAddress);
    void setup();
};
extern BmsModbus bmsModbus;

