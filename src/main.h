#ifndef MAIN_H
#define MAIN_H

#define IO15 15
#define LED_PORT IO15
#define LED_ON digitalWrite(LED_PORT,LOW)
#define LED_OFF digitalWrite(LED_PORT,HIGH)
#define LED_TOGGLE digitalRead(LED_PORT) == HIGH ? LED_ON : LED_OFF

#define HOLE_CT_A 600.0
#define HOLE_CT_V 4000.0

#define MAX_AMPERE_FILTER_FACTOR 0.1

#define HOLE_CT_RATIO (HOLE_CT_A/HOLE_CT_V)
typedef struct
{
  uint8_t EpromValidStart;
  uint8_t InstalledCells;
  uint8_t ModbusAddress;
  uint8_t WEBSERVERPORT;
  uint16_t Max1161_RefVolt;
  int16_t Max1161_CellOffset;
  uint16_t Max1161_CellGain;
  uint32_t BAUDRATE;
  uint32_t IPADDRESS;
  uint32_t GATEWAY;
  uint32_t SUBNETMASK;
  uint16_t UseHoleCT;
  int16_t TempOffset;
  int16_t AmpereOffset;
  uint16_t AmpereGain;
  uint16_t TotalVoltageOffset;
  uint16_t TotalVoltageGain;
  uint8_t Reserved8;
  char SSID[32];
  char PASS[32];
  uint8_t EpromValidEnd;
} nvmSystemSet;
extern nvmSystemSet nvmSet;
#endif