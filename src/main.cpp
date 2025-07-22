#include "Arduino.h"
#include <max14921.h>
#include <SPI.h>
#include "bluetoothtask.h"
#include <HardwareSerial.h>
#include <driver/uart.h>
#include <WebServer.h>
#include <EEPROM.h>
#include "bmsModbus.h"
#include "main.h"
#include <esp_task_wdt.h>
#include "esp32SelfUploder.h"
#include "../../../version.h"
#include <BluetoothSerial.h>
extern BluetoothSerial SerialBT;

#define EVERY_100 100
#define EVERY_200 200
#define EVERY_250 250
#define EVERY_500 500
#define EVERY_1000 1000
#define EVERY_2000 2000

ESP32SelfUploder selfUploder;

TaskHandle_t *h_pxblueToothTask;
max14921 _max14921(SEL_MAX11163 , SEL_MAX14921, SAMPLPIN_MAX14921);

// ThreeWire max11163wire(DATAIN_MISO, SPICLOCK, SEL_MAX11163 );

extern WebServer webServer;
BmsModbus bmsModbus;
nvmSystemSet nvmSet;
long elaspedTime100 = 0;
long elaspedTime250 = 0;
long elaspedTime500 = 0;
long elaspedTime1000 = 0;
long elaspedTime2000 = 0;
void readAndWriteEprom(){
  EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet));
  if (nvmSet.EpromValidStart != 0x55 && nvmSet.EpromValidEnd != 0xAA)
  {
     Serial.println("Initializing default values...");
      memset(&nvmSet, 0, sizeof(nvmSystemSet));
      nvmSet.EpromValidStart = 0x55;
      nvmSet.InstalledCells = 16;
      nvmSet.ModbusAddress = 1;
      nvmSet.WEBSERVERPORT = 80;
      nvmSet.Max1161_RefVolt = 4096;
      nvmSet.Max1161_CellOffset = 0;
      nvmSet.Max1161_CellGain = 1220;
      nvmSet.BAUDRATE = 9600;
      nvmSet.IPADDRESS = (uint32_t)IPAddress(192, 168, 1, 100);
      nvmSet.GATEWAY = (uint32_t)IPAddress(192, 168, 1, 1);
      nvmSet.SUBNETMASK = (uint32_t)IPAddress(255, 255, 255, 0);
      nvmSet.UseHoleCT = 0;
      nvmSet.TempOffset = 4600;
      nvmSet.AmpereOffset = 0; 
      nvmSet.AmpereGain = 1220;
      nvmSet.TotalVoltageOffset = 0;
      nvmSet.TotalVoltageGain = 1220;
      nvmSet.Reserved8 = 0;
      nvmSet.EpromValidEnd = 0xAA;
      EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet));
      EEPROM.commit();
  }
}
void upLoder(){
    selfUploder.setLed(LED_PORT);
    selfUploder.begin("AndroidHotspot1953", "87654321", "https://raw.githubusercontent.com/kimjinhwa/PoscoYardT53Sensor/refs/heads/main/uploadFirmware");
    Serial.println("Booting Sketch...");
    WiFi.mode(WIFI_AP_STA);
    if(strlen(nvmSet.SSID) == 0){
        strncpy(nvmSet.SSID, selfUploder.ssid, sizeof(nvmSet.SSID));
    }
    if(strlen(nvmSet.PASS) == 0){
        strncpy(nvmSet.PASS, selfUploder.password, sizeof(nvmSet.PASS));
    }
    EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet));
    WiFi.begin(nvmSet.SSID, nvmSet.PASS);

    int retry = 0;
    
    while (WiFi.waitForConnectResult() != WL_CONNECTED  && retry  < (5 + nvmSet.ModbusAddress%5) ) {
        esp_task_wdt_reset();
        WiFi.begin(selfUploder.ssid, selfUploder.password);
        delay(100);  // 5초 대기
        Serial.println("WiFi failed, retrying.");
        retry++;
    }
    if(WiFi.isConnected()){
        Serial.println("WiFi connected");
        Serial.println(WiFi.localIP());
        Serial.println(WiFi.subnetMask());
        Serial.println(WiFi.gatewayIP());
        Serial.println(WiFi.dnsIP());
        #if AUTOUPDATE == 1
        Serial.printf("Free heap before SSL: %d\n", ESP.getFreeHeap());
        if (selfUploder.checkNewVersion(selfUploder.update_url)) {
            Serial.println("New version available!");
            if (selfUploder.tryAutoUpdate(selfUploder.updateFile_url.c_str())) {
                return;
            }
        } else {
            Serial.println("Already on latest version");
        }
        #endif
    }
    else{
        Serial.println("WiFi connection failed");
    }   
    // selfUploder.httpUpdater.setup(&selfUploder.httpServer);
    // selfUploder.httpServer.begin();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    Serial.println("\nWiFi Offed");
}
uint16_t timerCount = 0;
void IRAM_ATTR onTimer(){
    timerCount++;
}
void useInterrupt(){
    hw_timer_t *timer = NULL;
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000, true); // 1000us = 1ms
    timerAlarmEnable(timer);
}
void initPin(){
    pinMode(LED_PORT, OUTPUT);
    LED_ON;
    pinMode(SPICLOCK, OUTPUT);
    digitalWrite(SPICLOCK, LOW);

    // ASEL 핀들을 INPUT_PULLUP으로 설정
    pinMode(ASEL1, INPUT_PULLUP);
    pinMode(ASEL2, INPUT_PULLUP);
    pinMode(ASEL3, INPUT_PULLUP);
    
    // 초기 상태 확인을 위한 디버깅
    delay(100); // 핀 상태 안정화 대기
}

temperatureClass temperature34;

void setup()
{
    Serial.begin(115200);
    EEPROM.begin(sizeof(nvmSystemSet));
    readAndWriteEprom();

    _max14921.SetCellNumber(nvmSet.InstalledCells);
    _max14921.VREF = float(nvmSet.Max1161_RefVolt) / 1000.0;
    Serial.printf("\nVersion: %s\n", VERSION);
    Serial.printf("\nInstalledCells: %d\n", nvmSet.InstalledCells);
    Serial.printf("ModbusAddress: %d\n", nvmSet.ModbusAddress);
    Serial.printf("WEBSERVERPORT: %d\n", nvmSet.WEBSERVERPORT);
    Serial.printf("Max1161_RefVolt: %d\n", nvmSet.Max1161_RefVolt);
    Serial.printf("Max1161_CellOffset: %d\n", nvmSet.Max1161_CellOffset);
    Serial.printf("Max1161_CellGain: %d\n", nvmSet.Max1161_CellGain);
    Serial.printf("BAUDRATE: %d\n", nvmSet.BAUDRATE);
    Serial.printf("IPADDRESS: %s\n", IPAddress(nvmSet.IPADDRESS).toString().c_str());
    Serial.printf("GATEWAY: %s\n", IPAddress(nvmSet.GATEWAY).toString().c_str());
    Serial.printf("SUBNETMASK: %s\n", IPAddress(nvmSet.SUBNETMASK).toString().c_str());
    Serial.printf("UseHoleCT Ratio: %d\n", nvmSet.UseHoleCT);
    // Serial1.begin(9600, SERIAL_8N1, 26, 22);
    // Serial1.setPins(26, 22, -1, 33);
    // Serial1.setMode(UART_MODE_RS485_HALF_DUPLEX);
    // SPI.setFrequency(SPI_SPEED );
    // SPI.setBitOrder(LSBFIRST);
    // SPI.setDataMode(SPI_MODE0);

    SPI.begin(SPICLOCK, DATAIN_MISO, DATAOUT_MOSI, SEL_MAX14921);
    esp_task_wdt_init(60*10, true);
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();
    upLoder(); 
    esp_task_wdt_reset();
    void bluetoothTask(void *pvParameters);
    xTaskCreate(bluetoothTask, "bluetoothTask", 1024 * 4,
                NULL, 1, h_pxblueToothTask); // 5120 6144 PCB 패턴문제로 사용하지 않는다.
    bmsModbus.setup();
    // Serial.printf("Free stack: %d\n", uxTaskGetStackHighWaterMark(NULL));
    if (_max14921.startCalibration())
    {
        Serial.println("Calibration completed successfully");
    }
    else
    {
        Serial.println("Calibration failed, using default values");
    }
    _max14921.initialize();
    useInterrupt();
}
#include <esp_adc_cal.h>
void readTemperature(){
    uint32_t adc_voltage1;
    uint32_t adc_voltage2;
    uint32_t adc_voltage3;
    esp_adc_cal_characteristics_t adc_chars;

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    esp_adc_cal_get_voltage(ADC_CHANNEL_3, &adc_chars, &adc_voltage1);  // IN_TH = GPIO39 = ADC1_CH3
    esp_adc_cal_get_voltage(ADC_CHANNEL_6, &adc_chars, &adc_voltage2);  // TH3 = GPIO34 = ADC1_CH6
    esp_adc_cal_get_voltage(ADC_CHANNEL_7, &adc_chars, &adc_voltage3);  // TH4 = GPIO35 = ADC1_CH7
    // float th1 = analogRead(IN_TH) * 3.3 / 4095.0;
    // float th2 = analogRead(TH3) * 3.3 / 4095.0;
    // float th3 = analogRead(TH4) * 3.3 / 4095.0;
    // ESP_LOGI("MAIN", "TH1: %3.3f(%d):%3.2f, TH2: %3.3f(%d):%3.2f, TH3: %3.3f(%d):%3.2f\n", 
    //     th1,adc_voltage1, _max14921.calTemperature(adc_voltage1), 
    //     th2,adc_voltage2, _max14921.calTemperature(adc_voltage2), 
    //     th3,adc_voltage3, _max14921.calTemperature(adc_voltage3));
    float tBoard = _max14921.calTemperature(adc_voltage1);
    float th3 = _max14921.calTemperature(adc_voltage2);
    float th4 = _max14921.calTemperature(adc_voltage3);

    if (xSemaphoreTake(max14921::dataMutex, portMAX_DELAY) == pdPASS)
    {
        temperature34.setTemperature(tBoard, th3, th4);  // 포인터 제거
        xSemaphoreGive(max14921::dataMutex);
    }
}
uint8_t readModbusAddress(){
    uint8_t address = 0;
    if(analogRead(ASEL1) > 1000) address += 1;
    if(analogRead(ASEL2) > 1000) address += 2;
    if(digitalRead(ASEL3) == HIGH) address += 4;
    if(nvmSet.ModbusAddress != address){
        nvmSet.ModbusAddress = address;
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet));
        EEPROM.commit();
        Serial.printf("ModbusAddress Changed to: %d\n", nvmSet.ModbusAddress);
    }
    return address;
}
void loop()
{
    esp_task_wdt_reset();
    long currentTime = millis();
    if(timerCount > 200)  // 200ms
    {
        if(nvmSet.UseHoleCT > 0) _max14921.readT123(AMPERE);
        timerCount = 0;
    }
    if (currentTime - elaspedTime250 > EVERY_250)
    {
        elaspedTime250 = currentTime;
        if (_max14921.openWireStatus == CELL_OPEN) LED_ON;
        else LED_TOGGLE;
        _max14921.MD_AK35_ScanCell(false);
        _max14921.readTotalVoltage();
        _max14921.readT123(TT1);
        _max14921.readT123(TT2);
        _max14921.balance_cells();
        readTemperature();
    }

    if (currentTime - elaspedTime1000 > EVERY_1000)
    {
        elaspedTime1000 = currentTime;
        //ESP_LOGI("MAIN", "ModbusAddress: %d", readModbusAddress());
        // Serial.printf("ASEL1 (GPIO32): %d\n", analogRead(ASEL1));
        // Serial.printf("ASEL2 (GPIO25): %d\n", analogRead(ASEL2));
        // Serial.printf("ASEL3 (GPIO27): %d\n", digitalRead(ASEL3));
        readModbusAddress();
    }

    if (currentTime - elaspedTime2000 > EVERY_2000)
    {
        elaspedTime2000 = currentTime;
        float totalVoltage = 0.0;
        uint16_t maxVol;
        uint16_t minVol=65535;
        for (int index = 0; index < nvmSet.InstalledCells; index++)
        {
            maxVol = max(maxVol, _max14921.cellVoltage[index]);
            minVol = min(minVol, _max14921.cellVoltage[index]);
            totalVoltage += _max14921.cellVoltage[index] * _max14921.VREF / 65536.0;
            Serial.printf("%3.3fV ", _max14921.cellVoltage[index] * _max14921.VREF / 65536.0);
            SerialBT.printf("%3.3fV ", _max14921.cellVoltage[index] * _max14921.VREF / 65536.0);
        }
        Serial.printf("\nT1: %2.1f C ", _max14921.T123[TT1] / 10.0);
        SerialBT.printf("\nT1: %2.1f C ", _max14921.T123[TT1] / 10.0);
        Serial.printf("T2: %2.1f C ", _max14921.T123[TT2] / 10.0);
        SerialBT.printf("T2: %2.1f C ", _max14921.T123[TT2] / 10.0);
        Serial.printf("AMP: %2.1fA ", _max14921.T123[AMPERE] / 10.0);
        SerialBT.printf("AMP: %2.1fA ", _max14921.T123[AMPERE] / 10.0);
        Serial.printf("->TV: %3.3fV ", 16.0 * _max14921.totalVoltage * _max14921.VREF / 65536.0);
        SerialBT.printf("TV(sum): %3.3fV ", totalVoltage);
        Serial.printf("Tv Diff: %3.3fV ", totalVoltage - 16.0 * _max14921.totalVoltage * _max14921.VREF / 65536.0);
        SerialBT.printf("Tv Diff: %3.3fV ", totalVoltage - 16.0 * _max14921.totalVoltage * _max14921.VREF / 65536.0);
        Serial.printf("\nMax: %3.3fV Min: %3.3fV ", maxVol * _max14921.VREF / 65536.0, minVol * _max14921.VREF / 65536.0);
        SerialBT.printf("\nMax: %3.3fV Min: %3.3fV ", maxVol * _max14921.VREF / 65536.0, minVol * _max14921.VREF / 65536.0);
        Serial.printf("diff : %3.3fV\n\n ", maxVol * _max14921.VREF / 65536.0 - minVol * _max14921.VREF / 65536.0);
        SerialBT.printf("\n");
        ESP_LOGI("MAIN", "IN_TH: %3.2f, TH3: %3.2f, TH4: %3.2f", temperature34.average_temperature_in_th, temperature34.average_temperature_3, temperature34.average_temperature_4);
        SerialBT.printf("\n");
        // uint8_t problemCellNum;
        // CellStatus status;
        // elaspedTime2000 = currentTime;

        // if (_max14921.CheckOpenWire())
        // {
        //    if (xSemaphoreTake(max14921::dataMutex, portMAX_DELAY) == pdPASS) 
        //    {
        //         for (int i = 0; i < nvmSet.InstalledCells; i++)
        //         {
        //             _max14921.cellVoltage[i] = 0.0;
        //         }
        //         xSemaphoreGive(max14921::dataMutex);
        //    }
        //     Serial.printf("\nOpen Wire Problem Cell: %d\n", problemCellNum);
        //     LED_ON;
        // }
    }
    // selfUploder.httpServer.handleClient();
    delay(5);
}
