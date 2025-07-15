#ifndef ESP32SELFUPLODER_H
#define ESP32SELFUPLODER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HTTPUpdateServer.h>
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

struct FirmwareInfo {
    char latest[100];
    char filename[100];
};

class ESP32SelfUploder {
    private:
        FirmwareInfo currentFirmware;
        static const int FIRMWARE_INFO_ADDR = 0;
        
    public:
        char ssid[32];
        char password[32];
        char update_url[256];
        String updateFile_url;
        WebServer httpServer;
        HTTPUpdateServer httpUpdater;
        unsigned int ledPin;
        void setLed(unsigned int pin){
            ledPin = pin;
            pinMode(ledPin, OUTPUT);
        };
        ESP32SelfUploder() : httpServer(80) {
        }

        void begin(const char* ssid, const char* password, const char* update_url);
        void loop();
        bool tryAutoUpdate(const char* firmware_url);
        bool checkNewVersion(const char* version_url);
};

extern ESP32SelfUploder selfUploder;
#endif
