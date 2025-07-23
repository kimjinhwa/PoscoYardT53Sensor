#ifndef MAIN_H
#define MAIN_H

#define LED_PORT GPIO_NUM_15
// 원래 핀 설정
#define ASEL1 GPIO_NUM_32
#define ASEL2 GPIO_NUM_25
#define ASEL3 GPIO_NUM_27

#define SENSOR_VP GPIO_NUM_36  //GPIO36 = ADC1_CH0
#define SENSOR_VN GPIO_NUM_39  //GPIO39 = ADC1_CH3
#define TH3 GPIO_NUM_34  //GPIO34 = ADC1_CH6
#define TH4 GPIO_NUM_35  //GPIO35 = ADC1_CH7
#define IN_TH SENSOR_VN 

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
  int16_t TotalVoltageOffset;
  uint16_t TotalVoltageGain;
  uint8_t Reserved8;
  char SSID[32];
  char PASS[32];
  uint8_t EpromValidEnd;
} nvmSystemSet;
extern nvmSystemSet nvmSet;


class temperatureClass
{
    private:
        uint8_t index;
        float coefficient_temperature =1.05;
    public:
        float IN_TH[10]; 
        float temperature_3[10]; 
        float temperature_4[10]; 
        float average_temperature_in_th;
        float average_temperature_3;
        float average_temperature_4;
        temperatureClass(){
            // IN_TH = (float *)ps_malloc(sizeof(float) * 10);
            // temperature_3 = (float *)ps_malloc(sizeof(float) * 10);
            // temperature_4 = (float *)ps_malloc(sizeof(float) * 10);
            // if(IN_TH == NULL || temperature_3 == NULL || temperature_4 == NULL){
            //     Serial.println("Failed to allocate memory for temperature");
            //     delay(3000);
            //     ESP.restart();
            // }
            index = 0;
        }
        ~temperatureClass(){
        }

        void setAverageTemperature(){
            average_temperature_in_th = 0.0;
            average_temperature_3 = 0.0;
            average_temperature_4 = 0.0;
            
            // index가 0이면 평균 계산하지 않음
            if(index == 0) return;
            
            for(int i = 0; i < index; i++){
                average_temperature_in_th += IN_TH[i];
                average_temperature_3 += temperature_3[i];
                average_temperature_4 += temperature_4[i];
            }
            average_temperature_in_th /= index;  // index로 나누기
            average_temperature_3 /= index;
            average_temperature_4 /= index;
        }
        void setTemperature(float in_th, float th3, float th4){
            in_th = in_th * coefficient_temperature;
            th3 = th3 * coefficient_temperature;
            th4 = th4 * coefficient_temperature;
            this->IN_TH[index] = in_th;  // 단일 값 저장
            this->temperature_3[index] = th3;
            this->temperature_4[index] = th4;
            this->index++;
            setAverageTemperature();
            if(this->index >= 10) this->index = 0;
            
            // 디버깅: 현재 저장된 값들 출력
            // ESP_LOGI("MAIN", "Index: %d, Current: IN_TH=%.2f, TH3=%.2f, TH4=%.2f", 
            //     index-1, in_th, th3, th4);
        }
};
extern temperatureClass temperature34;
#endif