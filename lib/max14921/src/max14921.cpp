/*
  MAX14921.h - Library for reading from a MAX14921.
  Created by Felix
*/

#include "max14921.h"
#include "Arduino.h"
#include <SPI.h>
#include <ThreeWire.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "../../src/main.h"
long ticks = 0;

/**-----------------------------------------------------------------------------
 *
 * Default delay timeouts
 *
 *------------------------------------------------------------------------------
 */
#define   MD_AK35_LEVEL_SHIFTING_DELAY_MAX   50   // us
#define   MD_AK35_T_SETTLING_TIME_MAX        10   // us

/**-----------------------------------------------------------------------------
 *
 * SPI bit defines
 *
 *------------------------------------------------------------------------------
 */
#define   MD_AK35_SPI_SAMPLB_ENABLE     0x04
#define   MD_AK35_SPI_SAMPLB_DISABLE    0x00
#define   MD_AK35_SPI_DIAG_ENABLE       0x02
#define   MD_AK35_SPI_DIAG_DISABLE      0x00
#define   MD_AK35_SPI_LOPW_ENABLE       0x01
#define   MD_AK35_SPI_LOPW_DISABLE      0x00
#define   MD_AK35_SPI_TXSEL_DIRECT      0x08
#define   MD_AK35_SPI_TXSEL_BUFFERED    0x18

#define   MD_AK35_SPI_TXSEL_1           0x40
#define   MD_AK35_SPI_TXSEL_2           0x20
#define   MD_AK35_SPI_TXSEL_3           0x60

#define   MD_AK35_SPI_CELLSEL_00   0x80
#define   MD_AK35_SPI_CELLSEL_01   0xC0
#define   MD_AK35_SPI_CELLSEL_02   0xA0
#define   MD_AK35_SPI_CELLSEL_03   0xE0
#define   MD_AK35_SPI_CELLSEL_04   0x90
#define   MD_AK35_SPI_CELLSEL_05   0xD0
#define   MD_AK35_SPI_CELLSEL_06   0xB0
#define   MD_AK35_SPI_CELLSEL_07   0xF0
#define   MD_AK35_SPI_CELLSEL_08   0x88
#define   MD_AK35_SPI_CELLSEL_09   0xC8
#define   MD_AK35_SPI_CELLSEL_10   0xA8
#define   MD_AK35_SPI_CELLSEL_11   0xE8
#define   MD_AK35_SPI_CELLSEL_12   0x98
#define   MD_AK35_SPI_CELLSEL_13   0xD8
#define   MD_AK35_SPI_CELLSEL_14   0xB8
#define   MD_AK35_SPI_CELLSEL_15   0xF8
#define   MD_AK35_SPI_CELLSEL_TOTAL   0x18


uint8_t gMD_AK35_TxSelect_Table[ 3 ] = {
  MD_AK35_SPI_TXSEL_1,
  MD_AK35_SPI_TXSEL_2,
  MD_AK35_SPI_TXSEL_3
};
/**-----------------------------------------------------------------------------
 *
 * Lookup table for quick access to cell select bits needed for SPI command
 *
 *------------------------------------------------------------------------------
 */
uint8_t gMD_AK35_CellSelect_Table[ 16 ] =
{
  MD_AK35_SPI_CELLSEL_00,
  MD_AK35_SPI_CELLSEL_01,
  MD_AK35_SPI_CELLSEL_02,
  MD_AK35_SPI_CELLSEL_03,
  MD_AK35_SPI_CELLSEL_04,
  MD_AK35_SPI_CELLSEL_05,
  MD_AK35_SPI_CELLSEL_06,
  MD_AK35_SPI_CELLSEL_07,
  MD_AK35_SPI_CELLSEL_08,
  MD_AK35_SPI_CELLSEL_09,
  MD_AK35_SPI_CELLSEL_10,
  MD_AK35_SPI_CELLSEL_11,
  MD_AK35_SPI_CELLSEL_12,
  MD_AK35_SPI_CELLSEL_13,
  MD_AK35_SPI_CELLSEL_14,
  MD_AK35_SPI_CELLSEL_15
};


/**-----------------------------------------------------------------------------
 *
 * Used to manage the device object.
 *
 *------------------------------------------------------------------------------
 */
MD_AK35_INSTANCE_T   MD_AK35_obj;

int readvalue;
SemaphoreHandle_t max14921::dataMutex = NULL;

void max14921::initFIFO() {
  for (int i = 0; i < 16; i++)  // 16 cells
  {
    cellFIFOs[i].head = 0;
    cellFIFOs[i].count = 0;
    for (int j = 0; j < FIFO_SIZE; j++)  // 20 samples per cell
    {
      cellFIFOs[i].buffer[j] = 0;
    }
  }
  totalVoltageFIFO.head = 0;
  totalVoltageFIFO.count = 0;
  for (int j = 0; j < FIFO_SIZE; j++)  // 20 samples per cell
  {
    totalVoltageFIFO.buffer[j] = 0;
  }
  ampereFIFO.head = 0;
  ampereFIFO.count = 0;
  for (int j = 0; j < FIFO_SIZE; j++)  // 20 samples per cell
  {
    ampereFIFO.buffer[j] = 0;
  }
}
uint16_t max14921::updateTotalVoltageFIFO(uint16_t newvalue) {
  totalVoltageFIFO.buffer[totalVoltageFIFO.head] = newvalue;
  totalVoltageFIFO.head = (totalVoltageFIFO.head + 1) % FIFO_SIZE;
  if(totalVoltageFIFO.count < FIFO_SIZE) {
    totalVoltageFIFO.count++;
  }
  uint32_t sum = 0;
  for(int i=0; i<totalVoltageFIFO.count; i++) {
    sum += totalVoltageFIFO.buffer[i];
  }
  return (uint16_t)(sum / totalVoltageFIFO.count);
}
int16_t max14921::updateAmpereFIFO(int16_t newvalue) {
  ampereFIFO.buffer[ampereFIFO.head] = newvalue;
  ampereFIFO.head = (ampereFIFO.head + 1) % FIFO_SIZE;
  if(ampereFIFO.count < FIFO_SIZE) {
    ampereFIFO.count++;
  }
  int32_t sum = 0;
  for(int i=0; i<ampereFIFO.count; i++) {
    sum += ampereFIFO.buffer[i];
  }
  return (int16_t)(sum / ampereFIFO.count);
}
uint16_t max14921::updateFIFO(uint8_t cellNum, uint16_t newvalue) {
  cellFIFOs[cellNum].buffer[cellFIFOs[cellNum].head] = newvalue;
  cellFIFOs[cellNum].head = (cellFIFOs[cellNum].head + 1) % FIFO_SIZE;
  if(cellFIFOs[cellNum].count < FIFO_SIZE) {
    cellFIFOs[cellNum].count++;
  }
  
  // uint32_t로 변경하여 오버플로우 방지
  uint32_t sum = 0;
  for(int i=0; i<cellFIFOs[cellNum].count; i++) {
    sum += cellFIFOs[cellNum].buffer[i];
  }
  
  // 디버깅을 위한 출력 추가
  // if(cellNum == 0) {  // 첫 번째 셀만 출력
  //   Serial.printf("FIFO Debug - Cell %d: New=%d, Avg=%d, Count=%d\n", 
  //                cellNum, newvalue, (uint16_t)(sum/cellFIFOs[cellNum].count), 
  //                cellFIFOs[cellNum].count);
  // }
  
  return (uint16_t)(sum / cellFIFOs[cellNum].count);
}


max14921::max14921(int ADC_CS_pin, int CS_pin, int SAMPLPIN_MAX14921_pin)
{
	  _ADC_CS_pin = ADC_CS_pin;
	  _CS_pin = CS_pin;
	  _SAMPLPIN_MAX14921_pin = SAMPLPIN_MAX14921_pin;

    if(dataMutex == NULL) dataMutex = xSemaphoreCreateMutex();

	  pinMode(ADC_CS_pin, OUTPUT);
	  pinMode(CS_pin, OUTPUT);
	  pinMode(SAMPLPIN_MAX14921_pin, OUTPUT);
	  pinMode(MAX_EN, OUTPUT);
    pinMode(_SAMPLPIN_MAX14921_pin ,OUTPUT);

    initialize();
    initFIFO();
}
void max14921::initialize(){
	  // disable device to start with
	  digitalWrite(_ADC_CS_pin, HIGH);
	  digitalWrite(_CS_pin, HIGH);
	  digitalWrite(_SAMPLPIN_MAX14921_pin, HIGH);
	  digitalWrite(MAX_EN, HIGH);

	  // SPI.setClockDivider( SPI_CLOCK_DIV8 ); // slow the SPI bus down
	  // SPI.setBitOrder(LSBFIRST);
	  // SPI.setDataMode(SPI_MODE0);    // SPI 0,0 as per MCP330x data sheet
	  // SPI.begin();


	  //
	  // Setup initial parameters of the AL38 object
	  //
	  MD_AK35_obj.acqCellNumber             = nvmSet.InstalledCells;
	  MD_AK35_obj.acqCellSampleTmrValueMsec = 4;   // ms
	  MD_AK35_obj.acqCellRepeatTmrValueMsec = 6;   // ms
	  MD_AK35_obj.acqCellSettlingTimeUsec   = MD_AK35_T_SETTLING_TIME_MAX;   // us

	  MD_AK35_obj.acqScanContinuous = false;

	  MD_AK35_obj.acqT1IsEnabled        = false;
	  MD_AK35_obj.acqT2IsEnabled        = false;
	  MD_AK35_obj.acqT3IsEnabled        = false;
	  MD_AK35_obj.acqT1SettlingTimeUsec = MD_AK35_T_SETTLING_TIME_MAX;   // us
	  MD_AK35_obj.acqT2SettlingTimeUsec = MD_AK35_T_SETTLING_TIME_MAX;   // us
	  MD_AK35_obj.acqT3SettlingTimeUsec = MD_AK35_T_SETTLING_TIME_MAX;   // us

	//  MD_AK35_obj.acqInterface    = MI_CLI_REASON_EXECUTE_USBHID;

	  MD_AK35_obj.spiBalanceC01_C08 = 0x00;
	  MD_AK35_obj.spiBalanceC09_C16 = 0x00;
	  MD_AK35_obj.spiSamplB         = MD_AK35_SPI_SAMPLB_DISABLE;
	  MD_AK35_obj.spiDiag           = MD_AK35_SPI_DIAG_DISABLE;
	  MD_AK35_obj.spiLoPw           = MD_AK35_SPI_LOPW_DISABLE;

	  MD_AK35_obj.calParErrTmrValueMsec = 40;
    long rxData = MD_AK35_SpiTransfer24(0x00, 0x00, 0x00);
    Serial.printf("\n--->MAX14921 POWER ON: 0x%06lx\n", rxData);

}
// initialize the library with the numbers of the interface pins: (SPI on Arduino UNO as standard)
// set up the number of cells used: Choice between 3 and 16 cells total. This is used to calculate pack voltage
void max14921::SetCellNumber(uint8_t cellNum)
{
	MD_AK35_obj.acqCellNumber = cellNum;
}
// set up the battery chemestry used: LiPo is standard sets parimeters for vMax/vMin per cell.
//LiPo - vMin per cell = 3.600v vMax per cell = 4.200v

// Enable balancing of cells - Option - ON/OFF
void max14921::SetBalancing(bool bal)
{
}

// Enable open-wire detection - Option - ON/OFF
void max14921::SetOpenWireDetection(bool openwire)
{
   uint8_t spiCmd=0;
   MAX14921_Command_t command;
   long value = 0;
   //command.bits.ECS = 1;
   if(openwire)
   {
      command.bits.DIAG = 1; 
      long value = MD_AK35_SpiTransfer24(0x00, 0x00, command.cmd);
   }
   else
   {
      command.bits.DIAG = 0; 
      command.bits.ECS = 1;
      value = MD_AK35_SpiTransfer24(0x00, 0x00, command.cmd);
      // delay(100);
      //Serial.printf("\nDIAG: 0x%06lx\n",  value);
   }
  digitalWrite (_SAMPLPIN_MAX14921_pin, HIGH); // Sampl_Enable
}

uint16_t max14921::CheckOpenWire() {
    float normalVoltage[16] = {0.0};
    float diagVoltage[16] = {0.0};
    // if( openWireStatus == CELL_OPEN)
    //   hasIssueCell = 0xFFFF;
    // 1. 정상 상태(DIAG=0)에서 전압 측정
    SetOpenWireDetection(true);
    delay(600);
    MD_AK35_ScanCell(false);
    MD_AK35_ScanCell(false);

    //Serial.printf("\nDIAG: ");
    for(int i=0; i<MD_AK35_obj.acqCellNumber; i++) {
        diagVoltage[i] = (float)cellVoltage[i]*VREF/65536.0;
        if(diagVoltage[i] < 0.5){
          hasIssueCell |= (1 << i);
        }
        else{
          hasIssueCell = hasIssueCell & ~(1 << i);
        }
        //Serial.printf("%3.3fV ", diagVoltage[i]);
    }

    Serial.printf("\n");
    SetOpenWireDetection(false);
    MD_AK35_ScanCell(false);
    MD_AK35_ScanCell(false);
    if(hasIssueCell == 0  )
      this->openWireStatus = CELL_NORMAL;
    else
      this->openWireStatus = CELL_OPEN;
    return hasIssueCell;
}

uint16_t max14921::CheckOpenWire2() {
    float initialVoltage[16] = {0};
    float currentVoltage = 0;
    
    // 1. 먼저 모든 밸런싱을 비활성화하고 초기 전압을 측정
    MD_AK35_obj.spiBalanceC01_C08 = 0x00;
    MD_AK35_obj.spiBalanceC09_C16 = 0x00;
    
    MD_AK35_ScanCell(false);
    for(int i = 0; i < MD_AK35_obj.acqCellNumber; i++) {
        initialVoltage[i] = 1000.0 * cellVoltage[i] * VREF/65536.0; // mV로 변환
        Serial.printf("Initial Cell %d Voltage: %3.3f mV\n", i+1, initialVoltage[i]);
    }
    
    // 2. 각 셀에 대해 순차적으로 방전 테스트
    for(uint8_t cellNum = 1; cellNum <= MD_AK35_obj.acqCellNumber; cellNum++) {
        // 해당 셀의 밸런싱 FET만 활성화
        if(cellNum <= 8) {
            MD_AK35_obj.spiBalanceC01_C08 = (1 << (cellNum-1));
        } else {
            MD_AK35_obj.spiBalanceC09_C16 = (1 << (cellNum-9));
        }
        
        // RBAL x CSAMPLE 시간만큼 대기 (14.8ms)
        vTaskDelay(100);
        
        // 전압 재측정
        MD_AK35_ScanCell(false);
        currentVoltage = 1000.0 * cellVoltage[cellNum-1] * VREF/65536.0;
        
        // 전압 변화 확인
        float voltageDiff = initialVoltage[cellNum-1] - currentVoltage;
        
        Serial.printf("Cell %d - Initial: %3.3f mV, Current: %3.3f mV, Diff: %3.3f mV\n", 
            cellNum, initialVoltage[cellNum-1], currentVoltage, voltageDiff);
        
        // 문제 감지:
        if(voltageDiff < 100) { // 방전이 거의 안됨 -> Open
            hasIssueCell |= (1 << cellNum);  // 해당 셀의 비트를 1로 설정
            this->openWireStatus = CELL_OPEN;
            Serial.printf("Open wire detected at Cell %d initial: %3.3f mV, current: %3.3f mV, diff: %3.3f mV\n", 
              cellNum, initialVoltage[cellNum-1], currentVoltage, voltageDiff);
            break;
        } else if(cellNum < MD_AK35_obj.acqCellNumber && 
                 (1000.0 * cellVoltage[cellNum] * VREF/65536.0) > initialVoltage[cellNum] + 500) {
            // 다음 셀 전압이 초기값보다 크게 증가 -> Short
            hasIssueCell |= (1 << cellNum);  // 해당 셀의 비트를 1로 설정
            this->openWireStatus = CELL_SHORT;
            Serial.printf("Short detected at Cell %d (Next cell voltage increase)\n", cellNum);
            break;
        }
        
        // 다음 셀 검사 전에 밸런싱 비활성화
        MD_AK35_obj.spiBalanceC01_C08 = 0x00;
        MD_AK35_obj.spiBalanceC09_C16 = 0x00;
        
        // 셀이 다시 충전되도록 잠시 대기
        vTaskDelay(10);
    }
    
    // 마지막으로 모든 밸런싱 비활성화
    MD_AK35_obj.spiBalanceC01_C08 = 0x00;
    MD_AK35_obj.spiBalanceC09_C16 = 0x00;
    
    if(hasIssueCell == 0) {
        this->openWireStatus = CELL_NORMAL;
        Serial.println("All cells are normal");
    }
    
    return hasIssueCell;
}

// Set Sample time: Option in ms. Standard 4 ms
void max14921::SetSampleTime(int sampletime)
{
	MD_AK35_obj.acqCellSampleTmrValueMsec = sampletime;
}

// Set Settling Time in us. Standard 50 us.
void max14921::SetSettlingTime(int settlingtime)
{
	MD_AK35_obj.acqCellSettlingTimeUsec = settlingtime;
}

// Set Repeat Time in ms. Standard 10 ms.
void max14921::SetRepeatTime(int repeattime)
{
	MD_AK35_obj.acqCellRepeatTmrValueMsec = repeattime;
}


/**-----------------------------------------------------------------------------
 *
 * Read 2 bytes from device using dummy data
 *
 * @return
 * int : 2 bytes of SPI Rx Data
 *
 * @constraints
 * Standard
 *
 *------------------------------------------------------------------------------
 */
ThreeWire max11163wire(DATAIN_MISO, SPICLOCK, SEL_MAX11163 );
uint16_t max14921::MAX11163_ReadData16_2( void )
{
  uint16_t adcData = 0;
  uint16_t spiData = 0;
  uint16_t hi = 0, lo = 0;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

  max11163wire.beginTransmission();
  lo = max11163wire.read16();
  max11163wire.endTransmission();
  return lo;
}
uint16_t max14921::MAX11163_ReadData16( void )
{
  uint16_t adcData = 0;
  uint16_t spiData = 0;
  uint16_t hi = 0, lo = 0;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

  max11163wire.beginTransmission();
  lo = max11163wire.read16();
  max11163wire.endTransmission();
  SPI.end();
  delayMicroseconds(10);
  SPI.begin(SPICLOCK, DATAIN_MISO, DATAOUT_MOSI, SEL_MAX14921);
  return lo;
}

/**-----------------------------------------------------------------------------
 *
 * Transfers 3 SPI data bytes and returns 3 bytes of read data
 *
 * @param byte1   in : SPI Data Tx Byte 1
 * @param byte2   in : SPI Data Tx Byte 2
 * @param byte3   in : SPI Data Tx Byte 3
 *
 * @return
 * MCS_U32_T : 24 bits of SPI Rx Data
 *
 * @constraints
 * Standard
 *
 *------------------------------------------------------------------------------
 */
long max14921::MD_AK35_SpiTransfer24(uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
  digitalWrite (_CS_pin, LOW); // chip-select MAX14921
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  long ak35Data = 0;
  byte spiData  = 0;

  spiData    = SPI.transfer(byte1);
  ak35Data   = spiData;
  ak35Data <<= 8;

  spiData    = SPI.transfer(byte2);
  ak35Data  |= spiData;
  ak35Data <<= 8;

  spiData    = SPI.transfer(byte3);
  ak35Data  |= spiData;


  SPI.endTransaction();
  digitalWrite (_CS_pin, HIGH); // chip-select MAX14921
  return( ak35Data );
}

uint16_t max14921::readTotalVoltage()
{
  uint8_t spiCmd = 0;

  digitalWrite (_SAMPLPIN_MAX14921_pin, HIGH); // Sampl_Enable
  digitalWrite (_SAMPLPIN_MAX14921_pin, LOW); // 이제 샘플링을 했으므로 읽기 모드로 들어간다. 

  MD_AK35_obj.spiBalanceC01_C08 = 0x00;
	MD_AK35_obj.spiBalanceC09_C16 = 0x00;
  delayMicroseconds(MD_AK35_LEVEL_SHIFTING_DELAY_MAX*4); // Datasheet max 50us
  spiCmd = MD_AK35_SPI_CELLSEL_TOTAL   ;
  //한번은 읽고 버린다.
  long rxData = MD_AK35_SpiTransfer24(0, 0, spiCmd);
  delayMicroseconds(100); // Datasheet 5us
  MAX11163_ReadData16();
  rxData = MD_AK35_SpiTransfer24(0, 0, spiCmd);
  delayMicroseconds(100); // Datasheet 5us

  uint16_t adcData = 0;
  uint32_t sumData = 0;  // Changed to uint32_t to prevent overflow
  for(int i = 0; i < 10; i++){
    adcData = MAX11163_ReadData16();
    adcData = adcData ;
    sumData += adcData;  // Changed to sumData
  }

  adcData = (uint16_t)(sumData/10) + nvmSet.TotalVoltageOffset;  // Changed to sumData
  adcData = adcData * nvmSet.TotalVoltageGain/1000;
  if (xSemaphoreTake(max14921::dataMutex, portMAX_DELAY) == pdPASS){
    //totalVoltage = adcData;
    totalVoltage = updateTotalVoltageFIFO(adcData);
    xSemaphoreGive(max14921::dataMutex);
  }
  // ESP_LOGI("MAX14921","TV: %3.3fV ", 16.0 * totalVoltage * VREF / 65536.0);
  digitalWrite (_SAMPLPIN_MAX14921_pin, HIGH); // Sampl_Enable
  return adcData;
}
/**-----------------------------------------------------------------------------
 *
 * Read 2 bytes from device using dummy data
 * Tnumber : 0, 1, 2
 * T1 : 0, Read Temperature 1
 * T2 : 1, Read Temperature 2
 * T3 : 2, Read Ampere
 * @return
 * int : 2 bytes of SPI Rx Data
 *
 *------------------------------------------------------------------------------
 */
float max14921::calTemperature(uint16_t adcData){
    // ESP_LOGI("MAX14921", "Vntc T%d: %f", Tnumber, Vntc);
    adcData = adcData + nvmSet.TempOffset;
    float Vntc = adcData * VREF / 65536.0;
    // ESP_LOGI("MAX14921", "TNUMBER %d: %3.3f", Tnumber, Vntc);
    float Rref = 10000.0;
    float Vt = 3.3;
    // Rntc = Rref*Vntc/(Vt-Vntc)
    float Rntc = Rref * Vntc / (Vt - Vntc);
    // ESP_LOGI("MAX14921", "Rntc T%d: %f", Tnumber, Rntc);
    float Beta = 3977.0;     //
    float ntcAt25 = 10000.0; // NTC 10K at 25 degree
    float T0 = 273.15 + 25;  // 25 degree
    // Beta parameter formula:
    // R0 : 10000 ( NTC 10K)
    // Beta : 3977
    // R : ADC data derived resistance
    // T = 1/(1/T0 + 1/Beta)*ln(R/R0)
    float temperature = 1 / (1 / T0 + (1 / Beta) * log(Rntc / ntcAt25));
    temperature = temperature - 273.15;
    if (temperature < -10)
      temperature = -35;

  return 0.0;
}
uint16_t max14921::readT123(T_NUMBER Tnumber)
{
  uint8_t spiCmd = 0;
  delayMicroseconds(MD_AK35_LEVEL_SHIFTING_DELAY_MAX); // Datasheet max 50us

  spiCmd = gMD_AK35_TxSelect_Table[Tnumber] | 0x10 | 0x08; // Through buffer
  // spiCmd = gMD_AK35_TxSelect_Table[Tnumber] | 0x00 | 0x08; //Direct
  spiCmd |= (MD_AK35_obj.spiSamplB | MD_AK35_obj.spiDiag | MD_AK35_obj.spiLoPw);

  // 쓰레기 값을 버리기 위해서 아래의 값을 읽고 버린다. 
  long rxData = MD_AK35_SpiTransfer24(0, 0, spiCmd);
  delayMicroseconds(50); // Datasheet 5us
  MAX11163_ReadData16();
  rxData = MD_AK35_SpiTransfer24(0, 0, spiCmd);
  delayMicroseconds(50); // Datasheet 5us



  rxData = 0;
  int readCount = 10;
  if (Tnumber == AMPERE)
  {
    //1ms 200번 읽기 
    readCount = 300;
    //uint32_t startTime = millis();
    SPI.begin(SPICLOCK, DATAIN_MISO, DATAOUT_MOSI, SEL_MAX14921);
    for (int i = 0; i < readCount; i++)
    {
      rxData += MAX11163_ReadData16_2();
      delayMicroseconds(50);
    }
    SPI.end();
    delayMicroseconds(10);
  }
  else
  {
    readCount = 10;
    for (int i = 0; i < readCount; i++)
    {
      rxData += MAX11163_ReadData16();
      delayMicroseconds(50);
    }
  }
  uint16_t adcData = (uint16_t)(rxData / readCount);
  // Vntc = Vt*Rntc/(Rref+Rntc)
  if (Tnumber == TT1 || Tnumber == TT2)
  {
    float temperature = calTemperature(adcData);
    // ESP_LOGI("MAX14921", "Temperature T%d: %f", Tnumber, temperature);
    T123[Tnumber] = (int)(temperature * 10);
    return temperature;
  }
  else
  {
    // OPAMP에서 1/2를 했으므로 다시 2배수를 해준다.
    //ESP_LOGE("MAX14921", "step0 adcData: %d", adcData);
    adcData = adcData + nvmSet.AmpereOffset;
    //ESP_LOGE("MAX14921", "step1 adcData: %d", adcData);
    adcData = adcData * nvmSet.AmpereGain / 1000.0;
    //ESP_LOGE("MAX14921", "step2 adcData: %d", adcData);
    float Vntc = adcData * VREF / 65536.0;
    //ESP_LOGE("MAX14921", "step3 Vntc: %f", Vntc);
    Vntc = Vntc - 2.0;
    //ESP_LOGE("MAX14921", "step3 Vntc: %f", Vntc);
    float voltage = 1000.0 * Vntc * 2.0; // mV 단위로 변환
    float ampere = 0.0;
    // ESP_LOGI("MAX14921", "step6 voltage: %f", voltage);
    // ESP_LOGI("MAX14921", "step7 voltage: %f", voltage);
    // ESP_LOGI("MAX14921", "AMP VOL %d: %3.3f", Tnumber, voltage);

    //ampere = voltage * HOLE_CT_RATIO;
    ampere = voltage * nvmSet.UseHoleCT/HOLE_CT_V;
    ampere = ampere * 10.0; //;이제 전류값에 10을 곱해준다.
    //ESP_LOGI("MAX14921", "AMP VOL %d: %3.3f", Tnumber, ampere);
    if (ampere < -nvmSet.UseHoleCT* 10.0 || ampere > nvmSet.UseHoleCT* 10.0)
    {
      ampere = 0.0;
    }
    if (xSemaphoreTake(max14921::dataMutex, portMAX_DELAY) == pdPASS)
    {
       //23[AMPERE] = T123[AMPERE] * (1.0 - MAX_AMPERE_FILTER_FACTOR) + MAX_AMPERE_FILTER_FACTOR * ampere;
      T123[AMPERE] = updateAmpereFIFO((int16_t)ampere);
      xSemaphoreGive(max14921::dataMutex);
    }
    return adcData;
  }
}

/**-----------------------------------------------------------------------------
 *
 * Performs readings of cell data including -
 *   - Disable SAMPL with SAMPL settle timing
 *   - Send cell selection SPI command to AK35
 *   - Delay AOUT settle time
 *   - Read ADC with timing constraints
 *
 * @param cellNum            in : Number of cells to read (1-16)
 * @param pAdcCellVoltage   out : Pointer to array of cell ADC readings
 * @param pSpiRxData        out : Pointer to array of SPI Rx data for AK35
 *                                  cell selection requests
 *
 * @constraints
 * Standard
 *
 *------------------------------------------------------------------------------
 */
/*
  Sample : SAMPLPIN_MAX14921_pin -> touch_high_volt_t
  and SMPLB -> LOW
  and ECS -> LOW, if High this is Voltage*16=  Total Voltage
  읽을 때는 둘중에 하나의 상태를 바꾼다.
*/
static int balanceToggleTest = 0;
static long readCount = 0;
void max14921::MD_AK35_Cmd_AcquireCell(uint8_t cellNum,
                              int *pAdcCellVoltage,
                              long *pSpiRxData )
{
  int cellIndex = 0;
  // Disable SAMPL and delay for data to be stored in AK35
  // digitalWrite (_SAMPLPIN_MAX14921_pin, HIGH); // Sampl_Disable
  // delayMicroseconds(2); //opend with in 0.5us
  //delayMicroseconds(MD_AK35_LEVEL_SHIFTING_DELAY_MAX*10);//Datasheet max 50us
  //digitalWrite (_SAMPLPIN_MAX14921_pin, LOW); // Sampl_Disable
  digitalWrite (_SAMPLPIN_MAX14921_pin, HIGH); // Sampl_Enable
  digitalWrite (_SAMPLPIN_MAX14921_pin, LOW); // 이제 샘플링을 했으므로 읽기 모드로 들어간다. 
  delayMicroseconds(MD_AK35_LEVEL_SHIFTING_DELAY_MAX);//Datasheet max 50us
  // 현재 SMPLB는 LOW로 되어 있어야 한다.

  readCount++;
  if(readCount % 1000 == 0)
  {
    ESP_LOGI("MAX14921", "balanceC01_C08: %02x, balanceC09_C16: %02x", MD_AK35_obj.modbusBalanceC01_C08, MD_AK35_obj.modbusBalanceC09_C16);
  }
  MD_AK35_obj.spiBalanceC01_C08 = REVERSE_BITS_8(MD_AK35_obj.modbusBalanceC01_C08);
  MD_AK35_obj.spiBalanceC09_C16 = REVERSE_BITS_8(MD_AK35_obj.modbusBalanceC09_C16);
  // if(balanceToggleTest == 0)
  // {
  //   MD_AK35_obj.spiBalanceC01_C08 = 0x00;
  //   MD_AK35_obj.spiBalanceC09_C16 = 0x00;
  //   balanceToggleTest = 1;
  // }
  // else
  // {
  //   MD_AK35_obj.spiBalanceC01_C08 = 0xff;
  //   MD_AK35_obj.spiBalanceC09_C16 = 0xff;
  //   balanceToggleTest = 0;
  // }
  if ( cellNum != 0 )
  {
    //
    // Read x cell voltages
    //
    uint8_t spiCmd=0;
    MAX14921_Command_t command;
    command.bits.ECS = 1;
    for ( cellIndex = (cellNum-1) ; cellIndex >= 0 ; cellIndex-- )
    //for ( cellIndex = 0 ; cellIndex < cellNum ; cellIndex++ )
    {
      // 쓰레기 값을 버리기 위해서 아래의 값을 읽고 버린다. 
      // 디버깅에서 발견 하였다
      if (cellIndex == 15)
      {
        command.bits.ECS = 1;
        for (int i = 0; i < 2; i++)
        {
          spiCmd = gMD_AK35_CellSelect_Table[i];
          spiCmd |= (MD_AK35_obj.spiSamplB | MD_AK35_obj.spiDiag | MD_AK35_obj.spiLoPw);
          MD_AK35_SpiTransfer24(MD_AK35_obj.spiBalanceC01_C08,
                                MD_AK35_obj.spiBalanceC09_C16,
                                spiCmd);
          delayMicroseconds(50); // Datasheet 5usA
          for (int i = 0; i < 1; i++)
          {
            MAX11163_ReadData16();
          }
        }
      }
      command.bits.ECS = 1;
      spiCmd  = gMD_AK35_CellSelect_Table[ cellIndex ];
      spiCmd |= ( MD_AK35_obj.spiSamplB | MD_AK35_obj.spiDiag | MD_AK35_obj.spiLoPw );
      pSpiRxData[ cellIndex ] = MD_AK35_SpiTransfer24( MD_AK35_obj.spiBalanceC01_C08,
                                                       MD_AK35_obj.spiBalanceC09_C16,
                                                       spiCmd );
      delayMicroseconds(50);//Datasheet 5usA
      uint32_t sumData = 0;  // Changed to uint32_t to prevent overflow
      for(int i = 0; i < 10; i++) {
        sumData += MAX11163_ReadData16();
      }
      pAdcCellVoltage[ cellIndex ] = (int)(sumData / 10);
      pAdcCellVoltage[ cellIndex ] = pAdcCellVoltage[ cellIndex ] + nvmSet.Max1161_CellOffset;
      pAdcCellVoltage[ cellIndex ] = pAdcCellVoltage[ cellIndex ] * nvmSet.Max1161_CellGain/1000;
      // if(cellIndex==0 )
      // {
      //   ESP_LOGI("--->MAX14921", "cellVoltage[%d]: %d", cellIndex, pAdcCellVoltage[cellIndex]*4096/65536);
      // }
      if (xSemaphoreTake(max14921::dataMutex, portMAX_DELAY) == pdPASS)   
      {
        //cellVoltage[cellIndex] = pAdcCellVoltage[cellIndex];
        cellVoltage[cellIndex] = updateFIFO(cellIndex, pAdcCellVoltage[cellIndex]);
        xSemaphoreGive(max14921::dataMutex);
      }
    }
  }
  digitalWrite (_SAMPLPIN_MAX14921_pin, HIGH); // 샘플링 모드로 놓는다.
}
bool max14921::startCalibration() 
{
    Serial.println("Starting self-calibration sequence...");
    
    // 1. 현재 상태 저장
    uint8_t prevDiag = MD_AK35_obj.spiDiag;
    uint8_t prevSamplB = MD_AK35_obj.spiSamplB;
    
    // 2. 캘리브레이션 모드로 스캔
    MD_AK35_ScanCell(true);
    
    // 3. 원래 상태로 복원
    MD_AK35_obj.spiDiag = prevDiag;
    MD_AK35_obj.spiSamplB = prevSamplB;
    
    // 4. 캘리브레이션 결과 검증
    bool calibrationSuccess = verifyCalibration();
    
    if (calibrationSuccess) {
        Serial.println("Calibration successful");
    } else {
        Serial.println("Calibration failed");
    }
    
    return calibrationSuccess;
}
bool max14921::verifyCalibration() {
    // 캘리브레이션 결과 검증
    // 예: 각 셀의 전압이 예상 범위 내에 있는지 확인
    for (int i = 0; i < MD_AK35_obj.acqCellNumber; i++) {
        float cellV = cellVoltage[i] * VREF / 65536.0;
        if (cellV > 5.0) {  // 예상 전압 범위
            return false;
        }
    }
    return true;
}
/**-----------------------------------------------------------------------------
 *
 * Scans thru requested cells and reads ADC data and returns data to PC
 *
 * @param calibrationData   in : TRUE  - Data read is Parasitic Error Cal data
 *                               FALSE - Data is normal cell voltage data
 *
 * @constraints
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void max14921::MD_AK35_ScanCell(bool calibrationData)
{
  int adcVal[16] = {0};
  long spiRxData[16] = {0};
  long rxData = 0;
  MAX14921_Command_t command;

  if (calibrationData)
  {
    Serial.println("Starting buffer amplifier calibration...");

    rxData = MD_AK35_SpiTransfer24(0x00, 0x00, 0x00);
    Serial.printf("\n--->MAX14921 Initialize Code: 0x%06lx\n", rxData);
    // 캘리브레이션 명령 설정
    uint8_t spiCmd = 0;
    command.bits.ECS = 1;
    command.bits.SC0 = 1;
    command.bits.SC1 = 0;
    command.bits.SC2 = 0;
    command.bits.SC3 = 0;
    command.bits.SAMPLB = 0;
    command.bits.DIAG = 1;
    command.bits.LOPW = 0;
    // spiCmd |= (1 << 16); // ECS = 1 (Cell selection enabled)
    // spiCmd |= (1 << 17); // SC0 = 1
    // SC1, SC2, SC3 = 0 (self-calibration mode)

    // SAMPL 비활성화
    digitalWrite(_SAMPLPIN_MAX14921_pin, LOW);
    delayMicroseconds(MD_AK35_LEVEL_SHIFTING_DELAY_MAX);

    // 캘리브레이션 시작
    rxData = MD_AK35_SpiTransfer24(0, 0, command.cmd);

    // CS rising edge로 캘리브레이션 시작
    digitalWrite(_CS_pin, HIGH);
    delayMicroseconds(MD_AK35_obj.acqCellSettlingTimeUsec);

    // 캘리브레이션 완료 대기
    delay(10); // 캘리브레이션 완료 대기

    Serial.println("Buffer amplifier calibration completed");

    // 일반 동작 모드로 복귀
    spiCmd = 0; // 모든 비트 클리어
    MD_AK35_SpiTransfer24(0, 0, spiCmd);
  }
  else
  {
    if (MD_AK35_obj.acqCellNumber != 0)
    {
      MD_AK35_Cmd_AcquireCell(MD_AK35_obj.acqCellNumber,
                              &adcVal[0],
                              &spiRxData[0]);
    }
  }
}
