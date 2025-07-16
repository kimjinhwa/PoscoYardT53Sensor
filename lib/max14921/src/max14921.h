/*
 MAX14921.h - Library for reading from a MAX14921.
 Created by Felix
 */

// ensure this library description is only included once
#ifndef MAX14921_h
#define MAX14921_h

#include "Arduino.h"
#include <inttypes.h>

// library interface description
/**-----------------------------------------------------------------------------
 *
 * Current object parameters
 *
 *------------------------------------------------------------------------------
 */
#define SEL_MAX11163 23    // chip-select MAX11163
#define DATAOUT_MOSI 13   // MOSI 
#define DATAIN_MISO 12    // MISO 
#define SPICLOCK 14  // Clock 
#define SPI_SPEED 1000000
#define MAX_EN 18


#define SEL_MAX14921 5 // chip-select MAX14921
#define SAMPLPIN_MAX14921 19 // SAMPLE pin MAX14921  
/*
ECS 16 R/W 0
0: Cell selection is disabled
1: Cell selection is enabled
SC0 17 R/W 0
[ECS, SC0, SC1, SC2, SC3]
1 – SC0, SC1, SC2, SC3: Selects the cell for voltage readout during hold phase.**
The selected cell voltage is routed to AOUT after the rising CS edge. See Table 2.
0 – 0, 0, 0, 0: AOUT is three-stated and sampling switches are configured for
parasitic capacitance error calibration.
0 – 1, 0, 0, 0: AOUT is three-stated and self-calibration of buffer amplifier offset
voltage is initiated after the following rising CS.
0 – SC0, SC1, 0, 1: Switches the T1, T2. T2 analog inputs directly to AOUT. See
Table 3.
0 – 0, 0, 1, 1: VP/12 (MAX14920) or VP/16 (MAX14921) voltage is routed to AOUT
on the next rising CS
0 – SC0, SC1, 1, 1: Routes and buffers the T1, T2. T3 to AOUT. See Table 3.
SC1 18 R/W 0
SC2 19 R/W 0
SC3 20 R/W 0
SMPLB 21 R/W 0
0: Device in sample phase if SAMPL input is logic-high
1: Device in hold phase
DIAG 22 R/W 0
0: Normal operation
1: Diagnostic enable, 10FA leakage is sunk on all CV_ inputs (CV0–CV16).
LOPW 23 R/W 0
0: Normal operation
1: Low-power mode enabled. Current into LDOIN is reduced to 125FA. Current
into VP is reduced to 1FA.*/
// MAX14921 명령어 비트 구조체 정의
typedef union {
  struct {
    uint8_t ECS : 1;    // Cell selection enabled
    uint8_t SC0 : 1;    // Self-calibration mode 0
    uint8_t SC1 : 1;    // Self-calibration mode 1
    uint8_t SC2 : 1;    // Self-calibration mode 2
    uint8_t SC3 : 1;    // Self-calibration mode 3
    uint8_t SAMPLB : 1; // Sample buffer enable
    uint8_t DIAG : 1;   // Diagnostic mode
    uint8_t LOPW : 1;   // Low power mode
  } bits;
  uint8_t cmd;
} MAX14921_Command_t;


typedef struct {
	uint8_t              tmrId;

  //MF_AFWRK_QUEUENODE_T     thdQNodes[ MD_AK35_THREAD_Q_SIZE ];    /**< Queue nodes  */
  //MF_AFWRK_QUEUE_T         thdQ;                                  /**< Thread queue */
  //MD_AK35_STATE_T          curState;

	uint8_t          acqCellNumber;
  int         acqCellSampleTmrValueMsec;
  int         acqCellRepeatTmrValueMsec;
  int         acqCellSettlingTimeUsec;
  bool        acqScanContinuous;
  bool        acqT1IsEnabled;
  bool        acqT2IsEnabled;
  bool        acqT3IsEnabled;
  int         acqT1SettlingTimeUsec;
  int         acqT2SettlingTimeUsec;
  bool        acqT3SettlingTimeUsec;
  //MI_CLI_REASON_T   acqInterface;

  uint8_t   spiBalanceC01_C08;
  uint8_t   spiBalanceC09_C16;
  uint8_t   spiSamplB;
  uint8_t   spiDiag;
  uint8_t   spiLoPw;
  long   calParErrTmrValueMsec;

  uint8_t   modbusBalanceC01_C08;
  uint8_t   modbusBalanceC09_C16;
  //MD_AK35_USERCFG_T   usrCfg;

} MD_AK35_INSTANCE_T;
typedef enum {TT1=0, TT2=1, AMPERE=2} T_NUMBER;
typedef enum {
    CELL_NORMAL = 0,
    CELL_OPEN = 1,
    CELL_SHORT = 2
} CellStatus;

#define FIFO_SIZE 20

typedef struct {
  uint16_t buffer[FIFO_SIZE];
  int head;
  int count;
} TotalVoltageFIFO;

typedef struct {
  int16_t buffer[FIFO_SIZE];
  int head;
  int count;
} AmpereFIFO;

typedef struct {
  uint16_t buffer[FIFO_SIZE];
  int head;
  int count;
} CellFIFO;
class max14921 {
public:
  CellFIFO cellFIFOs[16];
  TotalVoltageFIFO totalVoltageFIFO;
  AmpereFIFO ampereFIFO;
  void initFIFO();
  uint16_t updateFIFO(uint8_t cellNum, uint16_t newvalue);
  uint16_t updateTotalVoltageFIFO(uint16_t newvalue);
  int16_t updateAmpereFIFO(int16_t newvalue);
  float VREF;
 	uint16_t cellVoltage[ 16 ]    = { 0 };
  uint16_t hasIssueCell = 0x0000;
  int totalVoltage;
  int T123[3]={0};
  static SemaphoreHandle_t dataMutex;
	max14921(int ADC_CS_pin, int CS_pin, int SAMPLPIN_MAX14921_pin);
	void SetCellNumber(uint8_t cellNum);
	void SetBalancing(bool bal);
	void SetOpenWireDetection(bool openwire);
	uint16_t CheckOpenWire();
	uint16_t CheckOpenWire2();
	bool startCalibration();
  bool verifyCalibration();
	void SetSampleTime(int sampletime);
	void SetSettlingTime(int settlingtime);
	void SetRepeatTime(int repeattime);
  void initialize();

	uint16_t MAX11163_ReadData16(void);
	uint16_t MAX11163_ReadData16_2(void);
	CellStatus openWireStatus;
	void MD_AK35_ScanCell(bool calibrationData);
	uint16_t readT123(T_NUMBER Tnumber);
  float calTemperature(uint16_t adcData);
	uint16_t readTotalVoltage();
private:
	long MD_AK35_SpiTransfer24(uint8_t byte1, uint8_t byte2, uint8_t byte3);
	void MD_AK35_Cmd_AcquireCell(uint8_t cellNum,
			int *pAdcCellVoltage,
			long *pSpiRxData );

	int _ADC_CS_pin;
	int _CS_pin;
	int _SAMPLPIN_MAX14921_pin;
};
extern max14921 _max14921;

// 비트 순서 뒤집기 매크로 (SPI MSB First 전송 순서에 맞춤)
#define REVERSE_BITS_8(x) (((x & 0x01) << 7) | \
                           ((x & 0x02) << 5) | \
                           ((x & 0x04) << 3) | \
                           ((x & 0x08) << 1) | \
                           ((x & 0x10) >> 1) | \
                           ((x & 0x20) >> 3) | \
                           ((x & 0x40) >> 5) | \
                           ((x & 0x80) >> 7))

#endif

