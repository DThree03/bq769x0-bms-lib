#ifndef	_INIT_BQ769x0_H__
#define	_INIT_BQ769x0_H__

// can be reduced to save some memory if smaller ICs are used
#define MAX_NUMBER_OF_CELLS 15
#define MAX_NUMBER_OF_THERMISTORS 3

// IC type/size
#define bq76920 1
#define bq76930 2
#define bq76940 3

#define BQ_Addr 		0x08
#define I2C_CLOCK      	0x27
#define WR		        0x00
#define RD              0x01

static volatile unsigned char	ErrSec = 1;

static int adcGain;    // uV/LSB
static int adcOffset;  // mV

//Measurement
static int numberOfCells;
static int cellVoltages[MAX_NUMBER_OF_CELLS];
static long batVoltage;
static long batCurrent;                                // mA
static signed int temperatures[MAX_NUMBER_OF_THERMISTORS];

static int thermistorBetaValue = 3435;
static float shuntResistorValue_mOhm = 0.8;

 // Cell voltage limits (mV)
static char idCellMaxVoltage;
static char idCellMinVoltage;
static int maxCellVoltage;
static int minCellVoltage;

// Temperature limits (°C/10)
static signed int minCellTempCharge;
static signed int minCellTempDischarge;
static int maxCellTempCharge;
static int maxCellTempDischarge;

static bit writeRegister(unsigned char Reg_Add, char datax);

static void writeBuffer(unsigned char Reg_Add, unsigned char *buff, int lenght);

unsigned char readRegister(unsigned char Reg_Add);

static void readBuffer(unsigned char Reg_Add, unsigned char *buff, int numbyte);

/*-------------------------------------------------------------------------------*/
bit determineAddressAndCrc(void);

bit BQ769x0_Init(char bqType);

int BQ769x0_checkStatus();

void BQ769x0_updateTemperatures();

void BQ769x0_updateCurrent(bit ignoreCCReadyFlag);

void BQ769x0_updateVoltages();

float BQ769x0_getCellVoltage(char idCell);

long BQ769x0_getBatteryVoltage();

int QB769x0_getTemp();

int QB769x0_getCurrent();

int QB769x0_setCurrent();

/******		Set Protection	*******/

long BQ769x0_setShortCircuitProtection(long current_mA, int delay_us);

long BQ769x0_setOvercurrentDischargeProtection(long current_mA, int delay_ms);

int BQ769x0_setCellUndervoltageProtection(int voltage_mV, int delay_s);

int BQ769x0_setCellOvervoltageProtection(int voltage_mV, int delay_s);

void BQ769x0_setTemperatureLimits(int minDischarge_degC, int maxDischarge_degC, int minCharge_degC, int maxCharge_degC);

/******		Control		******/

bit BQ769x0_enableCharging(unsigned char state);

bit BQ769x0_enableDischarging(unsigned char state);

#endif
