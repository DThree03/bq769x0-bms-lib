#ifndef	_INIT_BQ769x0_H__
#define	_INIT_BQ769x0_H__

// can be reduced to save some memory if smaller ICs are used
#define MAX_NUMBER_OF_CELLS 15
#define MAX_NUMBER_OF_THERMISTORS 3

// IC type/size
#define bq76920 1
#define bq76930 2
#define bq76940 3

//#define BQ_Addr 		0x18
#define I2C_CLOCK      	0x27
#define WR		        0x00
#define RD              0x01

int readRegister(unsigned char Reg_Add);
/*-------------------------------------------------------------------------------*/
bit determineAddressAndCrc(void);

bit BQ769x0_Init(char bqType);

int BQ769x0_handleStatus();

void BQ769x0_updateTemperatures();

void BQ769x0_updateVoltages();

int BQ769x0_updateCurrent();

int BQ769x0_getCellVoltage(char idCell);

int BQ769x0_getBatteryVoltage();

int BQ769x0_getTemp();

double BQ769x0_getCurrentmA(char *sign);

void SetshuntResistorValue(float a);
/******		Set Protection	*******/

long BQ769x0_setShortCircuitProtection(unsigned char current_mA, unsigned char delay_us);

long BQ769x0_setOvercurrentDischargeProtection(unsigned char current_mA, unsigned char delay_ms);

int BQ769x0_setCellUndervoltageProtection(int voltage_mV, unsigned char delay_s);

int BQ769x0_setCellOvervoltageProtection(int voltage_mV, unsigned char delay_s);

void BQ769x0_setTemperatureLimits(int minDischarge_degC, int maxDischarge_degC, int minCharge_degC, int maxCharge_degC);

/******		Control		******/

bit BQ769x0_Charging(unsigned char state, int alert);

bit BQ769x0_Discharging(unsigned char state, int alert);

unsigned char Control_Value(void);

void BQ769x0_OffBalancing(void);
	
void BQ769x0_updateCellBalancing(void);

void BQ769x0_setBalancingVoltage(int voltage);

void BQ769x0_setMaxDifferenceBalanceVol(unsigned char mVol);

unsigned char getBalancingValue(int section);

#endif
