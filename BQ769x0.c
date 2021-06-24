/*
TO DO:
- update for bms 10 cell, 15 cell

*/

#include <math.h>
#include "BQ769x0.h"
#include "registers.h"

#include "Function_define.h"

static unsigned char _crc8_ccitt_update (unsigned char inCrc, unsigned char inData)
{
  unsigned char i;
  unsigned char datax;
  datax = inCrc ^ inData;

  for ( i = 0; i < 8; i++ )
  {
    if (( datax & 0x80 ) != 0 )
    {
      datax <<= 1;
      datax ^= 0x07;
    }
    else datax <<= 1;
  }

  return datax;
}
/*	I2C Configtion zone	*/
static void I2C_SI_Check(void)
{
	if (I2STAT == 0x00)
	{
		set_STO;
		SI = 0;
		if(SI)
		{
			clr_I2CEN;
			set_I2CEN;
			clr_SI;
			clr_I2CEN;		
		} 	
	}	
}
static bit I2C_Start()
{
	set_STA;                                /* Send Start bit to I2C EEPROM */
    clr_SI;
    while (!SI);
    if (I2STAT != 0x08){return 0;}
    else{return 1;}
}

static void I2C_Stop()
{
	clr_SI;
    set_STO;
	while (STO)                        /* Check STOP signal */
	{
		I2C_SI_Check();
	}
}

static bit writeRegister(unsigned char Reg_Add, char datax)
{
	unsigned char crc = 0;
	crc = _crc8_ccitt_update(crc, 0x10);
	crc = _crc8_ccitt_update(crc, Reg_Add);
	crc = _crc8_ccitt_update(crc, datax);
	/* Step1 */
	set_STA;                                /* Send Start bit to I2C EEPROM */
    clr_SI;
    while (!SI);
    if (I2STAT != 0x08){return 0;}
	/* Step2 */
	clr_STA; 
	I2DAT = ((unsigned char)BQ_Addr<<1)|WR;   
    clr_SI;
    while (!SI);
    if (I2STAT != 0x18){return 0;}
    /* Step3 */
    I2DAT = Reg_Add;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x28){return 0;}
    
    I2DAT = datax;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x28){return 0;}
	
	I2DAT = crc;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x28){return 0;}
    
    I2C_Stop();
	return 1;
}

static void writeBuffer(unsigned char Reg_Add, unsigned char *buff, int lenght)
{
	int i;
	/* Step1 */
	I2C_Start();
	/* Step2 */
	I2DAT = ((unsigned char)BQ_Addr<<1)|WR;
    clr_STA;                                /* Clear STA and Keep SI value in I2CON */    
    clr_SI;
    while (!SI);
    if (I2STAT != 0x18){return;}
    /* Step3 */
    I2DAT = Reg_Add;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x28){return;}
    for(i=0;i<lenght;i++)
	{
    	I2DAT = buff[i];                      /* Send data to EEPROM */
        clr_SI;
        while (!SI);
        if (I2STAT != 0x28){return;} 
	}
	I2C_Stop();
}
unsigned char readRegister(unsigned char Reg_Add)
{
	char x;	
	/* Step1 */
	I2C_Start();
	/* Step2 */
	I2DAT = ((unsigned char)BQ_Addr<<1)|WR;
    clr_STA;                                /* Clear STA and Keep SI value in I2CON */    
    clr_SI;
    while (!SI);
    if (I2STAT != 0x18){return 0;}
    /* Step3 */
    I2DAT = Reg_Add;          
    clr_SI;
    while (!SI);
    if (I2STAT != 0x28){return 0;}
    /* Step4 */
    set_STA;                                /* Repeated START */
    clr_SI; 
    while (!SI);
    if (I2STAT != 0x10){return 0;}
    /* Step4 */
    clr_STA;
    I2DAT = ((unsigned char)BQ_Addr<<1)|RD;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x40){return 0;}
    /* Step5 */
    set_AA;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x50){return 0;}
    x = I2DAT;
     /* Step6 */
    clr_AA;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x58){return 0;}
    I2C_Stop();
	return x;
}
static void readBuffer(unsigned char Reg_Add, unsigned char *buff, int numbyte)
{
	int i;
	/* Step1 */
	I2C_Start();
	/* Step2 */
	I2DAT = ((unsigned char)BQ_Addr<<1)|WR;
    clr_STA;                                /* Clear STA and Keep SI value in I2CON */    
    clr_SI;
    while (!SI);
    if (I2STAT != 0x18){return;}
    /* Step3 */
    I2DAT = Reg_Add;          
    clr_SI;
    while (!SI);
    if (I2STAT != 0x28){return;}
    /* Step4 */
    set_STA;                                /* Repeated START */
    clr_SI; 
    while (!SI);
    if (I2STAT != 0x10){return;}

    I2DAT = ((unsigned char)BQ_Addr<<1)|RD;
	clr_STA;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x40){return;}
	
	
    for(i=0;i<numbyte;i++)
    {
    	set_AA;
	    clr_SI;
	    while (!SI);
	    if (I2STAT != 0x50){return;}
	    buff[i] = I2DAT;
	}
	clr_AA;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x58){return;}
    I2C_Stop();
}

/*--------------------------------*/
static bit determineAddressAndCrc(void)
{
	if(writeRegister(CC_CFG, 0x19)){
		char x = readRegister(CC_CFG);
		if (x == 0x19)
		{
			return 1;
		}
	}
  	return 0;
}
bit BQ769x0_Init(char bqType)
{
	int i;
	if (bqType == bq76920) {numberOfCells = 5;}
	else if (bqType == bq76930) {numberOfCells = 10;}
	else {numberOfCells = 15;}
	
	for (i = 0; i < numberOfCells; i++) {
		cellVoltages[i] = 0;
	}
	
	if(determineAddressAndCrc())
	{
		if(!writeRegister((char)SYS_CTRL1, 0x18)) {return 0;}
		if(!writeRegister((char)SYS_CTRL2, 0x23)) {return 0;}		// switch CC_EN on, OFF charge, OFF Discharge
		//Set interupt Pin
			
		adcOffset = (signed int) readRegister(ADCOFFSET);  // convert from 2's complement
		adcGain = 365 + (((readRegister(ADCGAIN1) & 0x0C) << 1) | ((readRegister(ADCGAIN2) & 0xE0) >> 5)); // uV/LSB
		return 1;
	}
	else
	{
		return 0;
	}
}

int BQ769x0_checkStatus()
{
	regSYS_STAT_t sys_stat;
	sys_stat.regByte = readRegister(SYS_STAT);
	
	if (sys_stat.bits.CC_READY == 1) {
		BQ769x0_updateCurrent(1);
 	}
	
	if(sys_stat.regByte&0x3F)
	{
		if (sys_stat.regByte & 0x20) { // XR error
			if (ErrSec%3 == 0) {
				writeRegister(SYS_STAT, 0x20);
			}
        }
		if (sys_stat.regByte & 0x10) { //Alert error
			if (ErrSec%3 == 0) {
				writeRegister(SYS_STAT, 0x10);
			}
        }
		if (sys_stat.regByte & 0x08) { // UV error
			BQ769x0_updateVoltages();
			if (cellVoltages[idCellMinVoltage] > minCellVoltage){
				writeRegister(SYS_STAT, 0x08);
			}
		}
		if (sys_stat.regByte & 0x04) { // OV error
			BQ769x0_updateVoltages();
			if (cellVoltages[idCellMaxVoltage] < maxCellVoltage) {
				writeRegister(SYS_STAT, 0x04); 
			}
		}
		if (sys_stat.regByte & 0x02) { // SCD
			if(ErrSec%30 == 0){
				writeRegister(SYS_STAT, 0x02);
			}
		}
		if (sys_stat.regByte & 0x01) { // OCD
			if(ErrSec%30 == 0){
				writeRegister(SYS_STAT, 0x01);
			}
		}
		ErrSec++;
	}
	else{
		ErrSec = 1;
	}
	return sys_stat.regByte;
}
/*	------------------MEASUREMENT-----------------------*/
void BQ769x0_updateTemperatures()
{
	float tmp = 0;
	unsigned int adcVal = 0;
	unsigned int vtsx = 0;
	unsigned int rts = 0;
	
	adcVal = ((unsigned int)readRegister(TS1_HI_BYTE)<<8)|readRegister(TS1_LO_BYTE);
	vtsx = adcVal * 0.382; // mV
	rts = 10000.0*vtsx/(3300.0-vtsx); // Ohm
	tmp = 1.0/(1.0/(273.15+25) + (1.0/thermistorBetaValue)*log(rts/10000.0));
	temperatures[0] = (tmp - 273.15) * 100.0;
}

void BQ769x0_updateCurrent(bit ignoreCCReadyFlag)
{
	unsigned int adcVal = 0;
	signed char stat = readRegister(SYS_STAT);
	
	if ((ignoreCCReadyFlag == 1)||((stat&0x80)!=0))
	{
		adcVal = ((unsigned int)readRegister(CC_HI_BYTE)<<8)|readRegister(CC_LO_BYTE);
		batCurrent = adcVal * (8.44/shuntResistorValue_mOhm);  // mA
		
		if (batCurrent > -10 && batCurrent < 10)
		{
		  batCurrent = 0;
		}
		
		writeRegister(SYS_STAT, 0x80);  // Clear CC ready flag	
    }
}

void BQ769x0_updateVoltages()
{
	long adcVal = 0;
	long adcValPack;
	unsigned char buff[4];
	int connectedCells = 0;
	int i;
	
	idCellMaxVoltage = 0; //resets to zero before writing values to these vars
  	idCellMinVoltage = 0;
	
	for(i=0;i<numberOfCells;i++)
	{
		readBuffer((char)VC1_HI_BYTE + i*2, buff, 4);
		// combine VCx_HI and VCx_LO bits and calculate cell voltage
		adcVal = ((unsigned int)(buff[0] & 0x3F) << 8) | buff[2]; 
		cellVoltages[i] = adcVal * adcGain/1000 + adcOffset;	// calculate real voltage in mV
		if (cellVoltages[i] > 500) {  connectedCells++;}
		if (cellVoltages[i] > cellVoltages[idCellMaxVoltage]) {
			idCellMaxVoltage = i;
		}
		if (cellVoltages[i] < cellVoltages[idCellMinVoltage] && cellVoltages[i] > 500) {
		  idCellMinVoltage = i;
		}
	}
	adcValPack = ((unsigned int)readRegister(BAT_HI_BYTE)<<8)|readRegister(BAT_LO_BYTE);
	batVoltage = 4 * adcGain * adcValPack / 1000 + (connectedCells * adcOffset);
}

float BQ769x0_getCellVoltage(char idCell)
{
  return cellVoltages[idCell-1]/1000.0;
}
long BQ769x0_getBatteryVoltage()
{
  return batVoltage;
}
int QB769x0_getCurrent(){
	return batCurrent;
}
int QB769x0_setCurrent()
{
	signed char stat = readRegister(SYS_CTRL2);
	if(!writeRegister((char)SYS_CTRL2, 0x20|stat)) {return 0;}
	else{return 1;}
}
int QB769x0_getTemp()
{
	return temperatures[0];
}
	
/*	----------------PROTECTIONS--------------------*/
long BQ769x0_setShortCircuitProtection(long current_mA, int delay_us)
{
	int i;
	regPROTECT1_t protect1;
	protect1.bits.RSNS = 1;
	
	protect1.bits.SCD_THRESH = 0;
	for(i = sizeof(SCD_threshold_setting)/sizeof(SCD_threshold_setting[0])-1;i>0;i--)
	{
		if(current_mA*shuntResistorValue_mOhm/1000 >= SCD_threshold_setting[i])
		{
			protect1.bits.SCD_THRESH = i;
			break;
		}
    }
	protect1.bits.SCD_DELAY = 0;
	for (i = sizeof(SCD_delay_setting)/sizeof(SCD_delay_setting[0])-1;i>0;i--)
	{
		if (delay_us >= SCD_delay_setting[i])
		{
		  protect1.bits.SCD_DELAY = i;
		  break;
		}
	}
	writeRegister(PROTECT1, protect1.regByte&0xDF);
	return (long)SCD_threshold_setting[protect1.bits.SCD_THRESH]*1000/shuntResistorValue_mOhm;
}
long BQ769x0_setOvercurrentDischargeProtection(long current_mA, int delay_ms)
{
	int i;
	regPROTECT2_t protect2;
	protect2.bits.OCD_THRESH = 0;
	for (i = sizeof(OCD_threshold_setting) / sizeof(OCD_threshold_setting[0]) - 1; i > 0; i--) {
		if (current_mA * shuntResistorValue_mOhm / 1000 >= OCD_threshold_setting[i]) {
		  protect2.bits.OCD_THRESH = i;
		  break;
		}
	}

	protect2.bits.OCD_DELAY = 0;
	for (i = sizeof(OCD_delay_setting) / sizeof(OCD_delay_setting[0]) - 1; i > 0; i--) {
		if (delay_ms >= OCD_delay_setting[i]) {
		  protect2.bits.OCD_DELAY = i;
		  break;
		}
	}
	writeRegister(PROTECT2, protect2.regByte&0x7F);
	return (long)OCD_threshold_setting[protect2.bits.OCD_THRESH]*1000/shuntResistorValue_mOhm;
}
int BQ769x0_setCellUndervoltageProtection(int voltage_mV, int delay_s)
{
	regPROTECT3_t protect3;
	unsigned char uv_trip = 0;
	int i;
	
	if(voltage_mV<1555){                          
		minCellVoltage = 1555;
	}
	else if(voltage_mV>3085){
		uv_trip = 0xFE;                          
		minCellVoltage = 3085;
	}
	else{
		minCellVoltage = voltage_mV;
		uv_trip = ((((long)voltage_mV - adcOffset) * 1000 / adcGain) >> 4) & 0x00FF;
	}
	
	uv_trip += 1;
	writeRegister(UV_TRIP, uv_trip);
	
	protect3.regByte = readRegister(PROTECT3);
	protect3.bits.UV_DELAY = 0;
	for (i = sizeof(UV_delay_setting)-1; i > 0; i--){
		if (delay_s >= UV_delay_setting[i]) {
		  protect3.bits.UV_DELAY = i;
		  break;
		}
	}
	writeRegister(PROTECT3, protect3.regByte);
	return ((1<<12)|(uv_trip << 4)) * adcGain/1000+adcOffset;
}
int BQ769x0_setCellOvervoltageProtection(int voltage_mV, int delay_s)
{
	regPROTECT3_t protect3;
	unsigned char ov_trip = 0;
	int i;		
	if(voltage_mV<3100){
		maxCellVoltage = 3100;
	}
	else if(voltage_mV>4641){
		maxCellVoltage  = 4641;
		ov_trip = 0xFF;
	}
	else{
		maxCellVoltage = voltage_mV;
		ov_trip = ((((long)voltage_mV - adcOffset)*1000/adcGain)>>4)&0x00FF;
	}
	writeRegister(OV_TRIP, ov_trip);
  
	protect3.regByte = readRegister(PROTECT3);
	protect3.bits.OV_DELAY = 0;
	for (i = sizeof(OV_delay_setting)-1; i > 0; i--) {
		if (delay_s >= OV_delay_setting[i]) {
		  protect3.bits.OV_DELAY = i;
		  break;
		}
	}
	writeRegister(PROTECT3, protect3.regByte);
  // returns the actual current threshold value
	return ((long)1 << 13 | ov_trip << 4) * adcGain / 1000 + adcOffset;
}
void BQ769x0_setTemperatureLimits(int minDischarge_degC, int maxDischarge_degC, int minCharge_degC, int maxCharge_degC)
{
  // Temperature limits (°C/10)
  minCellTempDischarge = minDischarge_degC * 100;
  maxCellTempDischarge = maxDischarge_degC * 100;
  minCellTempCharge = minCharge_degC * 100;
  maxCellTempCharge = maxCharge_degC * 100;  
}
/*	----------------CONTROL--------------------*/
bit BQ769x0_enableCharging(unsigned char state)
{
  if ((BQ769x0_checkStatus()&0x3F) == 0 &&
	cellVoltages[idCellMaxVoltage] < maxCellVoltage &&
	temperatures[0] < maxCellTempCharge &&
    temperatures[0] > minCellTempCharge)
  {
    unsigned char sys_ctrl2;
    sys_ctrl2 = readRegister(SYS_CTRL2);
	if(state){writeRegister(SYS_CTRL2, sys_ctrl2|state);}  // switch CHG on
	else{writeRegister(SYS_CTRL2, sys_ctrl2&0xFE);}
    return 1;
  }
  else {return 0;}
}
bit BQ769x0_enableDischarging(unsigned char state)
{
	if ((BQ769x0_checkStatus()&0x3F) == 0 &&
    cellVoltages[idCellMinVoltage] > minCellVoltage &&
    temperatures[0] < maxCellTempDischarge &&
    temperatures[0] > minCellTempDischarge)
	{
		unsigned char sys_ctrl2;
		sys_ctrl2 = readRegister(SYS_CTRL2);
		   
		if(state){writeRegister(SYS_CTRL2, sys_ctrl2|0x02);}  // switch DSG on
		else{writeRegister(SYS_CTRL2, sys_ctrl2&0xFD);}
		return 1;
	}
	else{return 0;}
}
