#include <math.h>
#include "BQ769x0.h"
#include "registers.h"

#include "Function_define.h"

static unsigned char BQ_Addr;
static unsigned char crcEnable;

static volatile unsigned char	ErrSec = 1;

static int adcGain;    // uV/LSB
static int adcOffset;  // mV

//Measurement
static int numberOfCells;
static int connectedCells;
static int cellVoltages[MAX_NUMBER_OF_CELLS];
static int batVoltage;
static double batCurrentmA;                               // mA
static char currentSign;
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
//Cell Balancing
static int balancingMinCellVoltage_mV = 3500;
static unsigned char balancingMaxVoltageDifference_mV = 3;
static bit balancingActive = 0;
static unsigned char balancingValue[3] = {0};

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

static bit writeRegister(unsigned char Reg_Add, unsigned char datax)
{
	unsigned char crc = 0;
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
	
    if(crcEnable){
        crc = _crc8_ccitt_update(crc, (BQ_Addr<<1)|WR);
        crc = _crc8_ccitt_update(crc, Reg_Add);
        crc = _crc8_ccitt_update(crc, datax);
        I2DAT = crc;
        clr_SI;
        while (!SI);
        if (I2STAT != 0x28){return 0;}
    }
    I2C_Stop();
	return 1;
}

static bit writeBuffer(unsigned char Reg_Add, unsigned char *buff, int lenght)
{
	int i;
    unsigned char crc = 0;
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
    
    if(crcEnable){
        crc = _crc8_ccitt_update(crc, (BQ_Addr<<1)|WR);
        crc = _crc8_ccitt_update(crc, Reg_Add);
    }
    
    for(i=0;i<lenght;i++)
	{
    	I2DAT = buff[i];                      /* Send data to EEPROM */
        clr_SI;
        while (!SI);
        if (I2STAT != 0x28){return 0;}
        if(crcEnable){crc = _crc8_ccitt_update(crc, buff[i]);}
            
	}
    if(crcEnable){
        I2DAT = crc;
        clr_SI;
        while (!SI);
        if (I2STAT != 0x28){return 0;}
    }
    
	I2C_Stop();
    return 1;
}
int readRegister(unsigned char Reg_Add)
{
	int x;	
	/* Step1 */
	I2C_Start();
	/* Step2 */
	I2DAT = (BQ_Addr<<1)|WR;
    clr_STA;                                /* Clear STA and Keep SI value in I2CON */    
    clr_SI;
    while (!SI);
    if (I2STAT != 0x18){return -1;}
    /* Step3 */
    I2DAT = Reg_Add;          
    clr_SI;
    while (!SI);
    if (I2STAT != 0x28){return -1;}
    /* Step4 */
    set_STA;                                /* Repeated START */
    clr_SI; 
    while (!SI);
    if (I2STAT != 0x10){return -1;}
    /* Step4 */
    clr_STA;
    I2DAT = (BQ_Addr<<1)|RD;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x40){return -1;}
    /* Step5 */
    set_AA;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x50){return -1;}
    x = I2DAT;
     /* Step6 */
    clr_AA;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x58){return -1;}
    I2C_Stop();
	return x;
}
static bit readBuffer(unsigned char Reg_Add, unsigned char *buff, int numbyte)
{
	int i;
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

    I2DAT = ((unsigned char)BQ_Addr<<1)|RD;
	clr_STA;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x40){return 0;}
	
    for(i=0;i<numbyte;i++)
    {
    	set_AA;
	    clr_SI;
	    while (!SI);
	    if (I2STAT != 0x50){return 0;}
	    buff[i] = I2DAT;
	}
	clr_AA;
    clr_SI;
    while (!SI);
    if (I2STAT != 0x58){return 0;}
    I2C_Stop();
    
    return 1;
}

/*--------------------------------*/
static void BQ769x0_getCurrentValue()
{
    //signed char status = readRegister(SYS_STAT);
    unsigned int adcVal = 0;
	unsigned char buff[2];
	readBuffer((unsigned char)CC_HI_BYTE, buff, 2);
    
    if(buff[0]&0x80){
        currentSign = 1;
        adcVal = (~buff[0])*256 + (~buff[1]);
    }
    else{
        currentSign = 0;
        adcVal = buff[0]*256 + buff[1];
    }
    batCurrentmA = ((float)adcVal*8.44)/shuntResistorValue_mOhm;  // mA
    if(batCurrentmA == 0){
        currentSign = 0;
    }
	writeRegister(SYS_STAT, 0x80);  // Clear CC ready flag
}

static bit determineAddressAndCrc(void)
{
    crcEnable = 0;
    
    BQ_Addr = 0x18;
	writeRegister(CC_CFG, 0x19);
    //printf("I2C Addr: 0x18, crcEn: 0\n");
    if (readRegister(CC_CFG) == 0x19){return 1;}
	
    BQ_Addr = 0x08;
	writeRegister(CC_CFG, 0x19);
    //printf("I2C Addr: 0x08, crcEn: 0\n");
    if (readRegister(CC_CFG) == 0x19){return 1;}
  	
    crcEnable = 1;
    
    BQ_Addr = 0x18;
	writeRegister(CC_CFG, 0x19);
    //printf("I2C Addr: 0x18, crcEn: 1\n");
    if (readRegister(CC_CFG) == 0x19){return 1;}
	
    BQ_Addr = 0x08;
	writeRegister(CC_CFG, 0x19);
    //printf("I2C Addr: 0x08, crcEn: 1\n");
    if (readRegister(CC_CFG) == 0x19){return 1;}
    
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
		if(!writeRegister((char)SYS_CTRL2, 0x01)) {return 0;}		// switch CC_EN on, OFF charge, OFF Discharge
		//Set interupt Pin
			
		adcOffset = (signed int) readRegister(ADCOFFSET);  // convert from 2's complement
		adcGain = 365 + (((readRegister(ADCGAIN1) & 0x0C) << 1) | ((readRegister(ADCGAIN2) & 0xE0) >> 5)); // uV/LSB
		
		writeRegister((unsigned char)(CELLBAL1), 0x00);
		return 1;
	}
	return 0;
}

int BQ769x0_handleStatus()
{
	unsigned char sys_stat = readRegister((unsigned char)SYS_STAT);
	
	if (sys_stat&0x80) {
		BQ769x0_getCurrentValue();
 	}
	
    if(sys_stat&0x3F)
	{
		if (sys_stat & 0x20) { // XR error
			if (ErrSec%3 == 0) {
				writeRegister(SYS_STAT, 0x20);
			}
        }
		if (sys_stat & 0x10) { //Alert error
			if (ErrSec%3 == 0) {
				writeRegister(SYS_STAT, 0x10);
			}
        }
		if (sys_stat & 0x08) { // UV error
			BQ769x0_updateVoltages();
			if (cellVoltages[idCellMinVoltage] > minCellVoltage){
				writeRegister(SYS_STAT, 0x08);
			}
		}
		if (sys_stat & 0x04) { // OV error
			BQ769x0_updateVoltages();
			if (cellVoltages[idCellMaxVoltage] < maxCellVoltage) {
				writeRegister(SYS_STAT, 0x04); 
			}
		}
		if (sys_stat & 0x02) { // SCD
			if(ErrSec%30 == 0){
				writeRegister(SYS_STAT, 0x02);
			}
		}
		if (sys_stat & 0x01) { // OCD
			if(ErrSec%30 == 0){
				writeRegister(SYS_STAT, 0x01);
			}
		}
		ErrSec++;
	}
	else if(ErrSec > 1){
		ErrSec = 1;
	}
	return sys_stat;
}
/*	------------------MEASUREMENT-----------------------*/
void BQ769x0_updateVoltages()
{
	long adcVal = 0;
	long adcValPack;
	unsigned char buff[4];
	int i;
	connectedCells = 0;
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
		if ((cellVoltages[i] < cellVoltages[idCellMinVoltage]) && (cellVoltages[i] > 500)) {
			idCellMinVoltage = i;
		}
	}
    buff[0] = readRegister(BAT_HI_BYTE);
    buff[1] = readRegister(BAT_LO_BYTE);
    if(((int)buff[0] != -1) && ((int)buff[1] != -1))
    {
       adcValPack = ((unsigned int)buff[0]<<8)|buff[1];
       batVoltage = (int)(4 * adcGain * adcValPack / 1000 + (connectedCells * adcOffset)); 
    }
    else{
       batVoltage = 0; 
    }
}
void BQ769x0_updateTemperatures()
{
	float tmp = 0;
	unsigned int adcVal = 0;
	unsigned int vtsx = 0;
	unsigned int rts = 0;
	
	//adcVal = ((unsigned int)readRegister(TS1_HI_BYTE)<<8)|readRegister(TS1_LO_BYTE);
    adcVal = (int)readRegister(TS1_HI_BYTE)*256 + (int)readRegister(TS1_LO_BYTE);
	vtsx = adcVal * 0.382; // mV
	rts = 10000.0*vtsx/(3300.0-vtsx); // Ohm
	tmp = 1.0/(1.0/(273.15+25) + (1.0/thermistorBetaValue)*log(rts/10000.0));
	temperatures[0] = (tmp - 273.15) * 100.0;
}

int BQ769x0_updateCurrent()
{
	unsigned char stat = readRegister(SYS_CTRL2);
    
	if(writeRegister(SYS_CTRL2, 0x20|stat)) {return 1;}
	else{return 0;}
}
double BQ769x0_getCurrentmA(char *sign)
{
    *sign = currentSign;
	return batCurrentmA;
}
int BQ769x0_getCellVoltage(char idCell)
{
  return cellVoltages[idCell-1];
}
int BQ769x0_getBatteryVoltage()
{
  return batVoltage;
}
int BQ769x0_getTemp()
{
	return temperatures[0];
}
void SetshuntResistorValue(float a)
{
	shuntResistorValue_mOhm = a;
}
/*	----------------PROTECTIONS--------------------*/
long BQ769x0_setShortCircuitProtection(unsigned char current_mA, unsigned char delay_us)
{
	regPROTECT1_t protect1;
	protect1.bits.RSNS = 1;
	protect1.bits.SCD_THRESH = current_mA;
	protect1.bits.SCD_DELAY = delay_us;
	writeRegister(PROTECT1, protect1.regByte&0xDF);
	return (long)SCD_threshold_setting[protect1.bits.SCD_THRESH]/shuntResistorValue_mOhm;
}
long BQ769x0_setOvercurrentDischargeProtection(unsigned char current_mA, unsigned char delay_ms)
{
	regPROTECT2_t protect2;
	protect2.bits.OCD_THRESH = current_mA;
	protect2.bits.OCD_DELAY = delay_ms;
	writeRegister(PROTECT2, protect2.regByte&0x7F);
	return (long)OCD_threshold_setting[protect2.bits.OCD_THRESH]*1000/shuntResistorValue_mOhm;
}
int BQ769x0_setCellUndervoltageProtection(int voltage_mV, unsigned char delay_s)
{
	regPROTECT3_t protect3;
	unsigned char uv_trip = 0;
	
	voltage_mV = 1500 + voltage_mV*100;
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
	protect3.bits.UV_DELAY = delay_s;
	writeRegister(PROTECT3, protect3.regByte);
	return ((1<<12)|(uv_trip << 4)) * adcGain/1000+adcOffset;
}
int BQ769x0_setCellOvervoltageProtection(int voltage_mV, unsigned char delay_s)
{
	regPROTECT3_t protect3;
	unsigned char ov_trip = 0;
	voltage_mV = 3100 + voltage_mV*100;		
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
	protect3.bits.OV_DELAY = delay_s;
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
bit BQ769x0_Charging(unsigned char state, int alert)
{
    regSYS_CTRL2_t sys_ctrl2;
    sys_ctrl2.regByte = readRegister(SYS_CTRL2);
    if(state)
    {
        if((cellVoltages[idCellMaxVoltage] < maxCellVoltage) && (alert&0x3F) == 0 &&
        temperatures[0] < maxCellTempCharge &&
        temperatures[0] > minCellTempCharge)  
        {
            sys_ctrl2.bits.CHG_ON = 1;
        }
        else{return 0;}	
    }
    else{sys_ctrl2.bits.CHG_ON = 0;}
    writeRegister(SYS_CTRL2, sys_ctrl2.regByte);
    return 1;
}
bit BQ769x0_Discharging(unsigned char state, int alert)
{
    regSYS_CTRL2_t sys_ctrl2;
    sys_ctrl2.regByte = readRegister(SYS_CTRL2);
    if(state)
    {
        if((cellVoltages[idCellMinVoltage] > minCellVoltage) && (alert&0x3F) == 0 &&
        temperatures[0] < maxCellTempCharge &&
        temperatures[0] > minCellTempCharge) 
        {
            sys_ctrl2.bits.DSG_ON = 1;
        }
        else{return 0;}
    }
    else{sys_ctrl2.bits.DSG_ON = 0;}
    writeRegister(SYS_CTRL2, sys_ctrl2.regByte);
    return 1;
}
unsigned char Control_Value(void)
{
	return (readRegister(SYS_CTRL2)&0x03);
}
void BQ769x0_OffBalancing(void)
{
	unsigned char numberOfSections = numberOfCells/5;
	unsigned char section;
	for(section = 0; section<numberOfSections; section++)
	{
		if(balancingValue[section] != 0){
			writeRegister((unsigned char)(CELLBAL1+section), 0);
			balancingValue[section] = 0;
		}			
	}
}
void BQ769x0_updateCellBalancing(void)
{
	unsigned char numberOfSections = numberOfCells/5;
	//Check time
	//Check balancing allowed
	if (BQ769x0_handleStatus() == 0 &&
    cellVoltages[idCellMaxVoltage] > balancingMinCellVoltage_mV &&
    (cellVoltages[idCellMaxVoltage] - cellVoltages[idCellMinVoltage]) > balancingMaxVoltageDifference_mV)
	{
		unsigned char i, section, CellBallValue;
		bit balancingFlags;
		for(section = 0; section<numberOfSections; section++)
		{
			CellBallValue = 0;
			balancingFlags = 0;
			for(i=0;i<5;i++)
            {
				if ((cellVoltages[section*5 + i] - cellVoltages[idCellMinVoltage]) > balancingMaxVoltageDifference_mV &&
					(section*5 + i + 1) != idCellMaxVoltage &&
					balancingFlags == 0)
				{
					if((cellVoltages[section*5 + i + 1]<200)&&
						((section*5 + i + 2) == idCellMaxVoltage))
					{
						balancingFlags = 0;
					}
					else{
						CellBallValue |= (1<<i);
						balancingFlags = 1;
					}
                }
				else{
					if(cellVoltages[section*5 + i]>200){
						balancingFlags = 0;
					}
				}
            }
			if(CellBallValue != balancingValue[section]){
				writeRegister((unsigned char)(CELLBAL1+section), CellBallValue);
				balancingValue[section] = CellBallValue;
			}
		}
		balancingActive = 1;
	}
	else if(balancingActive == 1)
	{
		unsigned char section;
		for(section = 0; section<numberOfSections;  section++){
			writeRegister((unsigned char)(CELLBAL1+section), 0x00);
		}
		balancingActive = 0;
	}
}
void BQ769x0_setBalancingVoltage(int voltage)
{
	if((voltage>3000)&&(voltage<4642)){
		balancingMinCellVoltage_mV = voltage;
	}
}
void BQ769x0_setMaxDifferenceBalanceVol(unsigned char mVol)
{
	if((mVol>0)&&(mVol<500)){
		balancingMaxVoltageDifference_mV = mVol;
	}
}
unsigned char getBalancingValue(int section)
{
	if(section<=(numberOfCells/5)){
		return balancingValue[section];
	}
	return 0;
}