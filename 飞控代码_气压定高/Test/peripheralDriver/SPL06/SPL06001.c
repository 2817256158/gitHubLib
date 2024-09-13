//#ifndef SPL06_01_C
//#define SPL06_01_C

#include "spl06001.h"
#include <stdio.h>
#include <math.h>
static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;

void spl0601_write(uint8_t hwadr, uint8_t regadr, uint8_t val);
uint8_t spl0601_read(uint8_t hwadr, uint8_t regadr);
void spl0601_get_calib_param(void);

/*****************************************************************************
 Function: spl0601_write
 Description: this function will write data to specofic register through software I2C bus
 Input:  uint8 hwadr   hardware I2C address
         uint8 regadr  register address
         uint8 val     write-in value          
 Output: 
 Return: 
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_write(uint8_t hwadr, uint8_t regadr, uint8_t val)
{
  HAL_I2C_Mem_Write(&hi2c2,hwadr<<1,regadr,I2C_MEMADD_SIZE_8BIT,&val,1,100);
}

/*****************************************************************************
 Function: spl0601_read
 Description: this function will read register data through software I2C bus
 Input: uint8 hwadr   hardware I2C address
        uint8 regadr  register address        
 Output: 
 Return: uint8 readout value
 Calls: 
 Called By: 
*****************************************************************************/
uint8_t spl0601_read(uint8_t hwadr, uint8_t regadr)
{
    static uint8_t val = 0;
     HAL_I2C_Mem_Read(&hi2c2,hwadr<<1,regadr,I2C_MEMADD_SIZE_8BIT,&val,1,100);
    return val;
}

/*****************************************************************************
 Function: spl0601_init
 Description: initialization
 Input: void             
 Output: 
 Return: void 
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_init(void)
{   uint8_t temp;
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
    HAL_I2C_Mem_Read(&hi2c2,(HW_ADR<<1),0x0d,I2C_MEMADD_SIZE_8BIT,&temp,1,HAL_MAX_DELAY);
    p_spl0601->chip_id = temp;
    printf("ID:0x%x\r\n",temp);
    spl0601_get_calib_param();
    // sampling rate = 4Hz; Pressure oversample = 32;
    spl0601_rateset(PRESSURE_SENSOR,128, 32);   
    // sampling rate = 4Hz; Temperature oversample = 32; 
    spl0601_rateset(TEMPERATURE_SENSOR,32, 8);
    //Start background measurement
    
}

/*****************************************************************************
 Function: spl0601_rateset
 Description: set sample rate and over sample rate per second for specific sensor
 Input:     uint8 u8OverSmpl  oversample rate         Maximal = 128
            uint8 u8SmplRate  sample rate(Hz) Maximal = 128
            uint8 iSensor     0: Pressure; 1: Temperature 
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_rateset(uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<4);
            break;
        case 4:
            reg |= (2<<4);
            break;
        case 8:
            reg |= (3<<4);
            break;
        case 16:
            reg |= (4<<4);
            break;
        case 32:
            reg |= (5<<4);
            break;
        case 64:
            reg |= (6<<4);
            break;
        case 128:
            reg |= (7<<4);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == PRESSURE_SENSOR)
    {
        p_spl0601->i32kP = i32kPkT;
        spl0601_write(HW_ADR, 0x06, reg);
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x04);
        }
        else
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg & (~0x04));
        }
    }
    if(iSensor == TEMPERATURE_SENSOR)
    {
        p_spl0601->i32kT = i32kPkT;
        spl0601_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x08);
        }
        else
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg & (~0x08));
        }
    }

}

/*****************************************************************************
 Function: spl0601_get_calib_param
 Description: obtain the calibrated coefficient
 Input: void     
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_get_calib_param(void)
{
    uint32_t h;
    uint32_t m;
    uint32_t l;
    h =  spl0601_read(HW_ADR, 0x10);
    l  =  spl0601_read(HW_ADR, 0x11);
    p_spl0601->calib_param.c0 = (int16_t)h<<4 | l>>4;
    p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0&0x0800)?(0xF000|p_spl0601->calib_param.c0):p_spl0601->calib_param.c0;
    h =  spl0601_read(HW_ADR, 0x11);
    l  =  spl0601_read(HW_ADR, 0x12);
    p_spl0601->calib_param.c1 = (int16_t)(h&0x0F)<<8 | l;
    p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1&0x0800)?(0xF000|p_spl0601->calib_param.c1):p_spl0601->calib_param.c1;
    h =  spl0601_read(HW_ADR, 0x13);
    m =  spl0601_read(HW_ADR, 0x14);
    l =  spl0601_read(HW_ADR, 0x15);
    p_spl0601->calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00&0x080000)?(0xFFF00000|p_spl0601->calib_param.c00):p_spl0601->calib_param.c00;
    h =  spl0601_read(HW_ADR, 0x15);
    m =  spl0601_read(HW_ADR, 0x16);
    l =  spl0601_read(HW_ADR, 0x17);
    p_spl0601->calib_param.c10 = (int32_t)(h&0x0F)<<16 | (int32_t)m<<8 | l;
    p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10&0x080000)?(0xFFF00000|p_spl0601->calib_param.c10):p_spl0601->calib_param.c10;
    h =  spl0601_read(HW_ADR, 0x18);
    l  =  spl0601_read(HW_ADR, 0x19);
    p_spl0601->calib_param.c01 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1A);
    l  =  spl0601_read(HW_ADR, 0x1B);
    p_spl0601->calib_param.c11 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1C);
    l  =  spl0601_read(HW_ADR, 0x1D);
    p_spl0601->calib_param.c20 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1E);
    l  =  spl0601_read(HW_ADR, 0x1F);
    p_spl0601->calib_param.c21 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x20);
    l  =  spl0601_read(HW_ADR, 0x21);
    p_spl0601->calib_param.c30 = (int16_t)h<<8 | l;
}

/*****************************************************************************
 Function: spl0601_start_temperature
 Description: start one measurement for temperature
 Input: void    
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_start_temperature(void)
{
    spl0601_write(HW_ADR, 0x08, 0x02);
}

/*****************************************************************************
 Function: spl0601_start_pressure
 Description: start one measurement for pressure
 Input: void       
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/

void spl0601_start_pressure(void)
{
    spl0601_write(HW_ADR, 0x08, 0x01);
}
/*****************************************************************************
 Function: spl0601_start_continuous
 Description: Select mode for the continuously measurement
 Input: uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature        
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_start_continuous(uint8_t mode)
{
    spl0601_write(HW_ADR, 0x08, mode+4);
}

void spl0601_stop(void)
{
    spl0601_write(HW_ADR, 0x08, 0);
}

/*****************************************************************************
 Function: spl0601_get_raw_temp
 Description:obtain the original temperature value and turn them into 32bits-integer 
 Input: void          
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_get_raw_temp(void)
{

    static uint8_t temp[3];
    HAL_I2C_Mem_Read(&hi2c2,HW_ADR<<1,0x03,I2C_MEMADD_SIZE_8BIT,temp,3,100);
    p_spl0601->i32rawTemperature = ((int32_t)temp[0]<<16 )| ((int32_t)temp[1]<<8 )| ((int32_t)temp[2]);
    p_spl0601->i32rawTemperature= (p_spl0601->i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
}

/*****************************************************************************
 Function: spl0601_get_raw_pressure
 Description: obtain the original pressure value and turn them into 32bits-integer
 Input: void       
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_get_raw_pressure(void)
{
    static uint8_t temp[3];
    HAL_I2C_Mem_Read(&hi2c2,HW_ADR<<1,0x00,I2C_MEMADD_SIZE_8BIT,temp,3,100);    
    p_spl0601->i32rawPressure = ((int32_t)temp[0]<<16) |( (int32_t)temp[1]<<8) | ((int32_t)temp[2]);
    p_spl0601->i32rawPressure= (p_spl0601->i32rawPressure&0x800000) ? (0xFF000000|p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
}

/*****************************************************************************
 Function: spl0601_get_temperature
 Description:  return calibrated temperature value base on original value.
 Input: void          
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
float spl0601_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 Function: spl0601_get_pressure
 Description: return calibrated pressure value base on original value.
 Input: void            
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/

float spl0601_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / (float)p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc* p_spl0601->calib_param.c30);
    qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}
/*后续添加*/

#define Total_Number_24 16777216.0
#define PRS_CFG_Addr 0x06
#define TMP_CFG_Addr 0x07

#define k_SPS1 524288.0
#define k_SPS2 1572864.0
#define k_SPS4 3670016.0
#define k_SPS8 7864320.0
#define k_SPS16 253952.0
#define k_SPS32 516096.0
#define k_SPS64 1040384.0
#define k_SPS128 2088960.0
 
#define PSR_B2_Addr 0x00
#define PSR_B1_Addr 0x01
#define PSR_B0_Addr 0x02
#define TMP_B2_Addr 0x03
#define TMP_B1_Addr 0x04
#define TMP_B0_Addr 0x05
#define PRS_CFG_Addr 0x06
#define TMP_CFG_Addr 0x07
#define MEAS_CFG_Addr 0x08
#define CFG_REG_Addr 0x09
#define RESET_Addr 0x0C
#define ID_Addr 0x0D
 
#define k_SPS1 524288.0
#define k_SPS2 1572864.0
#define k_SPS4 3670016.0
#define k_SPS8 7864320.0
#define k_SPS16 253952.0
#define k_SPS32 516096.0
#define k_SPS64 1040384.0
#define k_SPS128 2088960.0
#define Temp_c0_Addr 0x10
#define Temp_c1_Addr  0x11
#define Temp_c2_Addr  0x12
 
#define Press_c0_Addr  0x13
#define Press_c1_Addr  0x14
#define Press_c2_Addr  0x15
#define Press_c3_Addr  0x16
#define Press_c4_Addr  0x17
#define Press_c5_Addr  0x18
#define Press_c6_Addr  0x19
#define Press_c7_Addr  0x1A
#define Press_c8_Addr  0x1B
#define Press_c9_Addr  0x1C
#define Press_c10_Addr  0x1D
#define Press_c11_Addr  0x1E
#define Press_c12_Addr  0x1F
#define Press_c13_Addr  0x20
#define Press_c14_Addr  0x21
 
#define Total_Number_24 16777216.0
#define Total_Number_20 1048576.0
#define Total_Number_16 65536.0
#define Total_Number_12 4096.0

void Config_Read(uint8_t* Config_Press,uint8_t* Config_Temp)
{
    *Config_Press = spl0601_read(HW_ADR,PRS_CFG_Addr);
    *Config_Temp = spl0601_read(HW_ADR,TMP_CFG_Addr);
}
float Scale_factor(uint8_t Config_k)
{
	float k;
	switch(Config_k)
	{
		case 0: k = k_SPS1;break;
		case 1: k = k_SPS2;break;
        case 2: k = k_SPS4;break;
		case 3: k = k_SPS8;break;
		case 4: k = k_SPS16;break;
		case 5: k = k_SPS32;break;
        case 6: k = k_SPS64;break;
        case 7:	k = k_SPS128;break;	 	
	}
	return k;
}
float Temperature_conversion(uint32_t Temp_Data,float k)
{
	float Temperature;
	int Temp;
	if(Temp_Data&0x800000)
	{
		Temp = Temp_Data-Total_Number_24;
	}
	else
	{
		Temp = Temp_Data;
	}
	Temperature = Temp/k;
	return Temperature;
}
float Pressure_conversion(uint32_t Pressure_Data,float k)
{
	float Pressure;
	int Press;
	if(Pressure_Data&0x800000)
	{
		Press = Pressure_Data-Total_Number_24;
	}
	else
	{
		Press = Pressure_Data;
	}
	Pressure = Press/k;
	return Pressure;
}

float Correcting_Pressure(int *Pressure_Para,float Pressure,float Temperature)
{
	float	Corr_Pressure;
	Corr_Pressure = Pressure_Para[0]+ Pressure*(Pressure_Para[1]+Pressure*(Pressure_Para[4]+Pressure*Pressure_Para[6]))+Temperature*Pressure_Para[2]+Temperature*Pressure*(Pressure_Para[3]+Pressure*Pressure_Para[5]);
	return Corr_Pressure;
}
 
float Correcting_Temperature(int *Temperature_Para,float Temperature)
{	
	float Corr_Temperature;
	Corr_Temperature = Temperature_Para[0]*0.5+Temperature_Para[1]*Temperature;
	return Corr_Temperature;
}
uint8_t SPL06_Read_Byte(uint8_t Seg_Addr)//补偿函数 Parameter_Reading
{
   return spl0601_read(HW_ADR,Seg_Addr);
}
void Parameter_Reading(int *Pressure_Para,int *Temperature_Para)
{
	uint8_t Temp_Config0,Temp_Config1,Temp_Config2;
	uint8_t Press_Config0,Press_Config1,Press_Config2,Press_Config3,Press_Config4;
	uint8_t Press_Config5,Press_Config6,Press_Config7,Press_Config8,Press_Config9;
	uint8_t Press_Config10,Press_Config11,Press_Config12,Press_Config13,Press_Config14;	 
	//Temperature
	Temp_Config0 = SPL06_Read_Byte(Temp_c0_Addr);
	Temp_Config1 = SPL06_Read_Byte(Temp_c1_Addr);
	Temp_Config2 = SPL06_Read_Byte(Temp_c2_Addr);
	Temperature_Para[0] = (Temp_Config0<<4)+((Temp_Config1&0xF0)>>4);
	if(Temperature_Para[0]&0x0800) Temperature_Para[0] = Temperature_Para[0]-Total_Number_12;
	Temperature_Para[1] = ((Temp_Config1&0x0F)<<8)+Temp_Config2;
	if(Temperature_Para[1]&0x0800) Temperature_Para[1] = Temperature_Para[1]-Total_Number_12;
	//Pressure
	Press_Config0 = SPL06_Read_Byte(Press_c0_Addr);
	Press_Config1 = SPL06_Read_Byte(Press_c1_Addr);
	Press_Config2 = SPL06_Read_Byte(Press_c2_Addr);
	Press_Config3 = SPL06_Read_Byte(Press_c3_Addr);
	Press_Config4 = SPL06_Read_Byte(Press_c4_Addr);
	Press_Config5 = SPL06_Read_Byte(Press_c5_Addr);
	Press_Config6 = SPL06_Read_Byte(Press_c6_Addr);
	Press_Config7 = SPL06_Read_Byte(Press_c7_Addr);
	Press_Config8 = SPL06_Read_Byte(Press_c8_Addr);
	Press_Config9 = SPL06_Read_Byte(Press_c9_Addr);
	Press_Config10 = SPL06_Read_Byte(Press_c10_Addr);
	Press_Config11 = SPL06_Read_Byte(Press_c11_Addr);
	Press_Config12 = SPL06_Read_Byte(Press_c12_Addr);
	Press_Config13 = SPL06_Read_Byte(Press_c13_Addr);
	Press_Config14 = SPL06_Read_Byte(Press_c14_Addr);
	Pressure_Para[0] = (Press_Config0<<12)+(Press_Config1<<4)+((Press_Config2&0xF0)>>4);//c00
	if(Pressure_Para[0]&0x80000) Pressure_Para[0] = Pressure_Para[0] - Total_Number_20;//c00
	Pressure_Para[1] = ((Press_Config2&0x0F)<<16)+ (Press_Config3<<8)+ Press_Config4;//c10
	if(Pressure_Para[1]&0x80000) Pressure_Para[1] = Pressure_Para[1] - Total_Number_20;//c10
	Pressure_Para[2] = (Press_Config5<<8)+Press_Config6;//c01
	if(Pressure_Para[2]&0x8000) Pressure_Para[2] = Pressure_Para[2] - Total_Number_16;//c01
	Pressure_Para[3] = (Press_Config7<<8)+Press_Config8;//c11
	if(Pressure_Para[3]&0x8000) Pressure_Para[3] = Pressure_Para[3] - Total_Number_16;//c11
	Pressure_Para[4] = (Press_Config9<<8)+Press_Config10;//c20
	if(Pressure_Para[4]&0x8000) Pressure_Para[4] = Pressure_Para[4] - Total_Number_16;//c20
	Pressure_Para[5] = (Press_Config11<<8)+Press_Config12;//c21
	if(Pressure_Para[5]&0x8000) Pressure_Para[5] = Pressure_Para[5] - Total_Number_16;//c21
	Pressure_Para[6] = (Press_Config13<<8)+Press_Config14;//c30
	if(Pressure_Para[6]&0x8000) Pressure_Para[6] = Pressure_Para[6] - Total_Number_16;//c30

}

#define STANDARD_ATMOSPHERIC_PRESSURE 101325.0 // 标准大气压（Pa）
#define SEA_LEVEL_PRESSURE 101325.0 // 海平面气压（Pa），通常用作参考压力
float pressure_to_altitude(float pressure) {
    // 确保压力值不为零或负值
    if (pressure <= 0) {
        return -1.0; // 错误值，表示压力值无效
    }

    // 使用海拔高度公式
    // H = 44330 * (1 - (P / P0)^(1 / 5.255))
    // H: 海拔高度（米）
    // P: 传感器读取的压力（Pa）
    // P0: 标准大气压（Pa）

    float altitude = 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 1.0 / 5.255));
		
    return altitude*100;
}

/*后续添加*/

float spl0601_altitude_read(float pressure)
{
	float altitude=0.0f;
	float alt_3=0.0f;
	alt_3 = ( 101000 - pressure ) / 1000.0f;
    altitude = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * ( 101000 - pressure ) * 100.0f ;
	return altitude;
}
float spl06_raw_pressure=0.0f;
float spl06_calibration_altitude_read()
{
	static float altitude_offset=0.0f;
    static float offset_t=0.0f;
	static uint16_t cnt=0;
	float altitude=0.0f;
	spl06_raw_pressure=spl0601_get_pressure();
	if(altitude_offset == 0.0f)//未校准
	{		
		if(cnt > 200)pressure_to_altitude(spl06_raw_pressure);
		else altitude_offset=pressure_to_altitude(spl06_raw_pressure);
		cnt++;
//		if(cnt > 200 && cnt < 300)offset_t += pressure_to_altitude(spl06_raw_pressure);
//    if(cnt == 300)altitude_offset = offset_t / 100.0;
//    if(cnt < 301) cnt++;
	}
	else											 //校准后
	{
		altitude=pressure_to_altitude(spl06_raw_pressure)-altitude_offset;
	}
	return altitude;
}




