#ifndef SPL06001_H
#define SPL06001_H

#include "main.h"
#include "i2c.h"

#define HW_ADR 0x76       //芯片5脚（SDO）接地，地址为0x76; 此脚悬空，地址Wie0x77
#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

typedef struct
{	
	struct  //高度数据
	{
		float rate;
		float bara_height; 
		float ultra_height;
		float ultra_baro_height;
	}High;		 
}_st_FlightData;

struct spl0601_calib_param_t {	
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
};

struct spl0601_t {	
    struct spl0601_calib_param_t calib_param;/**<calibration data*/	
    uint8_t chip_id; /**<chip id*/	
    int32_t i32rawPressure;
    int32_t i32rawTemperature;
    int32_t i32kP;    
    int32_t i32kT;
};

void  spl0601_init(void);
void  spl0601_rateset(uint8_t iSensor, uint8_t u8OverSmpl, uint8_t u8SmplRate);
void  spl0601_start_temperature(void);
void  spl0601_start_pressure(void);
void  spl0601_start_continuous(uint8_t mode);
void  spl0601_get_raw_temp(void);
void  spl0601_get_raw_pressure(void);
float spl0601_get_temperature(void);
float spl0601_get_pressure(void);//读取气压原始数据

void Config_Read(uint8_t* Config_Press,uint8_t* Config_Temp);
float Scale_factor(uint8_t Config_k);
float Temperature_conversion(uint32_t Temp_Data,float k);
float Pressure_conversion(uint32_t Pressure_Data,float k);
float Correcting_Pressure(int *Pressure_Para,float Pressure,float Temperature);
float Correcting_Temperature(int *Temperature_Para,float Temperature);
void Parameter_Reading(int *Pressure_Para,int *Temperature_Para);

float pressure_to_altitude(float pressure);
float spl0601_altitude_read(float pressure);//读取气压转化后的海拔数据
float spl06_calibration_altitude_read();//读取校准后的海拔数据
void High_Data_Calc(uint8_t dT_ms);//高度数据融合
#endif

