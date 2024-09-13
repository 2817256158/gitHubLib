#include "pid.h" 
#include <math.h>

#define MUX_SUM_V2 1000
#define High_MAX_ERR 1000
/*Yaw只用了单级PID*/
/*一级参数*/
/*-80 -0.01 -0.0*/
float Kp_pitch_balance=-20;
float Kd_pitch_balance=-0.01;
float Ki_pitch_balance=-0.0;
/*-80 -0.01 -0.0*/
float Kp_roll_balance=-20;
float Kd_roll_balance=-0.01;
float Ki_roll_balance=-0.0;
/*20 0.。8 0.0*/
// float Kp_yaw_balance=30.0;
// float Kd_yaw_balance=1.0;
// float Ki_yaw_balance=0.0;
float Kp_yaw_balance=-0.0;
float Kd_yaw_balance=-0.0;
float Ki_yaw_balance=-0.0;
/*二级参数*/
/*0.85 3.80 0.0*/
float Kp_gyroy_balance_v2=0.75;
float Kd_gyroy_balance_v2=3.40;
float Ki_gyroy_balance_v2=0.0;
/*0.85 3.80 0.0*/
float Kp_gyrox_balance_v2=0.75;
float Kd_gyrox_balance_v2=3.40;
float Ki_gyrox_balance_v2=0.0;
/*0.65 2.8 0.0*/
float Kp_gyroz_balance_v2=0.65;
float Kd_gyroz_balance_v2=2.80;
float Ki_gyroz_balance_v2=0.0;

float gyrox_accumulate_err=0.0f;
float gyroy_accumulate_err=0.0f;
float gyroz_accumulate_err=0.0f;

float gyrox_last_err=0.0f;
float gyroy_last_err=0.0f;
float gyroz_last_err=0.0f;

/*aim Angle*/
float aim_pitch=0.0;
float aim_roll=0.0;
float aim_yaw=0.0;

extern volatile float P;
extern volatile float I;
extern volatile float D;

int pitch_balance(float Angle,short gyro)
{
    volatile static float last_pitch=0;
    volatile static float pitch_err=0;
    float err;
    int pwm_balance;
    float angle_num=0.0f;
    err=Angle-0.2;//计算角度误差
    if(fabs(pitch_err+err)>=1000)pitch_err=pitch_err+err;//积分限幅
    pwm_balance=Kp_pitch_balance*err+gyro*1.0*Kd_pitch_balance+Ki_pitch_balance*pitch_err;//计算pwm输出
    last_pitch=Angle;
    return pwm_balance;
}
int roll_balance(float Angle,short gyro)
{
    volatile static float last_roll=0;
    volatile static float roll_err=0;
    float err;   
    int pwm_balance;
    float angle_num=0.0f;
    err=Angle-angle_num;
    if(fabs(roll_err+err)>=1000)roll_err=roll_err+err;
    pwm_balance=Kp_roll_balance*err+gyro*1.0*Kd_roll_balance+Ki_roll_balance*roll_err;//计算pwm输出
    last_roll=Angle;
    return pwm_balance;
}
int yaw_balance(float Angle,short gyro)
{
    volatile static float last_yaw=0;
    volatile static float yaw_err=0;
    float err;
    int pwm_balance;
    float angle_num=0.0f;
    err=Angle-angle_num;
    if(fabs(yaw_err+err)>=10000)yaw_err=yaw_err+err;
    pwm_balance=Kp_yaw_balance*err+gyro*1.0*Kd_yaw_balance+Ki_yaw_balance*yaw_err;//计算pwm输出
    last_yaw=Angle;
    return pwm_balance;
}


#define MAX_Gyro_NUM 1000
int PID_Version2(int ex_value,short re_value,int num)/*0:gyrox 1:gyroy 2:gyroz*/
{
    int pwm_out=0;
    float err=0.0f;
    err=re_value*1.0-ex_value*1.0;
    if(num==0)
    {
        //积分分离 当偏差太大时不积分
        if(fabs(gyrox_accumulate_err+err)>MUX_SUM_V2)
        {
            if(fabs(err)<MAX_Gyro_NUM)gyrox_accumulate_err=0;
            else gyrox_accumulate_err+=err;
        }
        pwm_out=Kp_gyrox_balance_v2*err + Ki_gyrox_balance_v2*gyrox_accumulate_err + Kd_gyrox_balance_v2*(err-gyrox_last_err);
        gyrox_last_err=err;
    }
    else if(num==1)
    {
        //积分分离 当偏差太大时不积分
        if(fabs(gyroy_accumulate_err+err)>MUX_SUM_V2)
        {
            if(fabs(err)<MAX_Gyro_NUM)gyroy_accumulate_err=0;
            else gyroy_accumulate_err+=err;
        }
        pwm_out=Kp_gyroy_balance_v2*err + Ki_gyroy_balance_v2*gyroy_accumulate_err + Kd_gyroy_balance_v2*(err-gyroy_last_err);
        gyroy_last_err=err;
    }
    else if(num==2)
    {
        //积分分离 当偏差太大时不积分
        if(fabs(gyroz_accumulate_err+err)>MUX_SUM_V2)
        {
            if(fabs(err)<MAX_Gyro_NUM)gyroz_accumulate_err=0;
            else gyroz_accumulate_err+=err;
        }
         pwm_out=Kp_gyroz_balance_v2*err + Ki_gyroz_balance_v2*gyroz_accumulate_err + Kd_gyroz_balance_v2*(err-gyroz_last_err);
         gyroz_last_err=err;
    }
    return pwm_out; 
}

float PID_Control(PID_Creat*object, float expect_val, float real_val)
{
	float PID_Out=0;
	float err=expect_val - real_val;
	object->accumulate_err+=err;
	if(object->accumulate_err > 1000 )object->accumulate_err = 1000;
	if(object->accumulate_err < -1000 )object->accumulate_err = -1000;
	PID_Out = object->kp * err + object->ki * object->accumulate_err + object->kd * (err - object->last_err);
	object->last_err=err;
	return PID_Out;
}
