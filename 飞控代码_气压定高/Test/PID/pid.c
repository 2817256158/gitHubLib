#include "pid.h" 
#include <math.h>


float PID_Control(PID_Creat*object, float expect_val, float real_val)
{
	float PID_Out=0;
	float err=expect_val - real_val;
	object->accumulate_err+=err;
	if(object->accumulate_err > 1000 )object->accumulate_err = 1000;
	if(object->accumulate_err < -1000 )object->accumulate_err = -1000;
    object->p_out = object->kp * err;
    object->i_out = object->ki * object->accumulate_err;
    object->d_out = object->kd * (err - object->last_err);
	PID_Out = object->p_out + object->i_out + object->d_out;
	object->last_err=err;
    /*输出限幅*/
    if(PID_Out > object->out_limit)PID_Out = object->out_limit;
    if(PID_Out < object->out_limit*-1.0)PID_Out = object->out_limit*-1.0;
	return PID_Out;
}
