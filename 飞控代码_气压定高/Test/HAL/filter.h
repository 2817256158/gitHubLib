#ifndef FILTER_H
#define FILTER_H

#include "config.h"


float sliding_filter(float new_data, float buffer[], int size);//滑动滤波
float Low_pass_filter(float new_val,float last_val, float low_pass_filter_param);//低通滤波
#endif