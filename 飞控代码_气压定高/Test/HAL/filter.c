#include "filter.h"


float sliding_filter(float new_data, float buffer[], int size)//滑动滤波
{
    float sum = 0.0;

    for (int i = size - 1; i > 0; i--) {
        buffer[i] = buffer[i - 1];
    }
    buffer[0] = new_data;  
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }

    return sum / size;
}
float Low_pass_filter(float new_val,float last_val, float low_pass_filter_param)//低通滤波
{
    return new_val*low_pass_filter_param + last_val*(1.0-low_pass_filter_param);
}