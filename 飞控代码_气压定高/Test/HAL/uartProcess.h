#ifndef UARTPROCESS_H
#define UARTPROCESS_H

#include "config.h"



int fputc(int ch, FILE *f);//串口重定向
void uart1_process(uint8_t Byte1);//串口1数据处理
void uart2_process(uint8_t Byte1);//串口1数据处理
#endif