#include "uartprocess.h"

int fputc(int ch, FILE *f)//串口重定向
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
void uart1_process(uint8_t Byte1)
{
    switch(Byte1)
    {                
        case 'F':break;
        case 'B':break;
        case 'L':break;
        case 'R':break;
        case 'U':break;
        case 'D':break;
        case 'W':Motor_Run = 1;break;
        case 'X':Motor_Run = 0;break;
        case 'Y':break;
        case 'Z':break;
        default:break;
    }
}