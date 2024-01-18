#include "hal_board_cfg.h"
#include <stdio.h>
#include <string.h>

#ifndef HAL_UART_H
#define HAL_UART_H

/* 宏定义 */
/********************************************************************/
#define ENTER      0x0D               // 按键定义
#define SPACE      0x20

#define CLKSPD  ( CLKCONCMD & 0x03 )  //系统时钟速率

 /* 串行UART 配置 */
#define HIGH_STOP                   0x02
#define LOW_STOP                    0x00
#define HIGH_START                  0x01
#define TRANSFER_MSB_FIRST          0x80
#define TRANSFER_MSB_LAST           0x00
#define FLOW_CONTROL_ENABLE         0x40
#define UART_ENABLE_RECEIVE         0x40


#define IO_PER_LOC_USART0_AT_PORT0_PIN2345() \
          do { PERCFG = (PERCFG&~0x01)|0x00; } while (0)

 /* 串行UART_E设置 */
#define BAUD_E(baud, clkDivPow) (     \
    (baud==2400)   ?  6  +clkDivPow : \
    (baud==4800)   ?  7  +clkDivPow : \
    (baud==9600)   ?  8  +clkDivPow : \
    (baud==14400)  ?  8  +clkDivPow : \
    (baud==19200)  ?  9  +clkDivPow : \
    (baud==28800)  ?  9  +clkDivPow : \
    (baud==38400)  ?  10 +clkDivPow : \
    (baud==57600)  ?  10 +clkDivPow : \
    (baud==76800)  ?  11 +clkDivPow : \
    (baud==115200) ?  11 +clkDivPow : \
    (baud==153600) ?  12 +clkDivPow : \
    (baud==230400) ?  12 +clkDivPow : \
    (baud==307200) ?  13 +clkDivPow : \
    0  )

 /* 串行UART_M设置 */
#define BAUD_M(baud) (      \
    (baud==2400)   ?  59  : \
    (baud==4800)   ?  59  : \
    (baud==9600)   ?  59  : \
    (baud==14400)  ?  216 : \
    (baud==19200)  ?  59  : \
    (baud==28800)  ?  216 : \
    (baud==38400)  ?  59  : \
    (baud==57600)  ?  216 : \
    (baud==76800)  ?  59  : \
    (baud==115200) ?  216 : \
    (baud==153600) ?  59  : \
    (baud==230400) ?  216 : \
    (baud==307200) ?  59  : \
  0)

 /* 串行UART设置 */
#define UART_SETUP(uart, baudRate, options)      \
   do {                                          \
      if ((options) & FLOW_CONTROL_ENABLE){      \
         if((uart) == 0){      /* USART0       */\
            if(PERCFG & 0x01){ /* Alt 2        */\
               P1SEL |= 0x3C;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x3C;                    \
            }                                    \
         }                                       \
         else {                /* USART1       */\
            if(PERCFG & 0x02){ /* Alt 2        */\
               P1SEL |= 0xF0;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x3C;                    \
            }                                    \
         }                                       \
      }                                          \
      else{                    /* 流控不时能 */  \
         if((uart) == 0){      /* USART0       */\
            if(PERCFG & 0x01){ /* Alt 2        */\
               P1SEL |= 0x30;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x0C;                    \
            }                                    \
         }                                       \
         else {                /* USART1       */\
            if(PERCFG & 0x02){ /* Alt 2        */\
               P1SEL |= 0xC0;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x30;                    \
            }                                    \
         }                                       \
      }                                          \
                                                 \
      U##uart##GCR = BAUD_E((baudRate), CLKSPD); \
      U##uart##BAUD = BAUD_M(baudRate);          \
                                                 \
      U##uart##CSR |= 0x80;                      \
                                                 \
      U##uart##UCR |= ((options) | 0x80);        \
                                                 \
      if((options) & TRANSFER_MSB_FIRST){        \
         U##uart##GCR |= 0x20;                   \
      }                                          \
   } while(0)
/********************************************************************/

/**********************EXTERN FUNCTION******************************/
extern void InitUart(void);          //串口初始化
extern void prints(char *s);         //打印字符串
extern uint8 send_a_byte(char c);   
extern void initUART1(void);
/*******************************************************************/

#endif