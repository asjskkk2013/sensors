
#define SCL P1_3
#define SDA P1_2
#define TRUE 1 
#define FALSE 0 

#include <iocc2530.h>
#include <stdio.h>
#include "iic.h"


void Delay_1u(unsigned int microSecs) {
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

void i2c_start() 
{ 
  WriteSDA1();
  WriteSCL1();
  Delay_1u(50);
  WriteSDA0();
  Delay_1u(50);
  WriteSCL0();
  Delay_1u(50);
}

void WriteSDA1(void)//SDA 输出1,相当于51里面的SDA=1    
{
       P1DIR |= 0x04;
       SDA = 1;
}
    
void WriteSDA0(void)//SDA 输出0    
{
     P1DIR |= 0x04;
     SDA = 0;
}

void WriteSCL1(void)//SCL 输出1    
{
     P1DIR |= 0x08;
     SCL = 1;
}

void WriteSCL0(void)//SCL 输出1    
{
     P1DIR |= 0x08;
     SCL = 0;
}

void ReadSDA(void)//这里设置SDA对应IO口DIR可以接收数据    
{
     P1DIR &= 0xFB;
}

void I2C_Stop_1(void)
{
    WriteSDA0();
    WriteSCL0();
    Delay_1u(50);
    WriteSCL1();
    Delay_1u(50);
    WriteSDA1();

}

/*发送0，在SCL为高电平时使SDA信号为低*/
void SEND_0_1(void)   /* SEND ACK */
{
    WriteSDA0();
    WriteSCL1();
    Delay_1u(50);
    WriteSCL0();
    Delay_1u(50);
}

/*发送1，在SCL为高电平时使SDA信号为高*/
void SEND_1_1(void)
{
    WriteSDA1();
    WriteSCL1();
    Delay_1u(50);
    WriteSCL0();
    Delay_1u(50);
}

/*发送完一个字节后检验设备的应答信号*/    
char Check_Acknowledge_1(void)
{
    WriteSDA1();
    WriteSCL1();
    Delay_1u(50);
    F0=SDA;
    Delay_1u(50);
    WriteSCL0();
    Delay_1u(50);
    if(F0==1)
        return FALSE;
    return TRUE;
}

void Write_Acknowledge_1(char stat)
{
    if(stat)WriteSDA0(); 
    else WriteSDA1(); 
    Delay_1u(50);
    WriteSCL1();   
    Delay_1u(50);
    WriteSCL0();   
    Delay_1u(50);
}

/*向I2C总线写一个字节*/
void WriteI2CByte_1(char b)
{
    char i;
    for(i=0;i<8;i++)
    {
      if((b<<i)&0x80)
      {
         SEND_1_1();
      }
      else
      {
         SEND_0_1();
      }
    }
}

/*从I2C总线读一个字节*/
char ReadI2CByte_1(unsigned int ack)
{
    char b=0,i;
    WriteSDA1();

    for(i=0;i<8;i++)
    {   
        WriteSCL0();
        Delay_1u(50);
        WriteSCL1(); 
        Delay_1u(50);

        ReadSDA();
        F0=SDA;//寄存器中的一位,用于存储SDA中的一位数据

    if(F0==1)
        {
          b=b<<1;
          b=b|0x01;
        }
        else
          b=b<<1;
    }
    WriteSCL0();
    if (!ack)
        Write_Acknowledge_1(0);       //发送nACK
    else
        Write_Acknowledge_1(1);         //发送ACK 
    return b; 
}

