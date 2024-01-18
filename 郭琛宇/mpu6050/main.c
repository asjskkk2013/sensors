/****************************************************************************
* 文 件 名: main.c
* 版    本: 1.0
* 描    述: 读取mpu6050的加速度、温度、陀螺仪的数据， 通关过串口（115200波特率）来发送
****************************************************************************/
#include <ioCC2530.h>
#include <string.h>
#include "iic.h"
#include "mpu6050.h"


/****************************************************************************
* 名    称: DelayMS()
* 功    能: 以毫秒为单位延时，系统时钟不配置时默认为16M(用示波器测量相当精确)
* 入口参数: msec 延时参数，值越大，延时越久
* 出口参数: 无
****************************************************************************/
void DelayMS(uint msec)
{ 
    uint i,j;
    
    for (i=0; i<msec; i++)
        for (j=0; j<535; j++);
}

void InitUart0(void)
{
    PERCFG = 0x00;           //外设控制寄存器 USART 0的IO位置:0为P0口位置1 
    P0SEL = 0x0c;            //P0_2,P0_3用作串口（外设功能）
    P2DIR &= ~0xC0;          //P0优先作为UART0
    
    U0CSR |= 0x80;           //设置为UART方式
    U0GCR |= 11;				       
    U0BAUD |= 216;           //波特率设为115200
    UTX0IF = 0;              //UART0 TX中断标志初始置位0
    U0CSR |= 0x40;           //允许接收 
    IEN0 |= 0x84;            //开总中断允许接收中断  
}

void Uart0SendString(char *Data, int len)
{
  uint i;
  for(i=0; i<len; i++)
  {
    U0DBUF = *Data++;
    while(UTX0IF == 0);
    UTX0IF = 0;
  }
}

void InitClockTo32M(void)
{   
    CLKCONCMD &= ~0x40;              //设置系统时钟源为 32MHZ晶振
    while(CLKCONSTA & 0x40);         //等待晶振稳定 
    CLKCONCMD &= ~0x47;              //设置系统主时钟频率为 32MHZ
}

/****************************************************************************
* 程序入口函数
****************************************************************************/
void main(void)
{
    int16_t ax, ay, az;
    int16_t temperature;
    int16_t gx, gy, gz;
    float f_temperature;
    
    char str[9]="M0:";    
    char strTemp[64];
    unsigned char tempV = 0;
    
    InitClockTo32M();                      //设置系统时钟源
    InitUart0();                     //串口初始化        
    IIC_Init();                     //IIC初始化

    while(1)
    {      
        tempV = MPU6050_GetDeviceID();
        //这个值如果打印为  [WHOE AM I]0x68， 那么表示已经正确辨认到mpu6050了
        sprintf(strTemp, "[WHOE AM I ?] chip ID = 0x%02x(if = 0x68, is OK)\n", tempV);  
        //串口打印该数据
        Uart0SendString(strTemp, strlen(strTemp));
        
        if(tempV == MPU6050_ADDRESS_AD0_LOW)          //测试是否 mpu6050 已经读到
        {           
            MPU6050_Initialize();                   //初始化mpu6050

            while((tempV = MPU6050_GetDeviceID()) == MPU6050_ADDRESS_AD0_LOW)   //连续测试是否 mpu6050 已经读到
            {
                //读取mpu6050的实时数据           
                MPU6050_GetRawAccelGyro(&ax, &ay, &az, &temperature, &gx, &gy, &gz);
                //转换成温度单位为度的数据
                f_temperature = (float)temperature/340.0 + 36.53;                
                sprintf(strTemp, "[%7d %7d %7d] : [%7d = %.02fC] : [%7d %7d %7d]\n", \
                    ax, ay, az, temperature, f_temperature, gx, gy, gz);

                //串口打印该数据
                Uart0SendString(strTemp, strlen(strTemp)); 
                DelayMS(2000);                       //延时函数使用定时器方式                
            }            
        }
        else
        {
            //mpu6050连接失败
            sprintf(strTemp, "MPU6050_Initialize FAIL");
            //串口打印该数据
            Uart0SendString(strTemp, strlen(strTemp));                
        }
        
        DelayMS(2000);                   //延时函数使用定时器方式
    }
}