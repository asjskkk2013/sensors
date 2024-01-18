#include <iocc2530.h>
#include <stdio.h>
#include "./uart/hal_uart.h"
#include "./iic/iic.h"
#include "ms5611.h"




uint32  Cal_C[7];
uint32 D1_Pres,D2_Temp; // 存放数字压力和温度  
float Pressure;             //温度补偿大气压  
float Temperature,Temperature2;
double dT;
double OFF,SENS;

__near_func int putchar(int c)//printf输出重定向
{
    UTX0IF = 0;
    U0DBUF = (char)c;
    while(UTX0IF == 0);
    return(c);
}

void ms5611_Reset(void){
  i2c_start();
  WriteI2CByte_1(ADDR_W);
  Check_Acknowledge_1();
  WriteI2CByte_1(CMD_RESET);
  Check_Acknowledge_1();
  I2C_Stop_1();
}



void MS5611_init(void)
{
  
  uint16 data1,data2;
   uint8  i=0;
   for(i=0;i<=6;i++)
   {
     /***********************************/

      i2c_start();
      WriteI2CByte_1(ADDR_W);//写入从设备地址，并设定为写入模式
      Check_Acknowledge_1();
      Delay_1u(100);
      WriteI2CByte_1(CMD_PROM_RD+i*2);//写入寄存器地址，准备读取数据
      Check_Acknowledge_1();
      Delay_1u(100);
      I2C_Stop_1();
      
      Delay_1u(100);
      i2c_start();
      WriteI2CByte_1(ADDR_R);//写入从设备地址，并设定为读取模式
      Check_Acknowledge_1();
      Delay_1u(100);
      data1=ReadI2CByte_1(1);
      data2=ReadI2CByte_1(0);
      I2C_Stop_1();
      Delay_1u(1000);
      Cal_C[i]=(data1<<8)|data2;
   }
   Delay_1u(1000);
   
   //get_ms5611_data();
    //MS561101BA_GetTemperature(CMD_ADC_512);
 }

uint32 MS561101BA_DO_CONVERSION(uint8 command)
{
        unsigned long int conversion=0;
        unsigned long int conv1=0,conv2=0,conv3=0;
        
        i2c_start();
        WriteI2CByte_1(ADDR_W);//写入从设备地址，并设定为写入模式
        Check_Acknowledge_1();
        WriteI2CByte_1(command);        //写入寄存器地址（指令），准备读取数据
        Check_Acknowledge_1();
        I2C_Stop_1();
        Delay_1u(20000);
        
        i2c_start();   
        WriteI2CByte_1(ADDR_W);
        Check_Acknowledge_1();
        WriteI2CByte_1(CMD_ADC_READ);
        Check_Acknowledge_1();
        I2C_Stop_1();//I2C ADC read sequence
        
        Delay_1u(20000);
        i2c_start();
        WriteI2CByte_1(ADDR_R);
        Check_Acknowledge_1();
        conv1=ReadI2CByte_1(1);
        conv2=ReadI2CByte_1(1);
        conv3=ReadI2CByte_1(0);

        I2C_Stop_1();
        Delay_1u(1000);
        conversion= (conv1<<16) + (conv2<<8) + conv3;

        return conversion;
        
}

void MS561101BA_GetTemperature(uint8 OSR_Temp)
{
   
        D2_Temp= MS561101BA_DO_CONVERSION(OSR_Temp);
        //send_a_byte(D2_Temp);        
        Delay_1u(9000);
        
        dT=D2_Temp - ((((unsigned long int)Cal_C[5])<<8));
        Temperature=2000+dT*((unsigned long int)Cal_C[6])/8388608;//2007等于20.07摄氏度
        //printf("%lf\r\n",D2_Temp);
}

void MS561101BA_GetPressure(uint8 OSR_Pres)
{
      float Aux,OFF2,SENS2;  //温度校验值
      D1_Pres=MS561101BA_DO_CONVERSION(OSR_Pres);
      Delay_1u(1000);
      OFF=(uint32)(Cal_C[2]<<16)+((uint32)Cal_C[4]*dT)/128.0;
      SENS=(uint32)(Cal_C[1]<<15)+((uint32)Cal_C[3]*dT)/256.0;
      //温度补偿
              if(Temperature < 2000)// second order temperature compensation when under 20 degrees C
        {
                Temperature2 = (dT*dT) / 0x80000000;//(0x80000000是2的31次方)
                Aux = (Temperature-2000)*(Temperature-2000);
                OFF2 = 2.5*Aux;
                SENS2 = 1.25*Aux;
                if(Temperature < -1500)
                {
                        Aux = (Temperature+1500)*(Temperature+1500);
                        OFF2 = OFF2 + 7*Aux;
                        SENS2 = SENS + 5.5*Aux;
                }
        }else  //(Temperature > 2000)
        {
                Temperature2 = 0;
                OFF2 = 0;
                SENS2 = 0;
        }
        
        Temperature = Temperature - Temperature2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;        

        Pressure=(D1_Pres*SENS/2097152.0-OFF)/32768.0;//100009= 1000.09 mbar  
        //一个标准大气压的压强是1.013x10^3mbar
}

void get_ms5611_data(void)      
{
      
      MS561101BA_GetTemperature(CMD_ADC_D2);
      MS561101BA_GetPressure(CMD_ADC_D1);
}

