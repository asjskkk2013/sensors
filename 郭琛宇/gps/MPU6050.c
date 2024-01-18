#include "./iic/iic.h"
#include <iocc2530.h>
#include "./uart/hal_uart.h"

#define	MPU6050_GYRO_CONFIG	0x10//陀螺仪配置寄存器
#define	MPU6050_ACCEL_CONFIG	0x10//加速度计配置寄存器


void Single_Write_MPU6050(uint8 REG_Address,uint8 REG_data)
{
    i2c_start();                  //起始信号
    WriteI2CByte_1(0x68);   //发送设备地址+写信号
    WriteI2CByte_1(REG_Address);    //内部寄存器地址，
    WriteI2CByte_1(REG_data);       //内部寄存器数据，
    I2C_Stop_1();                   //发送停止信号
}


uint8 Single_Read_MPU6050(uint8 REG_Address)
{
  uint8 REG_data;
  i2c_start();                   //起始信号
  WriteI2CByte_1(0X68);    //发送设备地址+写信号
  WriteI2CByte_1(REG_Address);     //发送存储单元地址，从0开始	
  i2c_start();                   //起始信号
  WriteI2CByte_1(0x68+1);  //发送设备地址+读信号
  REG_data=ReadI2CByte_1(1);       //读出寄存器数据
  I2C_Stop_1();                    //停止信号
  return REG_data;
}

void InitMPU6050()
{
	Single_Write_MPU6050(0x6B, 0x00);	// #define PWR_MGMT_1 0x6B
	Single_Write_MPU6050(0x19, 0x07);	// #define SMPLRT_DIV 0x19
	Single_Write_MPU6050(0x1A, 0x06);		// #define CONFIG	  0x1A
	Single_Write_MPU6050(0x1B, 0x18);	// #define GYRO_CONFIG 0x1B
	Single_Write_MPU6050(0x1C, 0x01);// #define ACCEL_CONFIG 0x1C
}

int GetData(uint8 REG_Address)
{
	uint8 H,L;
	H=Single_Read_MPU6050(REG_Address);	// 相应寄存器地址
	L=Single_Read_MPU6050(REG_Address+1);
	return ((H<<8)+L);   
}


