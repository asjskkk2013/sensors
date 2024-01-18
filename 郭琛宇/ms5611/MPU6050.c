#include "./iic/iic.h"
#include <iocc2530.h>
#include "./uart/hal_uart.h"

#define	MPU6050_GYRO_CONFIG	0x10//���������üĴ���
#define	MPU6050_ACCEL_CONFIG	0x10//���ٶȼ����üĴ���


void Single_Write_MPU6050(uint8 REG_Address,uint8 REG_data)
{
    i2c_start();                  //��ʼ�ź�
    WriteI2CByte_1(0x68);   //�����豸��ַ+д�ź�
    WriteI2CByte_1(REG_Address);    //�ڲ��Ĵ�����ַ��
    WriteI2CByte_1(REG_data);       //�ڲ��Ĵ������ݣ�
    I2C_Stop_1();                   //����ֹͣ�ź�
}


uint8 Single_Read_MPU6050(uint8 REG_Address)
{
  uint8 REG_data;
  i2c_start();                   //��ʼ�ź�
  WriteI2CByte_1(0X68);    //�����豸��ַ+д�ź�
  WriteI2CByte_1(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ	
  i2c_start();                   //��ʼ�ź�
  WriteI2CByte_1(0x68+1);  //�����豸��ַ+���ź�
  REG_data=ReadI2CByte_1(1);       //�����Ĵ�������
  I2C_Stop_1();                    //ֹͣ�ź�
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
	H=Single_Read_MPU6050(REG_Address);	// ��Ӧ�Ĵ�����ַ
	L=Single_Read_MPU6050(REG_Address+1);
	return ((H<<8)+L);   
}


