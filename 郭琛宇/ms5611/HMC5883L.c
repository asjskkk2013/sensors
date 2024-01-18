#include "./iic/iic.h"
#include <iocc2530.h>
#include "HMC5883L.h"
#include "HMC5883_I2C.h"
#include "./uart/hal_uart.h"
#include <math.h>

int x,y,z;
double angle;
uchar ge,shi,bai,qian,wan; 

void delay_us1(uint32 delay_us)
{
  volatile unsigned int num;
  volatile unsigned int t;


  for (num = 0; num < delay_us; num++)
  {
    t = 11;
    while (t != 0)
    {
      t--;
    }
  }
}
//���뼶����ʱ
void delay_ms1(uint16 delay_ms)
{
  volatile unsigned int num;
  for (num = 0; num < delay_ms; num++)
  {
    delay_us1(1000);
  }
}


void hmc5883l_init(void)
{
   Single_Write_HMC5883(0x02,0x00);  //
   Single_Write_HMC5883(0x01,0x00);  //
}

void conversion(uint temp_data)
{  
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;   //ȡ������
	qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //ȡ������
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //ȡ������
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //ȡ������
    ge=temp_data+0x30; 	
}

HMC5883L_Data HMC5883L_ReadData() {
  HMC5883L_Data data;
  i2c_start();
  WriteI2CByte_1(0x3C);
  //send_a_byte(Check_Acknowledge_1());
  WriteI2CByte_1(0x03); 
  //send_a_byte(Check_Acknowledge_1());
  I2C_Stop_1();
  i2c_start(); 
  WriteI2CByte_1(0x3c+1);  
  //send_a_byte(Check_Acknowledge_1());
  data.raw_x = (ReadI2CByte_1(1) << 8) | ReadI2CByte_1(1); 
  data.raw_z = (ReadI2CByte_1(1) << 8) | ReadI2CByte_1(1);  
  data.raw_y = (ReadI2CByte_1(1) << 8) | ReadI2CByte_1(1);  
  I2C_Stop_1();// ���е�λ����
  float sensitivity = 0.92;  // �����ȣ���λ��LSB/Gauss��
  float offset = 100.0;     // ƫ��������λ����˹��
  data.x = (data.raw_x * sensitivity) + offset;
  data.y = (data.raw_y * sensitivity) + offset;
  data.z = (data.raw_z * sensitivity) + offset;
  return data;
}

void ShowHMC5883L(void)
{

  Multiple_Read_HMC5883();	//�����������ݣ��洢��BUF��
  x = BUF[0] << 8 | BUF[1]; 
  y = BUF[4] << 8 | BUF[5]; 
  z = BUF[2] << 8 | BUF[3];
  angle= atan2((double)y,(double)x) * (180 / 3.14159265) + 180; // angle in degrees
  conversion(angle); 
  int angle1 = bai*100+shi*10+ge;

  printf("Bx: %d, By: %d, Bz: %d\n, angle: ", x, y, z);
  send_a_byte(bai);
  send_a_byte(shi);
  send_a_byte(ge);
  send_a_byte('\r');
  delay_ms1(50);
}