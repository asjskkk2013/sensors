/****************************************************************************
* �� �� ��: main.c
* ��    ��: 1.0
* ��    ��: ��ȡmpu6050�ļ��ٶȡ��¶ȡ������ǵ����ݣ� ͨ�ع����ڣ�115200�����ʣ�������
****************************************************************************/
#include <ioCC2530.h>
#include <string.h>
#include "iic.h"
#include "mpu6050.h"


/****************************************************************************
* ��    ��: DelayMS()
* ��    ��: �Ժ���Ϊ��λ��ʱ��ϵͳʱ�Ӳ�����ʱĬ��Ϊ16M(��ʾ���������൱��ȷ)
* ��ڲ���: msec ��ʱ������ֵԽ����ʱԽ��
* ���ڲ���: ��
****************************************************************************/
void DelayMS(uint msec)
{ 
    uint i,j;
    
    for (i=0; i<msec; i++)
        for (j=0; j<535; j++);
}

void InitUart0(void)
{
    PERCFG = 0x00;           //������ƼĴ��� USART 0��IOλ��:0ΪP0��λ��1 
    P0SEL = 0x0c;            //P0_2,P0_3�������ڣ����蹦�ܣ�
    P2DIR &= ~0xC0;          //P0������ΪUART0
    
    U0CSR |= 0x80;           //����ΪUART��ʽ
    U0GCR |= 11;				       
    U0BAUD |= 216;           //��������Ϊ115200
    UTX0IF = 0;              //UART0 TX�жϱ�־��ʼ��λ0
    U0CSR |= 0x40;           //������� 
    IEN0 |= 0x84;            //�����ж���������ж�  
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
    CLKCONCMD &= ~0x40;              //����ϵͳʱ��ԴΪ 32MHZ����
    while(CLKCONSTA & 0x40);         //�ȴ������ȶ� 
    CLKCONCMD &= ~0x47;              //����ϵͳ��ʱ��Ƶ��Ϊ 32MHZ
}

/****************************************************************************
* ������ں���
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
    
    InitClockTo32M();                      //����ϵͳʱ��Դ
    InitUart0();                     //���ڳ�ʼ��        
    IIC_Init();                     //IIC��ʼ��

    while(1)
    {      
        tempV = MPU6050_GetDeviceID();
        //���ֵ�����ӡΪ  [WHOE AM I]0x68�� ��ô��ʾ�Ѿ���ȷ���ϵ�mpu6050��
        sprintf(strTemp, "[WHOE AM I ?] chip ID = 0x%02x(if = 0x68, is OK)\n", tempV);  
        //���ڴ�ӡ������
        Uart0SendString(strTemp, strlen(strTemp));
        
        if(tempV == MPU6050_ADDRESS_AD0_LOW)          //�����Ƿ� mpu6050 �Ѿ�����
        {           
            MPU6050_Initialize();                   //��ʼ��mpu6050

            while((tempV = MPU6050_GetDeviceID()) == MPU6050_ADDRESS_AD0_LOW)   //���������Ƿ� mpu6050 �Ѿ�����
            {
                //��ȡmpu6050��ʵʱ����           
                MPU6050_GetRawAccelGyro(&ax, &ay, &az, &temperature, &gx, &gy, &gz);
                //ת�����¶ȵ�λΪ�ȵ�����
                f_temperature = (float)temperature/340.0 + 36.53;                
                sprintf(strTemp, "[%7d %7d %7d] : [%7d = %.02fC] : [%7d %7d %7d]\n", \
                    ax, ay, az, temperature, f_temperature, gx, gy, gz);

                //���ڴ�ӡ������
                Uart0SendString(strTemp, strlen(strTemp)); 
                DelayMS(2000);                       //��ʱ����ʹ�ö�ʱ����ʽ                
            }            
        }
        else
        {
            //mpu6050����ʧ��
            sprintf(strTemp, "MPU6050_Initialize FAIL");
            //���ڴ�ӡ������
            Uart0SendString(strTemp, strlen(strTemp));                
        }
        
        DelayMS(2000);                   //��ʱ����ʹ�ö�ʱ����ʽ
    }
}