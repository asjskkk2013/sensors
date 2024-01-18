//2016��3��18��20:03:31
//ʹ�÷����������Դ���txd��P02,rxd��P03,������Ϊ9600
//P1_0��ʱ����SCL��P1_1��������SDA
//������ʾ0-360�Ƕȣ�0�ȱ�ʾX��ָ�����ϣ�

#include <ioCC2530.h>
#include  <math.h>
#include "HMC5883_I2C.h"

#define uint  unsigned int
#define uchar unsigned char

#define	SlaveAddress   0x3C	  //��������5883��IIC�����еĴӵ�ַ

int x,y,z;
double angle;
uchar ge,shi,bai,qian,wan;           //��ʾ����

//*********************************
//����˿�
#define LED1 P1_0	//����LED1ΪP10�ڿ���
#define LED2 P1_1	//����LED2ΪP11�ڿ���
#define LED3 P1_4	//����LED3ΪP14�ڿ���

//*********************************

//��������
void ON_32MOSC();	//����ϵͳ��ʱ��Ƶ��Ϊ32MHZ
void Delay(uint);	//��ʱ����
void Init_IO(void);	//��ʼ��LED����IO�ں���



//*********************************

//����ϵͳ��ʱ��Ƶ��Ϊ32MHZ
void ON_32MOSC()
{
    CLKCONCMD &= ~0x40;        //����ϵͳʱ��ԴΪ32MHZ����
    while(CLKCONSTA & 0x40);   //�ȴ������ȶ�
    CLKCONCMD &= ~0x47;        //����ϵͳ��ʱ��Ƶ��Ϊ32MHZ 
}

//��ʱ������Delay(1000)��Ϊ1��
void Delay(uint n)     
{
    uint a,b,c;
    for(a = 0;a<n;a++)
    for(b=6;b>0;b--)
    for(c=230;c>0;c--);
}

//��ʼ��LED����IO�ں���
void Init_IO(void)
{
    P1DIR |= 0x13;  //P10��P11��P14����Ϊ���
    LED1 = 1;
    LED2 = 1;
    LED3 = 1;	   //LED�Ƴ�ʼ��Ϊ��
}

//����0��ʼ��
void uar0_cfg()   //����0����
{
    PERCFG &=0xFE; //�Ѵ���0�Ľ�λ�������ڱ���λ��1 ��P0_2  P0_3
    P0SEL |=0x0C;  //P0_2  P0_3������Ƭ������ģʽ,��������ͨIO��,����P0_2ΪRX  P0_3ΪTX
    U0CSR |=0x80;  //��������
    U0UCR =0x00;   //�������� 
    U0GCR =8;      //���������� 9600
    U0BAUD=59;     //����������
    UTX0IF = 0;    //UART0 TX�жϱ�־��ʼ��λ0
}

//���ڷ����ַ�������			
void Send_char(unsigned char ch)
{
    U0DBUF=ch;
    while(UTX0IF == 0);
    UTX0IF = 0;
}


//��ʼ��HMC5883��������Ҫ��ο�pdf�����޸�****
void Init_HMC5883()
{
     Single_Write_HMC5883(0x02,0x00);  //
	 Single_Write_HMC5883(0x01,0x00);  //
}

//���ݼ���
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

/********������***********/
void main(void)
{   	
	ON_32MOSC();
	uar0_cfg();
    Init_IO();
	Init_HMC5883();
	Delay(10);
	  
	while(1)
	{  
	  	Multiple_Read_HMC5883();	//�����������ݣ��洢��BUF��
	  	x = BUF[0] << 8 | BUF[1]; 
    	y = BUF[4] << 8 | BUF[5]; 
		z = BUF[2] << 8 | BUF[3];
		angle= atan2((double)y,(double)x) * (180 / 3.14159265) + 180; // angle in degrees
		conversion(angle);       	//�������ݺ���ʾ
		
		Send_char(bai);
		Send_char(shi);
		Send_char(ge);
		Send_char('\n');      //���з�		
	  	
		LED1 = !LED1;    //LED1����һ��
		Delay(200);
	}
}
/**************************/

