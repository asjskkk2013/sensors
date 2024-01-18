#ifndef _HMC5883_IIC_H_
#define _HMC5883_IIC_H_

#define	SCL   P1_0      //IICʱ�����Ŷ���
#define	SDA   P1_1     //IIC�������Ŷ���

#define uchar unsigned char   //����uchar������Ϊ�޷�����
#define uint unsigned int     //����uint������Ϊ�޷�����

#define	SlaveAddress   0x3C	  //��������5883��IIC�����еĴӵ�ַ

#define SDADirOut P1DIR|=0x02;      //xxxx1M01
#define SDADirIn  P1DIR&=~0x02;
typedef unsigned char BYTE;


BYTE BUF[8];                         //�������ݻ����� 

/**************************************
��ʱ1΢��
��ͬ�Ĺ�������,��Ҫ�����˺�����ע��ʱ�ӹ���ʱ��Ҫ�޸�
������1T��MCUʱ,���������ʱ����
**************************************/
#pragma optimize=none
void Delayus(unsigned int usec)
{
     usec>>= 1;
    while(usec--)
    {
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
    }
}

/**************************************
��ʼ�ź�
**************************************/
void HMC5883_Start()
{
    SDADirOut;
    Delayus(5);  
    SDA = 1;                    //����������
    SCL = 1;                    //����ʱ����
    Delayus(5);                 //��ʱ
    SDA = 0;                    //�����½���
    Delayus(5);                 //��ʱ
    SCL = 0;                    //����ʱ����
    Delayus(5);  
}

/**************************************
ֹͣ�ź�
**************************************/
void HMC5883_Stop()
{   
    SCL=0;                       //����ʱ����
    Delayus(1);  
    SDADirOut;
    Delayus(1);  
    SDA = 0;                    //����������
    Delayus(5);                 //��ʱ
    SDA = 1;                    //����������
    Delayus(5);                 //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void HMC5883_SendACK(char ack)
{   
    SCL = 0;
    Delayus(5);                
    SDADirOut;
    Delayus(5);      
    SDA = ack;                  //дӦ���ź�
    SCL = 1;                    //����ʱ����
    Delayus(5);                 //��ʱ
    SCL = 0;                    //����ʱ����
    Delayus(5);                 //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
char HMC5883_RecvACK()
{
    SCL=0;
    Delayus(5);
    SDADirOut;
    SDA=1;
    SDADirIn;
    Delayus(5);          
    SCL=1;
    Delayus(5);
    if(SDA==1)
    {
      SCL=0;
      return 0;  //er
    }
    SCL=0;
    return 1;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
void HMC5883_SendByte(BYTE dat)
{
    BYTE i;
    SDADirOut;

    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;              //�Ƴ����ݵ����λ
        SDA = CY;               //�����ݿ�
        SCL = 1;                //����ʱ����
        Delayus(5);             //��ʱ
        SCL = 0;                //����ʱ����
        Delayus(5);             //��ʱ
    }
    HMC5883_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
BYTE HMC5883_RecvByte()
{
    BYTE i;
    BYTE dat = 0;

    SCL=0;
    SDADirOut;
    SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    Delayus(5);
    SDADirIn;
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL = 1;                //����ʱ����
        Delayus(5);             //��ʱ
        dat |= SDA;             //������               
        SCL = 0;                //����ʱ����
        Delayus(5);             //��ʱ
    }
    return dat;
}

//***************************************************
//����д������
void Single_Write_HMC5883(uchar REG_Address,uchar REG_data)  
{
    HMC5883_Start();                  //��ʼ�ź�
    HMC5883_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
    HMC5883_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ����ο�����pdf 
    HMC5883_SendByte(REG_data);       //�ڲ��Ĵ������ݣ���ο�����pdf
    HMC5883_Stop();                   //����ֹͣ�ź�
}

//******************************************************
//��������HMC5883�ڲ��Ƕ����ݣ���ַ��Χ0x3~0x5
void Multiple_Read_HMC5883(void)              //�����Ķ�ȡ�ڲ��Ĵ�������
{   uchar i;
    HMC5883_Start();                          //��ʼ�ź�
    HMC5883_SendByte(SlaveAddress);           //�����豸��ַ+д�ź�
    HMC5883_SendByte(0x03);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
    HMC5883_Start();                          //��ʼ�ź�
    HMC5883_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
	 for (i=0; i<7; i++)                      //������ȡ6����ַ���ݣ��洢��BUF
    {
        BUF[i] = HMC5883_RecvByte();          //BUF[0]�洢0x32��ַ�е�����
        if (i == 6)
        {
           HMC5883_SendACK(1);                //���һ��������Ҫ��NOACK
        }
        else
        {
          HMC5883_SendACK(0);                //��ӦACK
       }
   }
    HMC5883_Stop();                          //ֹͣ�ź�
    Delayus(1000);
}


#endif
