#ifndef _HMC5883_IIC_H_
#define _HMC5883_IIC_H_

#define	SCL   P1_0      //IIC时钟引脚定义
#define	SDA   P1_1     //IIC数据引脚定义

#define uchar unsigned char   //定义uchar型数据为无符号型
#define uint unsigned int     //定义uint型数据为无符号型

#define	SlaveAddress   0x3C	  //定义器件5883在IIC总线中的从地址

#define SDADirOut P1DIR|=0x02;      //xxxx1M01
#define SDADirIn  P1DIR&=~0x02;
typedef unsigned char BYTE;


BYTE BUF[8];                         //接收数据缓存区 

/**************************************
延时1微秒
不同的工作环境,需要调整此函数，注意时钟过快时需要修改
当改用1T的MCU时,请调整此延时函数
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
起始信号
**************************************/
void HMC5883_Start()
{
    SDADirOut;
    Delayus(5);  
    SDA = 1;                    //拉高数据线
    SCL = 1;                    //拉高时钟线
    Delayus(5);                 //延时
    SDA = 0;                    //产生下降沿
    Delayus(5);                 //延时
    SCL = 0;                    //拉低时钟线
    Delayus(5);  
}

/**************************************
停止信号
**************************************/
void HMC5883_Stop()
{   
    SCL=0;                       //拉高时钟线
    Delayus(1);  
    SDADirOut;
    Delayus(1);  
    SDA = 0;                    //拉低数据线
    Delayus(5);                 //延时
    SDA = 1;                    //产生上升沿
    Delayus(5);                 //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void HMC5883_SendACK(char ack)
{   
    SCL = 0;
    Delayus(5);                
    SDADirOut;
    Delayus(5);      
    SDA = ack;                  //写应答信号
    SCL = 1;                    //拉高时钟线
    Delayus(5);                 //延时
    SCL = 0;                    //拉低时钟线
    Delayus(5);                 //延时
}

/**************************************
接收应答信号
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
向IIC总线发送一个字节数据
**************************************/
void HMC5883_SendByte(BYTE dat)
{
    BYTE i;
    SDADirOut;

    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;              //移出数据的最高位
        SDA = CY;               //送数据口
        SCL = 1;                //拉高时钟线
        Delayus(5);             //延时
        SCL = 0;                //拉低时钟线
        Delayus(5);             //延时
    }
    HMC5883_RecvACK();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
BYTE HMC5883_RecvByte()
{
    BYTE i;
    BYTE dat = 0;

    SCL=0;
    SDADirOut;
    SDA = 1;                    //使能内部上拉,准备读取数据,
    Delayus(5);
    SDADirIn;
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        Delayus(5);             //延时
        dat |= SDA;             //读数据               
        SCL = 0;                //拉低时钟线
        Delayus(5);             //延时
    }
    return dat;
}

//***************************************************
//单个写入数据
void Single_Write_HMC5883(uchar REG_Address,uchar REG_data)  
{
    HMC5883_Start();                  //起始信号
    HMC5883_SendByte(SlaveAddress);   //发送设备地址+写信号
    HMC5883_SendByte(REG_Address);    //内部寄存器地址，请参考中文pdf 
    HMC5883_SendByte(REG_data);       //内部寄存器数据，请参考中文pdf
    HMC5883_Stop();                   //发送停止信号
}

//******************************************************
//连续读出HMC5883内部角度数据，地址范围0x3~0x5
void Multiple_Read_HMC5883(void)              //连续的读取内部寄存器数据
{   uchar i;
    HMC5883_Start();                          //起始信号
    HMC5883_SendByte(SlaveAddress);           //发送设备地址+写信号
    HMC5883_SendByte(0x03);                   //发送存储单元地址，从0x3开始	
    HMC5883_Start();                          //起始信号
    HMC5883_SendByte(SlaveAddress+1);         //发送设备地址+读信号
	 for (i=0; i<7; i++)                      //连续读取6个地址数据，存储中BUF
    {
        BUF[i] = HMC5883_RecvByte();          //BUF[0]存储0x32地址中的数据
        if (i == 6)
        {
           HMC5883_SendACK(1);                //最后一个数据需要回NOACK
        }
        else
        {
          HMC5883_SendACK(0);                //回应ACK
       }
   }
    HMC5883_Stop();                          //停止信号
    Delayus(1000);
}


#endif
