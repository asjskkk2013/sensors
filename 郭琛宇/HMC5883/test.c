//2016年3月18日20:03:31
//使用方法：将电脑串口txd接P02,rxd接P03,波特率为9600
//P1_0接时钟线SCL，P1_1接数据线SDA
//电脑显示0-360角度（0度表示X轴指向正南）

#include <ioCC2530.h>
#include  <math.h>
#include "HMC5883_I2C.h"

#define uint  unsigned int
#define uchar unsigned char

#define	SlaveAddress   0x3C	  //定义器件5883在IIC总线中的从地址

int x,y,z;
double angle;
uchar ge,shi,bai,qian,wan;           //显示变量

//*********************************
//定义端口
#define LED1 P1_0	//定义LED1为P10口控制
#define LED2 P1_1	//定义LED2为P11口控制
#define LED3 P1_4	//定义LED3为P14口控制

//*********************************

//函数声明
void ON_32MOSC();	//设置系统主时钟频率为32MHZ
void Delay(uint);	//延时函数
void Init_IO(void);	//初始化LED控制IO口函数



//*********************************

//设置系统主时钟频率为32MHZ
void ON_32MOSC()
{
    CLKCONCMD &= ~0x40;        //设置系统时钟源为32MHZ晶振
    while(CLKCONSTA & 0x40);   //等待晶振稳定
    CLKCONCMD &= ~0x47;        //设置系统主时钟频率为32MHZ 
}

//延时函数，Delay(1000)即为1秒
void Delay(uint n)     
{
    uint a,b,c;
    for(a = 0;a<n;a++)
    for(b=6;b>0;b--)
    for(c=230;c>0;c--);
}

//初始化LED控制IO口函数
void Init_IO(void)
{
    P1DIR |= 0x13;  //P10、P11、P14定义为输出
    LED1 = 1;
    LED2 = 1;
    LED3 = 1;	   //LED灯初始化为关
}

//串口0初始化
void uar0_cfg()   //串口0配置
{
    PERCFG &=0xFE; //把串口0的脚位置配置在备用位置1 即P0_2  P0_3
    P0SEL |=0x0C;  //P0_2  P0_3工作在片上外设模式,而不是普通IO口,其中P0_2为RX  P0_3为TX
    U0CSR |=0x80;  //典型配置
    U0UCR =0x00;   //典型配置 
    U0GCR =8;      //波特率设置 9600
    U0BAUD=59;     //波特率设置
    UTX0IF = 0;    //UART0 TX中断标志初始置位0
}

//串口发送字符串函数			
void Send_char(unsigned char ch)
{
    U0DBUF=ch;
    while(UTX0IF == 0);
    UTX0IF = 0;
}


//初始化HMC5883，根据需要请参考pdf进行修改****
void Init_HMC5883()
{
     Single_Write_HMC5883(0x02,0x00);  //
	 Single_Write_HMC5883(0x01,0x00);  //
}

//数据计算
void conversion(uint temp_data)
{  
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;   //取余运算
	qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //取余运算
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //取余运算
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //取余运算
    ge=temp_data+0x30; 	
}

/********主函数***********/
void main(void)
{   	
	ON_32MOSC();
	uar0_cfg();
    Init_IO();
	Init_HMC5883();
	Delay(10);
	  
	while(1)
	{  
	  	Multiple_Read_HMC5883();	//连续读出数据，存储在BUF中
	  	x = BUF[0] << 8 | BUF[1]; 
    	y = BUF[4] << 8 | BUF[5]; 
		z = BUF[2] << 8 | BUF[3];
		angle= atan2((double)y,(double)x) * (180 / 3.14159265) + 180; // angle in degrees
		conversion(angle);       	//计算数据和显示
		
		Send_char(bai);
		Send_char(shi);
		Send_char(ge);
		Send_char('\n');      //换行符		
	  	
		LED1 = !LED1;    //LED1灯闪一次
		Delay(200);
	}
}
/**************************/

