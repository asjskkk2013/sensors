#include <iocc2530.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "./uart/hal_uart.h"
#include "./iic/iic.h"
#include "ms5611.h"
#include "stdint.h"
#include "HMC5883L.h"

#define uchar unsigned char
#define uint unsigned int 
#define uint8 uchar
#define uint16 uint
#define TRUE 1
#define FALSE 0

//�������LED�ƵĶ˿�
#define LED1 P1_0	//����LED1ΪP10�ڿ���
#define LED2 P1_1	//����LED2ΪP11�ڿ���

uchar temp;
uchar temp1;
uint8 receive_buf[100];
uint8 flag=0,flag2=0,flag3=0;
uint8 count;
unsigned char index=0;
double latitude = 0.0;
double longitude = 0.0;
/****************************
//��ʱ����
*****************************/
void delay_us(uint32 delay_us)
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
void delay_ms(uint16 delay_ms)
{
  volatile unsigned int num;
  for (num = 0; num < delay_ms; num++)
  {
    delay_us(1000);
  }
}


void InitLed(void)
{
    P1DIR |= 0x03;   //P1_0��P1_1����Ϊ���
    LED1 = 1;       //LED1��Ϩ��
    LED2 = 1;	    //LED2��Ϩ��
}

void processNmeaData(uint8_t *nmeaData)
{
    // ��������Ƿ���"$"��ͷ���Ա�֤����Ч��NMEAЭ������
    if (nmeaData[0] == '$') {
        // ��������Ƿ���GGAЭ�飬GGAЭ���а�����γ����Ϣ
        if (strncmp((const char*)nmeaData + 1, "GGA", 3) == 0) {
            // ʹ��strtok�����ָ������ֶ�
            char *token = strtok((char*)nmeaData, ",");
            uint8_t fieldIndex = 0;
            while (token != NULL) {
                // ��γ����Ϣͨ���ڵ�3���͵�5���ֶ���
                if (fieldIndex == 2) {
                    // ����γ����Ϣ���Զ�Ϊ��λ��
                    latitude = atof(token);
                    //�ڴ˴�����γ����Ϣ�Ĵ���
                } else if (fieldIndex == 4) {
                    // ����������Ϣ���Զ�Ϊ��λ��
                    longitude = atof(token);
                    //�ڴ˴����о�����Ϣ�Ĵ���
                }
                token = strtok(NULL, ",");
                fieldIndex++;
            }
        }
    }
}
void getMessage(){
  while(1){
    while(flag)
      {
        if(temp1=='$'){
          receive_buf[0]=temp1;
          index++;
        }
        else if(temp1=='\n')
        {
          flag2=1;
          flag3=1;
          processNmeaData(receive_buf);
        }
        else
        {
          if(receive_buf[0]=='$')
          {
            receive_buf[index++]=temp1;
          }
        }
        flag=0;
        temp1=0;
      }  
      if(flag2)
      {
        for(uint8 i=0;i<=index;i++)
        {
          send_a_byte(receive_buf[i]);
        }
        index=0;
        flag2=0;
        
        memset(receive_buf, 0, 100);
      }
    if(flag3)
    {
      flag3=0;
      break;
    }
  }
  
  
}

void main(void)
{

    
    InitUart();    // baudrate:57600

    initUART1();
    hmc5883l_init();
    delay_ms(50);
    //prints("input: 11----->led1 on   10----->led1 off   21----->led2 on   20----->led2 off\r\n");
    //char a[5]="hello";
    //char b='1';

    //send_a_byte(b);
    //prints(a);
    ms5611_Reset();
    
    MS5611_init();
    //float h = 0;
    //float t = 0;
    //float p = 0;
    //float temp=(1.0/5.257);
    while(1)
    {   
        ShowHMC5883L();
        //HMC5883L_Data data = HMC5883L_ReadData();
        //printf("Bx: %f, By: %f, Bz: %f\n", data.x, data.y, data.z);
      //getMessage();
      //printf("latitude:%.6lf \r\n",latitude);
      //printf("longitude:%.6lf \r\n",longitude);
      //get_ms5611_data();
      
      //t=Temperature/100;
      //p=Pressure/100;
      
      //h=(1-pow(1013.25/p,temp))*44330;
      /*printf("temperature:%.2lf C\r\n",t);
      printf("presure:%.2lf mbar\r\n",p);
      printf("height:%.2lf m\r\n",h);*/
      
      
     
      //if( temp1 != 0)
       //   {
       //       send_a_byte(temp1);
              /*if((temp=='$'))  //'\r���س���Ϊ�����ַ� //����ܽ���3���ַ�
              {                                                     
                receive_buf[counter++] = temp;
              }
              else
              {
                RT_flag = 3;                   //����led����״̬
              }
              if(counter == 3)  RT_flag = 3;
              temp  = 0;*/
          //}
        /*if(RT_flag == 1)			//����
        {
          if( temp != 0)
          {
              if((temp!='\r')&&(counter<3))  //'\r���س���Ϊ�����ַ� //����ܽ���3���ַ�
              {                                                     
                receive_buf[counter++] = temp;
              }
              else
              {
                RT_flag = 3;                   //����led����״̬
              }
              if(counter == 3)  RT_flag = 3;
              temp  = 0;
          }
        }
    
        if(RT_flag == 3)			 //led״̬����
        {
            U0CSR &= ~0x40;		         //��ֹ����
            receive_buf[2] = '\0';
           // prints(receive_buf);      prints("\r\n");
            if(receive_buf[0] == '1')
            { 
              if(receive_buf[1] == '1')        { LED1 = 0;   prints("led1 on\r\n");   }    
              else if(receive_buf[1] == '0')   { LED1 = 1;   prints("led1 off\r\n");  }
            }
            else if(receive_buf[0] == '2')
            { 
              if(receive_buf[1] == '1')        { LED2 = 0;   prints("led2 on\r\n");   }   
              else if(receive_buf[1] == '0')   { LED2 = 1;   prints("led2 off\r\n");  }
            }
            U0CSR |= 0x40;		        //�������
            RT_flag = 1;		        //�ָ�������״̬
            counter = 0;			//ָ���0
        }*/
      
    }
}

/****************************************************************
*�������� �����ڽ���һ���ַ�					
*��ڲ��� : ��						
*�� �� ֵ ����				
*˵    �� ��������ɺ�򿪽���				
****************************************************************/
#pragma vector = URX0_VECTOR
 __interrupt void UART0_ISR(void)
{
    URX0IF = 0;				//���жϱ�־
    temp = U0DBUF;
    
}

#pragma vector = URX1_VECTOR 
__interrupt void UART1_ISR(void) 
{ 
   temp1 = U1DBUF;
   //flag=1;
  send_a_byte(temp1);
}