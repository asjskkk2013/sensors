#include "hal_board_cfg.h"
#include <stdio.h>
#include <string.h>
#include "hal_uart.h"



/*********************************************************************
 * �������ƣ�uartGetkey
 * ��    �ܣ��Ӵ��ڻ�ȡPC���̰���ֵ
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ��c    �õ��ļ�ֵ
 ********************************************************************/
char uartGetkey(void)
{
  char c;
  uint8 status;

  status = U0CSR;
  U0CSR |= UART_ENABLE_RECEIVE; // ���ڽ���ʹ��

  while (!URX0IF );
  c = U0DBUF;
  URX0IF = FALSE;
 
  U0CSR = status;     // �洢״̬

  return c;
}

char uart1Getkey(void)
{
  char c;
  uint8 status;

  status = U1CSR;
  U1CSR |= UART_ENABLE_RECEIVE; // ���ڽ���ʹ��

  while (!URX1IF );
  c = U1DBUF;
  URX1IF = FALSE;
 
  U1CSR = status;     // �洢״̬

  return c;
}


/*********************************************************************
 * �������ƣ�initUART
 * ��    �ܣ���ʼ������UART
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void InitUart(void)
{
  U0CSR = 0xC0;
  IO_PER_LOC_USART0_AT_PORT0_PIN2345();// ����UART IO�˿�����

  HAL_BOARD_INIT();//�弶��ʼ��

  UART_SETUP(0, 9600, HIGH_STOP);  // ��������
  UTX0IF = 1;

    U0CSR |= 0X40;				//�������
    IEN0 |= 0x84;				//�����жϣ������ж�
    
}

void initUART1(void)
{ 
  PERCFG = 0x00;		//λ��1  P0.4/P0.5��
  P0SEL |= 0x30;		//P0.4,P0.5�������ڣ��ⲿ�豸���ܣ�
  U1BAUD |= 59;                 //���ò�����9600
  U1GCR |= 8;
  U1UCR |= 0x80;
  U1CSR |= 0xC0;
  UTX1IF = 0;                   //�жϱ�־λ
  URX1IF = 0;
  URX1IE = 1;
  EA=1;
}


/*********************************************************************
 * �������ƣ�send_a_byte
 * ��    �ܣ�����һ���ַ�
 * ��ڲ�����c  ���͵��ַ�
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
uint8 send_a_byte(char c)  
{
  if(c == '\n')  
 {
    while(!UTX0IF);
   U0DBUF = 0x0d;   
 }
  
   while (!UTX0IF);
   UTX0IF = 0;
   return (U0DBUF = c);
}


/*********************************************************************
 * �������ƣ�prints
 * ��    �ܣ�����һ���ַ�
 * ��ڲ�����s         �ַ���ָ��
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void prints(char *s)
{
  uint8 i = 0;
 
  while(s[i] != 0)    // �ж��ַ����Ƿ����
  {
    send_a_byte(s[i]);// ����һ�ֽ�
    i++;
  }
}

