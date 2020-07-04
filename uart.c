/*
***********************************************************
* @file stm8s_uart.c
* @author STMicroelectronics - MCD Application Team
* @version V0.0.0
* @date 15-March-2016
* ��ʼ��UART���������жϷ�ʽ�����ַ����Բ�ѯ��ʽ����
* UARTͨѶ������9600bps,8λ���ݣ�1λֹͣλ����У��
* ע�⣺��PC���������ߣ�2��3����Ҫ���档
***********************************************************
*/ 
#include"iostm8s103F3.h"
/*�����������Ͷ���*/
typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned long     uint32_t;
#define FlagStatus     bool
#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define EnableInterrupt  __enable_interrupt() 
#define HSIClockFreq 16000000
#define BaudRate  115200
#define UART1_FLAG_TXE  (uint8_t)0x80  /*!< Transmit Data Register Empty flag */
#define UART1_FLAG_RXNE (uint8_t)0x20 /*!< Read Data Register Not Empty flag */
#define RxBufferSize 64
u8 RxBuffer[RxBufferSize];
u8 UART_RX_NUM=0;
/**************************************************************
       ���ò����ʣ���
       (1) ������дBRR2
       (2) BRR1��ŵ��Ƿ�Ƶϵ���ĵ�11λ����4λ��
       (3) BRR2��ŵ��Ƿ�Ƶϵ���ĵ�15λ����12λ���͵�3λ����0λ
       ���磺������λ9600ʱ����Ƶϵ��=2000000/9600=208
       ��Ӧ��ʮ��������Ϊ00D0��BBR1=0D,BBR2=00
***************************************************************/
void Init_UART1(void)
{
      UART1_CR1=0x00;
      UART1_CR2=0x00;
      UART1_CR3=0x00;
      UART1_BRR2=0x00;
      UART1_BRR1=0x0d;
      UART1_CR2=0x2c;//b3 = 1,������
                    // b2 = 1,�������
                    // b5 = 1,������������ж� 
}

void UART1_sendchar(unsigned char ch) //����һ���ַ�
{
      while((UART1_SR & 0x80)==0x00); // �����ͼĴ������գ���ȴ�
      UART1_DR=ch;                    // ��Ҫ���͵��ַ��͵����ݼĴ���
} 

#pragma vector=0x14
__interrupt void UART1_RX_IRQHandler(void)
{ 
   u8 Res;
    if(UART1_SR & UART1_FLAG_RXNE)  
    {/*�����ж�(���յ������ݱ�����0x0d 0x0a��β)*/
	Res =(uint8_t)UART1_DR;
        /*(USART1->DR);��ȡ���յ�������,���������ݺ��Զ�ȡ��RXNE���жϱ�־λ*/
	if(( UART_RX_NUM&0x80)==0)/*����δ���*/
	{
	    if( UART_RX_NUM&0x40)/*���յ���0x0d*/
		{
		  if(Res!=0x0a) UART_RX_NUM=0;/*���մ���,���¿�ʼ*/
		  else  UART_RX_NUM|=0x80;	/*��������� */
		}
            else /*��û�յ�0X0D*/
              {	
                if(Res==0x0d) UART_RX_NUM|=0x40;
                else
                  {
                    RxBuffer[ UART_RX_NUM&0X3F]=Res ;
                     UART_RX_NUM++;
                      if( UART_RX_NUM>63) UART_RX_NUM=0;/*�������ݴ���,���¿�ʼ����*/  
                  }		 
	      }
	 }  		 
      }
}

//*************************************************************************
// * ��������UART1_SendByte
// * ����  ��uart����һ���ֽ�
// * ����  ��u8 data
// *
// * ���  ����
// * ����  ���� 
// * ����  ���ⲿ���� 
// * ����  ��UART1_SendByte('a')
// ************************************************************************
//void UART1_SendByte(u8 data)
//{
//   UART1_DR=data;
//   while (!(UART1_SR & UART1_FLAG_TXE));
//}

//*******************************************************************************
// * ��������UART1_SendByte
// * ����  ��uart�����ַ���
// * ����  ��u8* Data,u16 len
// *
// * ���  ����
// * ����  ���� 
// * ����  ���ⲿ���� 
// * ����  ��UART1_SendString("iCreate STM8������",sizeof("iCreate STM8������"))
//*******************************************************************************
//void UART1_SendString(u8* Data,u16 len) //����һ���ַ�
//{
// u16 i=0;
//  for(;i<len;i++)
//  UART1_SendByte(Data[i]);
//}

//*********************************************************************************
// * ��������UART1_ReceiveByte
// * ����  ��uart��ѯ����һ���ֽ�
// * ����  ����
// *
// * ���  ����
// * ����  ��һ���ֽ� 
// * ����  ���ⲿ���� 
// * ����  ��temp=UART1_ReceiveByte()
//*********************************************************************************
//u8 UART1_ReceiveByte(void)
//{
//     u8 USART1_RX_BUF; 
//     while (!(UART1_SR & UART1_FLAG_RXNE));
//     USART1_RX_BUF=(uint8_t)UART1_DR;
//     return  USART1_RX_BUF;
//}
