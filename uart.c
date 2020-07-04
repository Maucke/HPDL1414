/*
***********************************************************
* @file stm8s_uart.c
* @author STMicroelectronics - MCD Application Team
* @version V0.0.0
* @date 15-March-2016
* 初始化UART，建议以中断方式接收字符，以查询方式发送
* UART通讯参数：9600bps,8位数据，1位停止位，无校验
* 注意：与PC机相连的线，2、3脚需要交叉。
***********************************************************
*/ 
#include"iostm8s103F3.h"
/*常用数据类型定义*/
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
       设置波特率：：
       (1) 必须先写BRR2
       (2) BRR1存放的是分频系数的第11位到第4位，
       (3) BRR2存放的是分频系数的第15位到第12位，和第3位到第0位
       例如：波特率位9600时，分频系数=2000000/9600=208
       对应的十六进制数为00D0，BBR1=0D,BBR2=00
***************************************************************/
void Init_UART1(void)
{
      UART1_CR1=0x00;
      UART1_CR2=0x00;
      UART1_CR3=0x00;
      UART1_BRR2=0x00;
      UART1_BRR1=0x0d;
      UART1_CR2=0x2c;//b3 = 1,允许发送
                    // b2 = 1,允许接收
                    // b5 = 1,允许产生接收中断 
}

void UART1_sendchar(unsigned char ch) //发送一个字符
{
      while((UART1_SR & 0x80)==0x00); // 若发送寄存器不空，则等待
      UART1_DR=ch;                    // 将要发送的字符送到数据寄存器
} 

#pragma vector=0x14
__interrupt void UART1_RX_IRQHandler(void)
{ 
   u8 Res;
    if(UART1_SR & UART1_FLAG_RXNE)  
    {/*接收中断(接收到的数据必须是0x0d 0x0a结尾)*/
	Res =(uint8_t)UART1_DR;
        /*(USART1->DR);读取接收到的数据,当读完数据后自动取消RXNE的中断标志位*/
	if(( UART_RX_NUM&0x80)==0)/*接收未完成*/
	{
	    if( UART_RX_NUM&0x40)/*接收到了0x0d*/
		{
		  if(Res!=0x0a) UART_RX_NUM=0;/*接收错误,重新开始*/
		  else  UART_RX_NUM|=0x80;	/*接收完成了 */
		}
            else /*还没收到0X0D*/
              {	
                if(Res==0x0d) UART_RX_NUM|=0x40;
                else
                  {
                    RxBuffer[ UART_RX_NUM&0X3F]=Res ;
                     UART_RX_NUM++;
                      if( UART_RX_NUM>63) UART_RX_NUM=0;/*接收数据错误,重新开始接收*/  
                  }		 
	      }
	 }  		 
      }
}

//*************************************************************************
// * 函数名：UART1_SendByte
// * 描述  ：uart发送一个字节
// * 输入  ：u8 data
// *
// * 输出  ：无
// * 返回  ：无 
// * 调用  ：外部调用 
// * 举例  ：UART1_SendByte('a')
// ************************************************************************
//void UART1_SendByte(u8 data)
//{
//   UART1_DR=data;
//   while (!(UART1_SR & UART1_FLAG_TXE));
//}

//*******************************************************************************
// * 函数名：UART1_SendByte
// * 描述  ：uart发送字符串
// * 输入  ：u8* Data,u16 len
// *
// * 输出  ：无
// * 返回  ：无 
// * 调用  ：外部调用 
// * 举例  ：UART1_SendString("iCreate STM8开发板",sizeof("iCreate STM8开发板"))
//*******************************************************************************
//void UART1_SendString(u8* Data,u16 len) //发送一串字符
//{
// u16 i=0;
//  for(;i<len;i++)
//  UART1_SendByte(Data[i]);
//}

//*********************************************************************************
// * 函数名：UART1_ReceiveByte
// * 描述  ：uart查询接收一个字节
// * 输入  ：无
// *
// * 输出  ：无
// * 返回  ：一个字节 
// * 调用  ：外部调用 
// * 举例  ：temp=UART1_ReceiveByte()
//*********************************************************************************
//u8 UART1_ReceiveByte(void)
//{
//     u8 USART1_RX_BUF; 
//     while (!(UART1_SR & UART1_FLAG_RXNE));
//     USART1_RX_BUF=(uint8_t)UART1_DR;
//     return  USART1_RX_BUF;
//}
