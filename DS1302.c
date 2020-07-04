/*************************************************************************
*  DS1302.C     
*  硬件设置说明：1.初始化时要先把控制寄存器的最高位(wp)置0，才能允许读写1302，
*                  初始化后再置1，禁止读写，防止干扰
*                2.初始化退出前一定要把秒寄存器的最高位(ch)标志位置0，否则时
*                  钟不走.
*                3.还有充电器也要注意设置(问题解决是．是掉电时脉冲干扰问题．
*                  用个１００微法的滤波电容 解决)
**************************************************************************/
#include "iostm8s105k6.h" //STM8系列头文件
/*常用数据类型定义*/
#define u8  unsigned char
#define u16 unsigned int
#define u32 unsigned long
#define enableInterrupts()   asm("rim")    /* enable interrupts */
#define disableInterrupts()  asm("sim")    /* disable interrupts*/
#define nop_()               asm("nop")    /* No Operation      */
#define trap()               asm("trap")   /* Trap (soft IT)    */
#define wfi()                asm("wfi")    /* Wait For Interrupt*/
#define halt()               asm("halt")   /* Halt              */

#define SCLK PB_ODR_ODR3
#define SDA_IN  PB_IDR_IDR2
#define SDA_OUT PB_ODR_ODR2
#define RST  PB_ODR_ODR1

//寄存器宏定义
void RTInputByte(u8 TimeNum) //写数据到DS1302
{ 
  u8 i;
  PB_DDR |= 0x04; // 0000 0100
  PB_CR1 |= 0x04; // 0000 0100
  //SDA 设置为输出
  for(i=8; i>0; i--)//写8位
  {
    if((TimeNum&0x01)!=0)//低位写起.如果最低位为1
      SDA_OUT=1;//SetTIO IO口置位
    else//如果最低位为0
      SDA_OUT=0;//ClrTIO IO口清位
    SCLK=1;//SetTCLK;
    nop_();
    SCLK=0;//ClrTCLK 送一时钟脉冲
    TimeNum>>=1;         //右移一位
  } 
  PB_DDR &= 0xFB; // 1111 1011
  PB_CR1 |= 0x04; // 0000 0100
  //SDA 设置为上拉输入
}

u8 RTOutputByte(void) 
{ 
  u8 i,TimeNum;
  PB_DDR &= 0xFB; // 1111 1011
  PB_CR1 |= 0x04; // 0000 0100
  //SDA 设置为上拉输入
  for(i=8; i>0; i--)
  {
    TimeNum>>=1;  //读取数据       
    if(SDA_IN!=0)//如果DS1302 IO口为1
      TimeNum|=0x80;
    SCLK=1;//SetTCLK;
    nop_();
    SCLK=0;//ClrTCLK ;//送一时钟脉冲
  } 
  return(TimeNum); 
}

void W1302(u8 ucAddr, u8 ucDa)//向DS1302 写数据 
{
  RST=0;//ClrTRST; RST 清零 
  SCLK=0;//ClrTCLK; CLK 清零
  nop_();
  RST=1;//SetTRST; RST 置位
  nop_();nop_();
  RTInputByte(ucAddr);//
  nop_();
  RTInputByte(ucDa); //   
  SCLK=1;//SetTCLK; CLK 置位
  RST=0;//ClrTRST; RST 清零
} 

u8 R1302(u8 ucAddr)//读取实时信息
{
  u8 ucData;
  W1302(0xBE,1); //写 1到 寄存器 BEH 允许读操作
  RST=0;//ClrTRST; RST 清零
  SCLK=0;//ClrTCLK ;CLK 清零
  nop_();
  RST=1;//SetTRST; RST 置位
  nop_();
  RTInputByte(ucAddr);  //读取相应的地址,比如年 8CH,月 88H
  ucData = RTOutputByte();//读取实时信息        
  SCLK=1;//SetTCLK; CLK 置位
  RST=0; //ClrTRST; RST 清零
  return(ucData);
}

void init_dS1302(void)//时钟初始化
{
  W1302(0x8E,0);     //允许读
  W1302(0x90,0xA5);  //激活电池充电相当于2mA   二极管+2千欧
  
  //以下为时间设定,设定一次就可以了,以后读就可以了,读出的是实时信息
  if(R1302(0x81)>=0x60)  
    W1302(0x80,0x00);//置秒 
  if(R1302(0x83)>=0x60)  
    W1302(0x82,0x08);//置分钟
   if(R1302(0x85)>=0x24) 
  {
    W1302(0x84,0x20);//置小时
    W1302(0x82,0x56);//置分钟
    W1302(0x8C,0x20);//置日期时间调整  年
    W1302(0x8A,0x07);//置星期
    W1302(0x88,0x06);//置月
    W1302(0x86,0x28);//置日
  }
  
  //×××××××××× 上电时间设置
  //W1302(0x80,0x00);//置秒 
  //W1302(0x82,0x58);//置分钟
  //W1302(0x84,0x18);//置小时
  //××× 上电 年 月 日 设置 
  //W1302(0x8C,0x11);//置日期时间调整  年
  //W1302(0x8A,0x01);//置星期
  //W1302(0x88,0x01);//置月
  //W1302(0x86,0x01);//置日
  //W1302(0x8E,0x80);//写保护,有干扰需要考虑
  
}
