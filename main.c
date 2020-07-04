//***********************************************************
//* @file stm8s103K6 _main.c                                *
//* @author STMicroelectronics - MCU Application Team       *
//* @version V0.0.0                                         *
//* @date 11-18-2019      21                                  *
//* 注意：beep需要在OPTION内设置
//***********************************************************
#include "iostm8s105k6.h" //STM8系列头文件
//#include "intrinsics.h"
#include "stdio.h"
#include "DS1302.c"

#define rim() asm("rim");  /* enable interrupts */
#define sim() asm("sim");  /* disable interrupts */
#define nop() asm("nop");  /* No Operation */

#define LED PD_ODR_ODR1    //SWIM
//HPDL1414 驱动管脚定义
#define D6  PD_ODR_ODR3    //数据总线
#define D5  PC_ODR_ODR2    //数据总线
#define D4  PC_ODR_ODR1    //数据总线
#define D3  PD_ODR_ODR2    //数据总线
#define D2  PD_ODR_ODR0    //数据总线
#define D1  PA_ODR_ODR2    //数据总线
#define D0  PA_ODR_ODR1    //数据总线
#define A0 PE_ODR_ODR5     //地址位0
#define A1 PB_ODR_ODR0     //地址位
#define WR1 PD_ODR_ODR7    //位1
#define WR2 PC_ODR_ODR4    //位2
#define WR3 PC_ODR_ODR3    //位3
#define WR4 PC_ODR_ODR5    //位4

#define BEEP_ON  BEEP_CSR =0x2E   //ON 1KHZ
#define BEEP_OFF BEEP_CSR =0x1E   //OFF
#define K1 PC_IDR_IDR6    //按键1
#define K2 PC_IDR_IDR7    //按键2

#define DHT_IN   PB_IDR_IDR5
#define DHT_OUT  PB_ODR_ODR5

//时间类变量
unsigned int  TIM_BASE;
unsigned char LOOP=0;
unsigned char temp,AD_FLAG=0;
unsigned char HPDL1414;
unsigned char HH=00,MM=00,SS=00,SSS=0; //时 分 秒 0.1秒
unsigned int CONV,AD_TEMP;
unsigned char TIMER_SEC,TIMER_MIN,TIMER_HOUR;
unsigned char TIMER_YEAR,TIMER_MON,TIMER_DAY,TIMER_WEEK;
unsigned char SET_YEAR,SET_MON,SET_DAY,SET_WEEK;
unsigned char SET_SEC,SET_MIN,SET_HOUR;
unsigned char MODE=0; //MODE=0 时钟  MODE=1 秒表
unsigned char key_flag;
unsigned int  key_delay=0; //按键延时
unsigned char beep_delay=0;


unsigned char DHT_TEMP,DHT_HUMI;
int calctemp(int temp);

void Delay_us(unsigned int p) 
{  
  int i;
    for(i=0;i<p;i++)   
    {
    asm("nop"); //一个asm("nop")函数经过示波器测试代表100ns 
    asm("nop"); 
    asm("nop"); 
    asm("nop");  
    }
}

//---- 2M晶振 毫秒级延时程序-----------------------   
void delay_ms(unsigned int time)   
{   
  unsigned int i;   
  while(time--)     
    for(i=113;i>0;i--)   
    {
      asm("nop"); 
      asm("nop");   
      asm("nop");   
      asm("nop");
    }
}  

void Delay31us()		//@2MHz
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

void Delay50us()		//@2MHz
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
unsigned int timeout;
unsigned char Cheakdata;
unsigned char DHT11_Us_Buf[4];
unsigned char DHT11_Recv_Buf[7];

unsigned char Readuchar(void)
{
	unsigned char i;
	unsigned char buf = 0;
	for (i = 0; i<8; i++)
	{
		DHT_OUT = 1;
		timeout = 0;
		while ((DHT_IN == 0) && ((timeout++)<500));
		Delay31us();
		buf = buf << 1;
		if (DHT_IN == 1)
			buf = buf | 0x01;
		timeout = 0;
		while ((DHT_IN == 1) && ((timeout++)<500));
	}
	return buf;
}


void DHT11_ReadRH(void)
{
	unsigned char i;
        
       PB_DDR=0x2B;
      PB_CR1=0x2B;
        PB_CR2=0;
        
	DHT_OUT = 0;
	delay_ms(20);//20ms
	DHT_OUT = 1;
        
       PB_DDR = 0x0B;
       PB_CR1 =0x2B;
       PB_CR2 =0;
       
	Delay50us();
	if (DHT_IN == 0)
	{
		timeout = 0;
		while ((DHT_IN == 0) && ((timeout++)<500));
		timeout = 0;
		while ((DHT_IN == 1) && ((timeout++)<500));
		DHT11_Recv_Buf[0] = Readuchar();
		DHT11_Recv_Buf[2] = Readuchar();
		DHT11_Recv_Buf[1] = Readuchar();
		DHT11_Recv_Buf[3] = Readuchar();
		Cheakdata = Readuchar();
		if ((DHT11_Recv_Buf[0] + DHT11_Recv_Buf[1] + DHT11_Recv_Buf[2] + DHT11_Recv_Buf[3]) == Cheakdata)
                {
                    DHT_TEMP = calctemp(DHT11_Recv_Buf[1]);
                    DHT_HUMI = DHT11_Recv_Buf[0];
                }
	}
}
//RAM
char Display_RAM[16];


//IO管脚定义
//DDR   CR1   CR2     引脚设置
// 0     0     0      悬浮输入
// 0     1     0      上拉输入
// 0     0     1      中断悬浮输入
// 0     1     1      中断上拉输入
// 1     0     0      开漏输出
// 1     1     0      推挽输出
// 1     X     1      输出（最快速度为10MHZ）
//输出引脚对应的寄存器为ODR，输入引脚对应的寄存器为IDR。
void Init_GPIO(void)
{
  PA_DDR = 0x06; //方向寄存器；00000110 
  PA_CR1 = 0x06; //控制寄存器1 00000110
  PA_CR2 = 0x00; //控制寄存器2 00000000
  
  PC_DDR = 0x3F; //00111110  
  PC_CR1 = 0xFF; //11111110 
  PC_CR2 = 0x00; //00000000 
  
  PB_DDR = 0x0B; //PB45 0000 1011
  PB_CR1 = 0x0B; 
  PB_CR2 = 0x00;
  
  PD_DDR = 0xBF;//10111101 (RX)6 (TX)5
  PD_CR1 = 0xBF;//10111101 BD
  PD_CR2 = 0x00;//01000000
  
  PE_DDR = 0xFF; //00010000 //PE5
  PE_CR1 = 0xFF; //00010000 
  PE_CR2 = 0x00; //00000000  
  
  PF_DDR = 0x00; //00000000 //PF4 - AN12
  PF_CR1 = 0x00; //00000000 
  PF_CR2 = 0x00; //00000000
}       

void Init_CLK(void)
{
  CLK_ICKR|=0x01;         //开启内部HSI
  while(!(CLK_ICKR&0x02));//HSI准备就绪
  CLK_SWR=0xe1;           //HSI为主时钟源
  //EP_CSR = 0x20|14;     //输出1KHz 
  //CLK_CKDIVR=0x00;         //HSI不分频 16MHZ
  CLK_CKDIVR=0x18;        //HSI8分频，2M，复位后的默认值
}

void TIM4_init(void)
{
  CLK_PCKENR1 |=0x10; //默认开启时钟.
  TIM4_IER=0X00;      //禁止中断
  TIM4_PSCR=0x07;     //预分频值:1,2,4,8,16,32,64,128  计数器时钟=主时钟/128=2MHZ/128=64uS
  TIM4_CNTR=70;       //计数器初始值                
  TIM4_ARR=70;        //8位向上计数(0->ARR)的自动重载计数器  定时周期=(ARR+1)*64us=4992uS ;ARR=70
  TIM4_EGR=0x01;      //计数器重新初始化并产生寄存器更新
  TIM4_IER=0x01;      //使能更新中断
  TIM4_CR1=0x01;      //使能计数器
}
int blinkcount =0;

void COLCK();
#pragma vector=TIM4_OVR_UIF_vector//0x19
__interrupt void TIM4_OVR_UIF_IRQHandler(void)//对应IAP的中断地址：0x8060
{
  TIM4_SR=0x00; //清中断标记
  TIM_BASE++;   //时间基准++ 2MHZ-5ms
  
  blinkcount++;
    if(blinkcount>200)
      blinkcount=0;
  
  if (key_flag==1)  {if (key_delay<604) key_delay++;}
  if (beep_delay>0)  beep_delay--; else  BEEP_OFF;
  switch(TIM_BASE)
  {
  case 20:  {SSS=1;break;}
  case 40:  {SSS=2;break;}
  case 60:  {SSS=3;break;}
  case 80:  {SSS=4;break;}
  case 100: {SSS=5;COLCK();break;}
  case 120: {SSS=6;break;}
  case 140: {SSS=7;break;}
  case 160: {SSS=8;
    DHT11_ReadRH();break;}
  case 180: {SSS=9;break;}
  case 200: {SSS=0;
  AD_FLAG=1;  //电池电压检测
  TIM_BASE=0; //秒计时基准
  SS++;//秒+1
  if (SS>59) {MM++;SS=0;}//分+1
  if (MM>59) {HH++;MM=0;}//时+1
  if ((HH==24)&(MM==0)&(SS==0)) {HH=0;} //小时复位
  
  break;}
  default:{break;}
  }
  return; //返回
}
#define u8 unsigned char
int setmode = 0;
int PP =0;
/********************************************************************************************************
** 	函数名称:			BCD2HEX(u8 val)
**	功能描述:			BCD转HEX
********************************************************************************************************/
u8	BCD2HEX(u8 val)
{
	return	((val >> 4) * 10) + (val & 0x0f);
}
/********************************************************************************************************
**	功能描述:			HEX转BCD
**	入口参数:			val:HAX码
********************************************************************************************************/
u8	HEX2BCD(u8 val)
{
	return	(((val % 100) / 10) << 4) | (val % 10);
}
void keyscan()
{
  if ((K1==0)||(K2==0)) //有按键
  { 
    delay_ms(20); //消抖动
    BEEP_ON;
    if (K2==0) {
      key_flag=1;  //开始按键计时
      while(!K2);  //等待按键恢复
      key_flag=0;  //停止按键计时
      setmode++;
      blinkcount = 0;
      if(setmode>6)
        setmode=0;
    }
    if (K1==0) {
      key_flag=1;  //开始按键计时
      while(!K1);  //等待按键恢复
      switch(setmode)
      {
      case 0:
        if(PP<400)
        {
          PP=399;
        }
        else if(PP<600)
          PP=599;
        else if(PP<800)
          PP=799;
        else
        {
          PP=199;
        }break;
      case 1:
        SET_MON = BCD2HEX(R1302(0x89));
        SET_MON++;
        if(SET_MON>7)
          SET_MON = 1;
          
        W1302(0x88,HEX2BCD(SET_MON));break;
      case 2:
        SET_DAY = BCD2HEX(R1302(0x87));
        SET_DAY++;
        if(SET_DAY>31)
          SET_DAY = 1;
          
        W1302(0x86,HEX2BCD(SET_DAY));break;
      case 3:
        SET_HOUR = BCD2HEX(R1302(0x85));
        SET_HOUR++;
        if(SET_HOUR>59)
          SET_HOUR = 0;
          
        W1302(0x84,HEX2BCD(SET_HOUR));break;
      case 4:
        SET_MIN = BCD2HEX(R1302(0x83));
        SET_MIN++;
        if(SET_MIN>59)
          SET_MIN = 0;
          
        W1302(0x82,HEX2BCD(SET_MIN));break;
      case 5:
          
        W1302(0x80,0);break;
      case 6:
        SET_WEEK = BCD2HEX(R1302(0x8B));
        SET_WEEK++;
        if(SET_WEEK>7)
          SET_WEEK = 1;
          
        W1302(0x8A,HEX2BCD(SET_WEEK));break;
      }
    }
  }
  key_delay=0;
}

void LED_EN1(void) //2
{
  asm("nop");asm("nop");
  WR1=0;//
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  WR1=1;//
}

void LED_EN2(void)
{
  asm("nop");asm("nop");
  WR2=0;//
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  WR2=1;//
}

void LED_EN3(void)
{
  asm("nop");asm("nop");
  WR3=0;//
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  WR3=1;//
}

void LED_EN4(void)
{
  asm("nop");asm("nop");
  WR4=0;//
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  WR4=1;//
}

void disp(char HPDL1414)
{
  if (HPDL1414&0x40) D6=1; else D6=0;//0100 0000
  if (HPDL1414&0x20) D5=1; else D5=0;//0010 0000
  if (HPDL1414&0x10) D4=1; else D4=0;//0001 0000
  if (HPDL1414&0x08) D3=1; else D3=0;
  if (HPDL1414&0x04) D2=1; else D2=0;
  if (HPDL1414&0x02) D1=1; else D1=0;
  if (HPDL1414&0x01) D0=1; else D0=0;
}

/**************************************************************************
* 函数名：ADC_init
* 描述  ：ADC模块初始化
*************************************************************************/
void ADC_init()
{
  ADC_CR1 =0x40;                       //ADC时钟输入频率为16MHz 这里设置分频系数为2  连续转换模式 先禁止ADC转换       
  ADC_CR2 =0x00;                       //设置数据左对齐  8位模式 禁止扫描模式
  ADC_CSR =0x00;                       //不用外部触发 禁止转换结束中断 设置转换通道
  ADC_TDRL=0x01;                       //关闭施密特触发器 ,如果不会无法进入接收中断
}

/**************************************************************************
* 函数名：ADC_conversion
* 描述  ：ADC转换函数
* 输入  ：PD5 5通道
* 输出  ：高8位值
*************************************************************************/
unsigned char ADC_conversion(unsigned char CH)
{
  unsigned int Result;
  unsigned char i;
  ADC_CSR&=(~0x80);   //清中断标记
  ADC_CSR =CH;        //选择ADC通道
  ADC_CR1|=0x01;      //启动AD转换
  for(i=0;i<100;i++); //延时一段时间，至少7uS，保证ADC 模块上电正常
  ADC_CR1|=0x01;      //启动AD转换
  while(!(ADC_CSR & 0x80)); // 等待AD转换结束
  Result=ADC_DRH;
  //Result =((((unsigned int)ADC_DRH)<<2)+ADC_DRL);  //10位AD值
  return(Result); //返回AD转换高8位值
}

void AIOControl(int index)
{
  switch(index)
  {
  case 0:
  A0=1,A1=1;
  LED_EN1();break;
  case 1: 
  A0=0,A1=1;
  LED_EN1();break;
  case 2: 
  A0=1,A1=0;
  LED_EN1();break;
  case 3: 
  A0=0,A1=0;
  LED_EN1();break;
  
  case 4:
  A0=1,A1=1;
  LED_EN2();break;  
  case 5:
  A0=0,A1=1;
  LED_EN2();break;
  case 6:
  A0=1,A1=0;
  LED_EN2();break;
  case 7:
  A0=0,A1=0;
  LED_EN2();break;
  
  case 8:
  A0=1,A1=1;
  LED_EN3();break;
  case 9:
  A0=0,A1=1;
  LED_EN3();break;
  case 10:
  A0=1,A1=0;
  LED_EN3();break;
  case 11:
  A0=0,A1=0;
  LED_EN3(); break;
  
  case 12: 
  A0=1,A1=1;
  LED_EN4();break;
  case 13:
  A0=0,A1=1;
  LED_EN4();break;
  case 14:
  A0=1,A1=0;
  LED_EN4();break;
  case 15:
  A0=0,A1=0;
  LED_EN4();break;
  }
}

void Refresh_GRAM(void)
{
  int i;
  for(i = 0;i<16;i++)
  {
      disp(Display_RAM[i]);
      AIOControl(i);
  }
}

void Clear(void)
{
  int i;
  for(i = 0;i<16;i++)
  {
      Display_RAM[i] = ' ';
  }
}

void Display(int Delat,char* ch)
{
  int j=0;
  while(ch[j]!='\0')
  {
    if(Delat+j>=0&&Delat+j<16)
    {
      Display_RAM[Delat+j] = ch[j];
    }
    j++;
  }
}


int cont_str(char *s)
{
	int i = 0;      
	while ( s[++i] != '\0')   ;
	return i;
}


char StrTime[10];
char StrDate[10];
char StrMonth[10];
char StrWeek[10];
char StrDay[10];
char StrHumi[20];
char StrTemp[20];

void COLCK()
{
  //*** 实时时钟数据读取 +显示  ***
  TIMER_SEC =R1302(0x81); //读秒信息 
  
  TIMER_MIN =R1302(0x83); //读分信息
  
  TIMER_HOUR=R1302(0x85); //读时信息  
  
  TIMER_DAY =R1302(0x87); //读日信息 
  TIMER_MON =R1302(0x89); //读月信息
  TIMER_WEEK =R1302(0x8B); //读星期信息
  TIMER_YEAR =R1302(0x8D); //读年信息  
  
}

int Current[10] = {0};
int Target[10] = {0};

void MotionControl()
{
  if(setmode==0)
  {
    if(PP==0)
    {
      Target[0] = 0;
      Target[1] = 8;
      Target[2] = -8;
      
      Target[3] = 16+8;
      Target[4] = 16+8;
      
      Current[0] = -8;
      Current[1] = 16;
      Current[2] = -8-8;
      Current[3] = 16+8;
      Current[4] = 16+8;
    }
    else if(PP==200)//显示
    {
      Target[0] = 8;
      Target[1] = 16+8;
      Target[2] = 0;
      
      Target[3] = 16+8;
      Target[4] = 16+8;
    }
    else if(PP==400)//显示星期
    {
      Target[0] = 0;
      Target[1] = 8;
      Target[2] = -8-8;
      Target[3] = 16+8;
      Target[4] = 16+8;
    }
    else if(PP==600)//显示湿度
    {
      Target[0] = 0;
      Target[1] = 16+8;
      Target[2] = -8-8;
      Target[3] = 8;
      Target[4] = 16+8;
    }
    else if(PP==800)//显示温度
    {
      Target[0] = 0;
      Target[1] = 16+8;
      Target[2] = -8-8;
      Target[3] = 16+8;
      Target[4] = 8;
    }
  
    PP++;
    if(PP>1000)
      PP=200;
  }
  else if(setmode < 6)
  {
      Target[0] = 8;
      Target[1] = 16+8;
      Target[2] = 0;
      
      Target[3] = 16+8;
      Target[4] = 16+8;
  }
  else
  {
      Target[0] = 0;
      Target[1] = 8;
      Target[2] = -8-8;
      Target[3] = 16+8;
      Target[4] = 16+8;
  }
}

void DampRun()
{
  int i;
  for(i=0;i<10;i++)
  {
  if(Current[i]>Target[i])
    Current[i] -- ;
  else if(Current[i]<Target[i])
    Current[i] ++ ;
  else 
    Current[i] = Target[i];
  }
}

void TimeRun()
{
  
    MotionControl();
    DampRun();
    Clear();
    
    {
      switch(TIMER_MON)
      {
      case 1:sprintf(StrMonth,"JAN");break;
      case 2:sprintf(StrMonth,"FEB");break;
      case 3:sprintf(StrMonth,"MAR");break;
      case 4:sprintf(StrMonth,"APR");break;
      case 5:sprintf(StrMonth,"MAY");break;
      case 6:sprintf(StrMonth,"JUN");break;
      case 7:sprintf(StrMonth,"JUL");break;
      case 8:sprintf(StrMonth,"AUG");break;
      case 9:sprintf(StrMonth,"SEP");break;
      case 0x10:sprintf(StrMonth,"OCT");break;
      case 0x11:sprintf(StrMonth,"NOV");break;
      case 0x12:sprintf(StrMonth,"DEC");break;
      default:sprintf(StrMonth,"UKW");break;
      }
      switch(TIMER_WEEK)
      {
      case 1:sprintf(StrWeek,"  MONDAY");break;
      case 2:sprintf(StrWeek," TUESDAY");break;
      case 3:sprintf(StrWeek,"WEDNSDAY");break;
      case 4:sprintf(StrWeek,"THURSDAY");break;
      case 5:sprintf(StrWeek,"  FRIDAY");break;
      case 6:sprintf(StrWeek,"SATURDAY");break;
      case 7:sprintf(StrWeek,"  SUNDAY");break;
      default:sprintf(StrWeek,"  UNKNOW");break;
      }
      if(blinkcount<=100||setmode)
        sprintf(StrTime,"%02X-%02X-%02X",TIMER_HOUR,TIMER_MIN,TIMER_SEC);
      else
        sprintf(StrTime,"%02X %02X %02X",TIMER_HOUR,TIMER_MIN,TIMER_SEC);
        
      sprintf(StrDay,"%2X",TIMER_DAY);
      sprintf(StrDate,"%s-%s )",StrMonth,StrDay);
    }
    if(blinkcount<=100)
    {
      switch(setmode)
      {
      case 1:sprintf(StrDate,"   -%s )",StrDay);break;
      case 2:sprintf(StrDate,"%s-   )",StrMonth);break;
      case 3:sprintf(StrTime,"  -%02X-%02X",TIMER_MIN,TIMER_SEC);break;
      case 4:sprintf(StrTime,"%02X-  -%02X",TIMER_HOUR,TIMER_SEC);break;
      case 5:sprintf(StrTime,"%02X-%02X-  ",TIMER_HOUR,TIMER_MIN);break;
      case 6:sprintf(StrWeek,"        ");break;
      }
    }
    
      sprintf(StrHumi,"HUMI-%02d%%",DHT_HUMI);
      sprintf(StrTemp,"TEMP-%02dC",DHT_TEMP);
    
    //else
    Display(Current[0],StrTime);
    Display(Current[1],StrWeek);
    Display(Current[2],StrDate);
    Display(Current[3],StrHumi);
    Display(Current[4],StrTemp);
    
    Refresh_GRAM();
}

int calctemp(int temp)
{
  static int defaulttemp;
  static int count=0;
  static int dlat=0;
  if(count == 0)
  {
    defaulttemp = temp;
    if(defaulttemp!=0)
      count ++;
    return defaulttemp;
  }
  else if(count < 20*60)
  {
    count ++;
    dlat = temp - defaulttemp;
    return defaulttemp;
  }
  else
  {
    return temp-dlat;
  }
}

//-----------------------------------
// Main Program主程序
//-----------------------------------
int main(void)
{
  asm("sim");  
  Init_CLK();
  Init_GPIO();   //初始化I/O
  TIM4_init();
  ADC_init();
  init_dS1302();
  COLCK();
  asm("rim");  //开全局中断
  WR1=1;WR2=1;WR3=1;WR4=1;//LED=0;
  BEEP_ON;
//     if(start_DHT11()==1)//检测是否有响应，结果有响应往下执行
   {      
 //    DHT11_Get_Data(); 
   }
     DHT11_ReadRH();
  while (1)
  {
    delay_ms(20); 
    TimeRun();
    keyscan();
    //***  电池电压检测 + 显示   约1秒刷新一次数据
    if (AD_FLAG==1)  //1秒1次 电池电压检测
    {
      AD_FLAG=0;
      AD_TEMP=ADC_conversion(12);
      CONV=(256*18)/AD_TEMP;
      CONV=CONV*10; //电池电压转换
    }
  }
}//main()
// [ program END 程序完 2]