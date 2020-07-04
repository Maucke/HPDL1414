//********************************************
//*  LCM1602 4线液晶驱动 并口 软件模拟程序   *
//*  控制芯片e : STM8S103F3                  *
//*  File name : 子程序                      *
//*  Date : 2016.9.17                        *
//--------------------------------------------
//长沙太阳人 SMC1602A产品引脚说明及演示连线
//PIN1: VSS  [电源地]-------------------VSS
//PIN2: VDD  [电源正极]-----------------+5V
//PIN3: Vo   [LCD偏压输入]------------- 接1K到VSS,接10K电阻到VDD,(或接10K可调,0为显示最深）
//PIN4: RS   [数据/命令选择端输入]------P2.5
//PIN5: RW   [读写控制信号输入]---------（接VSS 硬件置位写状态）
//PIN6: E    [使能信号输入]-------------P2.4
//PIN7: D0   [Data I/O]-----------------NC
//PIN8: D1   [Data I/O]-----------------NC
//PIN9: D2   [Data I/O]-----------------NC
//PIN10:D3   [Data I/O]-----------------NC
//PIN11:D4   [Data I/O]-----------------PC4（4bit）
//PIN12:D5   [Data I/O]-----------------PC5（4bit）
//PIN13:D6   [Data I/O]-----------------PC6（4bit）
//PIN14:D7   [Data I/O]-----------------PC7（4bit）
//PIN15:BLA  [背光源正极]---------------接10欧电阻通过2TY到+5V.
//PIN16:BLK  [背光源负极]---------------VSS
//*******************************************

#define Set_EN PA_ODR_ODR1=1  //配置EN脚
#define Clr_EN PA_ODR_ODR1=0
#define Set_RS PA_ODR_ODR2=1  //配置RS脚
#define Clr_RS PA_ODR_ODR2=0
#define LCD_DATA PC_ODR       //配置4为输出（PC7-PC4：高4位有效）

//*************************************************
//24M 1个int a延时 0.68uS
//16M 1个int a延时 1.02us
//2M  1个int a延时 8.16us
//*************************************************
void delay_us(unsigned int a)
{
while(a--);
}

//*********************
//*     输入使能      *
//*********************
 void LCD_en (void)
 {
 Set_EN; 
 delay_us(2); //可缩短！
 Clr_EN;
 delay_us(2); //可缩短！
 }

//*********************
//* 指令或控制字输入  *
//*********************
 void LCM1602A_cmd (unsigned char cmd)
 {
 delay_us(5); //40us延时
 Clr_RS;     //指令 
 LCD_DATA &= 0x0F;           //清I/O高四位 
 LCD_DATA |= (cmd&0xF0);     //指令高四位到 I/O高四位输出
 LCD_en();
 LCD_DATA &= 0x0F;           //清I/O高四位 
 LCD_DATA |= (cmd&0x0F)<<4;  //指令低四位到 I/O高四位输出
 LCD_en(); 
 }

//*********************
//*      数据输入     *
//*********************
void LCM1602A_dat (unsigned char dat)
 {
 delay_us(5); //40us
 Set_RS;  //数据
 LCD_DATA &= 0x0F;  //清I/O 高四位 
 LCD_DATA |= (dat&0xF0); //数据高四位到 I/O高四位输出 
 LCD_en();
 LCD_DATA &= 0x0F; //清I/O 高四位 
 LCD_DATA |= (dat&0x0F)<<4; //数据低四位到 I/O高四位输出
 LCD_en();
 }

//*********************
//*      液晶清屏     *
//*********************
 void LCD_clr (void)
 {
 LCM1602A_cmd(0x01);
 delay_us(250); //1700 us
 }

//*****************************************
//*              液晶定位                 *
//功能:设置(XPOS,YPOS)字符位置的DDRAM地址.*
//XPOS:位置范围(0到15)                    *
//YPOS:位置范围(0到1)                     *
//*****************************************
 void LCM1602A_pos (unsigned char XPOS, unsigned char YPOS)
 {
    if(YPOS==0)                   //(第一行)X: 第0----15个字符
        LCM1602A_cmd(XPOS|0x80);  //    DDRAM:   0----0FH
    else                          //(第二行)X: 第0----15个字符
        LCM1602A_cmd(XPOS|0xC0);  //    DDRAM:  40----4FH        
 }

//*************************
//*    单字符定位输出     *
//*************************
 void LCM1602A_printc (unsigned char x, unsigned char y, unsigned char c)
 {
 LCM1602A_pos(x,y);
 LCM1602A_dat(c);
 //Delayms(150); //1700 us
 }

//***************************
//*      字符串定位输出     *
//***************************
 void LCM1602A_prints (unsigned char x, unsigned char y, char const *s)
 {
 LCM1602A_pos(x,y);
 while(*s) {LCM1602A_dat(*s); s++; }
 }

//*******************************
//*      4线液晶初始化          *
//*功能:液晶显示控制器初始化.   *
//*******************************
void init4_LCM1602A(void)
 {
  delay_us(1500);//上电延时10ms
  LCM1602A_cmd(0x30);
  delay_us(450);//3ms
  LCM1602A_cmd(0x30);
  delay_us(450);
  LCM1602A_cmd(0x30);
  delay_us(450);
  LCM1602A_cmd(0x02);
  delay_us(450);
  LCM1602A_cmd(0x28); //四线显示
  delay_us(450);
  LCM1602A_cmd(0x08); //显示关闭
  LCM1602A_cmd(0x01); //显示清屏
  delay_us(1500);     //延时10MS
  LCM1602A_cmd(0x06); //显示光标移动设置
  LCM1602A_cmd(0x0C); //显示开及光标设置
  LCM1602A_dat (0x00);
 }

