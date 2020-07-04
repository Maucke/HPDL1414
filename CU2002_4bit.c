//********************************************
//*  LCM1602 4��Һ������ ���� ���ģ�����   *
//*  ����оƬe : STM8S103F3                  *
//*  File name : �ӳ���                      *
//*  Date : 2016.9.17                        *
//--------------------------------------------
//��ɳ̫���� SMC1602A��Ʒ����˵������ʾ����
//PIN1: VSS  [��Դ��]-------------------VSS
//PIN2: VDD  [��Դ����]-----------------+5V
//PIN3: Vo   [LCDƫѹ����]------------- ��1K��VSS,��10K���赽VDD,(���10K�ɵ�,0Ϊ��ʾ���
//PIN4: RS   [����/����ѡ�������]------P2.5
//PIN5: RW   [��д�����ź�����]---------����VSS Ӳ����λд״̬��
//PIN6: E    [ʹ���ź�����]-------------P2.4
//PIN7: D0   [Data I/O]-----------------NC
//PIN8: D1   [Data I/O]-----------------NC
//PIN9: D2   [Data I/O]-----------------NC
//PIN10:D3   [Data I/O]-----------------NC
//PIN11:D4   [Data I/O]-----------------PC4��4bit��
//PIN12:D5   [Data I/O]-----------------PC5��4bit��
//PIN13:D6   [Data I/O]-----------------PC6��4bit��
//PIN14:D7   [Data I/O]-----------------PC7��4bit��
//PIN15:BLA  [����Դ����]---------------��10ŷ����ͨ��2TY��+5V.
//PIN16:BLK  [����Դ����]---------------VSS
//*******************************************

#define Set_EN PA_ODR_ODR1=1  //����EN��
#define Clr_EN PA_ODR_ODR1=0
#define Set_RS PA_ODR_ODR2=1  //����RS��
#define Clr_RS PA_ODR_ODR2=0
#define LCD_DATA PC_ODR       //����4Ϊ�����PC7-PC4����4λ��Ч��

//*************************************************
//24M 1��int a��ʱ 0.68uS
//16M 1��int a��ʱ 1.02us
//2M  1��int a��ʱ 8.16us
//*************************************************
void delay_us(unsigned int a)
{
while(a--);
}

//*********************
//*     ����ʹ��      *
//*********************
 void LCD_en (void)
 {
 Set_EN; 
 delay_us(2); //�����̣�
 Clr_EN;
 delay_us(2); //�����̣�
 }

//*********************
//* ָ������������  *
//*********************
 void LCM1602A_cmd (unsigned char cmd)
 {
 delay_us(5); //40us��ʱ
 Clr_RS;     //ָ�� 
 LCD_DATA &= 0x0F;           //��I/O����λ 
 LCD_DATA |= (cmd&0xF0);     //ָ�����λ�� I/O����λ���
 LCD_en();
 LCD_DATA &= 0x0F;           //��I/O����λ 
 LCD_DATA |= (cmd&0x0F)<<4;  //ָ�����λ�� I/O����λ���
 LCD_en(); 
 }

//*********************
//*      ��������     *
//*********************
void LCM1602A_dat (unsigned char dat)
 {
 delay_us(5); //40us
 Set_RS;  //����
 LCD_DATA &= 0x0F;  //��I/O ����λ 
 LCD_DATA |= (dat&0xF0); //���ݸ���λ�� I/O����λ��� 
 LCD_en();
 LCD_DATA &= 0x0F; //��I/O ����λ 
 LCD_DATA |= (dat&0x0F)<<4; //���ݵ���λ�� I/O����λ���
 LCD_en();
 }

//*********************
//*      Һ������     *
//*********************
 void LCD_clr (void)
 {
 LCM1602A_cmd(0x01);
 delay_us(250); //1700 us
 }

//*****************************************
//*              Һ����λ                 *
//����:����(XPOS,YPOS)�ַ�λ�õ�DDRAM��ַ.*
//XPOS:λ�÷�Χ(0��15)                    *
//YPOS:λ�÷�Χ(0��1)                     *
//*****************************************
 void LCM1602A_pos (unsigned char XPOS, unsigned char YPOS)
 {
    if(YPOS==0)                   //(��һ��)X: ��0----15���ַ�
        LCM1602A_cmd(XPOS|0x80);  //    DDRAM:   0----0FH
    else                          //(�ڶ���)X: ��0----15���ַ�
        LCM1602A_cmd(XPOS|0xC0);  //    DDRAM:  40----4FH        
 }

//*************************
//*    ���ַ���λ���     *
//*************************
 void LCM1602A_printc (unsigned char x, unsigned char y, unsigned char c)
 {
 LCM1602A_pos(x,y);
 LCM1602A_dat(c);
 //Delayms(150); //1700 us
 }

//***************************
//*      �ַ�����λ���     *
//***************************
 void LCM1602A_prints (unsigned char x, unsigned char y, char const *s)
 {
 LCM1602A_pos(x,y);
 while(*s) {LCM1602A_dat(*s); s++; }
 }

//*******************************
//*      4��Һ����ʼ��          *
//*����:Һ����ʾ��������ʼ��.   *
//*******************************
void init4_LCM1602A(void)
 {
  delay_us(1500);//�ϵ���ʱ10ms
  LCM1602A_cmd(0x30);
  delay_us(450);//3ms
  LCM1602A_cmd(0x30);
  delay_us(450);
  LCM1602A_cmd(0x30);
  delay_us(450);
  LCM1602A_cmd(0x02);
  delay_us(450);
  LCM1602A_cmd(0x28); //������ʾ
  delay_us(450);
  LCM1602A_cmd(0x08); //��ʾ�ر�
  LCM1602A_cmd(0x01); //��ʾ����
  delay_us(1500);     //��ʱ10MS
  LCM1602A_cmd(0x06); //��ʾ����ƶ�����
  LCM1602A_cmd(0x0C); //��ʾ�����������
  LCM1602A_dat (0x00);
 }

