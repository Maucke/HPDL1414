/*************************************************************************
*  DS1302.C     
*  Ӳ������˵����1.��ʼ��ʱҪ�Ȱѿ��ƼĴ��������λ(wp)��0�����������д1302��
*                  ��ʼ��������1����ֹ��д����ֹ����
*                2.��ʼ���˳�ǰһ��Ҫ����Ĵ��������λ(ch)��־λ��0������ʱ
*                  �Ӳ���.
*                3.���г����ҲҪע������(�������ǣ��ǵ���ʱ����������⣮
*                  �ø�������΢�����˲����� ���)
**************************************************************************/
#include "iostm8s105k6.h" //STM8ϵ��ͷ�ļ�
/*�����������Ͷ���*/
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

//�Ĵ����궨��
void RTInputByte(u8 TimeNum) //д���ݵ�DS1302
{ 
  u8 i;
  PB_DDR |= 0x04; // 0000 0100
  PB_CR1 |= 0x04; // 0000 0100
  //SDA ����Ϊ���
  for(i=8; i>0; i--)//д8λ
  {
    if((TimeNum&0x01)!=0)//��λд��.������λΪ1
      SDA_OUT=1;//SetTIO IO����λ
    else//������λΪ0
      SDA_OUT=0;//ClrTIO IO����λ
    SCLK=1;//SetTCLK;
    nop_();
    SCLK=0;//ClrTCLK ��һʱ������
    TimeNum>>=1;         //����һλ
  } 
  PB_DDR &= 0xFB; // 1111 1011
  PB_CR1 |= 0x04; // 0000 0100
  //SDA ����Ϊ��������
}

u8 RTOutputByte(void) 
{ 
  u8 i,TimeNum;
  PB_DDR &= 0xFB; // 1111 1011
  PB_CR1 |= 0x04; // 0000 0100
  //SDA ����Ϊ��������
  for(i=8; i>0; i--)
  {
    TimeNum>>=1;  //��ȡ����       
    if(SDA_IN!=0)//���DS1302 IO��Ϊ1
      TimeNum|=0x80;
    SCLK=1;//SetTCLK;
    nop_();
    SCLK=0;//ClrTCLK ;//��һʱ������
  } 
  return(TimeNum); 
}

void W1302(u8 ucAddr, u8 ucDa)//��DS1302 д���� 
{
  RST=0;//ClrTRST; RST ���� 
  SCLK=0;//ClrTCLK; CLK ����
  nop_();
  RST=1;//SetTRST; RST ��λ
  nop_();nop_();
  RTInputByte(ucAddr);//
  nop_();
  RTInputByte(ucDa); //   
  SCLK=1;//SetTCLK; CLK ��λ
  RST=0;//ClrTRST; RST ����
} 

u8 R1302(u8 ucAddr)//��ȡʵʱ��Ϣ
{
  u8 ucData;
  W1302(0xBE,1); //д 1�� �Ĵ��� BEH ���������
  RST=0;//ClrTRST; RST ����
  SCLK=0;//ClrTCLK ;CLK ����
  nop_();
  RST=1;//SetTRST; RST ��λ
  nop_();
  RTInputByte(ucAddr);  //��ȡ��Ӧ�ĵ�ַ,������ 8CH,�� 88H
  ucData = RTOutputByte();//��ȡʵʱ��Ϣ        
  SCLK=1;//SetTCLK; CLK ��λ
  RST=0; //ClrTRST; RST ����
  return(ucData);
}

void init_dS1302(void)//ʱ�ӳ�ʼ��
{
  W1302(0x8E,0);     //�����
  W1302(0x90,0xA5);  //�����س���൱��2mA   ������+2ǧŷ
  
  //����Ϊʱ���趨,�趨һ�ξͿ�����,�Ժ���Ϳ�����,��������ʵʱ��Ϣ
  if(R1302(0x81)>=0x60)  
    W1302(0x80,0x00);//���� 
  if(R1302(0x83)>=0x60)  
    W1302(0x82,0x08);//�÷���
   if(R1302(0x85)>=0x24) 
  {
    W1302(0x84,0x20);//��Сʱ
    W1302(0x82,0x56);//�÷���
    W1302(0x8C,0x20);//������ʱ�����  ��
    W1302(0x8A,0x07);//������
    W1302(0x88,0x06);//����
    W1302(0x86,0x28);//����
  }
  
  //�������������������� �ϵ�ʱ������
  //W1302(0x80,0x00);//���� 
  //W1302(0x82,0x58);//�÷���
  //W1302(0x84,0x18);//��Сʱ
  //������ �ϵ� �� �� �� ���� 
  //W1302(0x8C,0x11);//������ʱ�����  ��
  //W1302(0x8A,0x01);//������
  //W1302(0x88,0x01);//����
  //W1302(0x86,0x01);//����
  //W1302(0x8E,0x80);//д����,�и�����Ҫ����
  
}
