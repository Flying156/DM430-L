/********************************************************************
//DM430-L����Сϵͳ�����2.8��TFT�����������λ��Ʋ��Գ���
//��ADC0��P60�ڣ��ɼ����ĵ�ѹֵ��ʾ�������ᣬ���Ƴɲ�������
//ADC0�����������ڲ�2.5V��׼�ο������2.5V������2.5V���������ֱ��
//�ó�������ο����û������и��ݸòο��������Ż�
//��TFT����12864�ӿڣ�������������Ҫ�öŰ������ӣ�������ο�ͼ��
//ע��ѡ��Һ���ĵ�Դ��λ�ڵ�λ����������ѡ5V��3.3V������Һ����ѹ����ѡ��Ĭ������Ϊ5V
//���Ի�����EW430 V5.30
//���ߣ�www.avrgcc.com
//ʱ�䣺2013.09.08
********************************************************************/
#include <msp430x14x.h>
#include "Config.h"                     //����ͷ�ļ�����Ӳ����ص��������������
#include "GUI.h"
#include "Ascii_8x16.h"                 //8x16��С�ַ�
#include "GB2424.h"                     //24x24���ش�С�ĺ���
#include "Chinese.h"                    //16x16���ش�С�ĺ���
#include "Touch.h"                      //TFT��������ͷ�ļ�
#include "Touch.c"                      //TFT����������ʼ��������
#include "TFT28.h"   
#include "DS18B20.c" //TFT��ʾͷ�ļ�
#include "TFT28.c"                      //TFT��ʾ������ʼ��������
#include "GUI.c"

//******************ȫ�ֱ���***************************
#define White          0xFFFF           //��ʾ��ɫ����
#define Black          0x0000
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0

uint Device_code;      //TFT����IC�ͺţ��°�2.8��ΪILI9320
extern void delayms(unsigned int count);

uchar key = 0x01;
static uchar Flag=0;                    //��־����
uint TEMP=0;                            //ADCֵ�ݴ���� 
static uint Precent=0;                  //����Ϊ��̬����Ҫ��Ϊ�˱�����һ�β���ֵ
uint count;
double yback=95;                      //���ڴ��ǰһ�εĵ�ѹ��ʾ���Y����
//**************�����ⲿ�����ͱ���********************
extern void delayms(uint count);

/********************************************************************
			��ʼ��IO���ӳ���
********************************************************************/
void Port_Init()
{
  P4SEL = 0x00;
  P4DIR = 0xFF;                                     //TFT���ݿ�
  P5SEL = 0x00;
  P5DIR|= BIT0 + BIT1 + BIT3 + BIT5 + BIT6 + BIT7;  //TFT��ʾ������
  
  P3SEL=0x00;                               //����IO��Ϊ��ͨI/Oģʽ
  P3DIR = 0xff;                             //����IO�ڷ���Ϊ���
  P3OUT = 0x00;                             //��ʼ����Ϊ00,LED������IO��
   P6DIR |= (1<<0);
   P1IE |= 0X03;
  P1IES |= 0X03;
  P1IFG &= 0X00;
}
//*************************************************************************
//	ADC��ʼ��������������ADC��ؼĴ���
//*************************************************************************
void ADC_Init()
{
  P6SEL|=0x01;                                    //ѡ��ADCͨ�������ö�ӦIO�ڵĹ���ģʽ
  ADC12CTL0|= ADC12ON + SHT0_2 + REF2_5V + REFON; //ADC��Դ���ƿ���16��CLK���ڲ���׼2.5V
  ADC12CTL1|= ADC12SSEL1 + ADC12SSEL0;            //SMCLK��ʱ��Դ
  ADC12MCTL0= SREF0 + INCH_0;                     //�ο�����λ��ͨ��ѡ��
  ADC12IE|= 0x01;                                 //�ж�����
  ADC12CTL0|= ENC;                                //ʹ��ת����
}
/********************************************************************
	LED��˸һ�Σ�������Ҫ�鿴����ִ��λ��ʱ������ã����ƶϵ�
********************************************************************/
void LED_Light()
{
    LED8=0x00;                              //����LED
    delay_ms(500);
    LED8=0xff;                              //Ϩ��LED
    delay_ms(500);
}

void  LCD_DisplayTemp(uchar A1,uchar B1,uchar C1, uchar D1)
{
  LCD_PutChar(222,94,A1+0x30,White,Black);
  LCD_PutChar(222,86,B1+0x30,White,Black);
  LCD_PutChar(222,78,'.',White,Black);
  LCD_PutChar(222,70,C1+0x30,White,Black);
  LCD_PutChar(222,62,D1,White,Black);
}


double getValue1(double value) {
  // ȷ������ֵ�����뷶Χ��
  if(value > 31)value = 31;
  if(value < 26) value = 26;
  
  
  // ����ӳ����ֵ
  double mappedValue = 189.0 - ((value - 26) * 30.0);
  
  return mappedValue;
}
double getValue2(uint temp_d) {
  uint value = temp_d / 10.0 * 1.8 + 32;
  // ȷ������ֵ�����뷶Χ��
  if(value > 89)value = 89;
  if(value < 79) value = 79;
  
  
  // ����ӳ����ֵ
  double mappedValue = 189.0 - ((value - 79) * 15.0);
  
  return mappedValue;
}

#pragma vector=ADC_VECTOR
__interrupt void ADC12ISR(void)
{
  uchar j;
  if (key == 0x01) {
    double y = getValue1(temp_value / 10.0); // ������ʾ
    while((ADC12CTL1&0x01)==1);           //���ADCæ����ȴ��������ȡADCת����ֵ
    Flag = 1 ;
    TEMP = ADC12MEM0;                     //��ȡADCת��ֵ
    count++;
    if (count < 235) {
      if (count > 1) {
        GUIline(30 + count - 1, yback, 30 + count, y, Red);
      }
      yback = y;
      GUIpoint(30 + count, y, Red);
      delay_ms(50);
    } else {
      count = 1;
      Show_RGB(31, 265, 39, 189, Black);
    }
  } else {
    double y = getValue2(temp_value);
    while((ADC12CTL1&0x01)==1);           //���ADCæ����ȴ��������ȡADCת����ֵ
    Flag = 1 ;
    TEMP = ADC12MEM0;                     //��ȡADCת��ֵ
    count++;
    if (count < 235) {
      if (count > 1) {
        GUIline(30 + count - 1, yback, 30 + count, y, Red);
      }
      yback = y;
      GUIpoint(30 + count, y, Red);
      delay_ms(50);
    } else {
      count = 1;
      Show_RGB(31, 265, 39, 189, Black);
    }
  }
}

#pragma vector = PORT1_VECTOR//�˿�1
__interrupt void PORT1_ISR(void)
{
  delay_ms(20);
    if((P1IN & 0X01)==0)
      {
        while((P1IN & 0X01)==0);
        delay_ms(20);
        key = 0x01; 
      }
      if((P1IN & 0X02)==0)
      {
        while((P1IN & 0X02)==0);
        delay_ms(20);
        key = 0x02;
      }
    P1IFG = 0;
}
/********************************************************************
                      ������
********************************************************************/
main()
{
  uint i,y,j;

  WDT_Init();                        //���Ź�����
  Clock_Init();                      //ϵͳʱ������
  Port_Init();                       //ϵͳ��ʼ��������IO������
  LED_Light();                       //LED����һ�Σ���������˵����Ƭ����ʼ���ɹ�   
  Device_code=0x9320;                //TFT����IC�ͺ�
  TFT_Initial();                     //��ʼ��LCD	
  while(1)
  {                                    //�ñ���ɫ����
  CLR_Screen(Black); 
  CLR_Screen(Black); 
  GUICoordinate1(30,190,280,180,2,Yellow,Black);   //���������ᣬ������Ҫ��ȫ����Ե���ƣ���������ش�ϸ����
  LCD_PutString(70,222,"The Temperature is: ",White,Black);     //��ʾԭ��0
  DS18B20_Reset();
  count = 1;                          //����ֵ��ʼ��Ϊ0
  ADC_Init();                         //��ʼ��ADC����
  _EINT();                            //ʹ���ж�
  Flag=1;                             //��־λ����1
    while(1)
    {
      ds1820_start();		          
      ds1820_read_temp();
      delay_ms(1000);
      while(Flag == 1) {
    ds1820_start();		          //����һ��ת��
    ds1820_read_temp();		          //��ȡ�¶���ֵ
    _EINT();
    //data_do(temp_value);                  //�������ݣ��õ�Ҫ��ʾ��ֵ
    if(key == 0x01){
        GUICoordinate1(30,190,280,180,2,Yellow,Black);  
        data_do1(temp_value);
    }
    else {
       GUICoordinate2(30,190,280,180,2,Yellow,Black);
        data_do2(temp_value);
    }
    LED8 &= ~(1<<0);                      //D1����˸��ʾ���ڲ����¶�
    for(j=0;j<30;j++)
    {	    
      if(key == 0x01)
     LCD_DisplayTemp(A1,A2,A3,'C');           //�����¶���ʾ�����������������ʾ
      else 
       LCD_DisplayTemp(A1,A2,A3,'F');   
     LED8 |= 1<<0;                        //LED����˸
    }
    ADC12CTL0 |= ADC12SC;           //����ת��
      ADC12CTL0 &= ~ADC12SC;          //����
      Flag=0;                         //�����־λ
      }
   }
   
  }
}