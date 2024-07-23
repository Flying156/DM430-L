/********************************************************************
//DM430-L型最小系统板控制2.8寸TFT彩屏横屏波形绘制测试程序
//将ADC0（P60口）采集到的电压值显示到坐标轴，绘制成波形曲线
//ADC0连续采样，内部2.5V基准参考，最大2.5V，大于2.5V的输入会是直线
//该程序仅作参考，用户可自行根据该参考做更改优化
//将TFT插入12864接口，触摸控制线需要用杜邦线连接，具体见参考图例
//注意选择液晶的电源，位于电位器附近，可选5V或3.3V，根据液晶电压进行选择，默认设置为5V
//调试环境：EW430 V5.30
//作者：www.avrgcc.com
//时间：2013.09.08
********************************************************************/
#include <msp430x14x.h>
#include "Config.h"                     //配置头文件，与硬件相关的配置在这里更改
#include "GUI.h"
#include "Ascii_8x16.h"                 //8x16大小字符
#include "GB2424.h"                     //24x24像素大小的汉字
#include "Chinese.h"                    //16x16像素大小的汉字
#include "Touch.h"                      //TFT触摸操作头文件
#include "Touch.c"                      //TFT触摸操作初始化及函数
#include "TFT28.h"   
#include "DS18B20.c" //TFT显示头文件
#include "TFT28.c"                      //TFT显示操作初始化及函数
#include "GUI.c"

//******************全局变量***************************
#define White          0xFFFF           //显示颜色代码
#define Black          0x0000
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0

uint Device_code;      //TFT控制IC型号，新版2.8寸为ILI9320
extern void delayms(unsigned int count);

uchar key = 0x01;
static uchar Flag=0;                    //标志变量
uint TEMP=0;                            //ADC值暂存变量 
static uint Precent=0;                  //设置为静态很重要，为了保留上一次采样值
uint count;
double yback=95;                      //用于存放前一次的电压显示点的Y坐标
//**************声明外部函数和变量********************
extern void delayms(uint count);

/********************************************************************
			初始化IO口子程序
********************************************************************/
void Port_Init()
{
  P4SEL = 0x00;
  P4DIR = 0xFF;                                     //TFT数据口
  P5SEL = 0x00;
  P5DIR|= BIT0 + BIT1 + BIT3 + BIT5 + BIT6 + BIT7;  //TFT显示控制线
  
  P3SEL=0x00;                               //设置IO口为普通I/O模式
  P3DIR = 0xff;                             //设置IO口方向为输出
  P3OUT = 0x00;                             //初始设置为00,LED灯所在IO口
   P6DIR |= (1<<0);
   P1IE |= 0X03;
  P1IES |= 0X03;
  P1IFG &= 0X00;
}
//*************************************************************************
//	ADC初始化程序，用于配置ADC相关寄存器
//*************************************************************************
void ADC_Init()
{
  P6SEL|=0x01;                                    //选择ADC通道，设置对应IO口的功能模式
  ADC12CTL0|= ADC12ON + SHT0_2 + REF2_5V + REFON; //ADC电源控制开，16个CLK，内部基准2.5V
  ADC12CTL1|= ADC12SSEL1 + ADC12SSEL0;            //SMCLK做时钟源
  ADC12MCTL0= SREF0 + INCH_0;                     //参考控制位及通道选择
  ADC12IE|= 0x01;                                 //中断允许
  ADC12CTL0|= ENC;                                //使能转换器
}
/********************************************************************
	LED闪烁一次，可在需要查看程序执行位置时灵活设置，类似断点
********************************************************************/
void LED_Light()
{
    LED8=0x00;                              //点亮LED
    delay_ms(500);
    LED8=0xff;                              //熄灭LED
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
  // 确保输入值在输入范围内
  if(value > 31)value = 31;
  if(value < 26) value = 26;
  
  
  // 计算映射后的值
  double mappedValue = 189.0 - ((value - 26) * 30.0);
  
  return mappedValue;
}
double getValue2(uint temp_d) {
  uint value = temp_d / 10.0 * 1.8 + 32;
  // 确保输入值在输入范围内
  if(value > 89)value = 89;
  if(value < 79) value = 79;
  
  
  // 计算映射后的值
  double mappedValue = 189.0 - ((value - 79) * 15.0);
  
  return mappedValue;
}

#pragma vector=ADC_VECTOR
__interrupt void ADC12ISR(void)
{
  uchar j;
  if (key == 0x01) {
    double y = getValue1(temp_value / 10.0); // 用于显示
    while((ADC12CTL1&0x01)==1);           //如果ADC忙，则等待，否则读取ADC转换数值
    Flag = 1 ;
    TEMP = ADC12MEM0;                     //读取ADC转换值
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
    while((ADC12CTL1&0x01)==1);           //如果ADC忙，则等待，否则读取ADC转换数值
    Flag = 1 ;
    TEMP = ADC12MEM0;                     //读取ADC转换值
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

#pragma vector = PORT1_VECTOR//端口1
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
                      主函数
********************************************************************/
main()
{
  uint i,y,j;

  WDT_Init();                        //看门狗设置
  Clock_Init();                      //系统时钟设置
  Port_Init();                       //系统初始化，设置IO口属性
  LED_Light();                       //LED点亮一次，如有现象，说明单片机初始化成功   
  Device_code=0x9320;                //TFT控制IC型号
  TFT_Initial();                     //初始化LCD	
  while(1)
  {                                    //用背景色清屏
  CLR_Screen(Black); 
  CLR_Screen(Black); 
  GUICoordinate1(30,190,280,180,2,Yellow,Black);   //绘制坐标轴，尽量不要完全靠边缘绘制，如果单像素粗细可以
  LCD_PutString(70,222,"The Temperature is: ",White,Black);     //显示原点0
  DS18B20_Reset();
  count = 1;                          //计数值初始化为0
  ADC_Init();                         //初始化ADC配置
  _EINT();                            //使能中断
  Flag=1;                             //标志位先置1
    while(1)
    {
      ds1820_start();		          
      ds1820_read_temp();
      delay_ms(1000);
      while(Flag == 1) {
    ds1820_start();		          //启动一次转换
    ds1820_read_temp();		          //读取温度数值
    _EINT();
    //data_do(temp_value);                  //处理数据，得到要显示的值
    if(key == 0x01){
        GUICoordinate1(30,190,280,180,2,Yellow,Black);  
        data_do1(temp_value);
    }
    else {
       GUICoordinate2(30,190,280,180,2,Yellow,Black);
        data_do2(temp_value);
    }
    LED8 &= ~(1<<0);                      //D1灯闪烁表示正在测试温度
    for(j=0;j<30;j++)
    {	    
      if(key == 0x01)
     LCD_DisplayTemp(A1,A2,A3,'C');           //调用温度显示函数，分离后依次显示
      else 
       LCD_DisplayTemp(A1,A2,A3,'F');   
     LED8 |= 1<<0;                        //LED灯闪烁
    }
    ADC12CTL0 |= ADC12SC;           //开启转换
      ADC12CTL0 &= ~ADC12SC;          //清零
      Flag=0;                         //清零标志位
      }
   }
   
  }
}