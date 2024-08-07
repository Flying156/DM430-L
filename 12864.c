//*************************************************************************
//			初始化IO口子程序
//*************************************************************************
void Port_init()
{

  P4SEL = 0x00;
  P4DIR = 0xFF;
  P5SEL = 0x00;
  P5DIR|= BIT0 + BIT1 + BIT5 + BIT6 + BIT7;
  PSB_SET;		  //液晶并口方式
  RST_SET;
}

//***********************************************************************
//	显示屏命令写入函数
//***********************************************************************
void LCD_write_com(unsigned char com) 
{	
  RS_CLR;
  RW_CLR;
  EN_SET;
  DataPort = com;
  delay_ms(5);
  EN_CLR;
}

//***********************************************************************
//	显示屏数据写入函数
//***********************************************************************
void LCD_write_data(unsigned char data) 
{
  RS_SET;
  RW_CLR;
  EN_SET;
  DataPort = data;
  delay_ms(5);
  EN_CLR;
}

//***********************************************************************
//	显示屏清空显示
//***********************************************************************

void LCD_clear(void) 
{
  LCD_write_com(0x01);
  delay_ms(5);
}

//***********************************************************************
//函数名称：DisplayCgrom(uchar hz)显示CGROM里的汉字
//***********************************************************************
void DisplayCgrom(uchar addr,uchar *hz)
{
  LCD_write_com(addr);
  delay_ms(5);
  while(*hz != '\0')  
  {
    LCD_write_data(*hz);
    hz++;
    delay_ms(5);
  }

} 

//***********************************************************************
//	显示屏单字符写入函数
//***********************************************************************
void LCD_write_char(unsigned char x,unsigned char y,unsigned char data) 
{
	
    if (y == 0) 
    {
    	LCD_write_com(0x80 + x);        //第一行显示
    }
    if(y == 1) 
    {
    	LCD_write_com(0x90 + x);        //第二行显示
    }
    if (y == 2) 
    {
    	LCD_write_com(0x88 + x);        //第三行显示
    }
    if(y == 3) 
    {
    	LCD_write_com(0x98 + x);        //第四行显示
    }
    delay_ms(1);
    LCD_write_data(data);
    delay_ms(1);
}

//***********************************************************************
//	显示屏字符串写入函数
//***********************************************************************
void LCD_write_str(unsigned char x,unsigned char y,unsigned char *s) 
{
	
    if (y == 0) 
    {
    	LCD_write_com(0x80 + x);        //第一行显示
    }
    if(y == 1) 
    {
    	LCD_write_com(0x90 + x);        //第二行显示
    }
    if (y == 2) 
    {
    	LCD_write_com(0x88 + x);        //第三行显示
    }
    if(y == 3) 
    {
    	LCD_write_com(0x98 + x);        //第四行显示
    }
    delay_ms(2);
    while (*s) 
    {
    	LCD_write_data( *s);
        delay_ms(2);
    	s ++;
    }
}

//***********************************************************************
//	显示屏初始化函数
//***********************************************************************
void LCD_init(void) 
{
  LCD_write_com(FUN_MODE);			//显示模式设置
  delay_ms(5);
  LCD_write_com(FUN_MODE);			//显示模式设置
  delay_ms(5);
  LCD_write_com(CURSE_DIR);			//显示模式设置
  delay_ms(5);
  LCD_write_com(DISPLAY_ON);			//显示开
  delay_ms(5);
  LCD_write_com(CLEAR_SCREEN);			//清屏
  delay_ms(5);
}

//***********************************************************************
//      液晶显示界面初始化
//***********************************************************************
void LCD_Desk(void)
{    
  LCD_clear();
  DisplayCgrom(0x80,"当前温度是：");
  DisplayCgrom(0x8A,"摄氏");
  delay_ms(250);
}

