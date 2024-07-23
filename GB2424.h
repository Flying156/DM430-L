// ------------------  汉字字模的数据结构定义 ------------------------ //
struct  typFNT_GB24                           // 汉字字模数据结构 
{
       unsigned char  Index[2];               // 汉字内码索引	
       unsigned char   Msk[72];               // 点阵码数据 
};

/////////////////////////////////////////////////////////////////////////
// 汉字字模表                                                          //
// 汉字库: 宋体16.dot,横向取模左高位,数据排列:从左到右从上到下         //
/////////////////////////////////////////////////////////////////////////
const  struct  typFNT_GB24 codeGB_24[] =          // 数据表 
{
/*--  文字:  欣  --*/
/*--  隶书18;  此字体下对应的点阵为：宽x高=24x24   --*/
"欣",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x0E,0x00,0x18,0xEE,0x00,0x1F,0xEE,0x30,0x0C,0x0F,0xF0,0x0C,0x0C,
0x70,0x0F,0xEC,0x80,0x0F,0x9E,0x00,0x0D,0xA6,0x00,0x0D,0xC7,0x80,0x19,0x8D,0xC0,
0x39,0x8C,0xF0,0x71,0x98,0x7C,0x70,0x70,0x3E,0x01,0xE0,0x1C,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

/*--  文字:  世  --*/
/*--  隶书18;  此字体下对应的点阵为：宽x高=24x24   --*/
"世",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x21,0x80,0x04,0x31,0x80,0x06,0x31,0x80,0x06,0x31,0x80,0x3F,0xFF,
0xF8,0x7F,0xFF,0xFE,0x46,0x31,0x9E,0x06,0x31,0x80,0x06,0x33,0x80,0x06,0x3F,0x80,
0x07,0x00,0x00,0x07,0x00,0x00,0x07,0xFF,0xF0,0x07,0xFF,0xF0,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

/*--  文字:  纪  --*/
/*--  隶书18;  此字体下对应的点阵为：宽x高=24x24   --*/
"纪",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0x00,0x00,0x03,0x3F,0xE0,0x06,0x38,0xE0,0x1C,0xC0,0x60,0x31,0x80,
0x60,0x3F,0x3F,0xE0,0x06,0x30,0x60,0x08,0x50,0x00,0x3F,0x90,0x00,0x1C,0x18,0x00,
0x00,0xD8,0x02,0x3F,0x9E,0x1E,0x7E,0x0F,0xFE,0x20,0x03,0xFC,0x00,0x00,0x78,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

/*--  文字:  电  --*/
/*--  隶书18;  此字体下对应的点阵为：宽x高=24x24   --*/
"电",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x60,0x00,0x00,0x60,0x00,0x3F,0xFF,0xC0,0x3F,0xF1,0xC0,0x38,0x60,
0xC0,0x1F,0xFF,0xC0,0x18,0x60,0xC0,0x18,0x60,0xC0,0x1F,0xFF,0xC0,0x1F,0xE3,0x80,
0x00,0x70,0x00,0x00,0x3C,0x0E,0x00,0x3F,0xFC,0x00,0x0F,0xFC,0x00,0x01,0xF0,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

/*--  文字:  子  --*/
/*--  隶书18;  此字体下对应的点阵为：宽x高=24x24   --*/
"子",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0xFF,0x80,0x03,0xFF,0xC0,0x07,0x03,0xC0,0x00,0x1F,0x00,0x00,0x1C,
0x00,0x3F,0xFF,0xFE,0x7F,0xFF,0xFE,0x78,0x0C,0x3E,0x00,0x0C,0x00,0x00,0x0E,0x00,
0x00,0x0E,0x00,0x00,0x1C,0x00,0x06,0x3C,0x00,0x01,0xF8,0x00,0x00,0x60,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

/*--  文字:  科  --*/
/*--  隶书18;  此字体下对应的点阵为：宽x高=24x24   --*/
"科",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xC0,0xE0,0x1F,0xC0,0x60,0x01,0x8E,0x60,0x11,0xC3,0x60,0x3F,0xE0,
0x60,0x03,0x9E,0x60,0x07,0x80,0x60,0x0F,0xE0,0x60,0x1D,0x9F,0xFE,0x39,0x98,0x64,
0xF1,0x80,0x60,0x01,0x80,0x60,0x03,0x80,0x60,0x03,0x80,0x60,0x01,0x00,0x40,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

/*--  文字:  技  --*/
/*--  隶书18;  此字体下对应的点阵为：宽x高=24x24   --*/
"技",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x03,0x03,0x00,0x03,0x03,0x00,0x03,0x03,0x00,0x1F,0xDF,0xF0,0x33,0x23,
0x00,0x03,0x0F,0xC0,0x03,0xD0,0x40,0x1F,0x1C,0x40,0x13,0x0E,0xC0,0x03,0x03,0x80,
0x03,0x03,0xE0,0x03,0x3E,0x7C,0x7F,0xF8,0x3E,0x3E,0x00,0x1C,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

/*--  文字:  欢  --*/
/*--  隶书18;  此字体下对应的点阵为：宽x高=24x24   --*/
"欢",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x04,0x00,0x00,0x06,0x00,0x1F,0xCC,0x00,0x1F,0xCF,0xF8,0x00,0xCC,
0x38,0x0C,0x9E,0x30,0x0F,0xA6,0x60,0x03,0x07,0x80,0x07,0x87,0x80,0x1C,0xCC,0xC0,
0x78,0x0C,0xE0,0x00,0x38,0x7C,0x00,0xF0,0x3E,0x03,0xC0,0x1C,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

/*--  文字:  迎  --*/
/*--  隶书18;  此字体下对应的点阵为：宽x高=24x24   --*/
"迎",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x04,0x00,0x0C,0x7C,0x00,0x0E,0x63,0xF0,0x03,0x63,0x30,0x00,0x63,
0x30,0x1F,0x63,0x30,0x02,0x63,0x20,0x02,0x7F,0xC0,0x03,0x63,0x00,0x02,0x03,0x00,
0x7F,0xF3,0x00,0x78,0x7F,0xFE,0x00,0x07,0xFE,0x00,0x01,0xFC,0x00,0x00,0x30,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

/*--  文字:  您  --*/
/*--  隶书18;  此字体下对应的点阵为：宽x高=24x24   --*/
"您",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xD8,0x00,0x0F,0xBF,0xE0,0x3F,0x33,0x20,0x1B,0x53,0x20,0x03,0x33,
0xC0,0x03,0x63,0x70,0x03,0x97,0x10,0x02,0x0E,0x00,0x03,0x18,0x80,0x1B,0xD9,0x80,
0x30,0xF2,0x00,0x30,0x1F,0x02,0x70,0x07,0xFE,0x20,0x01,0xFC,0x00,0x00,0x78,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

};