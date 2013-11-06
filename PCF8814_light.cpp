//***************************************************************************
//  File........: PCF8814.h
//  Author(s)...: Chiper
//  Porting.....: Igorok107
//  URL(s)......: http://digitalchip.ru
//  Description.: ������� LCD-����������� �� Nokia1100 � ������������ ���������
//  Data........: 02.11.13
//  Version.....: 2.1.0
//***************************************************************************
#include "PCF8814_light.h"
#include "PCF8814_font.h" // ���������� ����� (����� �������� � ����������� ������)

// ����� � ������� ��������� ������� � ��������� Arduino
volatile byte LCD_SCLK, LCD_SDA, LCD_CS, LCD_RST;

PCF8814::PCF8814(byte _LCD_SCLK, byte _LCD_SDA, byte _LCD_CS, byte _LCD_RST)
{
	// �������������� ���� �� ����� ��� ������ � LCD-������������
	pinMode(_LCD_SCLK,OUTPUT);
	pinMode(_LCD_SDA,OUTPUT);
	pinMode(_LCD_CS,OUTPUT);
	pinMode(_LCD_RST,OUTPUT);
  
	LCD_SCLK	=	_LCD_SCLK;
	LCD_SDA		=	_LCD_SDA;
	LCD_CS		=	_LCD_CS;
	LCD_RST		=	_LCD_RST;
}

//******************************************************************************
// �������� ����� (������� ��� ������) �� LCD-����������
//  mode: CMD_LCD_MODE - �������� �������
//		  DATA_LCD_MODE - �������� ������
//  c: �������� ������������� �����
void PCF8814::SendByte(char mode,unsigned char c)
{
  CS_LCD_RESET;
  SCLK_LCD_RESET;
  if (mode)
  {	
    SDA_LCD_SET;
  }
  else SDA_LCD_RESET;

  SCLK_LCD_SET;

  byte i;
  for(i=0;i<8;i++)
  {
    delayMicroseconds(LCD_MIN_DELAY/2);
    SCLK_LCD_RESET;

    if(c & 0x80) SDA_LCD_SET;
    else	     SDA_LCD_RESET;
    delayMicroseconds(LCD_MIN_DELAY/2);
    SCLK_LCD_SET;
    c <<= 1;	
  }	
  CS_LCD_SET;
  SCLK_LCD_RESET;
  SDA_LCD_RESET;
}

//******************************************************************************
// ������������� �����������
void PCF8814::Init(void)
{
  SCLK_LCD_RESET;
  SDA_LCD_RESET;
  CS_LCD_RESET;
  RST_LCD_RESET;
  delay(10);            // ������� �� ����� 5�� ��� ��������� ����������(����� 5 �� ����� ����������)
  RST_LCD_SET;

  SendByte(CMD_LCD_MODE,0xE2); // *** SOFTWARE RESET 
  SendByte(CMD_LCD_MODE,0x3A); // *** Use internal oscillator
  SendByte(CMD_LCD_MODE,0xEF); // *** FRAME FREQUENCY:
  SendByte(CMD_LCD_MODE,0x04); // *** 80Hz
  SendByte(CMD_LCD_MODE,0xD0); // *** 1:65 divider
  SendByte(CMD_LCD_MODE,0x20); // ������ � ������� Vop 
  SendByte(CMD_LCD_MODE,0x8D); // // ���������� ������������� [0x80-0x9F]
  SendByte(CMD_LCD_MODE,0xA4); // all on/normal display
  SendByte(CMD_LCD_MODE,0x2F); // Power control set(charge pump on/off)
  SendByte(CMD_LCD_MODE,0x40); // set start row address = 0
  SendByte(CMD_LCD_MODE,0xB0); // ���������� Y-����� = 0
  SendByte(CMD_LCD_MODE,0x10); // ���������� X-�����, ������� 3 ����
  SendByte(CMD_LCD_MODE,0x00);  // ���������� X-�����, ������� 4 ����

  //SendByte(CMD_LCD_MODE,0xC8); // mirror Y axis (about X axis)
  SendByte(CMD_LCD_MODE,0xA1); // ������������� ����� �� �����������

  SendByte(CMD_LCD_MODE,0xAC); // set initial row (R0) of the display
  SendByte(CMD_LCD_MODE,0x07);
  SendByte(CMD_LCD_MODE,0xAF); // ����� ���/����

  Clear(); // clear LCD
}

//******************************************************************************
// ������� ������
void PCF8814::Clear(void)
{
  SendByte(CMD_LCD_MODE,0x40); // Y = 0
  SendByte(CMD_LCD_MODE,0xB0);
  SendByte(CMD_LCD_MODE,0x10); // X = 0
  SendByte(CMD_LCD_MODE,0x00);

  SendByte(CMD_LCD_MODE,0xAE); // disable display;
  unsigned int i;
  for(i=0;i<864;i++) SendByte(DATA_LCD_MODE,0x00);
  SendByte(CMD_LCD_MODE,0xAF); // enable display;
}

//******************************************************************************
// �������������� LCD-������ �� ��� x � y ��������������.
//  ON: ��������
//  OFF: �� ���������
void PCF8814::Mirror(byte x, byte y)
{
	SendByte(CMD_LCD_MODE,0xA0 | x);
	SendByte(CMD_LCD_MODE,0xC0 | y<<3);
}

//******************************************************************************
// ������������� LCD-������.
//  �: ��������� �������� �� 0 �� 31.
void PCF8814::Contrast(byte c)
{
	if (c >= 0x20) c = 0x1F;
		SendByte(CMD_LCD_MODE,0x20);
		SendByte(CMD_LCD_MODE,0x80+c); // ���������� ������������� [0x80-0x9F]
}

//******************************************************************************
// ����� ������� �� LCD-����� � ������� �����
//  c: ��� �������
void PCF8814::Putc(unsigned char c)
{
  if (c < 208){
    byte i;
    for ( i = 0; i < 5; i++ )
      SendByte(DATA_LCD_MODE,pgm_read_byte(&(lcd_Font[c-32][i])));

    SendByte(DATA_LCD_MODE,0x00); // ����� ����� ��������� �� ����������� � 1 �������
  }
}

//******************************************************************************
// ����� �������� ������� �� LCD-����� � ������� �����
//  c: ��� �������
void PCF8814::PutcWide(unsigned char c)
{
  if (c < 208){ 	// ������� ������ ���� � ��������� UTF8

    byte i;
    for ( i = 0; i < 5; i++ )
    {
      unsigned char glyph = pgm_read_byte(&(lcd_Font[c-32][i]));
      SendByte(DATA_LCD_MODE,glyph);
      SendByte(DATA_LCD_MODE,glyph);
    }

    SendByte(DATA_LCD_MODE,0x00); // ����� ����� ��������� �� ����������� � 1 �������
    //	SendByte(DATA_LCD_MODE,0x00); // ����� ������� ��� �����
  }
}

//******************************************************************************
// ����� ������ �������� �� LCD-����� � ������� �����. ���� ������ �������
// �� ����� � ������� ������, �� ������� ����������� �� ��������� ������.
//  message: ��������� �� ������ ��������. 0x00 - ������� ����� ������.
void PCF8814::Print(const char * message)
{
  while (*message) Putc(*message++); // ����� ������ ��������� �����
}

//******************************************************************************
// ����� ������ �������� ������� ������ �� LCD-����� � ������� �����
// �� ����������� ������. ���� ������ ������� �� ����� � ������� ������, �� �������
// ����������� �� ��������� ������.
//  message: ��������� �� ������ �������� � ����������� ������. 0x00 - ������� ����� ������.
void PCF8814::PrintWide(char * message)
{
  while (*message) PutcWide(*message++);  // ����� ������ ��������� �����
}

//******************************************************************************
// ����� ������ �������� �� LCD-����� NOKIA 1100 � ������� ����� �� ����������� ������.
// ���� ������ ������� �� ����� � ������� ������, �� ������� ����������� �� ��������� ������.
//  message: ��������� �� ������ �������� � ����������� ������. 0x00 - ������� ����� ������.
void PCF8814::PrintF(char * message)
{
  byte data;
  while (data=pgm_read_byte(message), data)
  { 
    Putc(data);
    message++;
  }
}

//******************************************************************************
// ������������� ������ � ����������� ���������. ������ ���������� � ������� 
// ����� ����. �� ����������� 16 ���������, �� ��������� - 8
//  x: 0..15
//  y: 0..7    
void PCF8814::GotoXY(byte x,byte y)
{
  x=x*6;	// ��������� �� ���������� � ����������� � ����������� � ��������

  SendByte(CMD_LCD_MODE,(0xB0|(y&0x0F)));      // ��������� ������ �� Y: 0100 yyyy         
  SendByte(CMD_LCD_MODE,(0x00|(x&0x0F)));      // ��������� ������ �� X: 0000 xxxx - ���� (x3 x2 x1 x0)
  SendByte(CMD_LCD_MODE,(0x10|((x>>4)&0x07))); // ��������� ������ �� X: 0010 0xxx - ���� (x6 x5 x4)
}
