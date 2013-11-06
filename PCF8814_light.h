//***************************************************************************
//  File........: PCF8814.h
//  Author(s)...: Chiper
//  Porting.....: Igorok107
//  URL(s)......: http://digitalchip.ru
//  Description.: ������� LCD-����������� �� Nokia1100 � ������������ ���������
//  Data........: 02.11.13
//  Version.....: 2.1.0
//***************************************************************************
#ifndef PCF8814_H
#define PCF8814_H

#include "Arduino.h"
//******************************************************************************
//******************************************************************************
// ��������� ������ ����� ��������. 
#define FULL_CHARSET 

// *****!!!!! ����������� ��������, ��� ������� �������� LCD-����������
#define LCD_MIN_DELAY	4
// *****!!!!! ����������� ���������������� ��� ���������� ����������

// �������, �����������, ��������� ����������
#define SCLK_LCD_SET    digitalWrite(LCD_SCLK,HIGH)
#define SDA_LCD_SET     digitalWrite(LCD_SDA,HIGH)
#define CS_LCD_SET      digitalWrite(LCD_CS,HIGH)
#define RST_LCD_SET     digitalWrite(LCD_RST,HIGH)
#define SCLK_LCD_RESET  digitalWrite(LCD_SCLK,LOW)
#define SDA_LCD_RESET   digitalWrite(LCD_SDA,LOW)
#define CS_LCD_RESET    digitalWrite(LCD_CS,LOW)
#define RST_LCD_RESET   digitalWrite(LCD_RST,LOW)

// ������� ��� ������ � ������
#define ClearBit(reg, bit)       reg &= (~(1<<(bit)))
#define SetBit(reg, bit)         reg |= (1<<(bit))	
#define InvBit(reg, bit)         reg ^= 1<<bit	

#define CMD_LCD_MODE	0
#define DATA_LCD_MODE	1

#define ON	1
#define OFF	0

// ���������� ������� � ��������
#define LCD_X_RES	96		// ���������� �� �����������
#define LCD_Y_RES	68		// ���������� �� ���������

//******************************************************************************
class PCF8814 {
	public:
		PCF8814(byte _LCD_SCLK, byte _LCD_SDA, byte _LCD_CS, byte _LCD_RST);
		void Init(void);
		void Clear(void);
		void Mirror(byte x, byte y);
		void Contrast(byte c = 0x0D);
		void SendByte(char mode,unsigned char c);
		void Putc(unsigned char c);
		void PutcWide(unsigned char c);
		void Print(const char * message);
		void PrintF(char * message);
		void PrintWide(char * message);
		void GotoXY(byte x = 0,byte y = 0);
};
#endif /* PCF8814_H */