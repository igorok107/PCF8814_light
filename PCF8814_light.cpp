//***************************************************************************
//  File........: PCF8814.h
//  Author(s)...: Chiper
//  Porting.....: Igorok107
//  URL(s)......: http://digitalchip.ru
//  Description.: Драйвер LCD-контроллера от Nokia1100 без графических функций
//  Data........: 07.11.13
//  Version.....: 2.2.0
//***************************************************************************
#include "PCF8814_light.h"
#include "PCF8814_font.h" // Подключаем шрифт (будет размещен в программной памяти)

// Порты к которым подключен дисплей в нумерации Arduino
volatile uint8_t LCD_SCLK, LCD_SDA, LCD_CS, LCD_RST;

#ifndef SOFT_SPI
	const uint8_t SPI_DELAY[] = {1,2,5,10,1,1,2}; //Задержка для первого бита
#endif

PCF8814::PCF8814(uint8_t _LCD_SCLK, uint8_t _LCD_SDA, uint8_t _LCD_CS, uint8_t _LCD_RST)
{
	// Инициализируем пины на вывод для работы с LCD-контроллером
	pinMode(_LCD_RST,OUTPUT);
	LCD_RST		=	_LCD_RST;	
#ifdef SOFT_SPI
	pinMode(_LCD_SCLK,OUTPUT);
	pinMode(_LCD_SDA,OUTPUT);
	pinMode(_LCD_CS,OUTPUT);	
	LCD_SCLK	=	_LCD_SCLK;
	LCD_SDA		=	_LCD_SDA;
	LCD_CS		=	_LCD_CS;	
#else // Инициализация аппаратного SPI
	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);	
	pinMode(SS, OUTPUT);
	SPCR = 0x10; //Определяем как MASTER.
	SPCR = (SPCR & ~0x03) | (SPI_CLOCK_DIV & 0x03); //Делитель частоты
	SPSR = (SPSR & ~1) | ((SPI_CLOCK_DIV >> 2) & 1);
#endif
	CS_LCD_SET;
}

uint8_t PCF8814::SPI_write(uint8_t cData) {
		SPDR = cData;
        while(!(SPSR & _BV(SPIF)));
        return SPDR;    
}

//******************************************************************************
// Передача байта (команды или данных) на LCD-контроллер
//  mode: CMD_LCD_MODE - передаем команду
//		  DATA_LCD_MODE - передаем данные
//  c: значение передаваемого байта
void PCF8814::SendByte(char mode,unsigned char c)
{
	#ifndef SOFT_SPI
		SPCR &= ~_BV(SPIE);
	#endif
	CS_LCD_RESET;
	(mode) ? SDA_LCD_SET : SDA_LCD_RESET;
	
	SCLK_LCD_SET;
	#ifndef SOFT_SPI		
		delayMicroseconds(SPI_DELAY[SPI_CLOCK_DIV]);
	#else
		delayMicroseconds(LCD_MIN_DELAY/2);
	#endif
	SCLK_LCD_RESET;
		
	#ifdef SOFT_SPI
		uint8_t i;
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
		SCLK_LCD_RESET;
	#else
		SPCR |= _BV(SPE);
		SPI_write(c);
		SPCR &= ~_BV(SPE);	
	#endif
	CS_LCD_SET;
	SDA_LCD_RESET;
}

//******************************************************************************
// Инициализация контроллера
void PCF8814::Init(void)
{
	SCLK_LCD_RESET;
	SDA_LCD_RESET;
	CS_LCD_RESET;
	RST_LCD_RESET;
	delay(10);            // выжидем не менее 5мс для установки генератора(менее 5 мс может неработать)
	RST_LCD_SET;
	CS_LCD_SET;

	SendByte(CMD_LCD_MODE,0xE2); // *** SOFTWARE RESET
	SendByte(CMD_LCD_MODE,0x3A); // *** Use internal oscillator
	SendByte(CMD_LCD_MODE,0xEF); // *** FRAME FREQUENCY:
	SendByte(CMD_LCD_MODE,0x04); // *** 80Hz
	SendByte(CMD_LCD_MODE,0xD0); // *** 1:65 divider
	SendByte(CMD_LCD_MODE,0xEB); // Включить температурную компенсацию
	SendByte(CMD_LCD_MODE,0x20); // Запись в регистр Vop
	SendByte(CMD_LCD_MODE,0x85); // Определяет контрастность
	SendByte(CMD_LCD_MODE,0xA4); // all on/normal display
	SendByte(CMD_LCD_MODE,0x2F); // Power control set(charge pump on/off)
	SendByte(CMD_LCD_MODE,0x40); // set start row address = 0
	SendByte(CMD_LCD_MODE,0xB0); // установить Y-адрес = 0
	SendByte(CMD_LCD_MODE,0x10); // установить X-адрес, старшие 3 бита
	SendByte(CMD_LCD_MODE,0x00);  // установить X-адрес, младшие 4 бита

	//SendByte(CMD_LCD_MODE,0xC8); // mirror Y axis (about X axis)
	SendByte(CMD_LCD_MODE,0xA1); // Инвертировать экран по горизонтали

	SendByte(CMD_LCD_MODE,0xAC); // set initial row (R0) of the display
	SendByte(CMD_LCD_MODE,0x07);
	SendByte(CMD_LCD_MODE,0xAF); // экран вкл/выкл

	Clear(); // clear LCD
}

//******************************************************************************
// Очистка экрана
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
// Зеркалирование LCD-экрана по оси x и y соответственно.
//  ON: Отразить
//  OFF: Не отражатьж
void PCF8814::Mirror(uint8_t x, uint8_t y)
{
	SendByte(CMD_LCD_MODE,0xA0 | x);
	SendByte(CMD_LCD_MODE,0xC0 | y<<3);
}

//******************************************************************************
// Контрасиность LCD-экрана.
//  с: принимает значения от 0 до 31.
void PCF8814::Contrast(uint8_t c)
{
	if (c >= 0x20) c = 0x1F;
	SendByte(CMD_LCD_MODE,0x20);
	SendByte(CMD_LCD_MODE,0x80+c); // Определяет контрастность [0x80-0x9F]
}

//******************************************************************************
// Вывод символа на LCD-экран в текущее место
//  c: код символа
void PCF8814::Putc(unsigned char c)
{
	if (c < 208){
		uint8_t i;
		for ( i = 0; i < 5; i++ )
		SendByte(DATA_LCD_MODE,pgm_read_byte(&(lcd_Font[c-32][i])));

		SendByte(DATA_LCD_MODE,0x00); // Зазор между символами по горизонтали в 1 пиксель
	}
}

//******************************************************************************
// Вывод широкого символа на LCD-экран в текущее место
//  c: код символа
void PCF8814::PutcWide(unsigned char c)
{
  if (c < 208){ 	// Урезаем первый байт в кодировке UTF8

    uint8_t i;
    for ( i = 0; i < 5; i++ )
    {
      unsigned char glyph = pgm_read_byte(&(lcd_Font[c-32][i]));
      SendByte(DATA_LCD_MODE,glyph);
      SendByte(DATA_LCD_MODE,glyph);
    }

    SendByte(DATA_LCD_MODE,0x00); // Зазор между символами по горизонтали в 1 пиксель
    //	SendByte(DATA_LCD_MODE,0x00); // Можно сделать две линии
  }
}

//******************************************************************************
// Вывод строки символов на LCD-экран в текущее место. Если строка выходит
// за экран в текущей строке, то остаток переносится на следующую строку.
//  message: указатель на строку символов. 0x00 - признак конца строки.
void PCF8814::Print(const char * message)
{
  while (*message) Putc(*message++); // Конец строки обозначен нулем
}

//******************************************************************************
// Вывод строки символов двойной ширины на LCD-экран в текущее место
// из оперативной памяти. Если строка выходит за экран в текущей строке, то остаток
// переносится на следующую строку.
//  message: указатель на строку символов в оперативной памяти. 0x00 - признак конца строки.
void PCF8814::PrintWide(char * message)
{
  while (*message) PutcWide(*message++);  // Конец строки обозначен нулем
}

//******************************************************************************
// Вывод строки символов на LCD-экран NOKIA 1100 в текущее место из программной памяти.
// Если строка выходит за экран в текущей строке, то остаток переносится на следующую строку.
//  message: указатель на строку символов в программной памяти. 0x00 - признак конца строки.
void PCF8814::PrintF(char * message)
{
  uint8_t data;
  while (data=pgm_read_byte(message), data)
  { 
    Putc(data);
    message++;
  }
}

//******************************************************************************
// Устанавливает курсор в необходимое положение. Отсчет начинается в верхнем 
// левом углу. По горизонтали 16 знакомест, по вертикали - 8
//  x: 0..15
//  y: 0..7    
void PCF8814::GotoXY(uint8_t x,uint8_t y)
{
  x=x*6;	// Переходим от координаты в знакоместах к координатам в пикселях

  SendByte(CMD_LCD_MODE,(0xB0|(y&0x0F)));      // установка адреса по Y: 0100 yyyy         
  SendByte(CMD_LCD_MODE,(0x00|(x&0x0F)));      // установка адреса по X: 0000 xxxx - биты (x3 x2 x1 x0)
  SendByte(CMD_LCD_MODE,(0x10|((x>>4)&0x07))); // установка адреса по X: 0010 0xxx - биты (x6 x5 x4)
}
