	// *************************************************************************************
	// LCD Include File for Philips PCF8833 STN RGB- 132x132x3 Driver
	//
	// Taken from Philips data sheet Feb 14, 2003
	// *************************************************************************************
	// Philips PCF8833 LCD controller command codes


#ifndef Lcd_h
#define Lcd_h

#include "stm32f1xx_hal.h"

    #define LCD_RESET_PIN GPIO_PIN_6
    #define LCD_CS_PIN GPIO_PIN_7


	#define LCD_TX_QUEUE_LEN 3

	#define NOP 0x00 // nop
	#define SWRESET 0x01 // software reset
	#define BSTROFF 0x02 // booster voltage OFF
	#define BSTRON 0x03 // booster voltage ON
	#define RDDIDIF 0x04 // read display identification
	#define RDDST 0x09 // read display status
	#define SLEEPIN 0x10 // sleep in
	#define SLEEPOUT 0x11 // sleep out
	#define PTLON 0x12 // partial display mode
	#define NORON 0x13 // display normal mode
	#define INVOFF 0x20 // inversion OFF
	#define INVON 0x21 // inversion ON
	#define DALO 0x22 // all pixel OFF
	#define DAL 0x23 // all pixel ON
	#define SETCON 0x25 // write contrast
	#define DISPOFF 0x28 // display OFF
	#define DISPON 0x29 // display ON
	#define CASET 0x2A // column address set
	#define PASET 0x2B // page address set
	#define RAMWR 0x2C // memory write
	#define RGBSET 0x2D // colour set
	#define PTLAR 0x30 // partial area
	#define VSCRDEF 0x33 // vertical scrolling definition
	#define TEOFF 0x34 // test mode
	#define TEON 0x35 // test mode
	#define MADCTL 0x36 // memory access control
	#define SEP 0x37 // vertical scrolling start address
	#define IDMOFF 0x38 // idle mode OFF
	#define IDMON 0x39 // idle mode ON
	#define COLMOD 0x3A // interface pixel format
	#define SETVOP 0xB0 // set Vop
	#define BRS 0xB4 // bottom row swap
	#define TRS 0xB6 // top row swap
	#define DISCTR 0xB9 // display control
	#define DOR 0xBA // data order
	#define TCDFE 0xBD // enable/disable DF temperature compensation
	#define TCVOPE 0xBF // enable/disable Vop temp comp
	#define EC 0xC0 // internal or external oscillator
	#define SETMUL 0xC2 // set multiplication factor
	#define TCVOPAB 0xC3 // set TCVOP slopes A and B
	#define TCVOPCD 0xC4 // set TCVOP slopes c and d
	#define TCDF 0xC5 // set divider frequency
	#define DF8COLOR 0xC6 // set divider frequency 8-color mode
	#define SETBS 0xC7 // set bias system
	#define RDTEMP 0xC8 // temperature read back


	// 12-bit color definitions
	#define WHITE 0xFFF
	#define BLACK 0x000
	#define RED 0xF00
	#define GREEN 0x0F0
	#define BLUE 0x00F
	#define CYAN 0x0FF
	#define MAGENTA 0xF0F
	#define YELLOW 0xFF0
	#define BROWN 0xB22
	#define ORANGE 0xFA0
	#define PINK 0xF6A


	// Font sizes
	#define SMALL 0
	#define MEDIUM 1
	#define LARGE 2


	// Booleans
	#define NOFILL 0
	#define FILL 1

    //MADCTL data (MEMORY DATA ACCESS CONTROL)
    #define MADCTL_DEFAULT 0x00
    #define MADCTL_MY 0x80
    #define MADCTL_MX 0x40
    #define MADCTL_V 0x20
    #define MADCTL_LAO 0x10
    #define MADCTL_RGB 0x08
	typedef struct
	{
		GPIO_TypeDef* Port;
		uint32_t Pin;
	} PinType;
	extern PinType ResetPin;
	extern volatile uint8_t busy_flag;
	extern volatile int8_t ScreenDirectoin;

	void LCDFlushTxBuffer(void);
	void LCDInit(int8_t ScrDir, uint8_t params);
	void LCDSetMADCTL (uint8_t val);
	void LCDSetRotation (uint8_t rot);
	void LCDClearScreen(void);
	void LCDSetXY(int x, int y);
	void LCDSetPixel(int x, int y, int color);
	void LCDSetLine(int x0, int y0, int x1, int y1, int color);
	void LCDSetRect(int x0, int y0, int x1, int y1, unsigned char fill, int color);
	void LCDSetCircle(int x0, int y0, int radius, int color);
	void LCDPutChar(char c, int x, int y, int size, int fColor, int bColor);
	void LCDPutCharRotate(char c, int x, int y, int size, int fColor, int bColor);
	void LCDPutStr(char *pString, int x, int y, int Size, int fColor, int bColor);
	void LCDPutStrROM(const char *pString, int x, int y, int Size, int fColor, int bColor);

#endif
