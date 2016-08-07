#include "LCD.h"
#include <stdlib.h>
#include <string.h>
//#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "LcdFont.h"

//#include "gpiodef.h"




void WriteSpiCommand(int command);
void WriteSpiData(int data);
// *****************************************************************************
//xQueueHandle LCD_TX_Queue;
uint8_t SPI_DMA_TX_Buffer[2][9]; // буфер, принимающий данные
uint8_t CurrentBuffer = 0;
uint8_t inbuf = 0; // принято байт
PinType ResetPin;
volatile uint8_t busy_flag = 0;
//uint8_t DMATransferComplete = 0;
volatile int8_t ScreenDirectoin = 0;


// *****************************************************************************
void SPI_StartDMA_TX (uint8_t LengthBufer, uint8_t * ar)
{
	busy_flag = 1;
	//DMATransferComplete = 0;
	DMA1_Channel5->CCR &= (uint16_t)(~DMA_CCR_EN); //запретить работу канала
	DMA1_Channel5->CMAR = (uint32_t)ar;
	DMA1_Channel5->CNDTR =  LengthBufer;      //загрузить количество данных для обмена
	DMA1->IFCR          |=  DMA_IFCR_CTCIF5;  //сбросить флаг окончания обмена
	GPIOB->BRR = (LCD_CS_PIN); //-> HIGHT
	DMA1_Channel5->CCR  |=  DMA_CCR_EN;      //разрешить работу канала
}
// *****************************************************************************
void LCDSetRotation (uint8_t rot)
{
    if (rot > 1) return;
    ScreenDirectoin = rot;

}
//*****************************************************************************
void LCDSetMADCTL (uint8_t val)
{
    WriteSpiCommand(MADCTL);
    WriteSpiData(val); // rotate 90 deg left
}
// *****************************************************************************
void SpiSend(uint16_t bits9)
{
	SPI_DMA_TX_Buffer[CurrentBuffer][inbuf] |= (uint8_t) ( bits9 >> (1 + inbuf));
	SPI_DMA_TX_Buffer[CurrentBuffer][inbuf + 1] = (uint8_t) (bits9 << (7 - inbuf));
	inbuf++;
	if (inbuf == 8)
	{
		//GPIOA->BSRR = GPIO_Pin_0;
		//GPIO_SetBits( GPIOA, GPIO_Pin_0); //Debug
		while (busy_flag==1);
		//GPIO_ResetBits( GPIOA, GPIO_Pin_0);//Debug
		//Send it!
		//GPIO_ResetBits( GPIOA, GPIO_Pin_1);
		//GPIOB->BRR = GPIO_Pin_12; //SPI CS -> LOW

		SPI_StartDMA_TX(9,SPI_DMA_TX_Buffer[CurrentBuffer]);
		CurrentBuffer++;
		CurrentBuffer &= 1;
		inbuf=0;

		memset(SPI_DMA_TX_Buffer[CurrentBuffer], 0, 9); //Clean TX buffer
	}
}
// *****************************************************************************
void LCDFlushTxBuffer(void)
{
	while (inbuf != 0)
	{
		SpiSend(0x00);
	}
}
// *****************************************************************************
void WriteSpiCommand(int command)
{
	uint16_t a = (uint16_t)(command & ~0x0100); //reset 9-th bit
	SpiSend (a);
}
// *****************************************************************************
void WriteSpiData(int data)
{
	uint16_t a = (uint16_t)(data | 0x0100); //set 9-th bit
	SpiSend(a);
}
// *****************************************************************************
/*
void SPI2_IRQHandler (void)
{
	if (SPI2->SR & SPI_SR_TXE)
	{

	}
}*/
// *****************************************************************************
void DMA1_Channel5_IRQHandler (void) //SPI TX DMA IRQ handler
{
  //Если обмен завершен
  if(DMA1->ISR & DMA_ISR_TCIF5)
  {
	//что-то делаем
	//SPI2->CR2 |= SPI_CR2_TXEIE; //Enable SPI TX interrupt, requared to wait for last byte will be transmitted
    DMA1->IFCR = DMA_IFCR_CTCIF5; //Очищаем бит прерывания

	// There is no interrupt for SPI BUSY flag, we have to wait end of transmission
	//on 9MGz SPI baud rate, delay will be ~167ns
	while (SPI2->SR & SPI_SR_BSY);
	//GPIOB->BSRR = GPIO_Pin_12; //SPI CS -> HIGHT
	//GPIO_SetBits( GPIOA, GPIO_Pin_1);
	busy_flag = 0;
	GPIOB->BSRR = (LCD_CS_PIN); //-> HIGHT

  }

  //Если передана половина буфера
  if(DMA1->ISR & DMA_ISR_HTIF5)
  {
	DMA1->IFCR |= DMA_IFCR_CHTIF5; //Очищаем бит прерывания
  }      //что-то делаем

  //Если произошла ошибка при обмене
  if(DMA1->ISR & DMA_ISR_TEIF5)
  {
  	//vPortFree (rx_frame.frame_array); //Очищаем память
	//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Включаем прерывания USART
	DMA1->IFCR |= DMA_IFCR_CTEIF5; //Очищаем бит прерывания
  }      //что-то делаем

  DMA1->IFCR |= DMA_IFCR_CGIF5; //очищаем бит глобального прерывания
}
// *****************************************************************************
void SPI_DrvInit ()
{

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; //SPI2 enable


	//GPIO for SPI
    GPIOB->CRH      &= ~(GPIO_CRH_MODE13 | GPIO_CRH_MODE15 | GPIO_CRH_CNF13 | GPIO_CRH_CNF15);
    GPIOB->CRH      |= (GPIO_CRH_MODE13 | GPIO_CRH_MODE15);//Output mode, max speed 50 MHz.
    GPIOB->CRH      |= (GPIO_CRH_CNF13_1 | GPIO_CRH_CNF15_1); //10: Alternate function output Push-pull



	//Init CS pin | RST pin
    GPIOB->CRL      &= ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF6 | GPIO_CRL_CNF7); //Reset pin settings
    GPIOB->CRL      |= (GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1);//Output mode, max speed 10 MHz.

    GPIOB->BRR = (LCD_CS_PIN); //-> LOW
    GPIOB->BSRR = LCD_RESET_PIN; //RST -> HIGHT

	//Включить тактирование DMA1
	if ((RCC->AHBENR & RCC_AHBENR_DMA1EN) != RCC_AHBENR_DMA1EN)
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	//Задать адрес источника и приемника и количество данных для обмена
	DMA1_Channel5->CPAR  =  (uint32_t)&SPI2->DR;       //адрес регистра перефирии
	DMA1_Channel5->CMAR  =  0;   				//адрес буфера в памяти, будет установлен при старте передачи
	DMA1_Channel5->CNDTR =  9;                         //количество данных для обмена
	//----------------- Манипуляции с регистром конфигурации  ----------------
	//Следующие действия можно обьединить в одну команду (разбито для наглядности)
	DMA1_Channel5->CCR   =  0;									//предочистка регистра конфигурации
	DMA1_Channel5->CCR  &= (uint16_t) (~DMA_CCR_CIRC);		//выключить циклический режим
	//DMA1_Channel5->CCR  &= (uint16_t) (~DMA_CCR3_DIR);         //направление: чтение в память
	DMA1_Channel5->CCR  |=  DMA_CCR_DIR;            			//направление: чтение из памяти
	//Настроить работу с переферийным устройством
	DMA1_Channel5->CCR   &= (uint16_t)(~DMA_CCR_PSIZE);		//размерность данных 8 бит
	DMA1_Channel5->CCR   &= (uint16_t)(~DMA_CCR_PINC);		//неиспользовать инкремент указателя
	//Настроить работу с памятью
	DMA1_Channel5->CCR   &= (uint16_t)(~DMA_CCR_MSIZE);		//размерность данных 8 бит
	DMA1_Channel5->CCR  |=  DMA_CCR_MINC;						//использовать инкремент указателя


	//Разрешить прерывание по завершении обмена:
	DMA1_Channel5->CCR |= DMA_CCR_TCIE;						//канал 7
	NVIC_EnableIRQ (DMA1_Channel5_IRQn);						//Разрешить прерывания от DMA
	NVIC_SetPriority (DMA1_Channel5_IRQn, 15);


	//Configure SPI
	SPI2->CR1 |= SPI_CR1_MSTR; //Set as master
	//SPI2->CR1 |= SPI_CR1_BR_2; //Baud rate SystemClock/32
	SPI2->CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0; //Baud rate SystemClock/8 - 9MGz
	SPI2->CR1 |= SPI_CR1_BIDIOE; //Output enable in bidirectional mode
	//SPI2->CR1 |= SPI_CR1_BIDIMODE; //1-line bidirectional data mode selected
	//SPI2->CR1 |= SPI_CR1_DFF//16-bit data frame format is selected for transmission/reception
	//SPI2->CR1 |= SPI_CR1_SSM; //Software NSS management (SSM = 1)
	SPI2->CR2 |= SPI_CR2_SSOE; //SS output is enabled in master mode and when the cell is enabled. The cell cannot work in a multimaster environment.
	SPI2->CR2 |= SPI_CR2_TXDMAEN;//Tx buffer DMA enabled
	//SPI2->CR2 |= SPI_CR2_TXEIE;//Tx interrupt enabled
	//SPI2->CR1 |= SPI_CR1_SSM; //Software slave management enabled. The external NSS pin remains free for other application uses
	SPI2->CR1 |= SPI_CR1_SPE; //Enable SPI
	//SPI2->CR1 |= SPI_CR1_SSI; //Slave select enable
	//SPI2->DR = 43;
	//SPI2->CR1 &= (uint16_t)(~SPI_CR1_SSI); //Slave select disable


	//NVIC_EnableIRQ (SPI2_IRQn);						//Разрешить прерывания от SPI
	//NVIC_SetPriority (SPI2_IRQn, 15);

	//LCD_TX_Queue = xQueueCreate (LCD_TX_QUEUE_LEN,sizeof(SPI_DMA_TX_Buffer));
	memset(SPI_DMA_TX_Buffer, 0, 18);



}

void LCD_Sender( void *pvParameters )
{
	while (1)
	{


	}
}


// *****************************************************************************
// Initializes the Philips PCF8833 LCD Controller
//
// Inputs: none
//
// Author: James P Lynch July 7, 2007
// *****************************************************************************
void LCDInit(int8_t ScrDir, uint8_t params)
{

    ScreenDirectoin = ScrDir;

	//GPIO_SetBits( ResetPin.Port, ResetPin.Pin);
	SPI_DrvInit ();

	//xTaskCreate( SPI_Sender,(signed char *)"SPI_Sender", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );

	// Hardware reset
	GPIOB->BRR = LCD_RESET_PIN; //RST -> LOW
	//GPIO_ResetBits( ResetPin.Port, ResetPin.Pin);
	vTaskDelay( 1000 / portTICK_RATE_MS );
	GPIOB->BSRR = LCD_RESET_PIN; //RST -> HIGHT
	vTaskDelay( 1000 / portTICK_RATE_MS );
	// Sleep out (command 0x11)
	WriteSpiCommand(SLEEPOUT);
	// Inversion on (command 0x21)
	//WriteSpiCommand(INVON); // seems to be required for this controller
	// Color Interface Pixel Format (command 0x3A)
	WriteSpiCommand(COLMOD);
	WriteSpiData(0x03); // 0x03 = 12 bits-per-pixel
	// Memory access controler (command 0x36)
	//WriteSpiData(MADCTL_V | MADCTL_LAO | MADCTL_MX); // mirror x
	WriteSpiCommand(MADCTL);
	//WriteSpiData(MADCTL_LAO | MADCTL_MX | MADCTL_MY | MADCTL_V); // rotate 90 deg left
	WriteSpiData(params);
	//WriteSpiData (0x36);
	// Write contrast (command 0x25)
	WriteSpiCommand(SETCON);
	WriteSpiData(0x3F); // contrast 0x30
	//for(int i = 0;i<20000;i++);
	// Display On (command 0x29)
	WriteSpiCommand(DISPON);
	LCDFlushTxBuffer();


}
// *****************************************************************************
// LCDClearScreen.c
//
// Clears the LCD screen to single color (BLACK)
//
// Inputs: none
//
// Author: James P Lynch July 7, 2007
// *****************************************************************************
void LCDClearScreen(void) {
	long i; // loop counter
	// Row address set (command 0x2B)
	WriteSpiCommand(PASET);
	WriteSpiData(0);
	WriteSpiData(131);
	// Column address set (command 0x2A)
	WriteSpiCommand(CASET);
	WriteSpiData(0);
	WriteSpiData(131);
	// set the display memory to BLACK
	WriteSpiCommand(RAMWR);
	for (i = 0; i < ((131 * 131) / 2); i++)
	{
		WriteSpiData((BLACK >> 4) & 0xFF);
		WriteSpiData(((BLACK & 0xF) << 4) | ((BLACK >> 8) & 0xF));
		WriteSpiData(BLACK & 0xFF);
	}
	LCDFlushTxBuffer();
}
// *****************************************************************************
//
// Sets the Row and Column addresses
//
// Inputs: x = row address (0 .. 131)
// y = column address (0 .. 131)
//
//
// Returns: nothing
//
// Author: James P Lynch July 7, 2007
// *****************************************************************************
void LCDSetXY(int x, int y)
{
	// Row address set (command 0x2B)
	WriteSpiCommand(PASET);
	WriteSpiData(x);
	WriteSpiData(x);
	// Column address set (command 0x2A)
	WriteSpiCommand(CASET);
	WriteSpiData(y);
	WriteSpiData(y);
}
// *************************************************************************************
//
// Lights a single pixel in the specified color at the specified x and y addresses
//
// Inputs: x = row address (0 .. 131)
// y = column address (0 .. 131)
// color = 12-bit color value rrrrggggbbbb
// rrrr = 1111 full red
// :
// 0000 red is off
//
// gggg = 1111 full green
// :
// 0000 green is off
//
// bbbb = 1111 full blue
// :
// 0000 blue is off
//
// Returns: nothing
//
// Note: see lcd.h for some sample color settings
//
// Author: James P Lynch July 7, 2007
// *************************************************************************************
void LCDSetPixel(int x, int y, int color)
{
    int temp;
    if (ScreenDirectoin == 1)
    {
        temp = x;
        x = y;
        y = 131 - temp;
    }
	LCDSetXY(x, y);
	WriteSpiCommand(RAMWR);
	WriteSpiData((unsigned char)((color >> 4) & 0xFFFF));
	WriteSpiData((unsigned char)(((color & 0x0F) << 4) | 0x00));
	WriteSpiCommand(NOP);
}
// *************************************************************************************************
// LCDSetLine.c
//
// Draws a line in the specified color from (x0,y0) to (x1,y1)
//
// Inputs: x = row address (0 .. 131)
// y = column address (0 .. 131)
// color = 12-bit color value rrrrggggbbbb
// rrrr = 1111 full red
// :
// 0000 red is off
//
// gggg = 1111 full green
// :
// 0000 green is off
//
// bbbb = 1111 full blue
// :
// 0000 blue is off
//
// Returns: nothing
//
// Note: good write-up on this algorithm in Wikipedia (search for Bresenham's line algorithm)
// see lcd.h for some sample color settings
//
// Authors: Dr. Leonard McMillan, Associate Professor UNC
// Jack Bresenham IBM, Winthrop University (Father of this algorithm, 1962)
//
// Note: taken verbatim from Professor McMillan's presentation:
// http://www.cs.unc.edu/~mcmillan/comp136/Lecture6/Lines.html
//
// *************************************************************************************************
void LCDSetLine(int x0, int y0, int x1, int y1, int color)
{
    int temp;
    if (ScreenDirectoin == 1)
    {
        temp = x0;
        x0 = y0;
        y0 = temp;
        temp = x1;
        x1 = y1;
        y1 = temp;
    }

    int dy = y1 - y0;
	int dx = x1 - x0;
	int stepx, stepy;

	if (dy < 0) { dy = -dy; stepy = -1; } else { stepy = 1; }
	if (dx < 0) { dx = -dx; stepx = -1; } else { stepx = 1; }
	dy <<= 1; // dy is now 2*dy
	dx <<= 1; // dx is now 2*dx
	LCDSetPixel(x0, y0, color);
	if (dx > dy)
	{
		int fraction = dy - (dx >> 1); // same as 2*dy - dx
		while (x0 != x1)
		{
			if (fraction >= 0)
			{
				y0 += stepy;
				fraction -= dx; // same as fraction -= 2*dx
			}
			x0 += stepx;
			fraction += dy; // same as fraction -= 2*dy
			LCDSetPixel(x0, y0, color);
		}
	}
	else
	{
		int fraction = dx - (dy >> 1);
		while (y0 != y1)
		{
			if (fraction >= 0)
			{
				x0 += stepx;
				fraction -= dy;
			}
			y0 += stepy;
			fraction += dx;
			LCDSetPixel(x0, y0, color);
		}
	}
}
// *****************************************************************************************
//
// Draws a rectangle in the specified color from (x1,y1) to (x2,y2)
// Rectangle can be filled with a color if desired
//
// Inputs: x = row address (0 .. 131)
// y = column address (0 .. 131)
// fill = 0=no fill, 1-fill entire rectangle
// color = 12-bit color value for lines rrrrggggbbbb
// rrrr = 1111 full red
// :
// 0000 red is off
//
// gggg = 1111 full green
// :
// 0000 green is off
//
// bbbb = 1111 full blue
// :
// 0000 blue is off
// Returns: nothing
//
// Notes:
//
// The best way to fill a rectangle is to take advantage of the "wrap-around" featute
// built into the Philips PCF8833 controller. By defining a drawing box, the memory can
// be simply filled by successive memory writes until all pixels have been illuminated.
//
// 1. Given the coordinates of two opposing corners (x0, y0) (x1, y1)
// calculate the minimums and maximums of the coordinates
//
// xmin = (x0 <= x1) ? x0 : x1;
// xmax = (x0 > x1) ? x0 : x1;
// ymin = (y0 <= y1) ? y0 : y1;
// ymax = (y0 > y1) ? y0 : y1;
//
// 2. Now set up the drawing box to be the desired rectangle
//
// WriteSpiCommand(PASET); // set the row boundaries
// WriteSpiData(xmin);
// WriteSpiData(xmax);
// WriteSpiCommand(CASET); // set the column boundaries
// WriteSpiData(ymin);
// WriteSpiData(ymax);
//
// 3. Calculate the number of pixels to be written divided by 2
//
// NumPixels = ((((xmax - xmin + 1) * (ymax - ymin + 1)) / 2) + 1)
//
// You may notice that I added one pixel to the formula.
// This covers the case where the number of pixels is odd and we
// would lose one pixel due to rounding error. In the case of
// odd pixels, the number of pixels is exact.
// in the case of even pixels, we have one more pixel than
// needed, but it cannot be displayed because it is outside
// the drawing box.
//
// We divide by 2 because two pixels are represented by three bytes.
// So we work through the rectangle two pixels at a time.
//
// 4. Now a simple memory write loop will fill the rectangle
//
// for (i = 0; i < ((((xmax - xmin + 1) * (ymax - ymin + 1)) / 2) + 1); i++) {
// WriteSpiData((color >> 4) & 0xFF);
// WriteSpiData(((color & 0xF) << 4) | ((color >> 8) & 0xF));
// WriteSpiData(color & 0xFF);
// }
//
// In the case of an unfilled rectangle, drawing four lines with the Bresenham line
// drawing algorithm is reasonably efficient.
//
// Author: James P Lynch July 7, 2007
// *****************************************************************************************
void LCDSetRect(int x0, int y0, int x1, int y1, unsigned char fill, int color)
{
	int xmin, xmax, ymin, ymax;
	int i;
	int temp;
	// check if the rectangle is to be filled
	if (fill == FILL)
	{
        if (ScreenDirectoin == 1)
        {
            temp = x0;
            x0 = y0;
            y0 = 131 - temp;
            temp = x1;
            x1 = y1;
            y1 = 131 - temp;
        }
		// best way to create a filled rectangle is to define a drawing box
		// and loop two pixels at a time
		// calculate the min and max for x and y directions
		xmin = (x0 <= x1) ? x0 : x1;
		xmax = (x0 > x1) ? x0 : x1;
		ymin = (y0 <= y1) ? y0 : y1;
		ymax = (y0 > y1) ? y0 : y1;
		// specify the controller drawing box according to those limits
		// Row address set (command 0x2B)
		WriteSpiCommand(PASET);
		WriteSpiData(xmin);
		WriteSpiData(xmax);
		// Column address set (command 0x2A)
		WriteSpiCommand(CASET);
		WriteSpiData(ymin);
		WriteSpiData(ymax);
		// WRITE MEMORY
		WriteSpiCommand(RAMWR);
		// loop on total number of pixels / 2
		for (i = 0; i < ((((xmax - xmin + 1) * (ymax - ymin + 1)) / 2) + 1); i++)
		{
			// use the color value to output three data bytes covering two pixels
			WriteSpiData((color >> 4) & 0xFF);
			WriteSpiData(((color & 0xF) << 4) | ((color >> 8) & 0xF));
			WriteSpiData(color & 0xFF);
		}
	}
	else
	{
		// best way to draw un unfilled rectangle is to draw four lines
		LCDSetLine(x0, y0, x1, y0, color);
		LCDSetLine(x0, y1, x1, y1, color);
		LCDSetLine(x0, y0, x0, y1, color);
		LCDSetLine(x1, y0, x1, y1, color);
	}
}

// *************************************************************************************
// LCDSetCircle.c
//
// Draws a line in the specified color at center (x0,y0) with radius
//
// Inputs: x0 = row address (0 .. 131)
// y0 = column address (0 .. 131)
// radius = radius in pixels
// color = 12-bit color value rrrrggggbbbb
//
// Returns: nothing
//
// Author: Jack Bresenham IBM, Winthrop University (Father of this algorithm, 1962)
//
// Note: taken verbatim Wikipedia article on Bresenham's line algorithm
// http://www.wikipedia.org
//
// *************************************************************************************
void LCDSetCircle(int x0, int y0, int radius, int color)
{
	int f = 1 - radius;
	int ddF_x = 0;
	int ddF_y = -2 * radius;
	int x = 0;
	int y = radius;
	LCDSetPixel(x0, y0 + radius, color);
	LCDSetPixel(x0, y0 - radius, color);
	LCDSetPixel(x0 + radius, y0, color);
	LCDSetPixel(x0 - radius, y0, color);
	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x + 1;
		LCDSetPixel(x0 + x, y0 + y, color);
		LCDSetPixel(x0 - x, y0 + y, color);
		LCDSetPixel(x0 + x, y0 - y, color);
		LCDSetPixel(x0 - x, y0 - y, color);
		LCDSetPixel(x0 + y, y0 + x, color);
		LCDSetPixel(x0 - y, y0 + x, color);
		LCDSetPixel(x0 + y, y0 - x, color);
		LCDSetPixel(x0 - y, y0 - x, color);
	}
}
// *****************************************************************************
// LCDPutChar.c
//
// Draws an ASCII character at the specified (x,y) address and color
//
// Inputs: c = character to be displayed
// x = row address (0 .. 131)
// y = column address (0 .. 131)
// size = font pitch (SMALL, MEDIUM, LARGE)
// fcolor = 12-bit foreground color value rrrrggggbbbb
// bcolor = 12-bit background color value rrrrggggbbbb
//
//
// Returns: nothing
//
//
// Notes: Here's an example to display "E" at address (20,20)
//
// LCDPutChar('E', 20, 20, MEDIUM, WHITE, BLACK);
//
// (27,20) (27,27)
// | |
// | |
// ^ V V
// : _ # # # # # # # 0x7F
// : _ _ # # _ _ _ # 0x31
// : _ _ # # _ # _ _ 0x34
// x _ _ # # # # _ _ 0x3C
// : _ _ # # _ # _ _ 0x34
// : _ _ # # _ _ _ # 0x31
// : _ # # # # # # # 0x7F
// : _ _ _ _ _ _ _ _ 0x00
//
// ------y------->
// ^ ^
// | |
// | |
// (20,20) (20,27)
//
//
// The most efficient way to display a character is to make use of the "wrap-around" feature
// of the Philips PCF8833 LCD controller chip.
//
// Assume that we position the character at (20, 20) that's a (row, col) specification.
// With the row and column address set commands, you can specify an 8x8 box for the SMALL and MEDIUM
// characters or a 16x8 box for the LARGE characters.
//
// WriteSpiCommand(PASET); // set the row drawing limits
// WriteSpiData(20); //
// WriteSpiData(27); // limit rows to (20, 27)
//
// WriteSpiCommand(CASET); // set the column drawing limits
// WriteSpiData(20); //
// WriteSpiData(27); // limit columns to (20,27)
//
// When the algorithm completes col 27, the column address wraps back to 20
// At the same time, the row address increases by one (this is done by the controller)
//
// We walk through each row, two pixels at a time. The purpose is to create three
// data bytes representing these two pixels in the following format (as specified by Philips
// for RGB 4 : 4 : 4 format (see page 62 of PCF8833 controller manual).
//
// Data for pixel 0: RRRRGGGGBBBB
// Data for Pixel 1: RRRRGGGGBBBB
//
// WriteSpiCommand(RAMWR); // start a memory write (96 data bytes to follow)
//
// WriteSpiData(RRRRGGGG); // first pixel, red and green data
// WriteSpiData(BBBBRRRR); // first pixel, blue data; second pixel, red data
// WriteSpiData(GGGGBBBB); // second pixel, green and blue data
// :
// and so on until all pixels displayed!
// :
// WriteSpiCommand(NOP); // this will terminate the RAMWR command
//
//
// Author: James P Lynch July 7, 2007
// *****************************************************************************
void LCDPutChar(char c, int x, int y, int size, int fColor, int bColor)
{
	extern const unsigned char FONT6x8[97][8];
	extern const unsigned char FONT8x8[97][8];
	extern const unsigned char FONT8x16[97][16];
	int i,j;
	unsigned int nCols;
	unsigned int nRows;
	unsigned int nBytes;
	unsigned char PixelRow;
	unsigned char Mask;
	unsigned int Word0;
	unsigned int Word1;
	unsigned char *pFont;
	unsigned char *pChar;

	unsigned char *FontTable[] =
	{
		(unsigned char *)FONT6x8,
		(unsigned char *)FONT8x8,
		(unsigned char *)FONT8x16
	};
	// get pointer to the beginning of the selected font table
	pFont = (unsigned char *)FontTable[size];
	// get the nColumns, nRows and nBytes
    nCols = *pFont;
    nRows = *(pFont + 1);
    nBytes = *(pFont + 2);

	// get pointer to the last byte of the desired character
	pChar = pFont + (nBytes * (c - 0x1F)) + nBytes - 1;
	// Row address set (command 0x2B)
	WriteSpiCommand(PASET);
	WriteSpiData(x);
	WriteSpiData(x + nRows - 1);
	// Column address set (command 0x2A)
	WriteSpiCommand(CASET);
	WriteSpiData(y);
	WriteSpiData(y + nCols - 1);
	// WRITE MEMORY
	WriteSpiCommand(RAMWR);
	// loop on each row, working backwards from the bottom to the top
	for (i = nRows - 1; i >= 0; i--)
	{
		// copy pixel row from font table and then decrement row
		PixelRow = *pChar--;
		// loop on each pixel in the row (left to right)
		// Note: we do two pixels each loop
		Mask = 0x80;
		for (j = 0; j < nCols; j += 2)
		{
			// if pixel bit set, use foreground color; else use the background color
			// now get the pixel color for two successive pixels
			if ((PixelRow & Mask) == 0)
			Word0 = bColor;
			else
			Word0 = fColor;
			Mask = Mask >> 1;
			if ((PixelRow & Mask) == 0)
			Word1 = bColor;
			else
			Word1 = fColor;
			Mask = Mask >> 1;
			// use this information to output three data bytes
			WriteSpiData((Word0 >> 4) & 0xFF);
			WriteSpiData(((Word0 & 0xF) << 4) | ((Word1 >> 8) & 0xF));
			WriteSpiData(Word1 & 0xFF);
		}
	}


}
// *************************************************************************************************
void LCDPutCharRotate(char c, int x, int y, int size, int fColor, int bColor)
{
	extern const unsigned char FONT6x8[97][8];
	extern const unsigned char FONT8x8[97][8];
	extern const unsigned char FONT8x16[97][16];
	int i,j;
	unsigned int nCols;
	unsigned int nRows;
	unsigned int nBytes;
	unsigned int Word0;
	unsigned int Word1;
	unsigned char *pFont;
	unsigned char *pChar;
    uint16_t t;
	unsigned char *FontTable[] =
	{
		(unsigned char *)FONT6x8,
		(unsigned char *)FONT8x8,
		(unsigned char *)FONT8x16
	};
	// get pointer to the beginning of the selected font table
	pFont = (unsigned char *)FontTable[size];
	// get the nColumns, nRows and nBytes
    nCols = *pFont;
    nRows = *(pFont + 1);
    nBytes = *(pFont + 2);

	// get pointer to the last byte of the desired character
	pChar = pFont + (nBytes * (c - 0x1F));
	// Row address set (command 0x2B)
	WriteSpiCommand(PASET);
	WriteSpiData(y);
	WriteSpiData(y + nCols - 1);
	// Column address set (command 0x2A)
	WriteSpiCommand(CASET);
	WriteSpiData(132 - x - nRows);
	WriteSpiData(132 - x - 1);
	// WRITE MEMORY
	WriteSpiCommand(RAMWR);
	// loop on each row, working backwards from the bottom to the top
	for (i = 0; i < nCols; i++)
	{
		for (j = 0; j < nRows; j += 2)
		{
			// if pixel bit set, use foreground color; else use the background color
			// now get the pixel color for two successive pixels
			t = *(pChar + j);
			//if ((*(pChar + j) & (1<<i)) == 0)
			if ((t & (128>>i)) == 0)
			Word0 = bColor;
			else
			Word0 = fColor;

			if ((*(pChar + j + 1) & (128>>i)) == 0)
			Word1 = bColor;
			else
			Word1 = fColor;

			// use this information to output three data bytes
			WriteSpiData((Word0 >> 4) & 0xFF);
			WriteSpiData(((Word0 & 0xF) << 4) | ((Word1 >> 8) & 0xF));
			WriteSpiData(Word1 & 0xFF);
		}
	}


}
// *************************************************************************************************
//
// Draws a null-terminates character string at the specified (x,y) address, size and color
//
// Inputs: pString = pointer to character string to be displayed
// x = row address (0 .. 131)
// y = column address (0 .. 131)
// Size = font pitch (SMALL, MEDIUM, LARGE)
// fColor = 12-bit foreground color value rrrrggggbbbb
// bColor = 12-bit background color value rrrrggggbbbb
//
//
// Returns: nothing
//
// Notes: Here's an example to display "Hello World!" at address (20,20)
//
// LCDPutChar("Hello World!", 20, 20, LARGE, WHITE, BLACK);
//
//
// Author: James P Lynch July 7, 2007
// *************************************************************************************************
void LCDPutStr(char *pString, int x, int y, int Size, int fColor, int bColor) {
	// loop until null-terminator is seen
	while (*pString != 0x00)
	{
		// draw the character
		if (ScreenDirectoin ==0) LCDPutChar(*pString++, x, y, Size, fColor, bColor);
		else LCDPutCharRotate (*pString++, x, y, Size, fColor, bColor);
		// advance the y position
		if (Size == SMALL)
		y = y + 6;
		else if (Size == MEDIUM)
		y = y + 8;
		else
		y = y + 8;
		// bail out if y exceeds 131
		if (y > 131) break;
	}
}
void LCDPutStrROM(const char *pString, int x, int y, int Size, int fColor, int bColor) {
	// loop until null-terminator is seen
	while (*pString != 0x00)
	{
		// draw the character
		if (ScreenDirectoin ==0)  LCDPutChar(*pString++, x, y, Size, fColor, bColor);
		else LCDPutCharRotate (*pString++, x, y, Size, fColor, bColor);
		// advance the y position
		if (Size == SMALL)
		y = y + 6;
		else if (Size == MEDIUM)
		y = y + 8;
		else
		y = y + 8;
		// bail out if y exceeds 131
		if (y > 131) break;
	}
}
