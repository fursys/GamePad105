#include "GUI.h"


int8_t dzPosX = 0;
int8_t dzPage = 0;
uint8_t dzeditMode = 0;

const char dzpageName [3][7]  = {"  val \0",
                               "center\0",
                               "  max \0"};

//------------------------------------------------------------------------------------------------
void dzPrintData (uint8_t page, uint8_t line, uint8_t cursor)
{
    char str[6];
    char stri[6];
    int len;

    memset(str, 32, 6);//init result string
    itoa (Parameters.ADC_Params.DeadZone[line], stri, &len );
    memcpy(str+(6-len),stri,len+1);

    if (cursor) LCDPutStr (str, DEADZONES_DATA_Y_POS - line*10, DEADZONES_DATA_COL_X_POS, SMALL, BLACK, dzeditMode ? GREEN : RED);
    else LCDPutStr (str, DEADZONES_DATA_Y_POS - line*10, DEADZONES_DATA_COL_X_POS, SMALL, GREEN, BLACK);
}
//------------------------------------------------------------------------------------------------
void dzPrintPage (uint8_t page)
{
    LCDPutStrROM (dzpageName[page], HEADER_Y_POS, 72, SMALL, GREEN, BLACK);
    for (int i = 0;i<ADC_CHANNELS;i++)
    {
        if (i==dzPosX) dzPrintData (page,i,1);
        else dzPrintData (page,i,0);
    }
}
//------------------------------------------------------------------------------------------------
void DeadZones_form (uint8_t event)
{
    	switch (event)
	{
		case NO_EVENT:

			break;
		case KEY_DOWN_EVENT:

		    if (dzeditMode)
            {
                Parameters.ADC_Params.DeadZone[dzPosX]-=DEADZONES_VALUE_INCREMENT;
                dzPrintData (dzPage,dzPosX,1);
            }
            else
            {
                dzPosX++;
                if  (dzPosX>=ADC_CHANNELS)
                {
                    dzPrintData (dzPage,dzPosX-1,0);
                    dzPosX = 0;
                    dzPrintData (dzPage,dzPosX,1);
                }
                else //двигаем курсор в пределах страницы
                {
                    dzPrintData (dzPage,dzPosX,1);
                    dzPrintData (dzPage,dzPosX-1,0);
                }
            }
            LCDFlushTxBuffer();

			break;
		case KEY_UP_EVENT:
		    if (dzeditMode)
            {
                Parameters.ADC_Params.DeadZone[dzPosX]+=DEADZONES_VALUE_INCREMENT;
                dzPrintData (dzPage,dzPosX,1);
            }
            else
            {
                dzPosX--;
                if (dzPosX<0)
                {
                    dzPrintData (dzPage,dzPosX+1,0);
                    dzPosX = ADC_CHANNELS-1;
                    dzPrintData (dzPage,dzPosX,1);
                }
                else //двигаем курсор в пределах страницы
                {
                    dzPrintData (dzPage,dzPosX,1);
                    dzPrintData (dzPage,dzPosX+1,0);
                }
            }
            LCDFlushTxBuffer();
			break;
		case KEY_ENTER_EVENT:
            dzeditMode++;
            dzeditMode &=1;
            dzPrintData (dzPage,dzPosX,1);
			break;
		case KEY_ESC_EVENT:
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
			break;
		case FORM_INIT_EVENT:
			//event = NO_EVENT;
            LCDSetRect(0, 0, 131, 131, FILL, BLACK);
            LCDPutStrROM ("Ch. name", DEADZONES_HEADER_Y_POS, 1, SMALL, GREEN, BLACK);
            for (int i = 0;i<CHANNELS;i++)
            {
                LCDPutStrROM (chName[i], DEADZONES_DATA_Y_POS - i*10, 1, SMALL, GREEN, BLACK);
            }
            dzPrintPage (0);



			LCDFlushTxBuffer();

			LCDFlushTxBuffer();
			break;
		 default:
			break;
	}
}
