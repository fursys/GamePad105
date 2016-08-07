#include "GUI.h"
#include <MyLib.h>



int8_t cpfPosX = 0;
int8_t cpfPage = 0;
uint8_t editMode = 0;

const char pageName [CHANNELS_PARAM_PAGES][7]  = {"  min \0",
                                                  "center\0",
                                                  "  max \0",
                                                  "D.Zons\0",
                                                  "Tranfr\0",
                                                  "Scale \0"};

void ChangeValue (int8_t direction, uint8_t mult)
{
    if (cpfPage == 3) Parameters.ADC_Params.DeadZone[cpfPosX]+=DEADZONES_VALUE_INCREMENT*direction*mult;
    else if (cpfPage == 4) Parameters.ADC_Params.Transform[cpfPosX]+=TRANSFORM_VALUE_INCREMENT*direction;
    else if (cpfPage == 5) Parameters.ADC_Params.ScaleFactor[cpfPosX]+=SCALE_VALUE_INCREMENT*direction*mult;
    else Parameters.ADC_Params.Calibration[cpfPage][cpfPosX]+=VALUE_INCREMENT*direction*mult;
}
//------------------------------------------------------------------------------------------------
void PrintData (uint8_t page, uint8_t line, uint8_t cursor)
{
    char str[6];
    char stri[6];
    int len;

    memset(str, 32, 6);//init result string
    if (cpfPage == 3) itoa (Parameters.ADC_Params.DeadZone[line], stri, &len );
    else if (cpfPage == 4) itoa (Parameters.ADC_Params.Transform[line], stri, &len );
    else if (cpfPage == 5) itoa (Parameters.ADC_Params.ScaleFactor[line], stri, &len );
    else itoa (Parameters.ADC_Params.Calibration[page][line], stri, &len );
    memcpy(str+(6-len),stri,len+1);

    if (cursor) LCDPutStr (str, DATA_Y_POS - line*10, DATA_COL_X_POS, SMALL, BLACK, editMode ? GREEN : RED );
    else LCDPutStr (str, DATA_Y_POS - line*10, DATA_COL_X_POS, SMALL, GREEN, BLACK);
}
//------------------------------------------------------------------------------------------------
void PrintPage (uint8_t page)
{
    LCDPutStrROM (pageName[page], HEADER_Y_POS, 72, SMALL, GREEN, BLACK);
    for (int i = 0;i<ADC_CHANNELS;i++)
    {
        if (i==cpfPosX) PrintData (page,i,1);
        else PrintData (page,i,0);
    }
}
//------------------------------------------------------------------------------------------------
void Calibration_params_form (uint8_t event)
{
    switch (event)
	{
		case NO_EVENT:

			break;
		case KEY_DOWN_EVENT:
		    if (editMode)
            {
                ChangeValue (-1, 1);
                PrintData (cpfPage,cpfPosX,1);
            }
            else
            {
                cpfPosX++;
                if  (cpfPosX>=ADC_CHANNELS) //прыгаем на следующую страницу
                {
                    cpfPage++;
                    if (cpfPage == CHANNELS_PARAM_PAGES) cpfPage = 0;
                    cpfPosX = 0;
                    PrintPage (cpfPage);
                }
                else //двигаем курсор в пределах страницы
                {
                    PrintData (cpfPage,cpfPosX,1);
                    PrintData (cpfPage,cpfPosX-1,0);
                }
            }
            LCDFlushTxBuffer();
			break;
		case KEY_UP_EVENT:
		    if (editMode)
            {
                ChangeValue (1, 1);
                PrintData (cpfPage,cpfPosX,1);
            }
            else
            {
                cpfPosX--;
                if (cpfPosX<0) //прыгаем на предыдущую страницу
                {
                    cpfPage--;
                    if (cpfPage == -1) cpfPage = CHANNELS_PARAM_PAGES - 1;
                    cpfPosX = ADC_CHANNELS-1;
                    PrintPage (cpfPage);
                }
                else //двигаем курсор в пределах страницы
                {
                    PrintData (cpfPage,cpfPosX,1);
                    PrintData (cpfPage,cpfPosX+1,0);
                }
            }
            LCDFlushTxBuffer();
			break;
        case KEY_RIGHT_EVENT:
            if (editMode)
            {
                ChangeValue (1, INCREMENTATION_MULTIPLIER);
                PrintData (cpfPage,cpfPosX,1);
            }
            else
            {
                //прыгаем на следующую страницу
                cpfPage++;
                if (cpfPage ==CHANNELS_PARAM_PAGES) cpfPage = 0;
                cpfPosX = 0;
                PrintPage (cpfPage);
            }
            break;
        case KEY_LEFT_EVENT:
            if (editMode)
            {
                ChangeValue (-1, INCREMENTATION_MULTIPLIER);
                PrintData (cpfPage,cpfPosX,1);
            }
            else
            {
                //прыгаем на предыдущую страницу
                cpfPage--;
                if (cpfPage == -1) cpfPage = CHANNELS_PARAM_PAGES - 1;
                cpfPosX = ADC_CHANNELS-1;
                PrintPage (cpfPage);
            }
            break;
		case KEY_ENTER_EVENT:
            editMode++;
            editMode &=1;
            PrintData (cpfPage,cpfPosX,1);
			break;
		case KEY_ESC_EVENT:
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
			break;
		case FORM_INIT_EVENT:
			//event = NO_EVENT;
            LCDSetRect(0, 0, 131, 131, FILL, BLACK);
            LCDPutStrROM ("Ch. name", HEADER_Y_POS, 1, SMALL, GREEN, BLACK);
            for (int i = 0;i<ADC_CHANNELS;i++)
            {
                LCDPutStrROM (chName[i], DATA_Y_POS - i*10, 1, SMALL, GREEN, BLACK);
            }
            PrintPage (0);



			LCDFlushTxBuffer();
			break;
		 default:
			break;
	}
}
