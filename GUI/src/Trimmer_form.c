#include "GUI.h"
#include <MyLib.h>

#define X_STICK 0
#define Y_STICK 1

#define X_DIRECTION 0
#define Y_DIRECTION 1

uint8_t stick = 0;
int16_t xCalibrated = 70;
int16_t yCalibrated = 70;

int16_t Tr_Calibrate (int16_t val, uint8_t index)
{
	int16_t scaledVal = 0;
	scaledVal = val;
	if (scaledVal > 0)
	{
		scaledVal = map (scaledVal,0,Parameters.ADC_Params.ScaleFactor[index],0,59);
		if (scaledVal > 59) scaledVal = 59; //проверяем выход за границу диапазона

	}
	else
	{
		scaledVal = map (scaledVal,-Parameters.ADC_Params.ScaleFactor[index],0,-59,0);
		if (scaledVal < -59) scaledVal = -59;//проверяем выход за границу диапазона

	}
	return scaledVal;
}
//------------------------------------------------------------------------------------------------
void UpdateScreen (uint8_t index, uint8_t direction)
{
    char str[6];
    int len;


    itoa(Parameters.ADC_Params.Trimmers[index],str,&len);
    str[5] = 0;
    if (len < 5)
    {
        len--;
        for (int i = 4;i>=0;i--)
        {
            if (len>=0) str[i] = str[len--];
            else str[i] = ' ';
        }

    }
    if (direction == X_DIRECTION)
    {
        LCDSetRect(2,xCalibrated-1,9,xCalibrated+1,1,BLACK);
        xCalibrated = 70 + Tr_Calibrate(Parameters.ADC_Params.Trimmers[index],index);
        LCDSetRect(2,xCalibrated-1,9,xCalibrated+1,1,GREEN);
        LCDPutStr(str,25,90,LARGE,GREEN,BLACK);
    }
    else if (direction == Y_DIRECTION)
    {
        LCDSetRect(yCalibrated-1,2,yCalibrated+1,9,1,BLACK);
        yCalibrated = 70 + Tr_Calibrate(Parameters.ADC_Params.Trimmers[index],index);
        LCDSetRect(yCalibrated-1,2,yCalibrated+1,9,1,GREEN);
        LCDPutStr(str,106,15,LARGE,GREEN,BLACK);
    }
    LCDFlushTxBuffer();
}
//------------------------------------------------------------------------------------------------

void Trimmer_form (uint8_t event)
{


    switch (event)
	{
		case NO_EVENT:

			break;
        case KEY_LEFT_EVENT:
            break;
        case KEY_RIGHT_EVENT:
            break;
		case KEY_DOWN_EVENT:
		    if (stick==Y_STICK)
            {
                Parameters.ADC_Params.Trimmers[RIGHT_Y]-=10;
                UpdateScreen (RIGHT_Y,Y_DIRECTION);
            }
            else
            {
                Parameters.ADC_Params.Trimmers[RIGHT_X] +=10;
                UpdateScreen (RIGHT_X,X_DIRECTION);
            }
			break;
		case KEY_UP_EVENT:
		    if (stick == Y_STICK)
            {
                Parameters.ADC_Params.Trimmers[RIGHT_Y]+=10;
                UpdateScreen (RIGHT_Y,Y_DIRECTION);
            }
            else
            {
                Parameters.ADC_Params.Trimmers[RIGHT_X] -=10;
                UpdateScreen (RIGHT_X,X_DIRECTION);
            }

			break;
		case KEY_ENTER_EVENT:
            stick++;
            stick &=1;
            //LCDSetRect(0, 0, 131, 131, FILL, BLACK);
		    if (stick == X_STICK)
            {
                LCDPutStrROM ("Trim X", 121, 20, SMALL, GREEN, BLACK);
                UpdateScreen (RIGHT_X,X_DIRECTION);
            }
            else
            {
                LCDPutStrROM ("Trim Y ", 121, 20, SMALL, GREEN, BLACK);
                UpdateScreen (RIGHT_Y,Y_DIRECTION);
            }

			break;
		case KEY_ESC_EVENT:
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
			break;
		case FORM_INIT_EVENT:
			//event = NO_EVENT;
            LCDSetRect(0, 0, 131, 131, FILL, BLACK);
            LCDPutStrROM ("Trim X", 121, 20, SMALL, GREEN, BLACK);
            LCDSetRect(1,10,10,130,0,GREEN);
            LCDSetRect(10,1,130,10,0,GREEN);

            LCDSetLine(10,70,20,70,GREEN);
            LCDSetLine(70,10,70,20,GREEN);
            LCDFlushTxBuffer();

            UpdateScreen (RIGHT_X,X_DIRECTION);
            UpdateScreen (RIGHT_Y,Y_DIRECTION);


			stick = 0;
			break;
		 default:
			break;
	}
}
