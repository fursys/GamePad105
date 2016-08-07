#include "GUI.h"
#include "ADC.h"
#include <MyLib.h>
//const char chName [CHANNELS][12]  = {"X          =","X          =","Right X(X) =","Right Y(Y) =","Left  Y(Z) =","Left  X(RX)=","Right V(RY)=","Left  V(RZ)="};
/*
const char strX[] =  "Right X(X) =";
const char strY[] =  "Right Y(Y) =";
const char strZ[] =  "Left  Y(Z) =";
const char strRX[] = "Left  X(RX)=";
const char strRY[] = "Right V(RY)=";
const char strRZ[] = "Left  V(RZ)=";
*/
void ADC_form (uint8_t event)
{
    char str[33];
    char stri[21];
    int len;
	switch (event)
	{
		case NO_EVENT:

            for (int i = 0;i<TOTAL_CHANNELS_COUNT;i++)
            {
                memset(str+12, 32, 9);//init result string
                itoa (ADC_data[i], stri, &len );
                memcpy(str,chName[i],12);
                memcpy(str+(16-len),stri,len);

                itoa (ADC_data_calibrated[i], stri, &len );
                memcpy(str+(21-len),stri,len+1);

                LCDPutStr (str, 121 - i*10, 1, SMALL, GREEN, BLACK);
            }
/*
            for (int i = 0;i<16;i++)
            {
                if (Buttons&(1<<i)) str[i] = '1';
                else str[i] = '0';
            }
            str[16] = 0;
            LCDPutStr (str, 11, 1, SMALL, GREEN, BLACK);


            for (int i = 16;i<BUTTONS_COUNT;i++)
            {
                if (Buttons&(1<<i)) str[i-16] = '1';
                else str[i-16] = '0';
            }
            str[32] = 0;
            LCDPutStr (str, 1, 1, SMALL, GREEN, BLACK);
*/

/*
            for (int i = 0;i<SWITCHES_COUNT;i++)
            {
                if (Switches&(1<<i)) str[i] = '1';
                else str[i] = '0';
            }

            str[SWITCHES_COUNT] = 0;
            LCDPutStr (str, 31, 1, SMALL, GREEN, BLACK);
*/
            LCDFlushTxBuffer();



			break;
		case KEY_DOWN_EVENT:
			break;
		case KEY_UP_EVENT:
			break;
		case KEY_ENTER_EVENT:
			break;
		case KEY_ESC_EVENT:
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
			break;
		case FORM_INIT_EVENT:
			//event = NO_EVENT;
            LCDSetRect(0, 0, 131, 131, FILL, BLACK);

            //LCDPutStrROM("Hello world!!!", 1, 1, SMALL, GREEN, BLACK);

            memset(str, 32, 10);


			LCDFlushTxBuffer();
			break;
		 default:
			break;
	}

}
