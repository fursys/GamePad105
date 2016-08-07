#include "GUI.h"
#include "flash.h"
#include "ADC.h"

void Reset_parameters_form (uint8_t event)
{
    	switch (event)
	{
		case NO_EVENT:

			break;
		case KEY_DOWN_EVENT:
			break;
		case KEY_UP_EVENT:
			break;
		case KEY_ENTER_EVENT:
		    //set default parameters
            //TODO: переделать обработчик для установления параметров по умолчанию
            //ADC_SetDefaultParameters(&Parameters.ADC_Params);
            Parameters.LCD_Britness = MAX_LCD_BRITNESS;
            Parameters.LCD_MADCTL = MADCTL_DEFAULT;
            Parameters.LCD_ScreenRotation = 0;
            ADC_SetDefaultParameters (&Parameters.ADC_Params);

            for (uint8_t i=0;i<TRANSMITTER_CHANNELS_COUNT;i++)
            {
                Parameters.RC_Channels_Mapping[i] = i;
            }
            for (uint8_t i=0;i<JOY_AXIS_COUNT;i++)
            {
                Parameters.Joy_Channels_Mapping[i] = i;
            }
            Parameters.RC_Channels_Mapping[6] = 10;
            Parameters.RC_Channels_Mapping[7] = 11;
            for (uint8_t i=0;i<32;i++)
            {
                Parameters.ButtonsMapping [i] = i;
            }

            SaveParametersAddr = (void*) LAST_PAGE;
            //save parameters
            flash_erase_page(SaveParametersAddr);
            WriteFlash(&Parameters, SaveParametersAddr, sizeof (Parameters));

            //Exit to main manu
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
			break;
		case KEY_ESC_EVENT:
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
			break;
		case FORM_INIT_EVENT:
			//event = NO_EVENT;
            LCDSetRect(0, 0, 131, 131, FILL, BLACK);
            LCDPutStrROM("Reset parameters?", 121, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("ENTER to reset", 111, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("ESC to exit", 101, 1, SMALL, GREEN, BLACK);
			LCDFlushTxBuffer();
			break;
		 default:
			break;
	}
}
