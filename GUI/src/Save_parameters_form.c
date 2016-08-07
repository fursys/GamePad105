#include "GUI.h"
#include "flash.h"

void Save_parameters_form (uint8_t event)
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
            SaveParametersAddr = FindNextAddr(sizeof (Parameters));
			WriteFlash(&Parameters, SaveParametersAddr, sizeof (Parameters));

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
            LCDPutStrROM("Save parameters?", 121, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("ENTER to save", 111, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("ESC to exit", 101, 1, SMALL, GREEN, BLACK);
			LCDFlushTxBuffer();
			break;
		 default:
			break;
	}
}
