#include "GUI.h"


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

			break;
		case KEY_ESC_EVENT:
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
			break;
		case FORM_INIT_EVENT:
			//event = NO_EVENT;
            LCDSetRect(0, 0, 131, 131, FILL, BLACK);
			LCDFlushTxBuffer();
			break;
		 default:
			break;
	}
}