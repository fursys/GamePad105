#include "GUI.h"
#include "ADC.h"

uint8_t Stage = 0;

void CalibrateProcess (void)
{
    switch (Stage)
    {
        case 0:
            LCDSetRect(0, 0, 131, 131, FILL, BLACK);
            LCDPutStrROM("Move gimbals to", 121, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("neutral position", 111, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("Press ENTER", 101, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("for next step", 91, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("ESC to exit", 81, 1, SMALL, GREEN, BLACK);
            Stage++;
            break;
        case 1:

            ADC_GetZeroLevel();

            LCDSetRect(0, 0, 131, 131, FILL, BLACK);
            LCDPutStrROM("Move gimbals to", 121, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("min and max pos", 111, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("Press ENTER", 101, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("for next step", 91, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("ESC to exit", 81, 1, SMALL, GREEN, BLACK);
            LCDFlushTxBuffer();
            //Set maximum possible values
            for (int i=0;i<ADC_CHANNELS;i++)
            {
                Parameters.ADC_Params.Calibration [0][i] = 0x7FFF;
                Parameters.ADC_Params.Calibration [2][i] = -0x7FFF;
            }
            ADC_StartCalibration();//Start calibration. see ADC.c module
            Stage++;
            break;
        case 2:
            ADC_StopCalibration();//Stop calibration.

            LCDSetRect(0, 0, 131, 131, FILL, BLACK);
            LCDPutStrROM("Calibration", 121, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("completed!", 111, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("Press ENTER", 101, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("to exit", 91, 1, SMALL, GREEN, BLACK);
            LCDFlushTxBuffer();
            Stage++;
            break;
        case 3://Exit
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
            break;
        default:
            break;
    }

}

void ADC_calibration_form (uint8_t event)
{
    	switch (event)
	{
		case NO_EVENT:

			break;
        case ENCODER_CLICK_EVENT:
            CalibrateProcess();
            break;
		case KEY_DOWN_EVENT:
			break;
		case KEY_UP_EVENT:
			break;
		case KEY_ENTER_EVENT:
            CalibrateProcess();
			break;
		case KEY_ESC_EVENT:
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
			break;
		case FORM_INIT_EVENT:
			//event = NO_EVENT;
            LCDSetRect(0, 0, 131, 131, FILL, BLACK);

            LCDPutStrROM("Gimbal calibration", 121, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("ENTER to start", 111, 1, SMALL, GREEN, BLACK);
            LCDPutStrROM("ESC to exit", 101, 1, SMALL, GREEN, BLACK);
			LCDFlushTxBuffer();
			Stage = 0;
			break;
		 default:
			break;

	}
}
