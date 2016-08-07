#include "GUI.h"




FORMFUNC CurrentFormProc;
uint8_t LastEvent = 0;

uint32_t ButtonsOld = 0;
uint32_t ButtonsCur = 0;

xQueueHandle event_queue;

const char chName [TOTAL_CHANNELS_COUNT][13]  = {"Right X(X) =\0","Right Y(Y) =\0","Left  Y(Z) =\0","Left  X(RX)=\0","Right V(RY)=\0","Left  V(RZ)=\0","X          =\0","X          =\0","{}         =\0","{}         =\0","{}         =\0","{}         =\0"};

void ButtonsEventTask ( void *pvParameters )
{
	while (1)
	{
        ButtonsOld &= ~ (ENCODER_UP_MASK | ENCODER_DOWN_MASK);
	    ButtonsCur = Buttons & (~ButtonsOld);
        Buttons &= ~ENCODER_RESETALL_MASK;
        //if (USB_connected && (Switches & 1))
        if (0)
        {
            //Если подключен USB и включен левый тумблер,
            //блокируем движение по меню

        }
        else
        {
            //if (ButtonsCur & ENCODER_UP_MASK) LastEvent = ENCODER_UP_EVENT;

            switch (ButtonsCur)
            {
                /*
                case KEY_UP://KEY_UP_EVENT
                    LastEvent = KEY_UP_EVENT;
                    xQueueSendToBack (event_queue, &LastEvent, 0);
                    break;
                case KEY_DOWN://KEY_DOWN_EVENT
                    LastEvent = KEY_DOWN_EVENT;
                    xQueueSendToBack (event_queue, &LastEvent, 0);
                    break;
                case KEY_RIGHT://KEY_RIGHT_EVENT
                    LastEvent = KEY_RIGHT_EVENT;
                    xQueueSendToBack (event_queue, &LastEvent, 0);
                    break;
                case KEY_LEFT://KEY_LEFT_EVENT
                    LastEvent = KEY_LEFT_EVENT;
                    xQueueSendToBack (event_queue, &LastEvent, 0);
                    break;
                case KEY_ENTER://KEY_ENTER_EVENT
                    LastEvent = KEY_ENTER_EVENT;
                    xQueueSendToBack (event_queue, &LastEvent, 0);
                    break;
                case KEY_FIRE_LEFT_UP://KEY_ESC_EVENT
                    LastEvent = KEY_ESC_EVENT;
                    xQueueSendToBack (event_queue, &LastEvent, 0);
                    break;
                */
                case ENCODER_BUTTON_MASK:
                    LastEvent = KEY_ENTER_EVENT; //ENCODER_CLICK_EVENT;
                    xQueueSendToBack (event_queue, &LastEvent, 0);
                    break;
                case ENCODER_LONGBTN_MASK:
                    LastEvent = KEY_ESC_EVENT; //ENCODER_LONGCLICK_EVENT;
                    xQueueSendToBack (event_queue, &LastEvent, 0);
                    break;
                case ENCODER_UP_MASK:
                    LastEvent = KEY_UP_EVENT;
                    xQueueSendToBack (event_queue, &LastEvent, 0);
                    break;
                case ENCODER_DOWN_MASK:
                    LastEvent = KEY_DOWN_EVENT;
                    xQueueSendToBack (event_queue, &LastEvent, 0);
                    break;
                case 0x05://две левые - получить нуль
                    break;
                case 0x0A://две правые - калибровка
                    break;
                case 0x0F: //четыре кнопки - сохранение
                    break;
                case 0x03: //Две верхние - подключить USB
                    break;
                default:
                    //Joystick_Send (buttons);
                    break;

            }
        }



        ButtonsOld = Buttons;
	    vTaskDelay( 10 / portTICK_RATE_MS );
	}


}
//------------------------------------------------------------------------------------------------


void GUI_Task ( void *pvParameters )
{
    //LCDInit(-1);
	LCDInit(Parameters.LCD_ScreenRotation, Parameters.LCD_MADCTL);
	LCDSetRect(0, 0, 131, 131, FILL, BLACK);
	//CurrentFormProc = ADC_form;
	CurrentFormProc = Menu_Form;
	CurrentFormProc(FORM_INIT_EVENT);

    uint8_t CurrEvent;
    BaseType_t GetEventResult;
	while (1)
	{
	    GetEventResult = xQueueReceive (event_queue, &CurrEvent, 100 / portTICK_RATE_MS);
	    if (GetEventResult == pdTRUE) CurrentFormProc(CurrEvent);
        else CurrentFormProc(NO_EVENT);
		//CurrentFormProc(LastEvent);
		//LastEvent = 0;//Reset event
		//vTaskDelay( 100 / portTICK_RATE_MS );
	}

}
//------------------------------------------------------------------------------------------------

void GUI_init(void)
{
    event_queue = xQueueCreate (EVENT_QUEUE_LEN,sizeof(LastEvent));
	xTaskCreate( GUI_Task,"GUI_Task", configMINIMAL_STACK_SIZE, NULL, LCDTask_PRIORITY, NULL );
    xTaskCreate( ButtonsEventTask,"ButtonsEventTask", configMINIMAL_STACK_SIZE, NULL, ButtonsEventTask_PRIORITY, NULL );
}
