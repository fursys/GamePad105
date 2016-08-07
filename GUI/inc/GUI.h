#ifndef GUI_H_INCLUDED
#define GUI_H_INCLUDED


#include "GlobalObjects.h"
#include "Buttons.h"
#include "LCD.h"
#include <MyLib.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define EVENT_QUEUE_LEN 10

#define NO_EVENT 0
#define FORM_INIT_EVENT 1
#define KEY_DOWN_EVENT 2
#define KEY_UP_EVENT 3
#define KEY_ESC_EVENT 4
#define KEY_ENTER_EVENT 5
#define STICK_UP_EVENT 6
#define STICK_DOWN_EVENT 7
#define KEY_LEFT_EVENT 8
#define KEY_RIGHT_EVENT 9

#define ENCODER_UP_EVENT 10
#define ENCODER_DOWN_EVENT 11
#define ENCODER_CLICK_EVENT 12
#define ENCODER_LONGCLICK_EVENT 13

#define RIGHT_X 0
#define RIGHT_Y 1
#define LEFT_X 2
#define LEFT_Y 3


#ifndef LCDTask_PRIORITY
    #define LCDTask_PRIORITY                ( tskIDLE_PRIORITY + 0 )
#endif
#ifndef ButtonsEventTask_PRIORITY
    #define ButtonsEventTask_PRIORITY       ( tskIDLE_PRIORITY + 0 )
#endif


typedef void (*FORMFUNC)(uint8_t event);
extern FORMFUNC CurrentFormProc;
extern uint8_t LastEvent;

extern const char chName [TOTAL_CHANNELS_COUNT][13];//  = {"Right X(X) =","Right Y(Y) =","Left  X(RX)=","Left  Y(Z) =","Right V(RY)=","Left  V(RZ)=","X          =","X          ="};


void ButtonsEventTask ( void *pvParameters );
void GUI_Task ( void *pvParameters );
void GUI_init(void);
//------------------------------------------------------------------------------------------------
//-----Calibration parameters form header
#define DATA_COL_X_POS 78
#define HEADER_Y_POS 121
#define DATA_Y_POS 111
#define VALUE_INCREMENT 10
#define CHANNELS_PARAM_PAGES 6
#define DEADZONES_VALUE_INCREMENT 1
#define TRANSFORM_VALUE_INCREMENT 1
#define SCALE_VALUE_INCREMENT  10
#define INCREMENTATION_MULTIPLIER 10

void Calibration_params_form (uint8_t event);

//------------------------------------------------------------------------------------------------
//-----Save parameters form header
void Save_parameters_form (uint8_t event);
//------------------------------------------------------------------------------------------------
//-----Reset parameters form header
void Reset_parameters_form (uint8_t event);
//------------------------------------------------------------------------------------------------
//-----ADC calibration form header
void ADC_calibration_form (uint8_t event);
//------------------------------------------------------------------------------------------------
//-----ADC form header
void ADC_form (uint8_t event);
//------------------------------------------------------------------------------------------------
//-----Trimmer form header
void Trimmer_form (uint8_t event);
//------------------------------------------------------------------------------------------------
//-----Buttons mapping form
void Buttons_Mapping_form (uint8_t event);
//------------------------------------------------------------------------------------------------
//LCD parameters form
void LCD_parameters_form (uint8_t event);
//------------------------------------------------------------------------------------------------
//-----Menu form header
#define MAX_MENU_ROWS 10
#define MENU_X_POS 115
#define MENU_Y_POS 1
void Menu_Form (uint8_t event);
//------------------------------------------------------------------------------------------------
#endif /* GUI_H_INCLUDED */
