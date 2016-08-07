#ifndef BUTTONS_H
#define BUTTONS_H

#include "GlobalObjects.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef BTN_READ_INTERVAL
    #define BTN_READ_INTERVAL 10
#endif

#define BUTTONS_COUNT           32
//#define SWITCHES_COUNT          2

#define KEY_UP                  64
#define KEY_DOWN                128
#define KEY_RIGHT               256
#define KEY_LEFT                32
#define KEY_ENTER               16
#define KEY_FIRE_RIGHT_UP       1
#define KEY_FIRE_RIGHT_DOWN     4
#define KEY_FIRE_LEFT_UP        2
#define KEY_FIRE_LEFT_DOWN      8



//extern int16_t Buttons;
//extern int16_t Switches;


void ButtonsInit(void);

#endif /* BUTTONS_H*/
