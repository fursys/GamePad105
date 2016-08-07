#ifndef ENCODER_H_INCLUDED
#define ENCODER_H_INCLUDED


#include <stdlib.h>
#include "GlobalObjects.h"
#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#define ENC_ARR 0xFF
#define ENC_MAX 0x03e8 //1000
#define PWM_PSC 0x16
#define ENC_Task_PRIORITY               ( tskIDLE_PRIORITY + 5 )

#ifndef LONG_CLICK_INTERVAL
    #define LONG_CLICK_INTERVAL 100
#endif

#ifndef ENCODER
extern int16_t EncoderValue;
extern int8_t EncoderDelta;
extern uint8_t EncoderSwitch;
#endif

void EncoderInit (void);

#endif /* ENCODER_H_INCLUDED */
