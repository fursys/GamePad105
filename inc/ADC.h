#ifndef __ADC_h
#define __ADC_h

#include "GlobalObjects.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "stm32f1xx_hal.h"

#define ADC_QUEUE_LEN 2


#ifndef ADC_Reader_PRIORITY
    #define ADC_Reader_PRIORITY				( tskIDLE_PRIORITY + 1 )
#endif
#ifndef A_EMA
    #define A_EMA  0.3333 	// 2/(N+1) N = 5 периодов
#endif
#ifndef ADC_CHANNELS
    #define ADC_CHANNELS 10 		// 0-6 -каналы управления
#endif
#ifndef ADC_READ_INTERVAL
    #define ADC_READ_INTERVAL 10
#endif




extern ADC_Parameters * ADC_Params;
//extern float ScaleDivider [ADC_CHANNELS];

extern xQueueHandle adc_queue;


void ADC_InitProc (ADC_Parameters * params);
void ADC_read (void);
void ADC_Reader( void *pvParameters );
void ADC_GetZeroLevel(void);
void ADC_SetDefaultParameters (ADC_Parameters * params);
void ADC_StartCalibration (void);
void ADC_StopCalibration (void);


#endif
