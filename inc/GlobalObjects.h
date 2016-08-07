#ifndef GLOBALOBJECTS_H_INCLUDED
#define GLOBALOBJECTS_H_INCLUDED

#include "stm32f1xx_hal.h"


//----------------ADC objects                               ------------------------------
#define ADC_CHANNELS 10
#define ADC_READ_INTERVAL 10
#define A_EMA  0.3333 	// 2/(N+1) N = 5 периодов


//-----------------------------------------------------------------------------------------
#define MAX_LCD_BRITNESS            10
#define JOY_AXIS_COUNT              8  //Channels sending to USB (Joystick)
#define TRANSMITTER_CHANNELS_COUNT  8  //Channels sending to PPM (RC controller)
#define TOTAL_CHANNELS_COUNT        12 //Total channels possible in the device
#define LONG_CLICK_INTERVAL         5
#define ENCODER_INTERVAL            100




typedef struct
{
	uint16_t ScaleFactor [TOTAL_CHANNELS_COUNT];//Channel value will be scaled to ScaleFactor. If ScaleFactor = 1000 then channel value will be between -1000 and 1000
	int8_t Transform [TOTAL_CHANNELS_COUNT]; //= {1,-1,1,1,1,1};
	int16_t Calibration [3][TOTAL_CHANNELS_COUNT];// Ќижнее значение|Ќоль|¬ерхнее значение
	int8_t DeadZone [TOTAL_CHANNELS_COUNT]; //Dead zone for each channel
	int16_t Trimmers [TOTAL_CHANNELS_COUNT]; //Trimmers
	float ScaleDivider [TOTAL_CHANNELS_COUNT];
}ADC_Parameters;

extern int32_t ADC_data [TOTAL_CHANNELS_COUNT]; //Analog channel average result
extern int32_t ADC_data_calibrated [TOTAL_CHANNELS_COUNT]; //Analog channel calibrated result
//----------------------------------------------------------------------------------------
#define BUTTONS_RESET_MASK 0x0FFFFFFF
extern uint32_t Buttons;
//----------------------------------------------------------------------------------------
typedef struct
{
	uint32_t header; //= PARAMETERS_HEADER; «аголовок нужен чтобы начало блока никода небыло равно 0xFFFFFFFF
    uint8_t LCD_Britness;
    uint8_t LCD_MADCTL; //LCD MEMORY DATA ACCESS CONTROL
    uint8_t LCD_ScreenRotation;
    ADC_Parameters ADC_Params;
    uint8_t RC_Channels_Mapping [TRANSMITTER_CHANNELS_COUNT];
    uint8_t Joy_Channels_Mapping [JOY_AXIS_COUNT];
    uint8_t ButtonsMapping [32];
}SaveDomain;

extern SaveDomain Parameters; //переменна€ типа SaveDomain дл€ хранени€ параметров системы
extern void * SaveParametersAddr;
//----------------------------------------------------------------------------------------
#define ENCODER

extern int16_t EncoderValue;
extern int8_t EncoderDelta;

#endif /* GLOBALOBJECTS_H_INCLUDED */
