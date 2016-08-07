/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  */

/* Private define ------------------------------------------------------------*/

#define GREEN_LED_Pin GPIO_PIN_8
#define GREEN_LED_GPIO_Port GPIOC



#define ENCODER_RESETALL_MASK   0xF0000000
#define ENCODER_RESUPDOWN_MASK  0xC0000000
#define ENCODER_DOWN_MASK       0x80000000
#define ENCODER_UP_MASK         0x40000000
#define ENCODER_BUTTON_MASK     0x20000000
#define ENCODER_LONGBTN_MASK    0x10000000
