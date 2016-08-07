/**
  ******************************************************************************
  * @file           : USB_DEVICE
  * @version        : v1.0_Cube
  * @brief          : Header for usb_device file.
  ******************************************************************************
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usb_device_H
#define __usb_device_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "usbd_def.h"

#include "GlobalObjects.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

typedef struct
{
	uint16_t buttons;
	int16_t X;
	int16_t Y;
	int16_t Z;
	int16_t RX;
	int16_t RY;
	int16_t RZ;
	int16_t An7;
	int16_t An8;
	//int16_t An9;
}gamepad_report_t;


extern USBD_HandleTypeDef hUsbDeviceFS;

/* USB_Device init function */

void USB_DEVICE_Init(void);
void OTG_FS_IRQHandler(void);

#ifdef __cplusplus
}
#endif
#endif /*__usb_device_H */

