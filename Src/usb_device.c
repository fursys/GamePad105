/**
  ******************************************************************************
  * @file           : USB_DEVICE
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

/* USB Device Core handle declaration */
USBD_HandleTypeDef hUsbDeviceFS;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

gamepad_report_t gamePadReport;
static void SendUSB( void *pvParameters );
void MX_USB_DEVICE_Init(void);
//------------------------------------------------------------------------------------------------
/* init function */
void MX_USB_DEVICE_Init(void)
{
  /* Init Device Library,Add Supported Class and Start the library*/
  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);

  USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID);

  USBD_Start(&hUsbDeviceFS);

}
//------------------------------------------------------------------------------------------------
/**
* @brief This function handles USB OTG FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
static void SendUSB( void *pvParameters )
{
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    int16_t * report;

    HAL_Delay(5000);
    while (1)
    {

        if ((hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED) || (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED));
        {
            report = (int16_t*) &gamePadReport;
            //report[0] = (uint16_t) Buttons;
            report[0] = 0;
            for (int i = 0;i<16;i++)
            {
                //Buttons mapping
                if (Buttons & (1<<Parameters.ButtonsMapping[i]))
                {
                    report[0] |= (1<<i);
                }
            }

            for (int i = 0;i<JOY_AXIS_COUNT;i++)
            {
                //Axis mapping
                report[i+1] = ADC_data_calibrated [Parameters.Joy_Channels_Mapping[i]];
            }

            USBD_HID_SendReport     (&hUsbDeviceFS, (uint8_t*) report, sizeof (gamePadReport));
            /*
            gamePadReport.buttons = (uint16_t) Buttons;// | (Switches<<14);
            gamePadReport.X = ADC_data_calibrated [Parameters.Channels_Mapping[0]];
            gamePadReport.Y = ADC_data_calibrated [Parameters.Channels_Mapping[1]];
            gamePadReport.RX = ADC_data_calibrated [Parameters.Channels_Mapping[2]];
            gamePadReport.Z = ADC_data_calibrated [Parameters.Channels_Mapping[3]];
            gamePadReport.RY = ADC_data_calibrated [Parameters.Channels_Mapping[4]];
            gamePadReport.RZ = ADC_data_calibrated [Parameters.Channels_Mapping[5]];
            gamePadReport.An7 = ADC_data_calibrated [Parameters.Channels_Mapping[6]];
            gamePadReport.An8 = ADC_data_calibrated [Parameters.Channels_Mapping[7]];
            */
            //USBD_HID_SendReport     (&hUsbDeviceFS, &gamePadReport, sizeof (gamePadReport));
        }

        HAL_Delay(32);
        //vTaskDelay( 32 / portTICK_RATE_MS );
    }


}
//------------------------------------------------------------------------------------------------
void USB_DEVICE_Init(void)
{
    gamePadReport.buttons = 0;
    gamePadReport.X = 0;
    gamePadReport.Y = 0;
    gamePadReport.RX = 0;
    gamePadReport.Z = 0;
    gamePadReport.RY = 0;
    gamePadReport.RZ = 0;
    gamePadReport.An7 = 0;
    gamePadReport.An8 = 0;
    xTaskCreate( SendUSB,"SendUSB", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );
}
