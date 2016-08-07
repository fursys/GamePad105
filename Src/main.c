#include "stm32f105xc.h"
//#include "gpiodef.h"

#include "stm32f1xx_hal.h"
#include "GlobalObjects.h"
#include "usb_device.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"


#include "ADC.h"
#include "flash.h"
#include "Buttons.h"
#include "GUI.h"
#include "Encoder.h"
#include "PPMsig.h"
//------------------------------------------------------------------------------------------------
#define Blink1_PRIORITY					( tskIDLE_PRIORITY + 0 )
#define PARAMETERS_HEADER 0xA0B0A1B1 //ћаркер начала блока параметров

//------------------------------------------------------------------------------------------------

/* Allocate two blocks of RAM for use by the heap.  The first is a block of 0x10000
bytes starting from address 0x80000000, and the second a block of 0xa0000 bytes
starting from address 0x90000000.  The block starting at 0x80000000 has the lower
start address so appears in the array fist. */
/*
јдрес начала блока пам€ти должен быть больше суммы
Data size и BSS size
которые вывод€тс€ линкером после сборки.
Ќужно следить чтобы размер не перекрывал стек.
*/
const HeapRegion_t xHeapRegions[] =
{
    { ( uint8_t * ) 0x20002800UL, configTOTAL_HEAP_SIZE },
    { NULL, 0 } /* Terminates the array. */
};
//------------------------------------------------------------------------------------------------

SaveDomain Parameters; //переменна€ типа SaveDomain дл€ хранени€ параметров системы
//uint32_t Buttons;

void * SaveParametersAddr;

//------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------
void ResetParameters (void)
{
    Parameters.LCD_Britness = MAX_LCD_BRITNESS;
    Parameters.LCD_MADCTL = MADCTL_DEFAULT;
    Parameters.LCD_ScreenRotation = 0;
    ADC_SetDefaultParameters (&Parameters.ADC_Params);

    for (uint8_t i=0;i<TRANSMITTER_CHANNELS_COUNT;i++)
    {
        Parameters.RC_Channels_Mapping[i] = i;
    }
    for (uint8_t i=0;i<JOY_AXIS_COUNT;i++)
    {
        Parameters.Joy_Channels_Mapping[i] = i;
    }
    for (uint8_t i=0;i<32;i++)
    {
        Parameters.ButtonsMapping [i] = i;
    }

    SaveParametersAddr = (void*) LAST_PAGE;
}
//------------------------------------------------------------------------------------------------

static void Blink1( void *pvParameters )
{
    //uint8_t calibr = 0;
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GREEN_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);


    //TODO: ѕеренести в LCD
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);


    //GPIOB->CRL      &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3 | GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    //GPIOB->CRL      |= (GPIO_CRL_MODE3 | GPIO_CRL_MODE5);//Output mode, max speed 50 MHz.
    //GPIOB->CRL      |= (GPIO_CRL_CNF5_1); //10: Alternate function output Push-pull
    //GPIOB->CRH      |= (GPIO_CRH_MODE13 | GPIO_CRH_MODE15);
    //GPIOB->CRH      |= (GPIO_CRH_CNF13_0 | GPIO_CRH_CNF15_0);//Output mode, max speed 50 MHz.
	//Init CS pin | RST pin


    //GPIOB->CRL      &= ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF6 | GPIO_CRL_CNF7); //Reset pin settings
    //GPIOB->CRL      |= (GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1);//Output mode, max speed 10 MHz.



	while( 1 )
    {
        /*
        if (Buttons == 2)
        {
            if (calibr)
            {
                calibr = 0;
                ADC_StopCalibration();
                SaveParametersAddr = FindNextAddr(sizeof (Parameters));
                WriteFlash(&Parameters, SaveParametersAddr, sizeof (Parameters));
            }
            else
            {
                calibr = 1;
                ADC_GetZeroLevel();
                ADC_StartCalibration();
            }
        }
        */
        HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
        //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6);
        //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
        HAL_Delay (100);
		//vTaskDelay( 100/ portTICK_RATE_MS );
	}
}
//------------------------------------------------------------------------------------------------
uint32_t HAL_GetTick(void)
{
    return xTaskGetTickCount();
}
void HAL_Delay(__IO uint32_t Delay)
{
    TickType_t ticks = Delay / portTICK_PERIOD_MS;
    vTaskDelay(ticks ? ticks : 1);          /* Minimum delay = 1 tick */
}

//------------------------------------------------------------------------------------------------
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV3;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  //HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  //__HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}
//------------------------------------------------------------------------------------------------

int main(void)
{



    /* HEAP initialization.
    Pass the array into vPortDefineHeapRegions(). */
    vPortDefineHeapRegions( xHeapRegions );

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();


// init values============================================================
	Parameters.header = PARAMETERS_HEADER;

	SaveParametersAddr = findLastBlock (sizeof (Parameters));
	if (SaveParametersAddr)
	{
		memcpy (&Parameters,SaveParametersAddr,sizeof (Parameters));
	}
	else
	{
        ResetParameters ();
	}
    flash_unlock();
    //flash_erase_page (LAST_PAGE);


    ButtonsInit();
    ADC_InitProc (&Parameters.ADC_Params);
    USB_DEVICE_Init ();
    GUI_init();
    EncoderInit();
    PPMinit();


	xTaskCreate( Blink1,"Blink1", configMINIMAL_STACK_SIZE, NULL, Blink1_PRIORITY, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();
}
