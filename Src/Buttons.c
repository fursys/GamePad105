#include "Buttons.h"




uint32_t Buttons;
//int16_t Switches;

//int16_t ButtonsOld;
//int16_t ButtonsCur;


void ButtonsTask ( void *pvParameters )
{
	while( 1 )
    {
        Buttons &= ~BUTTONS_RESET_MASK;

        //Switches switch 2 has interception with button 24!!!
        //TODO: replace switch bindings to virtual axis interface
        if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))
        {
            Buttons |= 0x01000000;
            ADC_data[10] = -1000;
            ADC_data_calibrated[10] = -1000;
        }
        else if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2))
        {
            Buttons |= 0x02000000;
            ADC_data[10] = 1000;
            ADC_data_calibrated[10] = 1000;
        }
        else
        {
            ADC_data[10] = 0;
            ADC_data_calibrated[10] = 0;
        }
        if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))
        {
            Buttons |= 0x04000000;
            ADC_data[11] = -1000;
            ADC_data_calibrated[11] = -1000;
        }
        else if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4))
        {
            Buttons |= 0x08000000;
            ADC_data[11] = 1000;
            ADC_data_calibrated[11] = 1000;
        }
        else
        {
            ADC_data[11] = 0;
            ADC_data_calibrated[11] = 0;
        }

        //Buttons
        for (int i=1;i<6;i++)
        {
            //Ставим пин в режим выхода
            GPIOC->CRH      &= ~(GPIO_CRH_CNF8 << 4*(i+2));
            GPIOC->CRH      |= (GPIO_CRH_MODE8_1 << 4*(i+2));

            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0<<(i+10),GPIO_PIN_SET);
            for (int j=1;j<6;j++)
            {
                if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0<<(j-1)))
                {
                    Buttons |= (1<<((i-1)*5 + (j-1)));
                }
            }
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0<<(i+10),GPIO_PIN_RESET);
            //Возвращаем в режим входа
            GPIOC->CRH      |= (GPIO_CRH_CNF8_0 << 4*(i+2));
            GPIOC->CRH      &= ~(GPIO_CRH_MODE8_1 << 4*(i+2));
        }



        HAL_Delay( BTN_READ_INTERVAL );
    }

}

//------------------------------------------------------------------------------------------------
void ButtonsInit(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
/*
    //Оставляем пины в режиме входа, на выход переключаем тольео перед использованием!!!
    GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12  | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
*/

    GPIO_InitStruct.Pin = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


//Switches

    GPIO_InitStruct.Pin = (GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9);
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = (GPIO_PIN_2);
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    xTaskCreate( ButtonsTask,"ButtonsTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
}
