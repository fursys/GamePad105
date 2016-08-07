#include "encoder.h"

int16_t EncoderValue = 0;
int8_t EncoderDelta = 0;
uint8_t EncoderSwitch = 0;
uint16_t EncButtonTime = 0;
uint8_t LastButtonState = 1;

//------------------------------------------------------------------------------------------------
/*
void TIM3_IRQHandler (void)
{

    TIM3->SR = 0;
    if (TIM3->CR1& TIM_CR1_DIR)
    {
        //reverse
    }
    else
    {
        //forward
    }
}
*/
//------------------------------------------------------------------------------------------------
void ENCinit12 (void)
{
    RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR    |= RCC_APB1ENR_TIM3EN;	//TIM3 RCC enable
    RCC->APB2ENR	|= RCC_APB2ENR_IOPCEN; //RCC enable for port B
    AFIO->MAPR      |=AFIO_MAPR_TIM3_REMAP_FULLREMAP;
/*
	//PB6, PB7
	GPIOB->CRL      &= ~(GPIO_CRL_CNF6 | GPIO_CRL_CNF7 | GPIO_CRL_MODE6 | GPIO_CRL_MODE7); //GPIO bits clear
	GPIOB->CRL      |= (GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1);  //01: Floating input (reset state)
    GPIOB->ODR      |= GPIO_ODR_ODR6 | GPIO_ODR_ODR7; //Set pull up
*/
    //PC6, PC7
	GPIOC->CRL      &= ~(GPIO_CRL_CNF6 | GPIO_CRL_CNF7 | GPIO_CRL_MODE6 | GPIO_CRL_MODE7); //GPIO bits clear
	GPIOC->CRL      |= (GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1);  //01: Floating input (reset state)
    GPIOC->ODR      |= GPIO_ODR_ODR6 | GPIO_ODR_ODR7; //Set pull up

    //Encoder switch PB12
    GPIOB->CRH      &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
    GPIOB->CRH      |= (GPIO_CRH_CNF12_1);//Input pull mode.
    GPIOB->ODR      |= GPIO_ODR_ODR12; //Set pull up

    TIM3->ARR       = ENC_ARR;
    //TIM3->CCR1      = 1000;

    TIM3->SMCR      |= TIM_SMCR_SMS_1;
    //TIM3->SMCR  |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; //To select Encoder Interface mode write SMS=001 in the TIMx_SMCR register if the counter
                                                    //is counting on TI2 edges only, SMS=010 if it is counting on TI1 edges only and SMS=011 if
                                                    //it is counting on both TI1 and TI2 edges.
    TIM3->CCER      = TIM_CCER_CC1P | TIM_CCER_CC2P;
    //TIM3->CCER |= TIM_CCER_CC3P | TIM_CCER_CC4P;
    //TIM3->CCER &= ~(TIM_CCER_CC3P | TIM_CCER_CC4P);
    TIM3->CCMR1     |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; //Map channels

    TIM3->CCMR1     |= TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1
                    | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1; //Apply filters

    TIM3->EGR       |= TIM_EGR_UG;      //Counter reset to 0
    TIM3->CNT       = ENC_ARR/2;
    TIM3->SR        = 0;

    //TIM3->DIER 		|= TIM_DIER_UIE;    //Update interrupt enable
    //TIM3->DIER 		|= TIM_DIER_CC1IE;
    //NVIC_EnableIRQ(TIM3_IRQn);

	TIM3->CR1       |= TIM_CR1_CEN;      //Start timer count
}

//------------------------------------------------------------------------------------------------
static void ENC_Task ( void *pvParameters )
{

    	while( 1 )
    	{
            EncoderDelta = (TIM3->CNT-ENC_ARR/2);
            TIM3->CNT = ENC_ARR/2; //reset counter
            EncoderValue += EncoderDelta*10;

            if (EncoderValue > ENC_MAX) EncoderValue = ENC_MAX;
            else if (EncoderValue < -ENC_MAX) EncoderValue = -ENC_MAX;

            EncoderSwitch = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);




            if ((EncoderSwitch) && (!LastButtonState)) //Button release event
            {
                if (EncButtonTime < LONG_CLICK_INTERVAL) Buttons |= ENCODER_BUTTON_MASK;
                else Buttons |=  ENCODER_LONGBTN_MASK;
                EncButtonTime = 0;
            }
            else if (!EncoderSwitch) //user keep button pressed
            {
                Buttons &= ~ENCODER_RESETALL_MASK;
                EncButtonTime++;
            }


            Buttons &= ~ENCODER_RESUPDOWN_MASK;
            if (EncoderDelta > 0) Buttons |= ENCODER_UP_MASK;
            else if (EncoderDelta < 0) Buttons |= ENCODER_DOWN_MASK;

            LastButtonState = EncoderSwitch;

    	    vTaskDelay( ENCODER_INTERVAL / portTICK_RATE_MS );
    	}
}
//------------------------------------------------------------------------------------------------
void EncoderInit (void)
{
    ENCinit12 ();

    xTaskCreate( ENC_Task,"ENC_Task", configMINIMAL_STACK_SIZE, NULL, ENC_Task_PRIORITY, NULL );
}
