#include "PPMsig.h"
#include <MyLib.h>
//---------------------------------------------------------------------------
uint16_t Values [2][17] = {{MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, PPM_SYNC_PULSE},
						{MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, MIN_PWM_VAL,
						PPM_PAUSE_VAL, PPM_SYNC_PULSE}};
uint8_t CurrentChannel = 0;
uint8_t CurrentArray = 0;
uint16_t CounterVal = 0;//PPM_PAUSE_VAL;
//uint16_t ScaleFactor [PPM_CHANNELS] = {};



//---------------------------------------------------------------------------
int16_t PPM_Calibrate (int16_t val, SaveDomain * Params, uint8_t index)
{
	int16_t scaledVal = 0;
	scaledVal = val + Params->ADC_Params.Calibration [1][index];
	if ((scaledVal < Params->ADC_Params.DeadZone[index]) && (scaledVal > -Params->ADC_Params.DeadZone[index])) scaledVal = 0;
    scaledVal += Params->ADC_Params.Trimmers [index];

	if (scaledVal > 0)
	{
		scaledVal = map (scaledVal,0,Params->ADC_Params.Calibration [2][index],0,SCALE_FACTOR);
		if (scaledVal > SCALE_FACTOR) scaledVal = SCALE_FACTOR;
	}
	else
	{
		scaledVal = map (scaledVal, Params->ADC_Params.Calibration [0][index],0,-SCALE_FACTOR,0);
		if (scaledVal < -SCALE_FACTOR) scaledVal = -SCALE_FACTOR;
	}
	//scaledVal *= Parameters.Transform[index];
	return scaledVal;
}
//---------------------------------------------------------------------------

void TIM1_CC_IRQHandler (void)
{

	uint32_t tmp = TIM1->SR;
	tmp &= TIM1->DIER;
	TIM1->SR = 0;


	if (tmp & TIM_SR_CC1IF)
	{
        if (CurrentChannel < 16)
		{
			CounterVal += Values[CurrentArray][CurrentChannel++];
			TIM1->CCR1 = CounterVal;
		}
        else if (CurrentChannel == 16)
		{
			CounterVal = Values[CurrentArray][CurrentChannel++];;
			TIM1->CCR1 = CounterVal;
		}
		else
        {
            CurrentChannel = 0;
            TIM1->CCR1 = PWM_ARR;
            CounterVal=0;
            //GPIOB->BSRR = GPIO_Pin_4; // -> HIGHT
        }


	}
	else if (tmp & TIM_SR_UIF)
    {

        CurrentChannel = 0;
        CounterVal = Values[CurrentArray][CurrentChannel++];;
		TIM1->CCR1 = CounterVal;



    }
	//else if (tmp & TIM_SR_CC1IF)
	//else if (tmp & TIM_SR_CC2IF)
	//else if (tmp & TIM_SR_CC4IF)



}
//---------------------------------------------------------------------------
void PPMinit (void)
{
	RCC->APB2ENR    |= RCC_APB2ENR_TIM1EN;	//TIM1 RCC enable
	RCC->APB2ENR	|= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN); //RCC enable for port B and alternative functions

    //GPIOB->BRR = GPIO_Pin_4; // -> LOW
	//TIM1 configure
    //AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1 | AFIO_MAPR_TIM1_REMAP_1; //AFIO_MAPR_SWJ_CFG_1 - 010: JTAG-DP Disabled and SW-DP Enabled;
                                                                //AFIO_MAPR_TIM1_REMAP_1 - 10: Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1)

	//PA8
	GPIOA->CRH      &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8); //GPIO bits clear
	GPIOA->CRH      |= (GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1);  //Out PP AF
	GPIOA->BSRR = GPIO_PIN_8; // -> HIGHT


	//TIM1->CR1       = 0x00000000;       //Reset timer settings
	TIM1->ARR       = PWM_ARR;           // Max value count to
	TIM1->CR1       |= TIM_CR1_ARPE;    // ARR buffering
	TIM1->PSC       = PWM_PSC;           // Prescaler
	TIM1->CCR1      = 0x10;	//Counter value

	//TIM1->CCMR2 =  = 0x00000000;

	//Set Output compare mode
	TIM1->CCMR1     |= TIM_CCMR1_OC1M_0
					| TIM_CCMR1_OC1M_1; //The output pin can keep its level (OCXM=000), be set
										//active (OCxM=001), be set inactive (OCxM=010)
										//or can toggle (OCxM=011) on match



	//TIM1->DIER 		|= TIM_DIER_UIE; //Update interrupt enable
	TIM1->DIER 		|= TIM_DIER_CC1IE; //CH1 Match interrupt enable
	TIM1->CCER      |= TIM_CCER_CC1E; //CH1 output enable

	NVIC_EnableIRQ(TIM1_CC_IRQn);              // Set interrupt procedure
    //GPIOB->BSRR = GPIO_Pin_4; // -> HIGHT
	TIM1->BDTR      |= TIM_BDTR_AOE;
	TIM1->EGR       |= TIM_EGR_UG;      //Counter reset to 0
	TIM1->CR1       |= TIM_CR1_CEN;      //Start timer count

}
//---------------------------------------------------------------------------
void PPMupdate (int32_t  * ar, SaveDomain* Params)
{
	uint8_t ar_index = CurrentArray;
	uint16_t ppm_len =0;
	ar_index ^= 1;
	/*
	for (int i = 0; i<8;i++)
	{
		Values [ar_index][i*2] = PPM_Calibrate (ar[i],Params,i) + MIN_PWM_VAL + SCALE_FACTOR - PPM_PAUSE_VAL;
		ppm_len += PPM_PAUSE_VAL + Values [ar_index][i*2];
	}*/

    for (int i = 0; i<8;i++)
	{
		Values [ar_index][i*2] = PPM_Calibrate (ar[Params->RC_Channels_Mapping[i]],Params,Params->RC_Channels_Mapping[i]) + MIN_PWM_VAL + SCALE_FACTOR - PPM_PAUSE_VAL;
		ppm_len += PPM_PAUSE_VAL + Values [ar_index][i*2];
	}

	/*for (int i = 6; i<8;i++)
	{
		Values [ar_index][i*2] = (swithes_val & (i-5)) ? MIN_PWM_VAL*2 : MIN_PWM_VAL;
		ppm_len += PPM_PAUSE_VAL + Values [ar_index][i*2];
	}*/
	//Values [ar_index][16] = 0xFF7F - (PPM_PAUSE_VAL + ppm_len); // устанавлиывем паузу до начала следующего цикла
	CurrentArray = ar_index;

}
