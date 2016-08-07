#include "ADC.h"
#include <MyLib.h>
#include "PPMSig.h"

int16_t Calibrate (int16_t val, uint8_t index);

xQueueHandle adc_queue;
uint32_t adc_buffer[ADC_CHANNELS];
int32_t ADC_data [TOTAL_CHANNELS_COUNT];
int32_t ADC_data_calibrated [TOTAL_CHANNELS_COUNT]; //Analog channel calibrated result

ADC_Parameters * ADC_Params;
uint8_t CalibrateMode = 0; // 0-NormalMode| 1-CalibrateMode



//------------------------------------------------------------------------------------------------
void ADC_StartCalibration (void)
{
    CalibrateMode = 1;
}
//------------------------------------------------------------------------------------------------
void ADC_StopCalibration (void)
{
    CalibrateMode = 0;
}
//------------------------------------------------------------------------------------------------
void ADC_SetDefaultParameters (ADC_Parameters * params)
{
    for (int i=0;i<TOTAL_CHANNELS_COUNT;i++)
    {
        params->Calibration [0][i] = 0x7FFF;
        params->Calibration [1][i] = 0;
        params->Calibration [2][i] = -0x7FFF;

        params->ScaleFactor [i] = 1000;
        params->Transform [i] = 1;
        params->DeadZone [i] = 0;
        params->Trimmers [i] = 0;
        params->ScaleDivider[i] = 1;
    }
    //TODO: this  to virtual axis interface
        params->Calibration [0][10] = -1000;
        params->Calibration [2][10] = 1000;
        params->Calibration [0][11] = -1000;
        params->Calibration [2][11] = 1000;

}
//------------------------------------------------------------------------------------------------
void ADC_Reader( void *pvParameters )
{
    uint32_t cur_adc [ADC_CHANNELS];


    while( 1 )
    {
        ADC_read ();
		xQueueReceive (adc_queue, cur_adc, portMAX_DELAY); //Get data from Queue
        for (int i = 0;i<ADC_CHANNELS;i++)
		{
            ADC_data[i] = (int) (A_EMA*cur_adc[i] + (1-A_EMA)* ADC_data[i]);
            if (CalibrateMode)
			{
				if (ADC_data[i]+ADC_Params->Calibration[1][i] < ADC_Params->Calibration[0][i]) ADC_Params->Calibration[0][i] = ADC_data[i] + ADC_Params->Calibration[1][i];
				else if (ADC_data[i]+ADC_Params->Calibration[1][i] > ADC_Params->Calibration[2][i]) ADC_Params->Calibration[2][i] = ADC_data[i] + ADC_Params->Calibration[1][i];
			}
			else
            {
                ADC_data_calibrated [i] = Calibrate (ADC_data[i],i);
            }
		}

		PPMupdate( ADC_data, &Parameters);

		vTaskDelay( ADC_READ_INTERVAL / portTICK_RATE_MS );
    }
}


//------------------------------------------------------------------------------------------------
int16_t Calibrate (int16_t val, uint8_t index)
{
	int16_t scaledVal = 0;
	scaledVal = val + ADC_Params->Calibration [1][index];
	if ((scaledVal < ADC_Params->DeadZone[index]) && (scaledVal > -ADC_Params->DeadZone[index])) scaledVal = 0;
	scaledVal += ADC_Params->Trimmers [index];

	if (scaledVal > 0)
	{
		scaledVal = map (scaledVal,0,ADC_Params->Calibration [2][index],0,ADC_Params->ScaleFactor[index]/ADC_Params->ScaleDivider[index]);
		if (scaledVal > ADC_Params->ScaleFactor[index]) scaledVal = ADC_Params->ScaleFactor[index]/ADC_Params->ScaleDivider[index]; //проверяем выход за границу диапазона

	}
	else
	{
		scaledVal = map (scaledVal, ADC_Params->Calibration [0][index],0,-ADC_Params->ScaleFactor[index]/ADC_Params->ScaleDivider[index],0);
		if (scaledVal < -ADC_Params->ScaleFactor[index]) scaledVal = -ADC_Params->ScaleFactor[index]/ADC_Params->ScaleDivider[index];//проверяем выход за границу диапазона

	}
	scaledVal *= ADC_Params->Transform[index];
	return scaledVal;
}
//------------------------------------------------------------------------------------------------
void ADC_GetZeroLevel(void)
{
	for (int i=0;i<TOTAL_CHANNELS_COUNT;i++)
	{
		ADC_Params->Calibration [1][i] = -ADC_data[i];
	}
}
//------------------------------------------------------------------------------------------------

void ADC1_2_IRQHandler (void)
{
    if (ADC1->SR & ADC_SR_EOC)
    {
        ADC1->SR &= ~(ADC_SR_EOC); //Сброс флага прерывания
        //adc_buffer[0] = ADC1->DR;


        portBASE_TYPE q_res = pdFALSE; //переменная для хранения результата постановки в очередь

        //Ставим сообщение в очередь обработчику пакета
        static portBASE_TYPE xHigherPriorityTaskWoken;

        xHigherPriorityTaskWoken = pdFALSE;
        q_res = xQueueSendToBackFromISR(adc_queue, &adc_buffer, &xHigherPriorityTaskWoken);
        // Это разблокирует задачу-обработчик. При этом приоритег задачи-обработчика выше приоритета выполняющейся в данный момент периодической задачи.
        //    Поэтому переключаем контекст принудительно - так мы добьемся того, что после выполнения обработчика прерывания управление получит задача обработчик.
        // Макрос, выполняющий переключение контекста. На других платформах имя макроса может быть другое!

        if (q_res != pdPASS) //Если очередь полная, вытаскиваем самое старое значение и кладем новое
        {
            //xQueueReceiveFromISR (adc_queue, NULL, 0);
            xQueueOverwriteFromISR(adc_queue, &adc_buffer, &xHigherPriorityTaskWoken);
        }
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE);
    }


}

//============================================================================================
void DMA1_Channel1_IRQHandler (void)//ADC DMA IRQ handler
{
  //Если обмен завершен
  if(DMA1->ISR & DMA_ISR_TCIF1)
  {
	//что-то делаем
	portBASE_TYPE q_res = pdFALSE; //переменная для хранения результата постановки в очередь

    DMA1->IFCR = DMA_IFCR_CTCIF1; //Очищаем бит прерывания
    //DMA1_Channel1->CCR  |=  DMA_CCR_EN;      //разрешить работу канала

	//Ставим сообщение в очередь обработчику пакета
	static portBASE_TYPE xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;
	q_res = xQueueSendToBackFromISR(adc_queue, &adc_buffer, &xHigherPriorityTaskWoken);
	/* Это разблокирует задачу-обработчик. При этом приоритег задачи-обработчика выше приоритета выполняющейся в данный момент периодической задачи. Поэтому переключаем контекст принудительно - так мы добьемся того, что после выполнения обработчика прерывания управление получит задача обработчик.*/
	/* Макрос, выполняющий переключение контекста. На других платформах имя макроса может быть другое! */

	if (q_res != pdPASS) //Если очередь полная, вытаскиваем самое старое значение и кладем новое
	{
		//xQueueReceiveFromISR (adc_queue, NULL, 0);
		xQueueOverwriteFromISR(adc_queue, &adc_buffer, &xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE);

    DMA1_Channel1->CCR  &=  ~DMA_CCR_EN;      //запретить работу канала
  }

  //Если передана половина буфера
  if(DMA1->ISR & DMA_ISR_HTIF1)
  {
	DMA1->IFCR |= DMA_IFCR_CHTIF1; //Очищаем бит прерывания
  }      //что-то делаем

  //Если произошла ошибка при обмене
  if(DMA1->ISR & DMA_ISR_TEIF1)
  {
	DMA1->IFCR |= DMA_IFCR_CTEIF1; //Очищаем бит прерывания
  }      //что-то делаем

  DMA1->IFCR |= DMA_IFCR_CGIF1; //очищаем бит глобального прерывания
}

//============================================================================================

void ADC_read (void)
{
	// Start the conversion
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	DMA1_Channel1->CNDTR =  ADC_CHANNELS;          //количество данных для обмена
	DMA1_Channel1->CCR  |=  DMA_CCR_EN;      //разрешить работу канала
	ADC1->CR2 |= ADC_CR2_SWSTART;
}
//********************************************************************************
//Function: инициализация DMA для работы с ADC 						          //
//********************************************************************************
void ADC_DMA_Init(void)
{
 //Включить тактирование DMA1
 if ((RCC->AHBENR & RCC_AHBENR_DMA1EN) != RCC_AHBENR_DMA1EN)
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
 //Задать адрес источника и приемника и количество данных для обмена
 DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;   //адрес регистра перефирии
 DMA1_Channel1->CMAR  = (uint32_t)adc_buffer;   	//адрес буфера в памяти
 DMA1_Channel1->CNDTR =  ADC_CHANNELS;             //количество данных для обмена
 //----------------- Манипуляции с регистром конфигурации  ----------------
 //Следующие действия можно обьединить в одну команду (разбито для наглядности)
 DMA1_Channel1->CCR   =  0;									//предочистка регистра конфигурации
 DMA1_Channel1->CCR  &= (uint16_t) (~DMA_CCR_CIRC);		//выключить циклический режим
 //DMA1_Channel1->CCR  |= DMA_CCR_CIRC;						//включить циклический режим
 DMA1_Channel1->CCR  &= (uint16_t) (~DMA_CCR_DIR);         //направление: чтение в память
 //Настроить работу с переферийным устройством
 //DMA1_Channel2->CCR  &= (uint16_t)(~DMA_CCR7_PSIZE); 		//размерность данных 8 бит
 DMA1_Channel1->CCR   |= DMA_CCR_PSIZE_1;          		//размерность данных 32 бит
 DMA1_Channel1->CCR   &= (uint16_t)(~DMA_CCR_PINC);		//не использовать инкремент указателя
 //Настроить работу с памятью
 DMA1_Channel1->CCR   |= DMA_CCR_MSIZE_1;					//размерность данных 32 бит
 DMA1_Channel1->CCR  |=  DMA_CCR_MINC;						//использовать инкремент указателя

 //Разрешить прерывание по завершении обмена:
 DMA1_Channel1->CCR |= DMA_CCR_TCIE;						//канал 1
 NVIC_EnableIRQ (DMA1_Channel1_IRQn);						//Разрешить прерывания от DMA
 NVIC_SetPriority (DMA1_Channel1_IRQn, 15);
 ADC1->CR2         |=  ADC_CR2_DMA;							//разрешить передачу ADC через DMA
 //DMA1_Channel1->CCR  |=  DMA_CCR_EN;      //разрешить работу канала
}

void ADC_InitProc (ADC_Parameters * params)
{

    ADC_Params = params;

	adc_queue = xQueueCreate (ADC_QUEUE_LEN,sizeof(adc_buffer));

	//RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1, ENABLE );

	RCC->APB2ENR	|= RCC_APB2ENR_IOPAEN;                    //включаю тактирование порта (если не включали ранее)
	RCC->APB2ENR	|= RCC_APB2ENR_IOPBEN;                    //включаю тактирование порта (если не включали ранее)
	RCC->APB2ENR	|= RCC_APB2ENR_ADC1EN;                    //тактирование ADC1

/* GPIO configuration ------------------------------------------------------*/
	//GPIOA->CRL  &= ~(GPIO_CRL_MODE0 | GPIO_CRL_MODE1 | GPIO_CRL_MODE2 | GPIO_CRL_MODE3 | GPIO_CRL_MODE4 | GPIO_CRL_MODE5 | GPIO_CRL_MODE6 | GPIO_CRL_MODE7);  //00: Input mode (reset state)
	//GPIOA->CRL  &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1 | GPIO_CRL_CNF2 | GPIO_CRL_CNF3 | GPIO_CRL_CNF4 | GPIO_CRL_CNF5 | GPIO_CRL_CNF6 | GPIO_CRL_CNF7);   //00: Analog mode
	GPIOA->CRL  &= ~(GPIO_CRL_MODE0 | GPIO_CRL_MODE1 | GPIO_CRL_MODE2 | GPIO_CRL_MODE3 | GPIO_CRL_MODE4 | GPIO_CRL_MODE5 | GPIO_CRL_MODE6 | GPIO_CRL_MODE7);  //00: Input mode (reset state)
	GPIOA->CRL  &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1 | GPIO_CRL_CNF2 | GPIO_CRL_CNF3 | GPIO_CRL_CNF4 | GPIO_CRL_CNF5 | GPIO_CRL_CNF6 | GPIO_CRL_CNF7);   //00: Analog mode

	GPIOB->CRL  &= ~(GPIO_CRL_MODE0 | GPIO_CRL_MODE1);  //00: Input mode (reset state)
	GPIOB->CRL  &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1);   //00: Analog mode

/* ADC1 configuration ------------------------------------------------------*/
    ADC1->CR1  &= ~(ADC_CR1_DUALMOD);   //0000: Independent mode.
    ADC1->CR1  |= ADC_CR1_SCAN;         //1: Scan mode enabled
    ADC1->CR2  &= ~(ADC_CR2_CONT);      //0: Single conversion mode
    //ADC1->CR2  &= ~(ADC_CR2_EXTTRIG);   //0: Conversion on external event disabled
    ADC1->CR2  &= ~(ADC_CR2_ALIGN);     //0: Right Alignment
    ADC1->CR2  |= ADC_CR2_EXTTRIG;      //External trigger conversion mode for regular channels
    ADC1->CR2  |= ADC_CR2_EXTSEL;       //External event select for regular group 111: SWSTART

/* ADC channels sequence configuration */
    //ADC1->SQR1 |=ADC_SQR1_L_3;//Regular channel sequence length = 8
    ADC1->SQR1 |= ADC_SQR1_L_0 | ADC_SQR1_L_3;//Regular channel sequence length = 10.  1 conversion  = 0000 -0
                                                                                    //2  conversions = 0001 -1
                                                                                    //6  conversions = 0101 -5
                                                                                    //10 conversions = 1001 -9

    ADC1->SQR3 &= ~(ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_1 | ADC_SQR3_SQ1_2 | ADC_SQR3_SQ1_3); //Set ch 0 as 1-t  in sequence
    ADC1->SQR3 |=ADC_SQR3_SQ2_0;                                                        //Set ch 1 as 2-d  in sequence
    ADC1->SQR3 |=ADC_SQR3_SQ3_1;                                                        //Set ch 2 as 3-d  in sequence
    ADC1->SQR3 |=ADC_SQR3_SQ4_0 | ADC_SQR3_SQ4_1;                                       //Set ch 3 as 4-th in sequence
    ADC1->SQR3 |=ADC_SQR3_SQ5_2;                                                        //Set ch 4 as 5-th in sequence
    ADC1->SQR3 |=ADC_SQR3_SQ6_2 | ADC_SQR3_SQ6_0;                                       //Set ch 5 as 6-th in sequence
    ADC1->SQR2 |=ADC_SQR2_SQ7_2 | ADC_SQR2_SQ7_1;                                       //Set ch 6 as 7-th in sequence
    ADC1->SQR2 |=ADC_SQR2_SQ8_2 | ADC_SQR2_SQ8_1 | ADC_SQR2_SQ8_0;                      //Set ch 7 as 8-th in sequence
    ADC1->SQR2 |=ADC_SQR2_SQ9_3;                                                        //Set ch 8 as 9-th in sequence
    ADC1->SQR2 |=ADC_SQR2_SQ10_3 | ADC_SQR2_SQ10_0;                                     //Set ch 9 as 10-th in sequence


/* ADC regular channels configuration */
    //ADC1->SMPR2 |= (ADC_SMPR2_SMP0_0|ADC_SMPR2_SMP0_2); //Sample time selection 101: 55.5 cycles
    /*
    ADC1->SMPR2 |= (ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_2); //Sample time selection 111: 239.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP2_0 | ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_2); //Sample time selection 111: 239.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP3_0 | ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_2); //Sample time selection 111: 239.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP4_0 | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_2); //Sample time selection 111: 239.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP5_0 | ADC_SMPR2_SMP5_1 | ADC_SMPR2_SMP5_2); //Sample time selection 111: 239.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP6_0 | ADC_SMPR2_SMP6_1 | ADC_SMPR2_SMP6_2); //Sample time selection 111: 239.5 cycles
    //ADC1->SMPR2 |= (ADC_SMPR2_SMP7_0|ADC_SMPR2_SMP7_2); //Sample time selection 101: 55.5 cycles
    */
    ADC1->SMPR2 |= (ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_2); //Sample time selection 101: 55.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_2); //Sample time selection 101: 55.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP2_0 | ADC_SMPR2_SMP2_2); //Sample time selection 101: 55.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP3_0 | ADC_SMPR2_SMP3_2); //Sample time selection 101: 55.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP4_0 | ADC_SMPR2_SMP4_2); //Sample time selection 101: 55.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP5_0 | ADC_SMPR2_SMP5_2); //Sample time selection 101: 55.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP6_0 | ADC_SMPR2_SMP6_2); //Sample time selection 101: 55.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP7_0 | ADC_SMPR2_SMP7_2); //Sample time selection 101: 55.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP8_0 | ADC_SMPR2_SMP8_2); //Sample time selection 101: 55.5 cycles
    ADC1->SMPR2 |= (ADC_SMPR2_SMP9_0 | ADC_SMPR2_SMP9_2); //Sample time selection 101: 55.5 cycles

/* Enable ADC1  */
	//ADC1->CR1 |= ADC_CR1_EOCIE; //Включаем прерывание по окончании преобразования
    //NVIC_EnableIRQ (ADC1_2_IRQn);
    //NVIC_SetPriority (ADC1_2_IRQn, 15);




    ADC1->CR2 |= ADC_CR2_ADON;  //Включаем ADC




/* Start calibration */
    ADC1->CR2 |= ADC_CR2_CAL;
/* Wait for the end of calibration  */
    while (ADC1->CR2 & ADC_CR2_CAL);

    ADC_DMA_Init();

    xTaskCreate( ADC_Reader, "ADC_Reader", configMINIMAL_STACK_SIZE, NULL, ADC_Reader_PRIORITY, NULL );
}
