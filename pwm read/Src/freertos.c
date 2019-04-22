/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

CH_Pwm_Val_t CH_Pwm_Val[MAX_RC_CH_NB];
uint8_t i;
uint16_t Sbus_CH[16]  	= {1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025};
TIM_IC_InitTypeDef sConfigIC;

#define TIMER_MAX_VAL 65535

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId lloop1Handle;
osThreadId sbustxHandle;
osSemaphoreId _sbustxHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void loop1(void const * argument);
void Do_sbustx(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of _sbustx */
  osSemaphoreDef(_sbustx);
  _sbustxHandle = osSemaphoreCreate(osSemaphore(_sbustx), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of lloop1 */
  osThreadDef(lloop1, loop1, osPriorityNormal, 0, 128);
  lloop1Handle = osThreadCreate(osThread(lloop1), NULL);

  /* definition and creation of sbustx */
  osThreadDef(sbustx, Do_sbustx, osPriorityIdle, 0, 128);
  sbustxHandle = osThreadCreate(osThread(sbustx), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_loop1 */
/**
  * @brief  Function implementing the lloop1 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_loop1 */
void loop1(void const * argument)
{

  /* USER CODE BEGIN loop1 */
  /* Infinite loop */
  for(;;)
	{
	for(i=0;i<8;i++)
  {
   Sbus_CH[i] = ((CH_Pwm_Val[i].Delta / PWM_Ratio) - 880) / 0.625f;
   CH_Pwm_Val[i].CH_Detected = CH_NOT_DETECTED; 	  	
   osDelay(1);
  }
  }
  /* USER CODE END loop1 */
}

/* USER CODE BEGIN Header_Do_sbustx */
/**
* @brief Function implementing the sbustx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Do_sbustx */
void Do_sbustx(void const * argument)
{
  /* USER CODE BEGIN Do_sbustx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Do_sbustx */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint8_t Ch_Idx;
	uint8_t Ch_Ct_Idx;

	/*****************************************
	 * 				CH 1 to 8 sampling
	 *****************************************/

	Ch_Idx 		= htim->Channel >> 1;

	switch(htim->Channel)
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
			Ch_Idx 		= 0;
			Ch_Ct_Idx 	= TIM_CHANNEL_1;
		break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
			Ch_Idx 		= 1;
			Ch_Ct_Idx 	= TIM_CHANNEL_2;
		break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			Ch_Idx 		= 2;
			Ch_Ct_Idx 	= TIM_CHANNEL_3;
		break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
			Ch_Idx 		= 3;
			Ch_Ct_Idx 	= TIM_CHANNEL_4;
		break;
	}

	if(htim->Instance == TIM2)
		Ch_Idx += TIMER_1_CH_NB;
	else if(htim->Instance == TIM3)
		Ch_Idx += TIMER_1_CH_NB + TIMER_2_CH_NB;

	// Raising edge
		if(CH_Pwm_Val[Ch_Idx].Current_Edge == RAISING)
		{
			// Set Edge detect flag
			CH_Pwm_Val[Ch_Idx].CH_Detected =  CH_DETECTED;

			// Set IC polarity to Falling
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
			HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, Ch_Ct_Idx);
			CH_Pwm_Val[Ch_Idx].Current_Edge = FALLING;

			// Store raising edge timer value
			CH_Pwm_Val[Ch_Idx].Rising = HAL_TIM_ReadCapturedValue(htim,Ch_Ct_Idx);
		}
		// Falling edge
		else{
			// Set IC polarity to Raising
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
			HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, Ch_Ct_Idx);
			CH_Pwm_Val[Ch_Idx].Current_Edge = RAISING;

			// Store falling edge timer value
			CH_Pwm_Val[Ch_Idx].Falling = HAL_TIM_ReadCapturedValue(htim,Ch_Ct_Idx);

			// Compute delta value between raising and falling edge
			if(CH_Pwm_Val[Ch_Idx].Rising < CH_Pwm_Val[Ch_Idx].Falling)
				CH_Pwm_Val[Ch_Idx].Delta = CH_Pwm_Val[Ch_Idx].Falling - CH_Pwm_Val[Ch_Idx].Rising;
			else
				CH_Pwm_Val[Ch_Idx].Delta = (TIMER_MAX_VAL - CH_Pwm_Val[Ch_Idx].Rising) + CH_Pwm_Val[Ch_Idx].Falling + 1;
		}

		// Start IC interrupt after polarity inversion
		HAL_TIM_IC_Start_IT(htim,Ch_Ct_Idx);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
