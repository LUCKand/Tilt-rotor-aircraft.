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
#include "stm32f1xx_hal.h"
#include "sbus.h"
#include "usart.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//extern uint8_t data[25];  
extern uint8_t txdata[25];
//extern uint16_t rcdata;
#ifdef TEST22
extern uint8_t data[22];  
#endif

#ifdef TEST23
extern uint8_t data[23];  
#endif

#ifdef TEST25
extern uint8_t data[25];  
#endif

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
//      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	  osDelay(100);
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
osSemaphoreWait(_sbustxHandle, 1);   
//HAL_UART_Transmit_IT(&huart2,txdata,SBUS_FRAME_LENGH);	
	
//	#ifdef UART1_
//    HAL_UART_Transmit(&huart1,txdata,SBUS_FRAME_LENGH,10);	
//	#endif
//	
//	#ifdef UART1_IT
//	HAL_UART_Transmit_IT(&huart1,txdata,SBUS_FRAME_LENGH);	
//	#endif
//	
//	#ifdef UART1_DMA
//    HAL_UART_Transmit_DMA(&huart1,txdata,SBUS_FRAME_LENGH);
//	
//	#endif
//	
//	#ifdef UART2_
//	HAL_UART_Transmit(&huart2,txdata,SBUS_FRAME_LENGH,10);	

//	#endif
//	
//	#ifdef UART2_IT
//	HAL_UART_Transmit_IT(&huart2,txdata,SBUS_FRAME_LENGH);	
//	#endif
//	
//	#ifdef UART1_DMA
//    HAL_UART_Transmit_DMA(&huart2,txdata,SBUS_FRAME_LENGH);    	
//	#endif
//	
	
	

	for(;;)
  {     		
//	if ( HAL_UART_Transmit_IT(&huart2,txdata,SBUS_FRAME_LENGH)==HAL_BUSY)
//{
//	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
// }  

// if (HAL_UART_Transmit_DMA(&huart2,txdata,SBUS_FRAME_LENGH)==HAL_BUSY)
// {
//	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
// } 
 
	  osDelay(1);
  }
//  
//      
//	  HAL_UART_Transmit_IT(&huart2,txdata,SBUS_FRAME_LENGH);
//	  if ( HAL_UART_Transmit_IT(&huart2,txdata,SBUS_FRAME_LENGH)==HAL_BUSY)
//{
// 
// }
  /* USER CODE END Do_sbustx */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	if(huart == &huart1)         
	{
//		HAL_UART_Receive_DMA(&huart1, data, 25u);
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

#ifdef TEST22
  HAL_UART_Receive_DMA(&huart1, data, 22u);             
#endif

#ifdef TEST23
  HAL_UART_Receive_DMA(&huart1, data, 23u);              
#endif

#ifdef TEST25
  HAL_UART_Receive_DMA(&huart1, data, 25u);             
#endif		
			
		RXEncoder();
		TXEncoder();
        osSemaphoreRelease(_sbustxHandle);
		
	#ifdef UART1_
    HAL_UART_Transmit(&huart1,txdata,SBUS_FRAME_LENGH,10);	
	#endif
	
	#ifdef UART1_IT
	HAL_UART_Transmit_IT(&huart1,txdata,SBUS_FRAME_LENGH);	
	#endif
	
	#ifdef UART1_DMA
    HAL_UART_Transmit_DMA(&huart1,txdata,SBUS_FRAME_LENGH);
	
	#endif
	
	#ifdef UART2_
	HAL_UART_Transmit(&huart2,txdata,SBUS_FRAME_LENGH,10);	

	#endif
	
	#ifdef UART2_IT
	HAL_UART_Transmit_IT(&huart2,txdata,SBUS_FRAME_LENGH);	
	#endif
	
	#ifdef UART2_DMA
//    HAL_UART_Transmit_DMA(&huart2,txdata,sizeof(txdata));    	
    HAL_UART_Transmit_DMA(&huart2,txdata,sizeof(txdata));    	
	
	#endif
	
//		HAL_UART_Transmit_IT(&huart2,txdata,SBUS_FRAME_LENGH);

		
  }

}




/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
