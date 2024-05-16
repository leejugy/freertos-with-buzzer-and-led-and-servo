/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t uart_rx_data;
extern void buzzer_control();
extern void led_pwm_control(uint32_t *pretime,uint16_t *led_pwm_speed);
extern void servo_control(int16_t *servo_tick);
extern void uart_send(uint8_t uart_num,char *fmt,...);
/* USER CODE END Variables */
/* Definitions for servo */
osThreadId_t servoHandle;
const osThreadAttr_t servo_attributes = {
  .name = "servo",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for buzzer */
osThreadId_t buzzerHandle;
const osThreadAttr_t buzzer_attributes = {
  .name = "buzzer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for led */
osThreadId_t ledHandle;
const osThreadAttr_t led_attributes = {
  .name = "led",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of servo */
  servoHandle = osThreadNew(StartDefaultTask, NULL, &servo_attributes);

  /* creation of buzzer */
  buzzerHandle = osThreadNew(StartTask02, NULL, &buzzer_attributes);

  /* creation of led */
  ledHandle = osThreadNew(StartTask03, NULL, &led_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the servo thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	int16_t servo_tick = 140;
  /* Infinite loop */
  for(;;)
  {
  	servo_control(&servo_tick);
  	if(uart_rx_data=='a'||uart_rx_data=='s'){
  		uart_rx_data = 0;
  	}
		osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
  	buzzer_control();
  	if(uart_rx_data=='q'||uart_rx_data=='w'||uart_rx_data=='e'
  			||uart_rx_data=='r'||uart_rx_data=='t'||uart_rx_data=='y'
  			||uart_rx_data=='u'||uart_rx_data=='i'){
  		uart_rx_data = 0;
  	}
  	osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	uint32_t pretime=HAL_GetTick();
	uint16_t led_fisrt_pwm_speed[2]={500,500};
  /* Infinite loop */
  for(;;)
  {
  	led_pwm_control(&pretime, &led_fisrt_pwm_speed[0]);
  	if(uart_rx_data=='1'||uart_rx_data=='2'
  			||uart_rx_data=='3'||uart_rx_data=='4'){
  		uart_rx_data = 0;
  	}
  	osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

