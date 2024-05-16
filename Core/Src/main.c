/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define OCTAVE1 1
#define OCTAVE2 2
#define OCTAVE3 4
#define OCTAVE4 8
#define OCTAVE5 16
#define OCTAVE6 32
#define OCTAVE7 64
#define OCTAVE8 128

typedef enum{
	C = 30581/OCTAVE4,
	D = 27247/OCTAVE4,
	E = 24271/OCTAVE4,
	F = 22909/OCTAVE4,
	G = 20408/OCTAVE4,
	A = 18181/OCTAVE4,
	B = 16198/OCTAVE4,
	C_= 15289/OCTAVE4
}pitch_arr;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart_send_data[64];
uint8_t uart_rx_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void uart_send(uint8_t uart_num,char *fmt,...);
void buzzer_control();
void led_pwm_control(uint32_t *pretime,uint16_t *led_pwm_speed);
void servo_control(int16_t *servo_tick);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance){
		UART_Start_Receive_DMA(&huart1, &uart_rx_data, 1);
	}
  UNUSED(huart);
}

void uart_send(uint8_t uart_num,char *fmt,...){
	va_list arg;
	va_start(arg,fmt);

	vsnprintf((char *)uart_send_data,64,fmt,arg);

	if(uart_num==1){
		HAL_UART_Transmit(&huart1, uart_send_data, 64, 10);
	}

	memset(uart_send_data,0,64);
	va_end(arg);
}

uint32_t pretime1;
bool buzzer_on=false;

void buzzer_control(){
	if(!buzzer_on){
		pretime1 = HAL_GetTick();
	}
	if((HAL_GetTick()-pretime1>=100)&&buzzer_on){
		pretime1=HAL_GetTick();
		htim9.Instance->CCR1 = 0;
		buzzer_on = false;
	}
	if(buzzer_on){
		return;
	}
	switch (uart_rx_data){
	case 'q':
		htim9.Instance->ARR = C;
		htim9.Instance->CCR1 = C/2;
		buzzer_on = true;
		uart_send(1,"C\n");
		break;
	case 'w':
		htim9.Instance->ARR = D;
		htim9.Instance->CCR1 = D/2;
		uart_send(1,"D\n");
		buzzer_on = true;
		break;
	case 'e':
		htim9.Instance->ARR = E;
		htim9.Instance->CCR1 = E/2;
		uart_send(1,"E\n");
		buzzer_on = true;
		break;
	case 'r':
		htim9.Instance->ARR = F;
		htim9.Instance->CCR1 = F/2;
		uart_send(1,"F\n");
		buzzer_on = true;
		break;
	case 't':
		htim9.Instance->ARR = G;
		htim9.Instance->CCR1 = G/2;
		uart_send(1,"G\n");
		buzzer_on = true;
		break;
	case 'y':
		htim9.Instance->ARR = A;
		htim9.Instance->CCR1 = A/2;
		uart_send(1,"A\n");
		buzzer_on = true;
		break;
	case 'u':
		htim9.Instance->ARR = B;
		htim9.Instance->CCR1 = B/2;
		uart_send(1,"B\n");
		buzzer_on = true;
		break;
	case 'i':
		htim9.Instance->ARR = C_;
		htim9.Instance->CCR1 = C_/2;
		uart_send(1,"C+\n");
		buzzer_on = true;
		break;
	default:
		break;
	}
}

void servo_control(int16_t *servo_tick){
	htim4.Instance->CCR1 = *servo_tick;
	if(uart_rx_data=='a'){
		if(*servo_tick<=140){
			uart_send(1,"servo minus degree is max!\n");
			*servo_tick=1300;
			return;
		}
		*servo_tick-=20;
		uart_send(1,"servo minus:%dtick,%d.%dms,%d'\n",
				*servo_tick,
				(*servo_tick/500),(*servo_tick/50)-(*servo_tick/500)*10,
				(*servo_tick-140)*180/1180);
	}
	else if(uart_rx_data=='s'){
		if(*servo_tick>=1320){
			uart_send(1,"servo plus degree is max!\n");
			*servo_tick=140;
			return;
		}
		*servo_tick+=20;
		uart_send(1,"servo plus:%dtick,%d.%dms,%d'\n",
				*servo_tick,
				(*servo_tick/500),(*servo_tick/50)-(*servo_tick/500)*10,
				(*servo_tick-140)*180/1180);
	}
}


void led_pwm_control(uint32_t *pretime,uint16_t *led_pwm_speed){
	uint16_t led_pwm_cal_val=500;
	if(HAL_GetTick()-*pretime>=50){
		htim3.Instance->CCR1=(htim3.Instance->CCR1+led_pwm_speed[0])%htim3.Init.Period;
		htim3.Instance->CCR2=(htim3.Instance->CCR2+led_pwm_speed[1])%htim3.Init.Period;
		*pretime=HAL_GetTick();
	}

	switch (uart_rx_data){
		case '1': //up led 1
			if(led_pwm_speed[0]<led_pwm_cal_val*4){
				led_pwm_speed[0]=led_pwm_speed[0]+led_pwm_cal_val;
				uart_send(1,"led 1 up speed, speed val : %d.\n",led_pwm_speed[0]);
			}
			else{
				uart_send(1,"led 1 pwm speed %d is too fast!\n",led_pwm_speed[0]);
			}
			break;
		case '2': //down led 1
			if(led_pwm_speed[0]>led_pwm_cal_val){
				led_pwm_speed[0]= led_pwm_speed[0]-led_pwm_cal_val;
				uart_send(1,"led 1 down speed, speed val : %d.\n",led_pwm_speed[0]);
			}
			else{
				uart_send(1,"led 1 pwm speed %d is too slow!\n",led_pwm_speed[0]);
			}
		  break;
		case '3': //up led 1
		  if(led_pwm_speed[1]<led_pwm_cal_val*4){
		   led_pwm_speed[1]=led_pwm_speed[1]+led_pwm_cal_val;
		   uart_send(1,"led 2 up speed, led speed val : %d.\n",led_pwm_speed[1]);
		  }
		  else{
		  	uart_send(1,"led 2 pwm speed %d is too fast!\n",led_pwm_speed[1]);
		  }
		  break;
		case '4': //down led 1
		  if(led_pwm_speed[1]>led_pwm_cal_val){
		  	led_pwm_speed[1]= led_pwm_speed[1]-led_pwm_cal_val;
		  	uart_send(1,"led 2 down speed, led speed val : %d.\n",led_pwm_speed[1]);
		  }
		  else{
		  	uart_send(1,"led 2 pwm speed %d is too slow!\n",led_pwm_speed[1]);
		  }
		  break;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  UART_Start_Receive_DMA(&huart1, &uart_rx_data, 1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
