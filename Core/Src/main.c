/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int8_t Channel;
	uint8_t Mode;
	uint8_t State;
} ADC_Params_t;

typedef struct {
	uint16_t last_RAW;
	double last_Volt;
	double avg_Volt;
	double rms_Volt;
} output_data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STAT_IDLE 0
#define STAT_MEASURE 1
#define STAT_ERROR 2

#define ADC_MODE_NONE 0
#define ADC_MODE_AVG 1
#define ADC_MODE_RMS 2
#define ADC_STATE_ENABLE 1
#define ADC_STATE_DISABLE 0

#define DEBUG_N
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId Data_receiveHandle;
osThreadId Led_statusHandle;
/* USER CODE BEGIN PV */
ADC_Params_t ADC_Params;
output_data_t ch0;
output_data_t ch1;
output_data_t ch2;
output_data_t ch3;

uint16_t ValueADC = 0;
uint8_t Status = STAT_IDLE; // voltmeter state
uint8_t TxDATA[24];
uint8_t RxDATA[24];
char buffer[64];
uint32_t Time;
uint8_t msg = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void Data_Receive_handler(void const * argument);
void Led_status_handler(void const * argument);
#ifdef DEBUG_N
void send_msg(char* msg);
#endif
void StatusTX();
void ValueTX(char* str_data, double data, uint8_t decimals);
void printNumber(char* str, uint32_t data);
void addChar(char* str, char ch);
void ReceiveDATA();
void Parsing();
void ADC_Routine(ADC_Params_t* params);
void ComputeVoltage(output_data_t* channel);
double avg(double new);
double rms(double new);
void TransmitMSG(char* msg);
void TransmitVOLTAGE(ADC_Params_t* params);
void Flush(void* data);
void VoltmeterInit();
void LED_SET(int8_t state);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	VoltmeterInit();
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Data_receive */
  osThreadDef(Data_receive, Data_Receive_handler, osPriorityNormal, 0, 128);
  Data_receiveHandle = osThreadCreate(osThread(Data_receive), NULL);

  /* definition and creation of Led_status */
  osThreadDef(Led_status, Led_status_handler, osPriorityIdle, 0, 128);
  Led_statusHandle = osThreadCreate(osThread(Led_status), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void StatusTX() {
	if (Status == STAT_IDLE) {
		HAL_UART_Transmit(&huart1, (uint8_t*) "Idle\n\r", 6, 100);
	} else if (Status == STAT_MEASURE) {
		HAL_UART_Transmit(&huart1, (uint8_t*) "Measuring...\n\r", 14, 100);
	} else if (Status == STAT_ERROR) {
		HAL_UART_Transmit(&huart1, (uint8_t*) "Error\n\r", 7, 100);
	}
}

void ValueTX(char* str_data, double data, uint8_t decimals) {
	if (data < 0) {
		addChar(str_data, '-'); //symbol -
		data = -data;
	}
	uint32_t integer = data;
	printNumber(str_data, integer);
	addChar(str_data, '.');; //symbol .
	data -= integer;
	for (uint8_t i = 0; i < decimals; i++) {
		data *= 10.0;
		printNumber(str_data, (uint8_t) data);
		data -= (uint8_t) data;
	}
}

void printNumber(char* str, uint32_t data) {
	int8_t bytes[10];
	uint8_t amount;
	for (uint8_t i = 0; i < 10; i++) {
		bytes[i] = data % 10;
		data /= 10;
		if (data == 0) {
			amount = i;
			break;
		}
	}
	for (int8_t i = amount; i >= 0; i--) {
		addChar(str, bytes[i] + '0');
	}
}

void addChar(char* str, char ch) {
	uint8_t i = 0;
	for (i = 0; str[i]!='\0';i++) {

	}
	str[i] = ch;
	str[i+1] = '\0';
}

void ReceiveDATA() {
	Flush(RxDATA);
	HAL_UART_Receive(&huart1, RxDATA, sizeof(RxDATA), 10);
	Parsing();
}

void Parsing()
{
	int validate = 0;
	//start measuring
	if (strstr((char*)RxDATA, "start")) {
		validate++;
		ADC_Params.State = ADC_STATE_ENABLE;
		char* ptr;
		if (ptr = strstr((char*) RxDATA, "ch")) {
			ADC_Params.Channel = atoi(ptr+2);
			validate++;
		}
		if (strstr((char*) RxDATA, "none")) {
			ADC_Params.Mode = ADC_MODE_NONE;
			validate++;
		} else if (strstr((char*) RxDATA, "avg")) {
			ADC_Params.Mode = ADC_MODE_AVG;
			validate++;
		} else if (strstr((char*) RxDATA, "rms")) {
			ADC_Params.Mode = ADC_MODE_RMS;
			validate++;
		}

		if (validate == 3){
			TransmitMSG("OK");
		}
		else
			TransmitMSG("Fail");
	}
	//stop measuring
	else if (strstr((char*)RxDATA, "stop")) {
		if (ADC_Params.State == ADC_STATE_ENABLE) {
			ADC_Params.State = ADC_STATE_DISABLE;
			TransmitMSG("->>-");
			Status = STAT_IDLE;
		}
		else
			TransmitMSG("Fail");
	}

	//get result
	else if (strstr((char*)RxDATA, "result")) {
		uint8_t last_ch = ADC_Params.Channel;
		validate ++;
		char *ptr;
		if (ptr = strstr((char*) RxDATA, "ch")) {
			ADC_Params.Channel = atoi(ptr + 2);
			validate++;
		}
		if (validate == 2) {
			TransmitVOLTAGE(&ADC_Params);
		}
		else
			TransmitMSG("Fail");
		ADC_Params.Channel = last_ch;
	}
	//get state
	else if (strstr((char*)RxDATA, "status")) {
		if (Status == STAT_IDLE) {
			TransmitMSG("Idle");
		} else if (Status == STAT_MEASURE) {
			TransmitMSG("Measuring...");
		} else if (Status == STAT_ERROR) {
			TransmitMSG("Error");
		}
	}
	ADC_Routine(&ADC_Params);
	Flush((char*)RxDATA);
}

void ADC_Routine(ADC_Params_t* params)
{
	static bool ADCstarted = false;
	if(params->State == ADC_STATE_ENABLE && !ADCstarted){
		if (HAL_ADC_Start(&hadc1) == HAL_OK) {
			Status = STAT_MEASURE;
			ADCstarted = true;
		} else
			Status = STAT_ERROR;

	}
	else if(params->State == ADC_STATE_DISABLE){
		if (HAL_ADC_Stop(&hadc1) == HAL_OK)
			ADCstarted = false;
		else
			Status = STAT_ERROR;
	}

	if (ADCstarted) {
		switch (params->Channel) {
		case 0:
			ComputeVoltage(&ch0);
			break;
		case 1:
			ComputeVoltage(&ch1);
			break;
		case 2:
			ComputeVoltage(&ch2);
			break;
		case 3:
			ComputeVoltage(&ch3);
			break;
		default:
			break;
		}
	}

}

void ComputeVoltage(output_data_t* channel){

	uint16_t lastRAW = 0;
	if (channel == &ch0) {
		lastRAW = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	} else if (channel == &ch1) {
		lastRAW = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	} else if (channel == &ch2) {
		lastRAW = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
	} else if (channel == &ch3) {
		lastRAW = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
	}
	channel->last_RAW = lastRAW;
	double last_V = 3.3/4096*lastRAW;
	channel->last_Volt = last_V;
	channel->avg_Volt = avg(last_V);
	channel->rms_Volt = rms(last_V);
}

double avg(double new) {
	static int t = 1;
	static double old = 0.0;
	static double summ = 0.0;
	double AVG = 0;
	if (t == 50) {
		summ = summ - old + new;
		AVG = summ/t;
		old = new;
		return AVG;
	}
	summ += new;
	AVG = summ/t;
	t++;
	old = new;
	return AVG;
}

double rms(double new) {
	static double old = 0.0;
    double RMS = 0.0;
    new = pow(new,2);
	static int t = 1;
	static double summ = 0.0;
	if (t == 50) {
		summ = summ - old + new;
		RMS = sqrt((double)summ/t);
		old = new;
		return RMS;
	}
	summ += new;
	RMS = sqrt(summ/t);
	t++;
	old = new;
	Time = HAL_GetTick();
}

void send_msg(char* msg) {
	if (strlen(msg) < 64 && strcmp(msg, RxDATA) != 0) {
		Flush(RxDATA);
		strcpy((char*)RxDATA, msg);
	}

}

void LED_SET(int8_t state) {
	if (state == 0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	} else if (state == 1) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}else if (state == -1) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}

void TransmitMSG(char* msg) {
#ifdef DEBUG_N
	Flush(TxDATA);
	strcpy((char*)TxDATA, msg);
#endif
	HAL_UART_Transmit(&huart1, msg, strlen(msg), 20);
}

void TransmitVOLTAGE(ADC_Params_t* params) {
	Flush(buffer);
	if (params->Mode == ADC_MODE_NONE) {
		switch (params->Channel) {
			case 0:
			ValueTX(buffer, ch0.last_Volt, 4);
			break;
		case 1:
			ValueTX(buffer, ch1.last_Volt, 4);
			break;
		case 2:
			ValueTX(buffer, ch2.last_Volt, 4);
			break;
		case 3:
			ValueTX(buffer, ch3.last_Volt, 4);
			break;
		default:
			break;
		}
	}
	else if (params->Mode == ADC_MODE_AVG) {
		switch (params->Channel) {
			case 0:
			ValueTX(buffer, ch0.avg_Volt, 4);
			break;
		case 1:
			ValueTX(buffer, ch1.avg_Volt, 4);
			break;
		case 2:
			ValueTX(buffer, ch2.avg_Volt, 4);
			break;
		case 3:
			ValueTX(buffer, ch3.avg_Volt, 4);
			break;
		default:
			break;
		}
	}
	else if (params->Mode == ADC_MODE_RMS) {
		switch (params->Channel) {
			case 0:
			ValueTX(buffer, ch0.rms_Volt, 4);
			break;
		case 1:
			ValueTX(buffer, ch1.rms_Volt, 4);
			break;
		case 2:
			ValueTX(buffer, ch2.rms_Volt, 4);
			break;
		case 3:
			ValueTX(buffer, ch3.rms_Volt, 4);
			break;
		default:
			break;
		}
	}
	TransmitMSG(buffer);
}

void Flush(void* data) {
	memset(data, 0, strlen(data));
}

void VoltmeterInit() {
	ADC_Params.Channel = -1;
	ADC_Params.Mode = ADC_MODE_NONE;
	ADC_Params.State = ADC_STATE_DISABLE;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		if (msg == 1) {
			send_msg("stop");
		} else if (msg == 2) {
			send_msg("start ch0 rms");
		} else if (msg == 3) {
			send_msg("result ch1");
		} else if (msg == 4) {
			send_msg("result ch0");
		} else if (msg == 5) {
			send_msg("status");
		}
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Data_Receive_handler */
/**
* @brief Function implementing the Data_receive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Data_Receive_handler */
void Data_Receive_handler(void const * argument)
{
  /* USER CODE BEGIN Data_Receive_handler */
  /* Infinite loop */
  for(;;)
  {
	  ReceiveDATA();
	  osDelay(1);
  }
  /* USER CODE END Data_Receive_handler */
}

/* USER CODE BEGIN Header_Led_status_handler */
/**
* @brief Function implementing the Led_status thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Led_status_handler */
void Led_status_handler(void const * argument)
{
  /* USER CODE BEGIN Led_status_handler */
	uint64_t timEn = HAL_GetTick();
	uint64_t timDis = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
		if (Status == STAT_IDLE) {
			if (HAL_GetTick() - timEn >= 50) {
				LED_SET(0);
				timEn = HAL_GetTick();
			}
			if (HAL_GetTick() - timDis >= 9950) {
				LED_SET(1);
				timDis = HAL_GetTick();
			}

		} else if (Status == STAT_MEASURE) {
			LED_SET(1);

		} else if (Status == STAT_ERROR) {
			if (HAL_GetTick() - timEn >= 50) {
				LED_SET(0);
				timEn = HAL_GetTick();
			}
			if (HAL_GetTick() - timDis >= 200) {
				LED_SET(1);
				timDis = HAL_GetTick();
			}
		}
    osDelay(1);
  }
  /* USER CODE END Led_status_handler */
}

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
	while (1) {
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

