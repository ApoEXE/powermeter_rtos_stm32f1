/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS3231 0x68 //timer
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId task_ADCHandle;
osThreadId task_nextionHandle;
osThreadId task_bluetoothHandle;
/* USER CODE BEGIN PV */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
uint32_t value[4];//adc channels
uint16_t AC_AMP = 0;
uint16_t AC_VOLT = 0;
uint16_t DC_AMP = 0;
uint16_t DC_VOLT = 0;
uint8_t Cmd_End[] = { 0xff, 0xff, 0xff };
uint16_t delay_sec = 100;
uint8_t rx_buff[7];
uint8_t DEVID = 0;
uint8_t time_buff[7];
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;

TIME time;
char buffer[50]; //save formated messag

uint8_t tempBuff[2];
float TEMP; //protect with mutex
float filter_AC_AMP = 0; //protect with mutex
float filter_AC_VOLT = 0; //protect with mutex
float filter_DC_AMP = 0; //protect with mutex
float filter_DC_VOLT = 0; //protect with mutex

static SemaphoreHandle_t fac_amp_mutex;
static SemaphoreHandle_t fac_volt_mutex;
static SemaphoreHandle_t fdc_amp_mutex;
static SemaphoreHandle_t fdc_volt_mutex;

ADC_ChannelConfTypeDef sConfig = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartADC(void const * argument);
void StartNextion(void const * argument);
void StartBluetooth(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void NEXTION_SendString(char *ID, char *string) {
	char buf[50];
	int len = sprintf(buf, "%s.txt=\"%s\"", ID, string);
	HAL_UART_Transmit(&huart1, (uint8_t*) buf, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}
void NEXTION_SendValue(char *ID, int value) {
	char buf[50];
	int len = sprintf(buf, "%s.val=%d", ID, value);
	HAL_UART_Transmit(&huart1, (uint8_t*) buf, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}
void NEXTION_SendGraphicValue(int id, int ch, int val) {
	char buf[50];
	int len = sprintf(buf, "add %d,%d,%d", id, ch, val);
	HAL_UART_Transmit(&huart1, (uint8_t*) buf, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}

int map_range(int OldValue, int OldMax, int OldMin, int NewMax, int NewMin) {

	return (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin))
			+ NewMin;
}

// Convert normal decimal numbers to binary coded decimal
uint8_t decToBcd(int val);
// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val);
void Set_Time(I2C_HandleTypeDef i2c_hdl, uint16_t address, uint8_t sec,
		uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month,
		uint16_t year);
void Get_Time(I2C_HandleTypeDef i2c_hdl, uint16_t address, uint8_t *get_time);

void force_temp_conv(I2C_HandleTypeDef i2c_hdl, uint16_t address);
float Get_Temp(I2C_HandleTypeDef i2c_hdl, uint16_t address, uint8_t *temp);

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, value, 4);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	fac_amp_mutex = xSemaphoreCreateMutex();
	fac_volt_mutex = xSemaphoreCreateMutex();
	fdc_amp_mutex = xSemaphoreCreateMutex();
	fdc_volt_mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of task_ADC */
  osThreadDef(task_ADC, StartADC, osPriorityNormal, 0, 128);
  task_ADCHandle = osThreadCreate(osThread(task_ADC), NULL);

  /* definition and creation of task_nextion */
  osThreadDef(task_nextion, StartNextion, osPriorityBelowNormal, 0, 128);
  task_nextionHandle = osThreadCreate(osThread(task_nextion), NULL);

  /* definition and creation of task_bluetooth */
  osThreadDef(task_bluetooth, StartBluetooth, osPriorityLow, 0, 128);
  task_bluetoothHandle = osThreadCreate(osThread(task_bluetooth), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	//printf("value[0]: %ld \n",value[0]);
}
void Set_Time(I2C_HandleTypeDef i2c_hdl, uint16_t address, uint8_t sec,
		uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month,
		uint16_t year) {
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	HAL_I2C_Mem_Write(&i2c_hdl, address, 0x00, 1, set_time, 7, 1000);
}
void Get_Time(I2C_HandleTypeDef i2c_hdl, uint16_t address, uint8_t *get_time) {

	HAL_I2C_Mem_Read(&i2c_hdl, address, 0x00, 1, get_time, 7, 1000);
	time.seconds = bcdToDec(get_time[0]);
	time.minutes = bcdToDec(get_time[1]);
	time.hour = bcdToDec(get_time[2]);
	time.dayofweek = bcdToDec(get_time[3]);
	time.dayofmonth = bcdToDec(get_time[4]);
	time.month = bcdToDec(get_time[5]);
	time.year = bcdToDec(get_time[6]);
}
uint8_t decToBcd(int val) {
	return (uint8_t) ((val / 10 * 16) + (val % 10));
}
// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val) {
	return (int) ((val / 16 * 10) + (val % 16));
}

float Get_Temp(I2C_HandleTypeDef i2c_hdl, uint16_t address, uint8_t *temp) {

	HAL_I2C_Mem_Read(&i2c_hdl, address, 0x11, 1, temp, 2, 1000);
	return ((temp[0]) + (temp[1] >> 6) / 4.0);
}
void force_temp_conv(I2C_HandleTypeDef i2c_hdl, uint16_t address) {
	uint8_t status = 0;
	uint8_t control = 0;
	HAL_I2C_Mem_Read(&i2c_hdl, address, 0x0F, 1, &status, 1, 100); // read status register
	if (!(status & 0x04))  // if the BSY bit is not set
	{
		HAL_I2C_Mem_Read(&i2c_hdl, address, 0x0E, 1, &control, 1, 100); // read control register
		HAL_I2C_Mem_Write(&i2c_hdl, address, 0x0E, 1,
				(uint8_t*) (control | (0x20)), 1, 100); // write modified control register with CONV bit as'1'
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	printf("\r\nLlego: %c %c \r\n", rx_buff[0],rx_buff[1]);
	if (rx_buff[0] == '$' && rx_buff[1] == '1') {
		delay_sec = 100;

	}

	if (rx_buff[0] == '$' && rx_buff[2] == '1') {

		delay_sec = 1000;

	}

	if (rx_buff[0] == '$' && rx_buff[3] == '1') {

		delay_sec = 10000;

	}
	//HAL_UART_Transmit(&huart1, (uint8_t*) rx_buff, sizeof(rx_buff), 1000);
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	HAL_UART_Receive_IT(&huart1, rx_buff, sizeof(rx_buff));
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartADC */
/**
 * @brief  Function implementing the task_adc thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartADC */
void StartADC(void const * argument)
{
  /* USER CODE BEGIN 5 */
	float ac_amp_prev = 0;
	float ac_volt_prev = 0;
	float dc_amp_prev = 0;
	float dc_volt_prev = 0;

	float a = 0.1;

	HAL_ADCEx_Calibration_Start(&hadc1);

	/* Infinite loop */
	for (;;) {

		//xSemaphoreTake(fac_amp_mutex, portMAX_DELAY);
		filter_AC_AMP = (1 - a) * ac_amp_prev + value[0] * a;
		ac_amp_prev = filter_AC_AMP;

		//xSemaphoreGive(fac_amp_mutex);

		filter_AC_VOLT = (1 - a) * ac_volt_prev + value[1]* a;
		ac_volt_prev = filter_AC_VOLT;


		filter_DC_AMP = (1 - a) * dc_amp_prev + value[2] * a;
		dc_amp_prev = filter_DC_AMP ;


		filter_DC_VOLT = (1 - a) * dc_volt_prev + value[3]* a;
		dc_volt_prev= filter_DC_VOLT;
		//printf("\r\tvale[0]: %ld \tvale[1]: %ld\tvale[2]: %ld\tvale[3]: %ld\r\n",value[0],value[1],value[2],value[3]);

		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartNextion */
/**
 * @brief Function implementing the task_nextion thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartNextion */
void StartNextion(void const * argument)
{
  /* USER CODE BEGIN StartNextion */

	uint8_t mapped = 0;
	uint8_t year = 23;
	uint8_t month = 1;
	uint8_t dm = 30;
	uint8_t dw = 1;
	uint8_t hour = 21;
	uint8_t min = 12;
	uint8_t sec = 00;

	//Set_Time(hi2c1, (uint16_t) DS3231 << 1, sec, min, hour, dw, dm, month,
			//year);
	printf("Nextion Thread Started\n");
	osDelay(100);
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	/* Infinite loop */
	for (;;) {

		//HAL_UART_Receive_IT(&huart1, rx_buff, sizeof(rx_buff));
		mapped = map_range((uint16_t)filter_AC_AMP, 4095, 0, 255, 0);
		NEXTION_SendValue("x0", mapped);
		NEXTION_SendGraphicValue(7, 0, mapped);
		mapped = map_range((uint16_t)filter_AC_VOLT, 4095, 0, 255, 0);
		NEXTION_SendValue("x1", mapped);
		NEXTION_SendGraphicValue(7, 1, mapped);
		mapped = map_range((uint16_t)filter_DC_AMP, 4095, 0, 255, 0);
		NEXTION_SendValue("x2", mapped);
		NEXTION_SendGraphicValue(7, 2, mapped);
		mapped = map_range((uint16_t)filter_DC_VOLT, 4095, 0, 255, 0);
		NEXTION_SendValue("x3", mapped);
		NEXTION_SendGraphicValue(7, 3, mapped);



		//Get_Time(hi2c1, (uint16_t) DS3231 << 1, time_buff);
		/*
		sprintf(buffer, "%02d/%02d/20%02d %02d:%02d:%02d", time.dayofmonth,
				time.month, time.year, time.hour, time.minutes, time.seconds);
		NEXTION_SendString("t1", buffer);

		printf("\r\n%s\r\n", buffer);
		*/
		switch (delay_sec) {
		case 100:
			NEXTION_SendString("t0", "delay 0.1sec");
			break;
		case 1000:
			NEXTION_SendString("t0", "delay 1sec");
			break;
		case 10000:
			NEXTION_SendString("t0", "delay 10sec");
			break;
		default:
			NEXTION_SendString("t0", "unknown");
			break;

		}

		osDelay(delay_sec);
	}
  /* USER CODE END StartNextion */
}

/* USER CODE BEGIN Header_StartBluetooth */
/**
 * @brief Function implementing the task_bluetooth thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBluetooth */
void StartBluetooth(void const * argument)
{
  /* USER CODE BEGIN StartBluetooth */


	/* Infinite loop */
	for (;;) {



		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		osDelay(100);
	}
  /* USER CODE END StartBluetooth */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
