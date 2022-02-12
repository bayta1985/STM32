/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
#include "ssd1306.h"
#include "image.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "bmp280.h"
#include "sd.h"
#include "usart_ring.h"
#include "usbd_cdc_if.h"
#include "st7789.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE_BF 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_1ms */
osThreadId_t Task_1msHandle;
const osThreadAttr_t Task_1ms_attributes = {
  .name = "Task_1ms",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_10ms */
osThreadId_t Task_10msHandle;
const osThreadAttr_t Task_10ms_attributes = {
  .name = "Task_10ms",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_50ms */
osThreadId_t Task_50msHandle;
const osThreadAttr_t Task_50ms_attributes = {
  .name = "Task_50ms",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Usart1Rx */
osThreadId_t Usart1RxHandle;
const osThreadAttr_t Usart1Rx_attributes = {
  .name = "Usart1Rx",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint32_t task1msCnt = 0;
uint32_t task10msCnt = 0;
uint32_t task50msCnt = 0;
uint8_t b1;

_Bool flag, btnPressed;
unsigned long t;
int val;
char* buffer;
uint8_t dataReceived=0;
uint8_t dataTransmitted=0;
int posScanLine = 6;
char* error_mesage = "";
char message[100];
int i = 0;
char* p;

//RTC
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
char trans_str[64] = {0,};

RTC_TimeTypeDef time;
RTC_DateTypeDef date;
HAL_StatusTypeDef res;
int hour, min, sec,subsec;
int dateRTC, monthRTC, yearRTC, dayOfWeek;

//BMP280
BMP280_HandleTypedef bmp280;
float pressure, temperature, humidity;
uint16_t size;
uint8_t Data[256];
bool bme280p, isForced;

//Универсальная газовая постоянная
float R = 8.31446261815324f;
//Молярная масса воздуха
float M = 28.96f;
//Ускорение свободного падения
float g = 9.8066f;
//Давление на уровне моря
float Po = 760.0f;
float Pmm = 0;
float h = 0;
//Переменная для подсчета ошибок чтения датчика
int errorCount = 0;
//PWM on A0
uint32_t iPWM = 37000;

//Переменные для использования microSD
volatile uint16_t Timer1=0;
//Ответ от СД карты
int sd_answer = 0;
//Масив куда сохраняем данные из блока
uint8_t sect[512];
extern char str1[60];
uint32_t byteswritten,bytesread;
uint8_t result;
extern char USERPath[4]; /* logical drive path */
FATFS SDFatFs;
FATFS *fs;
FIL MyFile;
//Массив для записи на MicroSD
//char buffer1[512] ="Lorem ipsum dolor sit amet, consectetur adipiscing elit. Cras tincidunt, ipsum et ullamcorper fermentum, turpis urna feugiat orci, ac aliquam ligula eros id est. Etiam vitae placerat est. Donec eget mauris odio. Donec nisl ipsum, tincidunt a convallis ut, sagittis et velit. Sed suscipit velit velit, sed tristique lacus tincidunt id. Aliquam ullamcorper nulla nec nunc dictum, eu maximus enim aliquam. Sed eleifend consequat diam non ultrices. Morbi porta quis turpis vitae sodales. Donec a porta urna pharetra.";
uint16_t i_for_SD;
char buf[32] ={};
int len = 0;
int dataFromUART = 0;
FRESULT resWriteFile; //результат выполнения
uint8_t wtext[] = "Hello from STM32!!!";//то что будем писать
uint8_t myTextForWrite[] = "It`s works! Привет!";

//список файлов и каталогов на микросд
FILINFO fileInfo;
char* fn;
DIR dir;
DWORD fre_clust, fre_sect, tot_sect;

//Файл лога
char* logFile = "log.log";
//Переменные для работы со временем
_Bool time_setting_mode = false;
_Bool showingTime = false;
_Bool logOn = false;
typedef struct{
	int year, month, day, hour, min, sec, dayOfWeek;
}TimeForSet;


char temperatureString[50];
char dateTempPlesureString[100];

//кнопка KEY
bool btnKeyState = false;

//USB Virtual COM-port
char str_tx[21];
bool SendToUSBvirtualPort = false;

//переменная для вывода приемного буфера на экран
char lastRX[32] = {0,};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void Task_1ms_handler(void *argument);
void Task_10ms_handler(void *argument);
void Task_50ms_handler(void *argument);
void Usart1Rx_handler(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Usart1Tx function */
void Usart1Tx()
{
	/* USER CODE BEGIN Usart1Tx */

}

int _write(int file, char *ptr, int len)
{
	while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY ){

	}
	HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
	return len;
}



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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_1ms */
  Task_1msHandle = osThreadNew(Task_1ms_handler, NULL, &Task_1ms_attributes);

  /* creation of Task_10ms */
  Task_10msHandle = osThreadNew(Task_10ms_handler, NULL, &Task_10ms_attributes);

  /* creation of Task_50ms */
  Task_50msHandle = osThreadNew(Task_50ms_handler, NULL, &Task_50ms_attributes);

  /* creation of Usart1Rx */
  Usart1RxHandle = osThreadNew(Usart1Rx_handler, NULL, &Usart1Rx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
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
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 7777;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 41999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|BLK_TFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RES_TFT_Pin|VCC_OLED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_TFT_Pin|GND_OLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_KEY_Pin */
  GPIO_InitStruct.Pin = BTN_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RES_TFT_Pin DC_TFT_Pin GND_OLED_Pin VCC_OLED_Pin */
  GPIO_InitStruct.Pin = RES_TFT_Pin|DC_TFT_Pin|GND_OLED_Pin|VCC_OLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BLK_TFT_Pin */
  GPIO_InitStruct.Pin = BLK_TFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLK_TFT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement);

////////////////////////////////////////////////////////////////////////////
// Блок вычисление дня недели из года, месяца и дня. Взято отсюда:
// How to calculate day of the week for RTC?
// https://electronics.stackexchange.com/questions/66285/how-to-calculate-day-of-the-week-for-rtc

/* Возвратит количество дней до начала указанного года, с учетом
 * високосных годов, но без смещения от Юлианского календаря
 * к Грегорианскому. Вместо этого Григореанский календарь
 * экстраполируется обратно во времени к гипотетическому
 * "нулевому" году. */
static int leap (int year)
{
	return year*365 + (year/4) - (year/100) + (year/400);
}

/* Вернет количество дней от 1 марта гипотетического года 0, без учета перехода
 * от Юлианского календаря к Грегорианскому, произошедщему в 16 веке. Алгоритм
 * основан на функции, известной как "Zeller's Congruence".
 * MOD 7 дает день недели, где 0 = понедельник и 6 = воскресенье. */
static int zeller (int year, int month, int day)
{
	year += ((month+9)/12) - 1;
	month = (month+9) % 12;
	return leap (year) + month*30 + ((6*month+5)/10) + day + 1;
}

/* Вернет день недели (1=понедельник, 7=воскресенье) от указанной даты. */
int dow (int year, int month, int day)
{
	return (zeller (year, month, day) % 7) + 1;
}
// Конец блока
////////////////////////////////////////////////////////////////////////////

//Отправка сообщения в UART, в блокируещем режиме
void SendToUART(char message[])
{
	int lenghtM = strlen(message);
	if(SendToUSBvirtualPort)
	{
		CDC_Transmit_FS((uint8_t*)message, lenghtM);
		HAL_Delay(1);
	}
	else
	{
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY ){}
		HAL_UART_Transmit(&huart1, (uint8_t*)message, lenghtM, 100);
	}
	//HAL_UART_Transmit_DMA(&huart1, (uint8_t*)message, lenghtM-1);
}

//Запись строк в файл (логирование)
void SendToFILE(char* message)
{
	if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
	{
		Error_Handler();
	}
	else
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
		if(f_open(&MyFile,logFile, FA_OPEN_ALWAYS|FA_WRITE)!=FR_OK)
		{
			Error_Handler();
		}
		else
		{
			int i = sprintf(dateTempPlesureString, "%d.%d.20%d %02d:%02d:%02d%s%s", dateRTC, monthRTC, yearRTC, hour, min, sec, message, "\r\n");
			if(f_lseek(&MyFile, f_size(&MyFile)) == FR_OK)
			{
				res=f_write(&MyFile,&dateTempPlesureString,i,(void*)&byteswritten);
			}
			if((byteswritten==0)||(resWriteFile!=FR_OK))
			{
				SendToUART("\r\nError write to file log.log\r\n");
				Error_Handler();
			}
			f_close(&MyFile);
		}
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,1);
}

//Сканер i2c шины
void I2C_Scaner()
{
	printf("\r\nStart scanner I2C bus\r\n");

	for (int i = 0; i < 128; ++i)
	{
		uint16_t address = i << 1;

		if (HAL_I2C_IsDeviceReady(&hi2c1, address, 1, HAL_MAX_DELAY) == HAL_OK)
		{
			printf("Address: 0x%X\r\n", address >> 1);
		}
	}

	printf("End test I2C bus\r\n");

	HAL_Delay(2000);
}


void OLEDview()
{

	//ssd1306_Init();

	/*
	for (int16_t i = 0; i < ssd1306_GetHeight() / 2; i += 2)
	  {
	    ssd1306_DrawRect(i, i, ssd1306_GetWidth() - 2 * i, ssd1306_GetHeight() - 2 * i);
	    ssd1306_UpdateScreen();
	    HAL_Delay(1);
	  }
	 */
	//ssd1306_Fill();
	/*
	if(i >= 100)
	{
		i = 0;
		ssd1306_Clear();
	}
	else
	{
		i++;
	}
	ssd1306_SetCursor(0, 20);
	ssd1306_DrawPixel(i, 0);
	drawProgressBarDemo(i);
	 */
	//string s = ("P: %.2f Pa, T: %.2f C\r\n", pressure, temperature);


	ssd1306_Clear();
	ssd1306_SetCursor((uint8_t)120, (uint8_t)53);
	ssd1306_WriteString("u", Font_7x10);
	ssd1306_UpdateScreen();

	ssd1306_SetCursor(0, 0);
	sprintf(message, "T=%.2f", temperature);
	ssd1306_WriteString(message, Font_7x10);

	ssd1306_SetCursor(0, 16);
	sprintf(message, "P=%.8f", pressure);
	ssd1306_WriteString(message, Font_7x10);

	ssd1306_SetCursor(71, 0);
	sprintf(message, "%02d:%02d:%02d", time.Hours, time.Minutes, time.Seconds);
	ssd1306_WriteString(message, Font_7x10);

	/*
	ssd1306_SetCursor(0, 26);
	sprintf(message, "Pmm=%.8f", Pmm);
	ssd1306_WriteString(message, Font_7x10);

	ssd1306_SetCursor(44, 53);
	sprintf(message, "PWM=%u", iPWM);
	ssd1306_WriteString(message, Font_7x10);

	ssd1306_SetCursor(0, 36);
	sprintf(message, "h=%.2f m", h);
	ssd1306_WriteString(message, Font_7x10);
	 */

	ssd1306_SetCursor(0, 27);
	sprintf(message, "RX:%s", lastRX);
	ssd1306_WriteString(message, Font_7x10);

	ssd1306_SetCursor(0, 38);
	sprintf(message, "TX:%s", "");
	ssd1306_WriteString(message, Font_7x10);
	/*
	if(logOn)
	{
		ssd1306_SetCursor(0, 53);
		ssd1306_WriteString("Log to SD on", Font_7x10);
	}
	else
	{
		ssd1306_SetCursor(0, 53);
		ssd1306_WriteString("Log to SD off", Font_7x10);
	}
	*/

	if(SendToUSBvirtualPort)
	{
			ssd1306_SetCursor(0, 53);
			ssd1306_WriteString("USB", Font_7x10);
		}
		else
		{
			ssd1306_SetCursor(0, 53);
			ssd1306_WriteString("UART", Font_7x10);
		}
	/*
	ssd1306_SetCursor(40, 53);
	sprintf(message, "SDans=%i", sd_answer);
	ssd1306_WriteString(message, Font_7x10);
	 */

	ssd1306_SetCursor(120, 53);
	ssd1306_WriteString(" ", Font_7x10);
	ssd1306_UpdateScreen();

}
//вычисления
void calculate()
{
	Pmm = pressure/133.3224;
	h = (R*temperature)/(M*g)*log(Po/Pmm);
}

void drawProgressBarDemo(int counter)
{
	char str[128];
	// draw the progress bar
	ssd1306_DrawProgressBar(0, 32, 120, 10, counter);

	// draw the percentage as String
	ssd1306_SetCursor(64, 16);
	sprintf(str, "%d", counter);
	ssd1306_WriteString(str, Font_7x10);
	ssd1306_UpdateScreen();
}

void timeRtc()
{
	res = HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	if(res != HAL_OK) {
		printf("HAL_RTC_GetTime failed: %d\r\n", res);
		return;
	}
	else
	{
		hour = time.Hours;
		min = time.Minutes;
		sec = time.Seconds;
		subsec = time.SubSeconds;
	}
	res = HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
	if(res != HAL_OK) {
		printf("HAL_RTC_GetDate failed: %d\r\n", res);
		return;
	}
	else
	{
		dateRTC = date.Date;
		monthRTC = date.Month;
		yearRTC = date.Year;
		dayOfWeek = date.WeekDay;
	}
	if(showingTime)
	{
		int count = sprintf(buf, "%s%d.%d.20%d %02d:%02d:%02d%s","tDate.txt=\"", dateRTC, monthRTC, yearRTC,hour,min, sec,"\"");
		HAL_UART_Transmit(&huart1, (uint8_t*)buf, count, 100);
		HAL_UART_Transmit(&huart1, (uint8_t*)"\xFF\xFF\xFF", 3, 100);

	}
}

void BMP280()
{
	while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity))
	{
		//size = sprintf((char *)Data,"failed\r\n");
		//HAL_UART_Transmit(&huart1, Data, size, 1000);
		HAL_Delay(2000);
		errorCount++;
	}
	//printf("Pressure: %f Pa\r\n", pressure);
}


FRESULT ReadLongFile(void)
{
	uint16_t i=0, i1=0;
	uint32_t ind=0;
	//uint32_t f_size = MyFile.fsize();
	uint32_t f_size = f_size(&MyFile);
	sprintf(str1,"\r\nfsize: %lu\r\n",(unsigned long)f_size);
	SendToUART(str1);
	ind=0;

	if(SendToUSBvirtualPort)
	{
	do
	{
		if(f_size<512)
		{
			i1=f_size;
		}
		else
		{
			i1=512;
		}
		f_size-=i1;
		f_lseek(&MyFile,ind);
		f_read(&MyFile,sect,i1,(UINT *)&bytesread);
		for(i=0;i<bytesread;i++)
		{
			CDC_Transmit_FS(sect+i, 1);
		}
		ind+=i1;
	}
	while(f_size>0);
	CDC_Transmit_FS((uint8_t*)"\r\n", 2);
	}
	else
	{
		do
			{
				if(f_size<512)
				{
					i1=f_size;
				}
				else
				{
					i1=512;
				}
				f_size-=i1;
				f_lseek(&MyFile,ind);
				f_read(&MyFile,sect,i1,(UINT *)&bytesread);
				for(i=0;i<bytesread;i++)
				{
					HAL_UART_Transmit(&huart1,sect+i,1,0x1000);

				}
				ind+=i1;
			}
			while(f_size>0);
			HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
	}
	return FR_OK;
}

int RTC_Set(TimeForSet tfs) {
	HAL_StatusTypeDef res;
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;

	memset(&time, 0, sizeof(time));
	memset(&date, 0, sizeof(date));

	date.WeekDay = dow(tfs.year, tfs.month, tfs.day);
	date.Year = tfs.year;
	date.Month = tfs.month;
	date.Date = tfs.day;

	res = HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
	if(res != HAL_OK) {
		SendToUART("HAL_RTC_SetDate failed\r\n");
		return -1;
	}

	time.Hours = tfs.hour;
	time.Minutes = tfs.min;
	time.Seconds = tfs.sec;

	res = HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
	if(res != HAL_OK) {
		SendToUART("HAL_RTC_SetTime failed\r\n");
		return -2;
	}

	return 0;
}

void UserSetTimeRTC(char* mes)
{
	TimeForSet t_for_set;
	showingTime = false;
	SendToUART("_____________________\r\n");
	SendToUART("Setting time and date\r\n");
	SendToUART("_____________________\r\n");
	SendToUART(mes);
	sscanf(mes,"1Y%2dM%2dD%2dh%2dm%2ds%2d",&t_for_set.year,&t_for_set.month,&t_for_set.day,&t_for_set.hour,&t_for_set.min,&t_for_set.sec);
	RTC_Set(t_for_set);
	showingTime = true;
}

void SendToVirtualPort()
{
	sprintf(str_tx,"USB Transmit\r\n");
	CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));
}

void changeUSBorUART()
{
	if(SendToUSBvirtualPort)
	{
		SendToUART("\r\nUART off\r\n");
		SendToUSBvirtualPort=false;
		SendToUART("\r\nUART on\r\n");
	}
	else
	{
		SendToUART("\r\nUART off\r\n");
		SendToUSBvirtualPort=true;
		SendToUART("\r\nUART on\r\n");
	}
}

void ECHO_UART()
{
	if(uart_available()) // есть ли что-то в приёмном буфере, тогда читаем
	{
		char reciveMessage[SIZE_BF] = {0,};
		uint8_t i = 0;

		while(uart_available())
		{
			reciveMessage[i++] = (char)uart_read(); // читаем байт

			if(i == SIZE_BF - 1)
			{
				reciveMessage[i] = '\0';
				break;
			}
		}
		reciveMessage[i] = '\0';
		strcpy(lastRX, reciveMessage);
		//SendToUART(reciveMessage);
		dataFromUART = atoi(reciveMessage);

		switch (dataFromUART){
		case 0://help
			SendToUART("\n\rThere will be a help on the commands :)\n\r");
			break;
		case 1://установка времени
			dataFromUART = 0;
			UserSetTimeRTC(reciveMessage);
			break;
		case 2://мигание светодиодом
			dataFromUART = 0;
			SendToUART("LED!!!\n\r");
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			break;
		case 3://чтение файла с микросд
			dataFromUART = 0;
			if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
			{
				Error_Handler();
			}
			else
			{
				if(f_open(&MyFile,"test.txt",FA_READ)!=FR_OK)
				{
					Error_Handler();
				}
				else
				{
					ReadLongFile();
					f_close(&MyFile);
				}
			}
			break;
		case 4://передача времени в юарт (старт/стоп)
			dataFromUART = 0;
			if (showingTime) {
				showingTime = false;
			} else {
				showingTime = true;
			}
			break;
		case 5://чтение файла с микросд
			dataFromUART = 0;
			if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
			{
				Error_Handler();
			}
			else
			{
				if(f_open(&MyFile,"cb.bmp",FA_READ)!=FR_OK)
				{
					SendToUART("File not exist.");
					//Error_Handler();
				}
				else
				{
					ReadLongFile();
					f_close(&MyFile);
				}
			}
			break;
		case 6://чтение файла с микросд
			dataFromUART = 0;
			if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
			{
				Error_Handler();
			}
			else
			{
				if(f_open(&MyFile,"island.bmp",FA_READ)!=FR_OK)
				{
					SendToUART("\r\nFile not exist.");
					//Error_Handler();
				}
				else
				{
					ReadLongFile();
					f_close(&MyFile);
				}
			}
			break;
		case 7://запись в файл
			if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
			{
				Error_Handler();
			}
			else
			{
				if(f_open(&MyFile,"myFile.txt", FA_OPEN_ALWAYS|FA_WRITE)!=FR_OK)
				{
					Error_Handler();
				}
				else
				{
					SendToUART("\r\nBegin write to file myFile.txt\r\n");
					sprintf(temperatureString, "%sT=%.2f%sP=%.8f",";", temperature,";", pressure);
					int i = sprintf(dateTempPlesureString, "%d.%d.20%d %02d:%02d:%02d%s%s", dateRTC, monthRTC, yearRTC, hour, min, sec, temperatureString, "\r\n");
					if(f_lseek(&MyFile, f_size(&MyFile)) == FR_OK)
					{
						res=f_write(&MyFile,&dateTempPlesureString,i,(void*)&byteswritten);
					}
					if((byteswritten==0)||(resWriteFile!=FR_OK))
					{
						SendToUART("\r\nError write to file myFile.txt\r\n");
						Error_Handler();
					}
					f_close(&MyFile);
					SendToUART("\r\nEnd write to file myFile.txt\r\n");
				}
			}
			break;
		case 8://чтение файла с микросд
			dataFromUART = 0;
			if(f_open(&MyFile,"myFile.txt",FA_READ)!=FR_OK)
			{
				SendToUART("\r\nFile not exist.\r\n");
				//Error_Handler();
			}
			else
			{
				ReadLongFile();
				f_close(&MyFile);
			}
			break;
		case 9://отвязать файловую систему
			FATFS_UnLinkDriver(USERPath);
			SendToUART("\r\nFilesystem is unlinked.\r\n");
			break;
		case 10:
			if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
			{
				Error_Handler();
			}
			else
			{
				result = f_opendir(&dir, "/");
				if (result == FR_OK)
				{
					while(1)
					{
						result = f_readdir(&dir, &fileInfo);
						if (result==FR_OK && fileInfo.fname[0])
						{
							fn = fileInfo.fname;
							SendToUART(fn);
							if(strlen(fn)) HAL_UART_Transmit(&huart1,(uint8_t*)fn,strlen(fn),0x1000);
							else //HAL_UART_Transmit(&huart1,(uint8_t*)fileInfo.fname,strlen((char*)fileInfo.fname),0x1000);
								SendToUART(fileInfo.fname);
							if(fileInfo.fattrib&AM_DIR)
							{
								//HAL_UART_Transmit(&huart1,(uint8_t*)" [DIR]",7,0x1000);
								SendToUART(" [DIR]");
							}
						}
						else break;
						//HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
						SendToUART("\r\n");
					}
					f_getfree("/", &fre_clust, &fs);

					sprintf(str1,"Free cluster: %lu\r\n",fre_clust);

					//HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
					SendToUART(str1);

					sprintf(str1,"All cluster (+2): %lu\r\n",fs->n_fatent);

					//HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
					SendToUART(str1);

					sprintf(str1,"Cluster size [sectors]: %d\r\n",fs->csize);

					//HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
					SendToUART(str1);

					tot_sect = (fs->n_fatent - 2) * fs->csize;

					sprintf(str1,"Total sector: %lu\r\n",tot_sect);

					//HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
					SendToUART(str1);
					fre_sect = fre_clust * fs->csize;

					sprintf(str1,"Free sector: %lu\r\n",fre_sect);

					//HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
					SendToUART(str1);
					sprintf(str1, "%lu KB total drive space.\r\n%lu KB available.\r\n",

							fre_sect/2, tot_sect/2);

					//HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
					SendToUART(str1);
					f_closedir(&dir);
				}
			}
			break;
		case 11:
			if(logOn)
			{
				logOn = false;
				SendToUART("\r\nLogging to SD - OFF\r\n");
			}
			else
			{
				logOn = true;
				SendToUART("\r\nLogging to SD - ON\r\n");
			}
			break;
		case 12:
			NVIC_SystemReset();
			break;
		case 13://чтение файла с микросд
			dataFromUART = 0;
			if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
			{
				Error_Handler();
			}
			else
			{
				if(f_open(&MyFile,logFile,FA_READ)!=FR_OK)
				{
					//Error_Handler();
				}
				else
				{
					ReadLongFile();
					f_close(&MyFile);
				}
			}
			break;
		case 14:
			SendToVirtualPort();
			break;
		case 15:
			changeUSBorUART();
			break;
		default://дефолтно, ничего не делать
			break;
		}

	}
}

void SendToLog()
{
	sprintf(temperatureString, "%sT=%.2f%sP=%.8f",";", temperature,";", pressure);
	SendToFILE(temperatureString);
}

void KeyEvent()
{
	if(HAL_GPIO_ReadPin(BTN_KEY_GPIO_Port, BTN_KEY_Pin) == GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			}
}

void Test_TFT()
{

	  ST7789_Init(240, 240);
	  ST7789_FillScreen(BLACK);
	  ST7789_SetBL(100);
	  uint16_t color = RGB565(255, 0, 0);
	  ST7789_DrawCircle(119, 119, 10, color);
	  ST7789_DrawCircle(119, 119, 20, color);
	  ST7789_DrawCircle(119, 119, 30, color);
	  ST7789_DrawCircle(119, 119, 40, color);
	  ST7789_DrawCircle(119, 119, 50, color);
	  ST7789_DrawLine(0, 119, 239, 119, color);
	  ST7789_DrawLine(119, 0, 119, 239,color);
	  HAL_Delay(3000);

	// Тест вывода основных цветов
	    ST7789_FillScreen(BLACK);
	    ST7789_SetBL(100);

	    color = RGB565(255, 0, 0);
	    ST7789_FillScreen(color);
	    HAL_Delay(500);

	    color = RGB565(0, 255, 0);
	    ST7789_FillScreen(color);
	    HAL_Delay(500);

	    color = RGB565(50, 55, 50);
	    ST7789_FillScreen(color);
	    HAL_Delay(500);

	    color = RGB565(0, 0, 255);
	    ST7789_FillScreen(color);
	    HAL_Delay(500);

	    color = RGB565(255, 255, 0);
	    ST7789_FillScreen(color);
	    HAL_Delay(500);

	    color = RGB565(255, 0, 255);
	    ST7789_FillScreen(color);
	    HAL_Delay(500);

	    color = RGB565(0, 255, 255);
	    ST7789_FillScreen(color);
	    HAL_Delay(500);

	    color = RGB565(255, 255, 255);
	    ST7789_FillScreen(color);
	    HAL_Delay(500);

	    ST7789_FillScreen(BLACK);
	    ST7789_SetBL(100);

			for (uint8_t y = 0; y<240 ; y++) {
				ST7789_DrawLine(120, 120, 239, y, RGB565(y+10, 0, 0));
			}

			for (uint8_t x = 0; x<240 ; x++) {
				ST7789_DrawLine(120, 120, x, 239, RGB565(0, x+10, 0));
			}

			for (uint8_t y = 0; y<240 ; y++) {
				ST7789_DrawLine(120, 120, 0, y, RGB565(0, 0, y+10));
			}

			for (uint8_t x = 0; x<240 ; x++) {
				ST7789_DrawLine(120, 120, x, 0, RGB565(x+10, x+10, x+10));
			}
	    HAL_Delay(1000);

	    ST7789_FillScreen(BLACK);
	    ST7789_SetBL(100);

	    for (uint8_t x = 0; x < 240 ; x = x + 20) {
				for (uint8_t y = 0; y < 240; y = y + 20) {
					ST7789_DrawRectangleFilled(x + 3, y + 3, x + 17, y + 17, RGB565(x, y, 0));
					ST7789_DrawRectangle(x + 2, y + 2, x + 19, y + 19, RGB565(250, 250, 250));
				}
			}
	    HAL_Delay(1000);

	    ST7789_FillScreen(BLACK);
	    ST7789_SetBL(100);

	    for (uint8_t x = 0; x < 240 ; x = x + 20) {
				for (uint8_t y = 0; y < 240; y = y + 20) {
	        ST7789_DrawCircleFilled(x + 10, y + 10, 8, RGB565(x, y, 0));
	        ST7789_DrawCircle(x + 10, y + 10, 9, RGB565(0, y, x));
				}
			}
	    HAL_Delay(1000);

	    ST7789_FillScreen(BLACK);
	    ST7789_SetBL(100);
	}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	__HAL_UART_ENABLE_IT(&MYUART, UART_IT_RXNE); // включить прерывания usart'a
	//Init OLED display

	len = sprintf(buf, "%s", "System started.\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);


	//Init_SD
	HAL_TIM_Base_Start_IT(&htim2);
	osDelay(200);
	//SD_PowerOn();
	//sd_answer = sd_ini();
	//len = sprintf(buf, "%s%d%s", "sd_ini=",sd_answer,"\r\n");
	//SendToUART(buf);
	disk_initialize(SDFatFs.drv);

	//read
	/*
	if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
	{
		Error_Handler();
	}
	else
	{
		if(f_open(&MyFile,"test.txt",FA_READ)!=FR_OK)
		{
			//Error_Handler();
		}
		else
		{
			ReadLongFile();
			f_close(&MyFile);
		}
	}
	 */

	//write
	/*
	if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
	{
	  Error_Handler();
	}
	else
	{
	  if(f_open(&MyFile,"mywrite.txt",FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK)
	  {
	    Error_Handler();
	  }
	  else
	  {
	    res=f_write(&MyFile,wtext,sizeof(wtext),(void*)&byteswritten);
	    if((byteswritten==0)||(resWriteFile!=FR_OK))
	    {
	    	Error_Handler();
	    }
	    f_close(&MyFile);
	  }
	}
	 */

	//read dir
	/*
	if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
	{
	  Error_Handler();
	}
	else
	{
		result = f_opendir(&dir, "/");
		if (result == FR_OK)
		{
			while(1)
			{
				result = f_readdir(&dir, &fileInfo);
				if (result==FR_OK && fileInfo.fname[0])
				{
					fn = fileInfo.fname;
					if(strlen(fn)) HAL_UART_Transmit(&huart1,(uint8_t*)fn,strlen(fn),0x1000);
					else HAL_UART_Transmit(&huart1,(uint8_t*)fileInfo.fname,strlen((char*)fileInfo.fname),0x1000);
					if(fileInfo.fattrib&AM_DIR)
					{
						HAL_UART_Transmit(&huart1,(uint8_t*)" [DIR]",7,0x1000);
					}
				}
				else break;
				HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
			}
			f_closedir(&dir);
		}
	}
	 */


	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_1ms_handler */
/**
 * @brief Function implementing the Task_1ms thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_1ms_handler */
void Task_1ms_handler(void *argument)
{
  /* USER CODE BEGIN Task_1ms_handler */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1 / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();


	//Init BMP280
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c2;
	bmp280.params.mode = BMP280_MODE_NORMAL;
	bmp280.params.filter = BMP280_FILTER_OFF;
	bmp280.params.standby = BMP280_STANDBY_125;
	bmp280.params.oversampling_pressure = BMP280_STANDARD;
	bmp280.params.oversampling_temperature = BMP280_STANDARD;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		size = sprintf((char *)Data, "BMP280 initialization failed\n");
		HAL_UART_Transmit(&huart1, Data, size, 1000);
		HAL_Delay(2000);
	}
	//HAL_Delay(1000);
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,1);
	/* Infinite loop */
	for(;;)
	{
		if (task1msCnt >= 2000)
		{


			//timeRtc();
			//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,1);

			//I2C_Scaner();
			BMP280();
			//OLEDview();
			Test_TFT();
			task1msCnt = 0;
			//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);

		}

		task1msCnt++;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
  /* USER CODE END Task_1ms_handler */
}

/* USER CODE BEGIN Header_Task_10ms_handler */
/**
 * @brief Function implementing the Task_10ms thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_10ms_handler */
void Task_10ms_handler(void *argument)
{
  /* USER CODE BEGIN Task_10ms_handler */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	ssd1306_Init();
	ssd1306_SetColor(White);
	ssd1306_FlipScreenVertically();

	osDelay(100);

	/* Infinite loop */
	for(;;)
	{
		if (task10msCnt == 10)
		{
			//timeRtc();
			//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,1);
			//I2C_Scaner();
			//BMP280();
			OLEDview();
			task10msCnt = 0;
			//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
		}
		task10msCnt++;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
  /* USER CODE END Task_10ms_handler */
}

/* USER CODE BEGIN Header_Task_50ms_handler */
/**
 * @brief Function implementing the Task_50ms thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_50ms_handler */
void Task_50ms_handler(void *argument)
{
  /* USER CODE BEGIN Task_50ms_handler */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();


	/* Infinite loop */
	for(;;)
	{
		ECHO_UART();
		if (task50msCnt == 20)
		{

			timeRtc();

			if(logOn)
			{
				SendToLog();
			}

			task50msCnt = 0;
		}
		KeyEvent();
		task50msCnt++;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
  /* USER CODE END Task_50ms_handler */
}

/* USER CODE BEGIN Header_Usart1Rx_handler */
/**
 * @brief Function implementing the Usart1Rx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Usart1Rx_handler */
void Usart1Rx_handler(void *argument)
{
  /* USER CODE BEGIN Usart1Rx_handler */
	/* Infinite loop */
	for(;;)
	{

		osDelay(1);
	}
  /* USER CODE END Usart1Rx_handler */
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
	if(htim==&htim2){
		Timer1++;
	}
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

