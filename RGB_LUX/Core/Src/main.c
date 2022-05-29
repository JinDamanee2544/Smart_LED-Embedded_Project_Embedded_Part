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
 * If no LICENSE0file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// LUX CONST
#define LUX_SCALE 14	// 2^14
#define RATIO_SCALE 9	// scale ratio to 2^5=512
#define K1T 0x0040 		// 0.125 * 2^RATIO_SCALE
#define B1T 0x01f2 		// 0.0304 * 2^LUX_SCALE
#define M1T 0x01be 		// 0.0272 * 2^LUX_SCALE
#define K2T 0x0080 		// 0.250 * 2^RATIO_SCALE
#define B2T 0x0214 		// 0.0325 * 2^LUX_SCALE
#define M2T 0x02d1		// 0.0440 * 2^LUX_SCALE
#define K3T 0x00c0		// 0.375 * 2^RATIO_SCALE
#define B3T 0x023f 		// 0.0351 * 2^LUX_SCALE
#define M3T 0x037b 		// 0.0544 * 2^LUX_SCALE
#define K4T 0x0100 		// 0.50 * 2^RATIO_SCALE
#define B4T 0x0270 		// 0.0381 * 2^LUX_SCALE
#define M4T 0x03fe 		// 0.0624 * 2^LUX_SCALE
#define K5T 0x0138 		// 0.61 * 2^RATIO_SCALE
#define B5T 0x016f 		// 0.0224 * 2^LUX_SCALE
#define M5T 0x01fc 		// 0.0310 * 2^LUX_SCALE
#define K6T 0x019a 		// 0.80 * 2^RATIO_SCALE
#define B6T 0x00d2 		// 0.0128 * 2^LUX_SCALE
#define M6T 0x00fb 		// 0.0153 * 2^LUX_SCALE
#define K7T 0x029a 		// 1.3 * 2^RATIO_SCALE
#define B7T 0x0018 		// 0.00146 * 2^LUX_SCALE
#define M7T 0x0012 		// 0.00112 * 2^LUX_SCALE
#define K8T 0x029a 		// 1.3 * 2^RATIO_SCALE
#define B8T 0x0000 		// 0.000 * 2^LUX_SCALE
#define M8T 0x0000 		// 0.000 * 2^LUX_SCALE

// RGB CONST
#define MAX_RED 	326
#define MAX_GREEN 	300
#define MAX_BLUE 	390

#define MIN_RED		54
#define MIN_GREEN	46
#define MIN_BLUE	61

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;
osThreadId LUXHandle;
osTimerId measureTimerHandle;
osTimerId autoUpdateHandle;
osMutexId UartMutexHandle;
osMutexId ESPUARTMutexHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void const *argument);
void luxFunc(void const *argument);
void rgbCount(void const *argument);
void update(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PIN
uint8_t S0; 	//PC5
uint8_t S1;  	//PC6
uint8_t S2 = 0; 	//PC8
uint8_t S3 = 0; 	//PC9

// FOR RGB
uint8_t flag = 0;
int counter = 0;
int countR = 0;
int countG = 0;
int countB = 0;

// FOR LUX
HAL_StatusTypeDef ret;
uint8_t R0C[1];
uint8_t R0D[1];
uint8_t R0E[1];
uint8_t R0F[1];
uint8_t S[2];
uint8_t uart[32];
uint8_t addr[1] = { 0x8C };
uint8_t config[1] = { 0x03 };

// CONTROL
int globalColor[3];
int globalLux[1];
char colorBuffer[100];
char luxBuffer[10];
char luxString[4];
//char printBuffer[50];

int map(int x,int in_min,int in_max,int out_min,int out_max){
	int calibrate = (x-in_min)*(out_max-out_min) / (in_max - in_min) + out_min;
	if(calibrate>255) return 255;
	if(calibrate<0) return 0;
	return calibrate;
}

void printMutexUART(char *buffer) {
	// UART MUTEX
	osMutexWait(UartMutexHandle, osWaitForever);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 1000);
	osMutexRelease(UartMutexHandle);
}
void settingNextFilter(int counter, uint8_t state) {
	// Testing color and setting next color filter
	//                s2 s3
	// RED FILTER   -> 0 0
	// BLUE FILTER 	-> 0 1
	// GREEN FILTER	-> 1 1

	if (state == 1) {	// RED
		//sprintf(printBuffer,"RED\t: %d\r\n",counter);
		//printMutexUART(printBuffer);

		//Setting next Filter : BLUE
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

	} else if (state == 2) {	// BLUE
		//sprintf(printBuffer,"BLUE\t: %d\r\n",counter);
		//printMutexUART(printBuffer);

		//Setting next Filter : GREEN
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

	} else if (state == 3) { // GREEN
		//sprintf(printBuffer,"GREEN\t: %d\r\n\n",counter);
		//printMutexUART(printBuffer);

		//Setting next Filter : RED
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
	}
}
/*
int calibrate(int counter,uint8_t color){
	int calibrate = 0;
	if(color==1){
		calibrate = counter * 255 / MAX_RED;
	} else if(color==2){
		calibrate = counter * 255 / MAX_BLUE;
	} else if(color==3){
		calibrate = counter * 255 / MAX_GREEN;
	}
	return calibrate;
}
*/
void sendDataToNodeMCU() {
	// Send lastest Data to node MCU

	// LED PIN CHECK
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	// SENDER TERMINAL
	sprintf(colorBuffer, "R : %d G: %d B :%d\r\n", globalColor[0],
			globalColor[1], globalColor[2]);
	printMutexUART(colorBuffer);
	sprintf(luxBuffer, "LUX : %d\r\n", globalLux[0]);
	printMutexUART(luxBuffer);

	// FORMAT LUX FROM INT TO STRING
	itoa(globalLux[0], luxString, 10);

	// SEND TO NODE MCU in String Format
	HAL_UART_Transmit(&huart1, globalColor, 3 * sizeof(int), 1000);
	HAL_UART_Transmit(&huart6, luxString, sizeof(luxString), 1000);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {	// WAVE CHANGEs
	if (GPIO_Pin == GPIO_PIN_13) {						// PUSH BUTTON REQEUST
		sendDataToNodeMCU();
		//HAL_Delay(100);
		while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
			;
	} else if (GPIO_Pin == GPIO_PIN_3) {		// COUNT FREQ OF LIGHT BY TIMER
		counter++; // count freq of color
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */

	// SET UP SCALING PIN
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1); // -> 1 0 means that 50% scaling
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
	// TURN ON LIGHT ON RGB
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1); // B2-LED

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of UartMutex */
	osMutexDef(UartMutex);
	UartMutexHandle = osMutexCreate(osMutex(UartMutex));

	/* definition and creation of ESPUARTMutex */
	osMutexDef(ESPUARTMutex);
	ESPUARTMutexHandle = osMutexCreate(osMutex(ESPUARTMutex));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */
	/* definition and creation of measureTimer */
	osTimerDef(measureTimer, rgbCount);
	measureTimerHandle = osTimerCreate(osTimer(measureTimer), osTimerPeriodic,
	NULL);

	/* definition and creation of autoUpdate */
	osTimerDef(autoUpdate, update);
	autoUpdateHandle = osTimerCreate(osTimer(autoUpdate), osTimerPeriodic,
	NULL);

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	osTimerStart(measureTimerHandle, 10);
	osTimerStart(autoUpdateHandle, 10000);
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of LUX */
	osThreadDef(LUX, luxFunc, osPriorityIdle, 0, 128);
	LUXHandle = osThreadCreate(osThread(LUX), NULL);

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 25000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, S0_Pin | S1_Pin | S2_Pin | S3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RGB_WAVE_Pin */
	GPIO_InitStruct.Pin = RGB_WAVE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RGB_WAVE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : S0_Pin S1_Pin S2_Pin S3_Pin */
	GPIO_InitStruct.Pin = S0_Pin | S1_Pin | S2_Pin | S3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	while (1) {
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_luxFunc */
/**
 * @brief Function implementing the LUX thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_luxFunc */
void luxFunc(void const *argument) {
	/* USER CODE BEGIN luxFunc */
	/* Infinite loop */

	for (;;) {
		ret = HAL_I2C_Mem_Write(&hi2c1, 0x39 << 1, 0xA0, I2C_MEMADD_SIZE_8BIT,
				config, 1, 100);
		if (ret != HAL_OK) {
			strcpy((char*) uart, "Error Write mem 0x0 by 0x03\r\n");
			printMutexUART(uart);
		} else {
			HAL_I2C_Master_Transmit(&hi2c1, 0x39 << 1, addr, 1, 100);
			HAL_I2C_Master_Receive(&hi2c1, 0x39 << 1, S, 2, 100);
			//			ret = HAL_I2C_Mem_Read(&hi2c1, 0x39 << 1 , 0x00, 1, R00, 1, 100);
			if (ret != HAL_OK) {
				strcpy((char*) uart, "Error Read mem 0x0 ");
				printMutexUART(uart);
			} else {
				osDelay(100);
				ret = HAL_I2C_Mem_Read(&hi2c1, 0x39 << 1, 0x8C, 1, R0C, 1, 100);
				ret = HAL_I2C_Mem_Read(&hi2c1, 0x39 << 1, 0x8D, 1, R0D, 1, 100);
				ret = HAL_I2C_Mem_Read(&hi2c1, 0x39 << 1, 0x8E, 1, R0E, 1, 100);
				ret = HAL_I2C_Mem_Read(&hi2c1, 0x39 << 1, 0x8F, 1, R0F, 1, 100);
				uint16_t data0;
				data0 = (uint16_t) R0C[0];
				data0 |= (uint16_t) (R0D[0] << 8);
				uint16_t data1;
				data1 = (uint16_t) R0E[0];
				data1 |= (uint16_t) (R0F[0] << 8);

				long ch0 = data0;
				long ch1 = data1;
				unsigned long ratio1 = 0;

				// calculate ratio and scale to 512 (2^9)
				if (ch0 != 0) {
					ratio1 = (ch1 << (RATIO_SCALE + 1)) / ch0;
				}

				// round ratio value
				unsigned long ratio = (ratio1 + 1) >> 1;

				// transform to LUX (p. 23)
				// FN package
				unsigned int b, m;

				if (ratio >= 0 && ratio <= K1T) {
					b = B1T;
					m = M1T;
				} else if (ratio <= K2T) {
					b = B2T;
					m = M2T;
				} else if (ratio <= K3T) {
					b = B3T;
					m = M3T;
				} else if (ratio <= K4T) {
					b = B4T;
					m = M4T;
				} else if (ratio <= K5T) {
					b = B5T;
					m = M5T;
				} else if (ratio <= K6T) {
					b = B6T;
					m = M6T;
				} else if (ratio <= K7T) {
					b = B7T;
					m = M7T;
				} else if (ratio <= K8T) {
					b = B8T;
					m = M8T;
				}

				unsigned long temp = (ch0 * b) - (ch1 * m);

				// avoid negativ LUX values
				if (temp < 0) {
					temp = 0;
				}

				// round the LSB (2^LUX-SCALE-1)
				temp += (1 << LUX_SCALE - 1);

				// cut the rest
				unsigned long lux = temp >> LUX_SCALE;
				globalLux[0] = (int) lux;

				if (ret != HAL_OK) {
					strcpy((char*) uart, "Error read mem 0x0C");
					printMutexUART(uart);
				} else {
					sprintf((char*) uart, "DATA0 %d DATA1 %d lux %d\r\n", data0,
							data1, lux);
				}
			}
		}
		//printMutexUART(uart);
		osDelay(1000);
	}

	/* USER CODE END luxFunc */
}

/* measure function */
void rgbCount(void const *argument) {
	/* USER CODE BEGIN measure */

	// Trigger every 10 ms to print out
	flag++;
	if (flag == 1) {				// RED
		//counter = calibrate(counter,1);
		counter = map(counter, MIN_RED, MAX_RED, 0, 255);
		countR = counter;
		/*
		if(counter<=255){
		} else {
			countR = 255;
		}*/
		settingNextFilter(counter, 1);
	} else if (flag == 2) {			// BLUE
		//counter = calibrate(counter,2);
		counter = map(counter, MIN_BLUE, MAX_BLUE, 0, 255);
		countB = counter;
		/*
		if(counter<=255){
		} else {
			countB = 255;
		}*/
		settingNextFilter(counter, 2);
	} else if (flag == 3) {			// GREEN
		//counter = calibrate(counter,3);
		counter = map(counter, MIN_GREEN, MAX_GREEN, 0, 255);
		countG = counter;
		/*
		if(counter<=255){
		} else {
			countG = 255;
		}*/
		settingNextFilter(counter, 3);
	} else if (flag == 4) {			// Update global and RESET
		// PUT IN Global Buffer
		globalColor[0] = countR;
		globalColor[1] = countG;
		globalColor[2] = countB;
		// RESET
		flag = 0;
		countR = countB = countG = 0;
	}
	counter = 0;					// RESET COUNTER after count any color
	/* USER CODE END measure */
}

/* update function */
void update(void const *argument) {
	/* USER CODE BEGIN update */

	// Auto Update Every 10 Second
	printMutexUART("\n--UPDATE--\r\n");
	sendDataToNodeMCU();
	printMutexUART("----------\r\n");
	osDelay(100);

	/* USER CODE END update */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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

