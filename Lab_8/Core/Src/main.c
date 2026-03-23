/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdarg.h"  
#include "stdio.h"   
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_spi.h"
#include <stdint.h>
#include <strings.h>
#include <string.h>
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myPrintf(const char* fmt, ...) 
{
  char buffer[512];

  // Step 2: Initialize the variadic argument list.
  va_list args;
  va_start(args, fmt);

  // Step 3: Format the final string using vsnprintf.
  // vsnprintf is "v" (takes a va_list) and "n" (prevents buffer overflow).
  int length = vsnprintf(buffer, sizeof(buffer), fmt, args);

  // Clean up the variadic argument list
  va_end(args);

  // Step 4: Transmit the string over UART using HAL_UART_Transmit.
  // We cast buffer to (uint8_t*) as required by the HAL library.
  if (length > 0) {
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, (uint16_t)length, HAL_MAX_DELAY);
  }
};


// TASK 2
// #define OUT_TEMP 0x26
// #define READ 0x80 
// uint8_t tx_buffer = READ | OUT_TEMP;
// uint8_t rx_buffer;
// int rx_complete = 0;

// //tx_buffer for temperature

// // If this function is called, that means 
// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
//   if (hspi->Instance == SPI1) {
//     HAL_SPI_Receive_IT(&hspi1, &rx_buffer, sizeof(rx_buffer));
//   } 
// };

// void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
//   if (hspi->Instance == SPI1) {
//     // End communication
//     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    
//     // Start new communication
//     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
//     HAL_SPI_Transmit_IT (&hspi1 , &tx_buffer , sizeof(tx_buffer));
    
//     //This can cause issues UART is a bit slower than SPI, so we will print in main loop
//     myPrintf("%00X \r\n", rx_buffer);
//   }
// }

// // We also have to enable the sensors 
// #define CTRL_REG1 0x20
// #define CTRL_REG1_VAL 0b00001111
// uint8_t specialtx[2] = { CTRL_REG1 , CTRL_REG1_VAL }; 
// //This will be called only once
// void start_gyro_reading() {

//     // We send the command to read temperature
//     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
//     HAL_SPI_Transmit(&hspi1, specialtx, sizeof(specialtx), HAL_MAX_DELAY);
//     HAL_GPIO_WritePin (GPIOE , GPIO_PIN_3 , GPIO_PIN_SET );

//     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
//     HAL_SPI_Transmit_IT(&hspi1, &tx_buffer, sizeof(tx_buffer));
// }

//Task 3
// --- Definitions ---
#define CTRL_REG1     0x20
#define CTRL_REG1_VAL 0b10001111 // Power on, 100Hz, all axes enabled
#define OUT_TEMP      0x26
#define READ 0x80 
#define MS_BIT        0x40 // Required for multi-byte reads

// Address 0x26 | READ | MS = 0xE6
// This will read Temp, Status, XL, XH, YL, YH, ZL, ZH in one go (8 bytes)
uint8_t tx_addr_packet = READ | MS_BIT | OUT_TEMP;
uint8_t rx_buffer[8]; // [0]=Temp, [1]=Status, [2-7]=X,Y,Z
volatile int data_ready = 0;

void start_gyro_reading() {
    // 1. Initialization (Blocking is fine here as it's once)
    uint8_t init_data[2] = { CTRL_REG1, CTRL_REG1_VAL };
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, init_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    // 2. Start the first non-blocking Interrupt chain
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi1, &tx_addr_packet, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        // Address sent, now receive the 8-byte burst
        HAL_SPI_Receive_IT(&hspi1, rx_buffer, 8);
    } 
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        // End current SPI transaction
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        
        data_ready = 1; // Signal main loop to process and print

        // Optional: Re-trigger immediately or via a Timer
        // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        // HAL_SPI_Transmit_IT(&hspi1, &tx_addr_packet, 1);
    }
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Task 1
  // 0x80 = Read operation
  // 0x0F = WHO_AM_I register for i3g4250d
  // uint8_t operation = 0x80 | 0x0F;
  // uint8_t dataRecieved;

  // Task 2
  start_gyro_reading();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // TASK 1
    // As we are doing polling, every loop we will Transmit and recieve also.
    // SET GPIO PE3 to LOW to recieve from i3g4250d -> Acts as selection, and start of communication
    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    // // SPI 1
    // HAL_SPI_Transmit(&hspi1, &operation, 1,10);
    // HAL_SPI_Receive(&hspi1, &dataRecieved, 1, 10);

    // // SET GPIO PE3 to HIGH to end communication;
    // HAL_GPIO_WritePin(GPIOE , GPIO_PIN_3 , GPIO_PIN_SET);


    // myPrintf("Sensor Data (i3g4250d): %003X \r\n", dataRecieved);
    // HAL_Delay(100);


    // Task 2
    HAL_Delay(80);

    // Task 3
    if (data_ready) {
        // 1. Combine bytes (Little Endian)
        int16_t x_raw = (int16_t)((rx_buffer[3] << 8) | rx_buffer[2]);
        int16_t y_raw = (int16_t)((rx_buffer[5] << 8) | rx_buffer[4]);
        int16_t z_raw = (int16_t)((rx_buffer[7] << 8) | rx_buffer[6]);
        uint8_t temp  = rx_buffer[0];

        // 2. Convert to DPS (Sensitivity 8.75 mdps/digit)
        float x_dps = x_raw * 0.00875f;
        float y_dps = y_raw * 0.00875f;
        float z_dps = z_raw * 0.00875f;

        // 3. Print in Decimal format for your Python script
        // Note: Use %d and %f, NOT %X, so Python's int() works!
        myPrintf("%d,%.2f,%.2f,%.2f\r\n", temp, x_dps, y_dps, z_dps);

        data_ready = 0;

        // 4. Trigger next read (if not doing it in callback)
        HAL_Delay(50); // Control sample rate so you don't overwhelm UART
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_SPI_Transmit_IT(&hspi1, &tx_addr_packet, 1);
  }
  /* USER CODE END 3 */
}
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
