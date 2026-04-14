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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// --- Definitions ---
#define RAD_TO_DEG          57.29577951f

// I3G4250D Gyroscope (SPI)
#define GYRO_WHO_AM_I       0x0F
#define GYRO_WHO_AM_I_VAL   0xD3
#define GYRO_CTRL_REG1      0x20
#define GYRO_CTRL_REG1_VAL  0x8F  // Power on, Enable X, Y, Z, 100Hz ODR
#define GYRO_OUT_X_L        0x28

// LSM303AGR Accelerometer (I2C)
// The 7-bit I2C address is 0x19 (0011001b). HAL requires it left-shifted by 1 -> 0x32
#define ACCEL_I2C_ADDR      (0x19 << 1)
#define ACCEL_WHO_AM_I      0x0F
#define ACCEL_WHO_AM_I_VAL  0x33
#define ACCEL_CTRL_REG1     0x20
#define ACCEL_CTRL_REG1_VAL 0x67  // 100Hz, Normal mode, X/Y/Z enabled
#define ACCEL_CTRL_REG4     0x23
#define ACCEL_CTRL_REG4_VAL 0x00  // Continuous update, +/- 2g, Normal mode
#define ACCEL_OUT_X_L       0x28

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// --- Data Structures ---
typedef struct {
    int16_t raw_acc_x, raw_acc_y, raw_acc_z;
    float acc_x, acc_y, acc_z;
    float acc_offset_x, acc_offset_y, acc_offset_z;

    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    float gyro_x, gyro_y, gyro_z;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;

    float angle_x;
    float angle_y;
    float angle_z;


} SensorData_t;

SensorData_t sensor = {0};

// --- SPI Interrupt Variables ---
volatile uint8_t spi_tx_buf[7];
volatile uint8_t spi_rx_buf[7];
volatile uint8_t spi_ready = 1;


// --- Functions ---

void myPrintf(const char* fmt, ...) {
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    int length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    
    if (length > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, (uint16_t)length, HAL_MAX_DELAY);
    }
}

// SPI Tx/Rx Complete Callback for Gyroscope
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        // Pull CS High to end communication
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); 
        spi_ready = 1;
    }
}

void gyro_init(void)
{
    // 0x80 = Read operation bit
    // 0x0F = WHO_AM_I register address
    uint8_t operation = 0x80 | 0x0F; 
    uint8_t dataRecieved = 0;

    // 1. Pull CS Low to start communication
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

    // 2. Transmit the register address (with the Read bit set)
    // We use a small timeout (10ms) as requested
    HAL_SPI_Transmit(&hspi1, &operation, 1, 10);

    // 3. Receive the data from the sensor
    // The SPI clock continues to run, allowing the sensor to push its ID onto MISO
    HAL_SPI_Receive(&hspi1, &dataRecieved, 1, 10);

    // 4. Pull CS High to end communication
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    // Print the result. Expected value for i3g4250d is 0xD3 (211 decimal)
    myPrintf("Sensor Data (i3g4250d) WHO_AM_I: 0x%02X \r\n", dataRecieved);

    // Proceed with actual configuration if the sensor is correct
    if (dataRecieved == 0xD3) {
        uint8_t config[2] = { GYRO_CTRL_REG1, GYRO_CTRL_REG1_VAL };
        
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, config, 2, 10);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        
        myPrintf("Gyroscope Configured Successfully.\r\n");
    } else {
        myPrintf("Error: Gyroscope not found or SPI communication failed.\r\n");
    }

    HAL_Delay(100);
}

void accel_init(void) {
    uint8_t id = 0;
    
    // Check WHO_AM_I
    HAL_I2C_Mem_Read(&hi2c1, ACCEL_I2C_ADDR, ACCEL_WHO_AM_I, 1, &id, 1, HAL_MAX_DELAY);
    if (id == ACCEL_WHO_AM_I_VAL) {
        myPrintf("LSM303AGR (Accel) OK\r\n");
    } else {
        myPrintf("LSM303AGR (Accel) Fail: 0x%02X\r\n", id);
    }

    // Configure CTRL_REG1_A and CTRL_REG4_A
    uint8_t ctrl1 = ACCEL_CTRL_REG1_VAL;
    HAL_I2C_Mem_Write(&hi2c1, ACCEL_I2C_ADDR, ACCEL_CTRL_REG1, 1, &ctrl1, 1, HAL_MAX_DELAY);
    
    uint8_t ctrl4 = ACCEL_CTRL_REG4_VAL;
    HAL_I2C_Mem_Write(&hi2c1, ACCEL_I2C_ADDR, ACCEL_CTRL_REG4, 1, &ctrl4, 1, HAL_MAX_DELAY);
}

void calibrate_sensors(SensorData_t *s) {
    myPrintf("Calibrating... Do not move the board.\r\n");
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint8_t samples = 20;

    for (int i = 0; i < samples; i++) {
        // --- Read Accel (Blocking) ---
        uint8_t raw_acc[6];
        // 0x80 sets the MSB to 1 for I2C auto-increment
        HAL_I2C_Mem_Read(&hi2c1, ACCEL_I2C_ADDR, ACCEL_OUT_X_L | 0x80, 1, raw_acc, 6, HAL_MAX_DELAY);
        int16_t rax = (raw_acc[1] << 8) | raw_acc[0];
        int16_t ray = (raw_acc[3] << 8) | raw_acc[2];
        int16_t raz = (raw_acc[5] << 8) | raw_acc[4];

        // 3.9 mg/LSB -> converted to g
        sum_ax += (rax * 3.9f) / 1000.0f;
        sum_ay += (ray * 3.9f) / 1000.0f;
        sum_az += ((raz * 3.9f) / 1000.0f) - 1.0f; // Subtract 1g for Gravity

        // --- Read Gyro (Blocking for calibration) ---
        // 0xC0 = 0x80 (Read) | 0x40 (Auto-increment address)
        uint8_t tx[7] = {GYRO_OUT_X_L | 0xC0}; 
        uint8_t rx[7] = {0};
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        
        int16_t rgx = (rx[2] << 8) | rx[1];
        int16_t rgy = (rx[4] << 8) | rx[3];
        int16_t rgz = (rx[6] << 8) | rx[5];

        // 8.75 mdps/LSB -> converted to dps
        sum_gx += (rgx * 8.75f) / 1000.0f;
        sum_gy += (rgy * 8.75f) / 1000.0f;
        sum_gz += (rgz * 8.75f) / 1000.0f;

        HAL_Delay(50);
    }

    s->acc_offset_x = sum_ax / samples;
    s->acc_offset_y = sum_ay / samples;
    s->acc_offset_z = sum_az / samples; 

    s->gyro_offset_x = sum_gx / samples;
    s->gyro_offset_y = sum_gy / samples;
    s->gyro_offset_z = sum_gz / samples;

    myPrintf("Calibration complete.\r\n");
}

// Initiates Non-Blocking Gyro Read via IT
void request_gyro_it(void) {
    if (spi_ready) {
        spi_ready = 0;
        // 0x80 (Read) | 0x40 (Auto Increment)
        spi_tx_buf[0] = GYRO_OUT_X_L | 0xC0; 
        
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // Pull CS Low
        HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx_buf, (uint8_t*)spi_rx_buf, 7);
    }
}

// Parses Gyro data after IT finishes
void process_gyro(SensorData_t *s) {
    s->raw_gyro_x = (int16_t)((spi_rx_buf[2] << 8) | spi_rx_buf[1]);
    s->raw_gyro_y = (int16_t)((spi_rx_buf[4] << 8) | spi_rx_buf[3]);
    s->raw_gyro_z = (int16_t)((spi_rx_buf[6] << 8) | spi_rx_buf[5]);

    s->gyro_x = ((s->raw_gyro_x * 8.75f) / 1000.0f) - s->gyro_offset_x;
    s->gyro_y = ((s->raw_gyro_y * 8.75f) / 1000.0f) - s->gyro_offset_y;
    s->gyro_z = ((s->raw_gyro_z * 8.75f) / 1000.0f) - s->gyro_offset_z;
}

// Blocks to read I2C Accel
void read_accel(SensorData_t *s) {
    uint8_t raw_acc[6];
    // 0x80 enables register auto-increment
    HAL_I2C_Mem_Read(&hi2c1, ACCEL_I2C_ADDR, ACCEL_OUT_X_L | 0x80, 1, raw_acc, 6, HAL_MAX_DELAY);
    
    s->raw_acc_x = (int16_t)((raw_acc[1] << 8) | raw_acc[0]);
    s->raw_acc_y = (int16_t)((raw_acc[3] << 8) | raw_acc[2]);
    s->raw_acc_z = (int16_t)((raw_acc[5] << 8) | raw_acc[4]);

    s->acc_x = ((s->raw_acc_x * 3.9f) / 1000.0f) - s->acc_offset_x;
    s->acc_y = ((s->raw_acc_y * 3.9f) / 1000.0f) - s->acc_offset_y;
    s->acc_z = ((s->raw_acc_z * 3.9f) / 1000.0f) - s->acc_offset_z;

    // Estimate X angle in degrees using Atan2
    s->angle_x = atan2f(s->acc_y, sqrtf(s->acc_x * s->acc_x + s->acc_z * s->acc_z)) * RAD_TO_DEG;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  HAL_Delay(100);

  // 2. Sensor Initialization
  gyro_init();
  accel_init();

  // 3. Calibration
  calibrate_sensors(&sensor);

  float filtered_angle_x = 0;
  float filtered_angle_y = 0;
  float filtered_angle_z = 0;
  float alpha = 0.98f;
  float dt = 0.1f; // 100ms delay

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    request_gyro_it();
        
        // Read I2C Accelerometer while SPI is transmitting in the background
        read_accel(&sensor);
        
        // Wait for SPI transfer to finish to safely parse gyro values
        while (!spi_ready) {
            // Yield / wait
        }
        process_gyro(&sensor);
        
        // Print requested outputs (gyro_x angular rate, acc_x tilt angle)
        //myPrintf("%.2f,%.2f\r\n", sensor.gyro_x, sensor.angle_x);
        filtered_angle_x = alpha * (filtered_angle_x + sensor.gyro_x * dt) + ((1.0f - alpha) * sensor.angle_x);
        filtered_angle_y = alpha * (filtered_angle_y + sensor.gyro_y * dt) + ((1.0f - alpha) * sensor.angle_y);
        filtered_angle_z = alpha * (filtered_angle_z + sensor.gyro_z * dt) + ((1.0f - alpha) * sensor.angle_z);
        // Print the fused, stable result
        myPrintf("%.2f , %.2f, %.2f \r\n", filtered_angle_x, filtered_angle_y, filtered_angle_z);
        //myPrintf("%.2f , %.2f, %.2f \r\n", sensor.angle_x, sensor.gyro_x, filtered_angle_x);
        HAL_Delay(100);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
