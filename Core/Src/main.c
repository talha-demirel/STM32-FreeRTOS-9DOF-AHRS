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
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "qmc5883p.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#ifdef USE_SEGGER_SYSVIEW
  #include "SEGGER_SYSVIEW.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACCEL_FLASH_SECTOR				FLASH_SECTOR_11
#define MAG_FLASH_SECTOR				FLASH_SECTOR_10

#define FLASH_SECTOR_11_ADDRESS			0x080E0000
#define FLASH_SECTOR_10_ADDRESS			0x080C0000

#define ACCEL_FLASH_SECTOR_ADDRESS		FLASH_SECTOR_11_ADDRESS
#define MAG_FLASH_SECTOR_ADDRESS		FLASH_SECTOR_10_ADDRESS
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_HandleTypeDef hmpu;
QMC5883P_HandleTypeDef hqmc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Bus_Recovery(GPIO_TypeDef *sclPort, uint16_t sclPin,
                       GPIO_TypeDef *sdaPort, uint16_t sdaPin);
void MPU6050_Print_Wrapper(const char* text);
void MPU6050_Button_Wait_Wrapper(void);

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  __HAL_DBGMCU_FREEZE_IWDG();
  __HAL_DBGMCU_FREEZE_TIM2();

  I2C_Bus_Recovery(GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7);
  MX_I2C1_Init();


  MPU6050_Set_Default(&hmpu);
  hmpu.hi2c = &hi2c1;


  if (MPU6050_Init(&hmpu) == HAL_OK) {
	  printf("[OK] MPU6050 initialized successfully!\r\n");
  } else {
      printf("[ERROR] Failed to initialize MPU6050\r\n");
      Error_Handler();
  }

  if (MPU6050_I2C_Bypass_Enable(&hmpu) == HAL_OK) {
  	  printf("[OK] MPU6050 I2C bypass enabled!\r\n");
  } else {
	  printf("[ERROR] Failed to enable MPU6050 I2C bypass!\r\n");
	  Error_Handler();
  }

  if (MPU6050_DRDY_INT_Enable(&hmpu) == HAL_OK) {
	  printf("[OK] MPU6050 DRDY interrupt enabled!\r\n");
  } else {
	  printf("[ERROR] Failed to enable MPU6050 DRDY interrupt!\r\n");
	  Error_Handler();
  }

  if (MPU6050_Gyro_Calibrate(&hmpu) == HAL_OK) {
      printf("[OK] Gyroscope calibrated succesfully!\r\n");
  } else {
      printf("[ERROR] Failed to calibrate gyroscope!\r\n");
      Error_Handler();
  }



  if (LoadAccelCalibrationFromFlash(&hmpu,ACCEL_FLASH_SECTOR_ADDRESS) == HAL_OK) {
	  printf("[OK] Accelerometer calibration loaded successfully!\r\n");
  }else{
	  printf("[ERROR] Failed to load accelerometer calibration! \r\n");
	  if(MPU6050_Accel_Calibrate_6pts(&hmpu,MPU6050_Button_Wait_Wrapper,MPU6050_Print_Wrapper) == HAL_OK){
		  printf("[OK] Accelerometer calibrated succesfully!\r\n");
		  if(SaveAccelCalibrationToFlash(&hmpu, ACCEL_FLASH_SECTOR, ACCEL_FLASH_SECTOR_ADDRESS) == HAL_OK){
			  printf("[OK] Accelerometer calibration saved to flash successfully!\r\n");
		  }
		  else{
			  printf("[ERROR] Failed to save accelerometer calibration to flash!\r\n");
		  }
	  }
	  else{
		  printf("[ERROR] Failed to calibrate accelerometer!\r\n");
	  }
  }

  /* A button can be added or the this block can be uncommented to calibrate accelerometer */
//  if(MPU6050_Accel_Calibrate_6pts(&hmpu,MPU6050_Button_Wait_Wrapper,MPU6050_Print_Wrapper) == HAL_OK){
//  	printf("[OK] Accelerometer calibrated succesfully!\r\n");
//  	if(SaveAccelCalibrationToFlash(&hmpu, ACCEL_FLASH_SECTOR, ACCEL_FLASH_SECTOR_ADDRESS) == HAL_OK){
//  		printf("[OK] Accelerometer calibration saved to flash successfully!\r\n");
//  	}
//  	else{
//  		printf("[ERROR] Failed to save accelerometer calibration to flash!\r\n");
//  	}
//  }
//  else{
//  	printf("[ERROR] Failed to calibrate accelerometer!\r\n");
//  }


  QMC5883P_Set_Default(&hqmc);
  hqmc.hi2c = &hi2c1;
  if(QMC5883P_Init(&hqmc) == HAL_OK){
	  printf("[OK] Magnetometer initialized successfully!\r\n");

  }
  else{
	   printf("[ERROR] Failed to initialize magnetometer!\r\n");
	   Error_Handler();
  }

  if (LoadMagCalibrationFromFlash(&hqmc,MAG_FLASH_SECTOR_ADDRESS) == HAL_OK) {

  	  printf("[OK] Magnetometer calibration loaded successfully!\r\n");

    }else{
  	  printf("[ERROR] Failed to load magnetometer calibration! \r\n");
  	  printf("Calibration started 50 sec!\r\n");
  	  if(QMC5883P_Calibrate(&hqmc, 50000) == HAL_OK){
  		  printf("[OK] Magnetometer calibrated succesfully!\r\n");
  		  if(SaveMagCalibrationToFlash(&hqmc, MAG_FLASH_SECTOR, MAG_FLASH_SECTOR_ADDRESS) == HAL_OK){
  			  printf("[OK] Magnetometer calibration saved to flash successfully!\r\n");
  		  }
  		  else{
  			  printf("[ERROR] Failed to save magnetometer calibration to flash!\r\n");
  		  }
  	  } else{

  		  printf("[ERROR] Failed to calibrate magnetometer!\r\n");
  		  Error_Handler();
  	  }
    }

  /* A button can be added or the this block can be uncommented to calibrate magnetometer */
//  printf("Calibration started 50 sec!\r\n");
//  if(QMC5883P_Calibrate(&hqmc, 50000) == HAL_OK){
//	  printf("[OK] Magnetometer calibrated succesfully!\r\n");
//	  if(SaveMagCalibrationToFlash(&hqmc, MAG_FLASH_SECTOR, MAG_FLASH_SECTOR_ADDRESS) == HAL_OK){
//		  printf("[OK] Magnetometer calibration saved to flash successfully!\r\n");
//	  }
//	  else{
//		  printf("[ERROR] Failed to save magnetometer calibration to flash!\r\n");
//	  }
//  } else{
//
//	printf("[ERROR] Failed to calibrate magnetometer!\r\n");
//	Error_Handler();;
//  }

#ifdef USE_SEGGER_SYSVIEW
  SEGGER_SYSVIEW_Conf();
  vSetVarulMaxPRIGROUPValue();
  SEGGER_SYSVIEW_Start();
#endif

  HAL_TIM_Base_Start(&htim2);

  MX_IWDG_Init();


  /* USER CODE END 2 */

  /* Init scheduler */
  MX_FREERTOS_Init();

  /* Start scheduler */
  vTaskStartScheduler();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

static void GPIO_Clock_Enable(GPIO_TypeDef *port)
{
    if (port == GPIOA)      __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();

}
void I2C_Bus_Recovery(GPIO_TypeDef *sclPort, uint16_t sclPin,
                       GPIO_TypeDef *sdaPort, uint16_t sdaPin){

	GPIO_Clock_Enable(sdaPort);
	GPIO_Clock_Enable(sclPort);

    GPIO_InitTypeDef gpio = {0};

    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = sclPin; HAL_GPIO_Init(sclPort, &gpio);
    gpio.Pin = sdaPin; HAL_GPIO_Init(sdaPort, &gpio);

    HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);
    HAL_Delay(1);

    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(sdaPort, sdaPin) == GPIO_PIN_RESET; i++)
    {
        HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_SET);
    HAL_Delay(1);

}

void MPU6050_Print_Wrapper(const char* text) {
    printf("%s", text);
}

void MPU6050_Button_Wait_Wrapper(void) {
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) { }
    HAL_Delay(50);
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) { }
    HAL_Delay(50);
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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

  HAL_Delay(100);

  __disable_irq();
  NVIC_SystemReset();
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
