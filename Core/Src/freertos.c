/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "madgwick.h"
#include "mpu6050.h"
#include "qmc5883p.h"
#include "iwdg.h"
#include "queue.h"
#include "event_groups.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief Structure to hold converted IMU data
 */
typedef struct {
    float AccelXYZ_g[3];
    float GyroXYZ_radps[3];
    float temp;
    uint32_t timestamp_us;
} IMU_Data_Batch_t;

/**
 * @brief Structure to hold magnetic sensor data
 */
typedef struct {
    float MagXYZ_Gauss[3];
    uint32_t timestamp_us;
} Mag_Data_Batch_t;

/**
 * @brief Structure to hold Quaternion representation of orientation
 */
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_t;

/**
 * @brief Structure to hold Euler angle representation of orientation
 */
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} EulerAngles_t;

/**
 * @brief Master telemetry structure combining all sensor fusions
 */
typedef struct {
    IMU_Data_Batch_t    imu_data_batch;
    Mag_Data_Batch_t    mag_data_batch;
    EulerAngles_t       EulerAngles;
    Quaternion_t        Quaternion;
} Telemetry_Data_Batch_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Task Notification Bits for IMU Task */
#define IMU_EXTI_READY_BIT          (1 << 0)
#define IMU_DMA_CMPLT_BIT           (1 << 1)
#define MAG_DMA_CMPLT_BIT           (1 << 2)

/* Watchdog Event Group Bits */
#define IMU_TASK_ALIVE_BIT          (1 << 0)
#define AHRS_TASK_ALIVE_BIT         (1 << 1)
#define TELEMETRY_TASK_ALIVE_BIT    (1 << 2)

#define ALL_TASKS_ALIVE_MASK        (IMU_TASK_ALIVE_BIT | AHRS_TASK_ALIVE_BIT | TELEMETRY_TASK_ALIVE_BIT)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD                  (M_PI / 180.0f)
#define G_TO_MPS2                   9.80665f

#define CURRENT_I2C_DEV_IMU         0
#define CURRENT_I2C_DEV_MAG         1

#define BETA_9DOF                   0.041f
#define BETA_6DOF 					0.15f



#define EXPECTED_MAG_NORM    		0.47f 	// Ankara/Turkiye
#define MAG_TOLERANCE        		0.15f
#define MAG_VALID_DEBOUNCE_COUNT   	20
#define MAG_TRANSITION_TIME_S      	1.5f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* External Handlers */
extern TIM_HandleTypeDef htim2;
extern MPU6050_HandleTypeDef hmpu;
extern QMC5883P_HandleTypeDef hqmc;
extern UART_HandleTypeDef huart2;

/* FreeRTOS Handles */
EventGroupHandle_t xWatchdogEventGroup;

TaskHandle_t hahrs_task;
TaskHandle_t himu_task;
TaskHandle_t hmonitor_task;
TaskHandle_t htelemetry_task;

QueueHandle_t xIMUQueue;
QueueHandle_t xMagQueue;
QueueHandle_t xTelemetryQueue;

/* State Variables */
static volatile uint8_t current_i2c_device = 0;
static volatile uint32_t isr_imu_timestamp_us = 0;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vAHRS_Task(void *pvParameters);
void vIMU_Task(void *pvParameters);
void vTelemetry_Task(void *pvParameters);
void vMonitor_Task(void *pvParameters);

uint32_t Get_us(TIM_HandleTypeDef *htim);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	__disable_irq();
	printf("[FATAL ERROR] STACK OVERFLOW DETECTED IN TASK: %s\r\n", pcTaskName);
	NVIC_SystemReset();
}
/* USER CODE END 4 */

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
    xIMUQueue = xQueueCreate(1, sizeof(IMU_Data_Batch_t));
    xMagQueue = xQueueCreate(1, sizeof(Mag_Data_Batch_t));
    xTelemetryQueue = xQueueCreate(1, sizeof(Telemetry_Data_Batch_t));

    if (xIMUQueue == NULL || xMagQueue == NULL || xTelemetryQueue == NULL) {
        Error_Handler();
    }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* USER CODE BEGIN RTOS_THREADS */

    if (xTaskCreate(vMonitor_Task, "Monitor_task", 128, NULL, 5, &hmonitor_task) != pdPASS) {
        Error_Handler();
    }
    if (xTaskCreate(vAHRS_Task, "AHRS_task", 1024, NULL, 4, &hahrs_task) != pdPASS) {
        Error_Handler();
    }
    if (xTaskCreate(vIMU_Task, "IMU_task", 256, NULL, 3, &himu_task) != pdPASS) {
        Error_Handler();
    }
    if (xTaskCreate(vTelemetry_Task, "Telemetry_task", 512, NULL, 1, &htelemetry_task) != pdPASS) {
        Error_Handler();
    }

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

    xWatchdogEventGroup = xEventGroupCreate();
    if (xWatchdogEventGroup == NULL) {
        Error_Handler();
    }

  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief Attitude and Heading Reference System (AHRS) Task
 *        Processes raw sensor data through the Madgwick filter to obtain orientation.
 */
void vAHRS_Task(void *pvParameters) {

    MadgwickFilter_t filter = {0};
    IMU_Data_Batch_t imu_data = {0};
    Mag_Data_Batch_t mag_data = {0};
    EulerAngles_t euler_angles = {0};
    Telemetry_Data_Batch_t telemetry_data = {0};

    float dt_imu;
    uint32_t last_imu_timestamp_us = Get_us(&htim2);
    uint32_t last_mag_timestamp_us = Get_us(&htim2);
    uint8_t mag_error_flag = 1;

    uint16_t mag_valid_streak   = 0;
    uint8_t  prev_mag_error_flag = 1;
    float    mag_transition_elapsed = 0.0f;

    MadgwickInit(&filter, BETA_6DOF);

    for (;;) {

        if (xQueueReceive(xIMUQueue, &imu_data, portMAX_DELAY) == pdTRUE) {


            uint32_t dt_imu_us = imu_data.timestamp_us - last_imu_timestamp_us;
            dt_imu = (float)dt_imu_us / 1000000.0f;
            last_imu_timestamp_us = imu_data.timestamp_us;


            if (dt_imu > 0.05f || dt_imu <= 0.0f) {
                dt_imu = 0.002f; /* Fallback to 500Hz nominal */
            }

            if (xQueueReceive(xMagQueue, &mag_data, 0) == pdTRUE) {

                last_mag_timestamp_us = mag_data.timestamp_us;
                mag_error_flag = 0;

                float mag_norm = sqrtf((mag_data.MagXYZ_Gauss[0] * mag_data.MagXYZ_Gauss[0]) +
                                       (mag_data.MagXYZ_Gauss[1] * mag_data.MagXYZ_Gauss[1]) +
                                       (mag_data.MagXYZ_Gauss[2] * mag_data.MagXYZ_Gauss[2]));
                uint8_t norm_ok = (mag_norm <= (EXPECTED_MAG_NORM + MAG_TOLERANCE)) &&
                                  (mag_norm >= (EXPECTED_MAG_NORM - MAG_TOLERANCE));
                if (norm_ok) {
                	if (mag_valid_streak < 0xFFFF) mag_valid_streak++;
                } else {
                	mag_valid_streak = 0;
                }

                mag_error_flag = (mag_valid_streak >= MAG_VALID_DEBOUNCE_COUNT) ? 0 : 1;

            } else {
                /* Check if Magnetometer data has timed out (Timeout: 0.5s) */
                uint32_t time_since_last_mag_us = Get_us(&htim2) - last_mag_timestamp_us;
                if (time_since_last_mag_us > 500000) {
                    mag_error_flag = 1;
                    mag_valid_streak = 0;
                }
            }

            if (!mag_error_flag) {
            	if(prev_mag_error_flag){	//	6DOF->9DOF
            		mag_transition_elapsed = 0.0f;
            	}
            	if (mag_transition_elapsed < MAG_TRANSITION_TIME_S) {
            		mag_transition_elapsed += dt_imu;
            	}

            	float ramp = mag_transition_elapsed / MAG_TRANSITION_TIME_S;
            	if (ramp > 1.0f) ramp = 1.0f;

            	filter.beta = ramp * BETA_9DOF;   /* 0 -> BETA_9DOF */
                MadgwickAHRSUpdate(&filter, imu_data.GyroXYZ_radps, imu_data.AccelXYZ_g, mag_data.MagXYZ_Gauss, dt_imu);
            } else {
                /* Fallback to 6-DOF IMU only if magnetometer fails/timeouts */
            	filter.beta = BETA_6DOF;
                MadgwickAHRSUpdateIMU(&filter, imu_data.GyroXYZ_radps, imu_data.AccelXYZ_g, dt_imu);
            }
            prev_mag_error_flag = mag_error_flag;


            /* Prepare telemetry packet */
            MadgwickGetEulerAngles(&filter, &euler_angles.roll_deg, &euler_angles.pitch_deg, &euler_angles.yaw_deg);
            telemetry_data.EulerAngles = euler_angles;
            telemetry_data.EulerAngles.pitch_deg = -telemetry_data.EulerAngles.pitch_deg; /* Pitch axis alignment */
            telemetry_data.imu_data_batch = imu_data;
            telemetry_data.mag_data_batch = mag_data;
            MadgwickGetQuaternions(&filter, &telemetry_data.Quaternion.q0,
                                            &telemetry_data.Quaternion.q1,
                                            &telemetry_data.Quaternion.q2,
                                            &telemetry_data.Quaternion.q3);


            xQueueOverwrite(xTelemetryQueue, &telemetry_data);
        }

        /* Signal watchdog that AHRS task is alive */
        xEventGroupSetBits(xWatchdogEventGroup, AHRS_TASK_ALIVE_BIT);
    }
}

/**
 * @brief IMU Data Acquisition Task
 *
 */
void vIMU_Task(void *pvParameters) {

    IMU_Data_Batch_t imu_batch = {0};
    Mag_Data_Batch_t mag_batch = {0};
    uint8_t mag_counter = 0;
    uint32_t notificationVal = 0;

    static uint8_t rawIMUDatas[14];
    static uint8_t rawMagDatas[6];

    /* State Machine Flags */
    uint8_t i2c_dma_is_busy = 0, imu_is_pending = 0, mag_is_pending = 0;

    __HAL_GPIO_EXTI_CLEAR_IT(INTA_Pin);
    HAL_NVIC_EnableIRQ(INTA_EXTI_IRQn);
    MPU6050_Is_Data_Ready(&hmpu); /* Clear initial INT status flag */

    for (;;) {

        /* Wait for Task Notifications from EXTI or DMA callbacks */
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notificationVal, pdMS_TO_TICKS(100)) == pdTRUE) {

            /* Hardware Data Ready (EXTI Triggered) */
            if (notificationVal & IMU_EXTI_READY_BIT) {
                imu_is_pending = 1;
                /* Decimate Magnetometer readings (Read 1 Mag for every 3 IMU readings) */
                if (++mag_counter >= 3) {
					mag_batch.timestamp_us = isr_imu_timestamp_us;
					mag_is_pending = 1;
					mag_counter = 0;
                }
            }

            /* IMU DMA Read Completed */
            if (notificationVal & IMU_DMA_CMPLT_BIT) {
                i2c_dma_is_busy = 0;

                MPU6050_Parse(&hmpu, rawIMUDatas);
                MPU6050_Accel_raw_to_g(&hmpu);
                MPU6050_Gyro_raw_to_dps(&hmpu);

                for (uint8_t i = 0; i < 3; i++) {
                    imu_batch.AccelXYZ_g[i] = hmpu.Accel.axyz_g[i];
                    imu_batch.GyroXYZ_radps[i] = hmpu.Gyro.xyz_dps[i] * DEG_TO_RAD;
                }
                imu_batch.temp = hmpu.temp;

                imu_batch.timestamp_us = isr_imu_timestamp_us;
                xQueueOverwrite(xIMUQueue, &imu_batch);
            }

            /* Magnetometer DMA Read Completed */
            if (notificationVal & MAG_DMA_CMPLT_BIT) {
                i2c_dma_is_busy = 0;

                Mag_Parse(&hqmc, rawMagDatas);
                Mag_raw_to_gauss(&hqmc);

                /* Axis Alignment: Map QMC5883P physical axes to MPU6050 coordinate frame */
                mag_batch.MagXYZ_Gauss[0] = hqmc.Data.xyz_gauss[1];
                mag_batch.MagXYZ_Gauss[1] = -hqmc.Data.xyz_gauss[0];
                mag_batch.MagXYZ_Gauss[2] = hqmc.Data.xyz_gauss[2];

                /* Guard against all-zero faulty readings */
                if (!(mag_batch.MagXYZ_Gauss[0] == 0 && mag_batch.MagXYZ_Gauss[1] == 0 && mag_batch.MagXYZ_Gauss[2] == 0)) {
                    xQueueOverwrite(xMagQueue, &mag_batch);
                }
            }

            /* I2C Bus Management */
            if (i2c_dma_is_busy == 0) {
                if (imu_is_pending == 1) {
					current_i2c_device = CURRENT_I2C_DEV_IMU;
                    if(MPU6050_Read_raw_DMA(&hmpu, rawIMUDatas) == HAL_OK){
						i2c_dma_is_busy = 1;
						imu_is_pending = 0;
                    }
                    __HAL_DMA_DISABLE_IT(hmpu.hi2c->hdmarx, DMA_IT_HT);
                } else if (mag_is_pending == 1) {
					current_i2c_device = CURRENT_I2C_DEV_MAG;
                    if(Mag_Read_raw_DMA(&hqmc, rawMagDatas) == HAL_OK){
						i2c_dma_is_busy = 1;
						mag_is_pending = 0;
                    }
                    __HAL_DMA_DISABLE_IT(hqmc.hi2c->hdmarx, DMA_IT_HT);
                }
            }
        }

        else {
            HAL_I2C_DeInit(hmpu.hi2c);
            HAL_I2C_Init(hmpu.hi2c);
            i2c_dma_is_busy = 0;
        }

        /* Signal watchdog that IMU task is alive */
        xEventGroupSetBits(xWatchdogEventGroup, IMU_TASK_ALIVE_BIT);
    }
}


/**
 * @brief Telemetry Task
 *        Transmits formatted system state data via UART DMA at a fixed frequency.
 */
void vTelemetry_Task(void *pvParameters) {

    Telemetry_Data_Batch_t telemetry_data;
    static char tx_buffer[256];

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); /* Fixed 50 Hz loop */

    for (;;) {

        if (xQueuePeek(xTelemetryQueue, &telemetry_data, 0) == pdTRUE) {

            /* Format Quaternion Data for 3D Viewer */
             uint8_t len = snprintf(tx_buffer, sizeof(tx_buffer),
             "%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",

			 telemetry_data.Quaternion.q0,
			 telemetry_data.Quaternion.q1,
			 telemetry_data.Quaternion.q2,
			 telemetry_data.Quaternion.q3,

             telemetry_data.EulerAngles.pitch_deg,
             telemetry_data.EulerAngles.roll_deg,
             telemetry_data.EulerAngles.yaw_deg,

             telemetry_data.imu_data_batch.AccelXYZ_g[0],
             telemetry_data.imu_data_batch.AccelXYZ_g[1],
             telemetry_data.imu_data_batch.AccelXYZ_g[2],

             telemetry_data.imu_data_batch.GyroXYZ_radps[0],
             telemetry_data.imu_data_batch.GyroXYZ_radps[1],
             telemetry_data.imu_data_batch.GyroXYZ_radps[2],

             telemetry_data.mag_data_batch.MagXYZ_Gauss[0],
             telemetry_data.mag_data_batch.MagXYZ_Gauss[1],
             telemetry_data.mag_data_batch.MagXYZ_Gauss[2],

			 telemetry_data.imu_data_batch.temp);


            /* Transmit via non-blocking DMA */
            HAL_UART_Transmit_DMA(&huart2, (uint8_t*)tx_buffer, len);
            __HAL_DMA_DISABLE_IT(huart2.hdmatx,DMA_IT_HT);

            /* Suspend task until DMA transmission completes */
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        /* Signal watchdog that Telemetry task is alive */
        xEventGroupSetBits(xWatchdogEventGroup, TELEMETRY_TASK_ALIVE_BIT);

        /* Block to maintain precise 50Hz frequency */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief Watchdog / System Monitor Task
 *        Monitors the alive flags of all critical system tasks and feeds the hardware IWDG.
 */
void vMonitor_Task(void *pvParameters) {
    uint32_t uxBits;
    const TickType_t xMaxExpectedDelay = pdMS_TO_TICKS(750); /* Global system timeout threshold */

    for (;;) {

        uxBits = xEventGroupWaitBits(
                    xWatchdogEventGroup,
                    ALL_TASKS_ALIVE_MASK,
                    pdTRUE,
                    pdTRUE,
                    xMaxExpectedDelay);

        if ((uxBits & ALL_TASKS_ALIVE_MASK) == ALL_TASKS_ALIVE_MASK) {

            HAL_IWDG_Refresh(&hiwdg);

        } else {
            /* Watchdog Failure Handling */
            HAL_UART_AbortTransmit(&huart2);

            if ((uxBits & IMU_TASK_ALIVE_BIT) == 0) {
                printf("[CRITICAL] IMU Task is locked!\r\n");
            }
            if ((uxBits & AHRS_TASK_ALIVE_BIT) == 0) {
                printf("[CRITICAL] AHRS Task is locked!\r\n");
            }
            if ((uxBits & TELEMETRY_TASK_ALIVE_BIT) == 0) {
                printf("[CRITICAL] Telemetry Task is locked!\r\n");
            }

            HAL_Delay(10);

        }
    }
}

/**
 * @brief Retrieves system microsecond timestamp from hardware timer
 */
uint32_t Get_us(TIM_HandleTypeDef *htim) {
    return __HAL_TIM_GET_COUNTER(htim);
}

/* -------------------------------------------------------------------------- */
/*                        		   ISR CALLBACKS                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief EXTI Line Callback (Triggers when IMU DRDY pin goes high)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == INTA_Pin) {
        isr_imu_timestamp_us = Get_us(&htim2);
        xTaskNotifyFromISR(himu_task, IMU_EXTI_READY_BIT, eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief I2C Memory Rx Complete Callback (Triggers when DMA finishes I2C transfer)
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        if (current_i2c_device == CURRENT_I2C_DEV_IMU) {
            xTaskNotifyFromISR(himu_task, IMU_DMA_CMPLT_BIT, eSetBits, &xHigherPriorityTaskWoken);
        } else if (current_i2c_device == CURRENT_I2C_DEV_MAG) {
            xTaskNotifyFromISR(himu_task, MAG_DMA_CMPLT_BIT, eSetBits, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/**
 * @brief UART Tx Complete Callback (Unblocks Telemetry task upon successful transmission)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART2) {
        vTaskNotifyGiveFromISR(htelemetry_task, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


/* USER CODE END Application */

