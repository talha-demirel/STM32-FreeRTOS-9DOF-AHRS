/*
 * mpu6050.h
 *
 *  Created on: 14 Apr 2026
 *      Author: Talha
 */

#ifndef SRC_MEMS_MPU6050_H_
#define SRC_MEMS_MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------- */
/*                                ENUMS                                       */
/* -------------------------------------------------------------------------- */

typedef enum {
    MPU6050_CLOCK_INTERNAL_8MHZ = 0,
    MPU6050_CLOCK_PLL_XGYRO,
    MPU6050_CLOCK_PLL_YGYRO,
    MPU6050_CLOCK_PLL_ZGYRO,
    MPU6050_CLOCK_PLL_EXT32K,
    MPU6050_CLOCK_PLL_EXT19M,
    MPU6050_CLOCK_KEEP_RESET
} MPU6050_ClockSource_t;

typedef enum {
    MPU6050_ACCEL_RANGE_2G = 0,
    MPU6050_ACCEL_RANGE_4G,
    MPU6050_ACCEL_RANGE_8G,
    MPU6050_ACCEL_RANGE_16G
} MPU6050_AccelRange_t;

typedef enum {
    MPU6050_GYRO_RANGE_250 = 0,
    MPU6050_GYRO_RANGE_500,
    MPU6050_GYRO_RANGE_1000,
    MPU6050_GYRO_RANGE_2000
} MPU6050_GyroRange_t;

typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ,
    MPU6050_DLPF_94HZ,
    MPU6050_DLPF_44HZ,
    MPU6050_DLPF_21HZ,
    MPU6050_DLPF_10HZ,
    MPU6050_DLPF_5HZ
} MPU6050_DLPF_t;

/* -------------------------------------------------------------------------- */
/*                                DATA STRUCTURES                             */
/* -------------------------------------------------------------------------- */

typedef struct {
    float   axyz_offset[3];
    float   axyz_scale[3];
    uint8_t is_calibrated;
} MPU6050_Accel_Cal_t;

typedef struct {
    int16_t axyz_raw[3];
    float   axyz_g[3];
    MPU6050_Accel_Cal_t Cal;
} MPU6050_Accel_Data_t;

typedef struct {
    float   gxyz_offset[3];
    float   gyro_scale;
    uint8_t is_calibrated;
} MPU6050_Gyro_Cal_t;

typedef struct {
    int16_t gxyz_raw[3];
    float   xyz_dps[3];
    MPU6050_Gyro_Cal_t Cal;
} MPU6050_Gyro_Data_t;

typedef struct {
    uint8_t               ClockSource;
    MPU6050_AccelRange_t  Accel_FullScale;
    MPU6050_GyroRange_t   Gyro_FullScale;
    uint8_t               SampleRateDivider;
    MPU6050_DLPF_t        DLPF;
} MPU6050_InitTypeDef;

typedef struct {
    I2C_HandleTypeDef    *hi2c;
    uint8_t              DeviceI2CAddress;
    MPU6050_InitTypeDef  Init;
    MPU6050_Accel_Data_t Accel;
    MPU6050_Gyro_Data_t  Gyro;
    float                temp;
} MPU6050_HandleTypeDef;

/* -------------------------------------------------------------------------- */
/*                                PUBLIC API                                  */
/* -------------------------------------------------------------------------- */

/* Initialization & Control */
HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *hmpu6050);
HAL_StatusTypeDef MPU6050_Set_Default(MPU6050_HandleTypeDef *hmpu6050);
HAL_StatusTypeDef MPU6050_Start(MPU6050_HandleTypeDef *hmpu6050);
HAL_StatusTypeDef MPU6050_Stop(MPU6050_HandleTypeDef *hmpu6050);

/* Interrupt & Bypass Configurations */
HAL_StatusTypeDef MPU6050_DRDY_INT_Enable(MPU6050_HandleTypeDef *hmpu6050);
HAL_StatusTypeDef MPU6050_I2C_Bypass_Enable(MPU6050_HandleTypeDef *hmpu6050);
uint8_t MPU6050_Is_Data_Ready(MPU6050_HandleTypeDef *hmpu6050);

/* Calibration Methods */
HAL_StatusTypeDef SaveAccelCalibrationToFlash(MPU6050_HandleTypeDef *hmpu6050, uint32_t Sector, uint32_t Address);
HAL_StatusTypeDef LoadAccelCalibrationFromFlash(MPU6050_HandleTypeDef *hmpu6050, uint32_t Address);
HAL_StatusTypeDef MPU6050_Accel_Calibrate_6pts(MPU6050_HandleTypeDef *hmpu6050, void (*waitbuttonFunc)(void), void (*printFunc)(const char*));
HAL_StatusTypeDef MPU6050_Gyro_Calibrate(MPU6050_HandleTypeDef *hmpu6050);

/* Data Acquisition & Processing */
HAL_StatusTypeDef MPU6050_Read_raw_DMA(MPU6050_HandleTypeDef *hmpu6050, uint8_t *rawDatas);
HAL_StatusTypeDef MPU6050_Parse(MPU6050_HandleTypeDef *hmpu6050, const uint8_t *rawDatastoParse);
HAL_StatusTypeDef MPU6050_Accel_raw_to_g(MPU6050_HandleTypeDef *hmpu6050);
HAL_StatusTypeDef MPU6050_Gyro_raw_to_dps(MPU6050_HandleTypeDef *hmpu6050);

#ifdef __cplusplus
}
#endif

#endif /* SRC_MEMS_MPU6050_H_ */
