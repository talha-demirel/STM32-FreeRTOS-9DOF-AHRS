/*
 * qmc5883p.h
 *
 *  Created on: 1 May 2026
 *      Author: Talha
 *  Description: QMC5883P 3-Axis Magnetic Sensor Driver Public Interface.
 */

#ifndef INC_QMC5883P_H_
#define INC_QMC5883P_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------- */
/*                                ENUMS                                       */
/* -------------------------------------------------------------------------- */

/* Control Register 1 Configuration */
typedef enum {
    QMC5883P_MODE_SUSPEND    = 0x00,
    QMC5883P_MODE_NORMAL     = 0x01,
    QMC5883P_MODE_SINGLE     = 0x02,
    QMC5883P_MODE_CONTINUOUS = 0x03
} QMC5883P_Mode_t;

typedef enum {
    QMC5883P_ODR_10HZ  = 0x00,
    QMC5883P_ODR_50HZ  = 0x01,
    QMC5883P_ODR_100HZ = 0x02,
    QMC5883P_ODR_200HZ = 0x03
} QMC5883P_ODR_t;

typedef enum {
    QMC5883P_OSR1_8 = 0x00,
    QMC5883P_OSR1_4 = 0x01,
    QMC5883P_OSR1_2 = 0x02,
    QMC5883P_OSR1_1 = 0x03
} QMC5883P_OSR1_t;

typedef enum {
    QMC5883P_OSR2_1 = 0x00,
    QMC5883P_OSR2_2 = 0x01,
    QMC5883P_OSR2_4 = 0x02,
    QMC5883P_OSR2_8 = 0x03
} QMC5883P_OSR2_t;

/* Control Register 2 Configuration */
typedef enum {
    QMC5883P_RNG_30G = 0x00,
    QMC5883P_RNG_12G = 0x01,
    QMC5883P_RNG_8G  = 0x02,
    QMC5883P_RNG_2G  = 0x03
} QMC5883P_RNG_t;

typedef enum {
    QMC5883P_SET_RESET_ON  = 0x00,
    QMC5883P_SET_ON        = 0x01,
    QMC5883P_SET_RESET_OFF = 0x02,
} QMC5883P_SET_RESET_t;

/* -------------------------------------------------------------------------- */
/*                                DATA STRUCTURES                             */
/* -------------------------------------------------------------------------- */

typedef struct {
    int16_t xyz_raw[3];
    float   xyz_gauss[3];
    float   Heading;
} QMC5883P_Data_t;

typedef struct {
    float    xyz_offset[3];
    float    xyz_scale[3];
    float sensitivity;
    uint32_t start_tick;
    uint8_t is_calibrated;
} QMC5883P_Cal_t;

typedef struct {
    QMC5883P_Mode_t      Mode;
    QMC5883P_ODR_t       ODR;
    QMC5883P_OSR1_t      OSR1;
    QMC5883P_OSR2_t      OSR2;
    QMC5883P_RNG_t       Range;
    QMC5883P_SET_RESET_t Set_Reset_Mode;
} QMC5883P_InitTypeDef;

typedef struct {
    I2C_HandleTypeDef    *hi2c;
    uint8_t              DeviceI2CAddress;
    QMC5883P_InitTypeDef Init;
    QMC5883P_Data_t      Data;
    QMC5883P_Cal_t       Cal;
} QMC5883P_HandleTypeDef;



typedef struct {
    uint32_t magic;
    float    mag_offset[3];
    float    mag_scale[3];
} QMC5883P_FlashCal_t;

/* -------------------------------------------------------------------------- */
/*                                PUBLIC API                                  */
/* -------------------------------------------------------------------------- */

/* Initialization & Configuration */
HAL_StatusTypeDef QMC5883P_Init(QMC5883P_HandleTypeDef *hqmc5883p);
HAL_StatusTypeDef QMC5883P_Set_Default(QMC5883P_HandleTypeDef *hqmc5883p);

/* Data Acquisition & Status */
uint8_t QMC5883P_Is_Data_Ready(QMC5883P_HandleTypeDef *hqmc5883p);
HAL_StatusTypeDef Mag_Read_raw(QMC5883P_HandleTypeDef *hqmc5883p);
HAL_StatusTypeDef Mag_Read_raw_DMA(QMC5883P_HandleTypeDef *hqmc5883p, uint8_t *rawDatas);
HAL_StatusTypeDef Mag_Parse(QMC5883P_HandleTypeDef *hqmc5883p, const uint8_t *rawDatas);
HAL_StatusTypeDef Mag_raw_to_gauss(QMC5883P_HandleTypeDef *hqmc5883p);

/* Calibration & Computation */
HAL_StatusTypeDef QMC5883P_Calibrate(QMC5883P_HandleTypeDef *hqmc5883p, uint32_t Duration);
HAL_StatusTypeDef SaveMagCalibrationToFlash(QMC5883P_HandleTypeDef *hqmc5883p, uint32_t Sector, uint32_t Address);
HAL_StatusTypeDef LoadMagCalibrationFromFlash(QMC5883P_HandleTypeDef *hqmc5883p, uint32_t Address);
#ifdef __cplusplus
}
#endif

#endif /* INC_QMC5883P_H_ */
