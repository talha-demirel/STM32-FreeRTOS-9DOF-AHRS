/*
 * mpu6050.c
 *
 *  Created on: 13 Apr 2026
 *      Author: Talha
 */

#include "mpu6050.h"
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/*                          PRIVATE DEFINES & MACROS                          */
/* -------------------------------------------------------------------------- */

#define MEM_ADDR_SIZE           I2C_MEMADD_SIZE_8BIT
#define I2C_TIMEOUT             100

/* I2C Device Addresses */
#define I2C_ADDR_AD0_LOW        (uint8_t)(0x68 << 1)
#define I2C_ADDR_AD0_HIGH       (uint8_t)(0x69 << 1)
#define WHO_AM_I_ID             (uint8_t)0x68

/* Register Addresses */
#define REG_WHO_AM_I            (uint8_t)0x75
#define REG_PWR_MGMT_1          (uint8_t)0x6B
#define REG_SMPRT_DIV           (uint8_t)0x19
#define REG_ACCEL_CONFIG        (uint8_t)0x1C
#define REG_GYRO_CONFIG         (uint8_t)0x1B
#define REG_CONFIG              (uint8_t)0x1A
#define REG_INT_PIN_CFG         (uint8_t)0x37
#define REG_INT_ENABLE          (uint8_t)0x38
#define REG_INT_STATUS          (uint8_t)0x3A

#define REG_ACCEL_XOUT_H        (uint8_t)0x3B
#define REG_GYRO_XOUT_H         (uint8_t)0x43

/* Register Bit Masks */
#define BIT_SLEEP               (uint8_t)(1 << 6)
#define BIT_CLKSEL              (uint8_t)(0x07)
#define BIT_AFSSEL              (uint8_t)(3 << 3)
#define BIT_FSSEL               (uint8_t)(3 << 3)
#define BIT_DLPF_CFG            (uint8_t)(0x07)

#define BIT_I2C_BYPASS_EN       (uint8_t)(1 << 1)
#define BIT_INT_RD_CLEAR        (uint8_t)(1 << 4)
#define BIT_LATCH_INT_EN        (uint8_t)(1 << 5)
#define BIT_INT_OPEN            (uint8_t)(1 << 6)
#define BIT_INT_LEVEL           (uint8_t)(1 << 7)
#define BIT_DATA_RDY_EN         (uint8_t)(1 << 0)

/* Calibration & Storage Macros */
#define ACCEL_CAL_SAMPLE_PER_POS 1000
#define GYRO_CAL_SAMPLE          1000
#define CAL_FLASH_SECTOR         FLASH_SECTOR_11
#define CAL_FLASH_ADDR           (uint32_t)(0x080E0000)
#define CAL_MAGIC                (uint32_t)(0xDEADBEEF)

/* -------------------------------------------------------------------------- */
/*                              PRIVATE STRUCTS                               */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint32_t magic;
    float    accel_offset[3];
    float    accel_scale[3];
} MPU6050_FlashCal_t;

/* -------------------------------------------------------------------------- */
/*                        PRIVATE FUNCTION PROTOTYPES                         */
/* -------------------------------------------------------------------------- */

static HAL_StatusTypeDef MPU6050_Mem_Write(MPU6050_HandleTypeDef *hmpu6050, uint16_t regAddr, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef MPU6050_Mem_Read(MPU6050_HandleTypeDef *hmpu6050, uint16_t regAddr, uint8_t *pData, uint16_t Size);

static HAL_StatusTypeDef Accel_Read_raw(MPU6050_HandleTypeDef *hmpu6050);
static HAL_StatusTypeDef Gyro_Read_raw(MPU6050_HandleTypeDef *hmpu6050);

/* -------------------------------------------------------------------------- */
/*                           PRIVATE FUNCTIONS                                */
/* -------------------------------------------------------------------------- */

static HAL_StatusTypeDef MPU6050_Mem_Write(MPU6050_HandleTypeDef *hmpu6050, uint16_t regAddr, uint8_t *pData, uint16_t Size) {
    if (hmpu6050 == NULL) return HAL_ERROR;
    if (HAL_I2C_Mem_Write(hmpu6050->hi2c, hmpu6050->DeviceI2CAddress, regAddr, MEM_ADDR_SIZE, pData, Size, I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef MPU6050_Mem_Read(MPU6050_HandleTypeDef *hmpu6050, uint16_t regAddr, uint8_t *pData, uint16_t Size) {
    if (hmpu6050 == NULL) return HAL_ERROR;
    if (HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->DeviceI2CAddress, regAddr, MEM_ADDR_SIZE, pData, Size, I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef Accel_Read_raw(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    uint8_t rawDatas[6];
    if (MPU6050_Mem_Read(hmpu6050, REG_ACCEL_XOUT_H, rawDatas, 6) != HAL_OK) return HAL_ERROR;

    for (uint8_t i = 0; i < 3; i++) {
        hmpu6050->Accel.axyz_raw[i] = (int16_t)((rawDatas[i * 2] << 8) | (rawDatas[i * 2 + 1]));
    }
    return HAL_OK;
}

static HAL_StatusTypeDef Gyro_Read_raw(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    uint8_t rawDatas[6];
    if (MPU6050_Mem_Read(hmpu6050, REG_GYRO_XOUT_H, rawDatas, 6) != HAL_OK) return HAL_ERROR;

    for (uint8_t i = 0; i < 3; i++) {
        hmpu6050->Gyro.gxyz_raw[i] = (int16_t)((rawDatas[i * 2] << 8) | (rawDatas[i * 2 + 1]));
    }
    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/*                            PUBLIC FUNCTIONS                                */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    if (HAL_I2C_IsDeviceReady(hmpu6050->hi2c, hmpu6050->DeviceI2CAddress, 3, I2C_TIMEOUT) != HAL_OK) return HAL_ERROR;

    uint8_t check_id;
    if (MPU6050_Mem_Read(hmpu6050, REG_WHO_AM_I, &check_id, 1) != HAL_OK) return HAL_ERROR;
    if (check_id != WHO_AM_I_ID) return HAL_ERROR;

    /* Configure PWR_MGMT_1 Register */
    uint8_t pwr_mgmt_1 = 0;
    if (MPU6050_Mem_Read(hmpu6050, REG_PWR_MGMT_1, &pwr_mgmt_1, 1) != HAL_OK) return HAL_ERROR;
    pwr_mgmt_1 &= ~BIT_SLEEP;
    pwr_mgmt_1 &= ~BIT_CLKSEL;
    pwr_mgmt_1 |= hmpu6050->Init.ClockSource;
    if (MPU6050_Mem_Write(hmpu6050, REG_PWR_MGMT_1, &pwr_mgmt_1, 1) != HAL_OK) return HAL_ERROR;

    HAL_Delay(50);

    /* Configure SMPRT_DIV Register */
    uint8_t smprt_div = hmpu6050->Init.SampleRateDivider;
    if (MPU6050_Mem_Write(hmpu6050, REG_SMPRT_DIV, &smprt_div, 1) != HAL_OK) return HAL_ERROR;

    /* Configure ACCEL_CONFIG Register */
    uint8_t accel_conf = 0;
    if (MPU6050_Mem_Read(hmpu6050, REG_ACCEL_CONFIG, &accel_conf, 1) != HAL_OK) return HAL_ERROR;
    accel_conf &= ~BIT_AFSSEL;
    accel_conf |= (uint8_t)((hmpu6050->Init.Accel_FullScale) << 3);
    if (MPU6050_Mem_Write(hmpu6050, REG_ACCEL_CONFIG, &accel_conf, 1) != HAL_OK) return HAL_ERROR;

    /* Configure GYRO_CONFIG Register */
    uint8_t gyro_conf = 0;
    if (MPU6050_Mem_Read(hmpu6050, REG_GYRO_CONFIG, &gyro_conf, 1) != HAL_OK) return HAL_ERROR;
    gyro_conf &= ~BIT_FSSEL;
    gyro_conf |= (uint8_t)((hmpu6050->Init.Gyro_FullScale) << 3);
    if (MPU6050_Mem_Write(hmpu6050, REG_GYRO_CONFIG, &gyro_conf, 1) != HAL_OK) return HAL_ERROR;

    /* Configure CONFIG Register (DLPF) */
    uint8_t conf = 0;
    if (MPU6050_Mem_Read(hmpu6050, REG_CONFIG, &conf, 1) != HAL_OK) return HAL_ERROR;
    conf &= ~BIT_DLPF_CFG;
    conf |= (uint8_t)((hmpu6050->Init.DLPF));
    if (MPU6050_Mem_Write(hmpu6050, REG_CONFIG, &conf, 1) != HAL_OK) return HAL_ERROR;

    /* Set Gyroscope Scale Factors */
    switch (hmpu6050->Init.Gyro_FullScale) {
        case MPU6050_GYRO_RANGE_250:  hmpu6050->Gyro.Cal.gyro_scale = 131.0f;  break;
        case MPU6050_GYRO_RANGE_500:  hmpu6050->Gyro.Cal.gyro_scale = 65.5f;   break;
        case MPU6050_GYRO_RANGE_1000: hmpu6050->Gyro.Cal.gyro_scale = 32.8f;   break;
        case MPU6050_GYRO_RANGE_2000: hmpu6050->Gyro.Cal.gyro_scale = 16.4f;   break;
        default:                      hmpu6050->Gyro.Cal.gyro_scale = 131.0f;  break;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Set_Default(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    hmpu6050->DeviceI2CAddress       = I2C_ADDR_AD0_LOW;
    hmpu6050->Init.ClockSource       = MPU6050_CLOCK_PLL_XGYRO;
    hmpu6050->Init.SampleRateDivider = 1;
    hmpu6050->Init.Accel_FullScale   = MPU6050_ACCEL_RANGE_4G;
    hmpu6050->Init.Gyro_FullScale    = MPU6050_GYRO_RANGE_500;
    hmpu6050->Init.DLPF              = MPU6050_DLPF_44HZ;

    hmpu6050->Accel.Cal.is_calibrated = 0;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Start(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    uint8_t pwr_mgmt_1 = 0;
    if (MPU6050_Mem_Read(hmpu6050, REG_PWR_MGMT_1, &pwr_mgmt_1, 1) != HAL_OK) return HAL_ERROR;
    pwr_mgmt_1 &= ~BIT_SLEEP;
    if (MPU6050_Mem_Write(hmpu6050, REG_PWR_MGMT_1, &pwr_mgmt_1, 1) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Stop(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    uint8_t pwr_mgmt_1 = 0;
    if (MPU6050_Mem_Read(hmpu6050, REG_PWR_MGMT_1, &pwr_mgmt_1, 1) != HAL_OK) return HAL_ERROR;
    pwr_mgmt_1 |= BIT_SLEEP;
    if (MPU6050_Mem_Write(hmpu6050, REG_PWR_MGMT_1, &pwr_mgmt_1, 1) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_DRDY_INT_Enable(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    uint8_t int_pin_conf = 0;
    if (MPU6050_Mem_Read(hmpu6050, REG_INT_PIN_CFG, &int_pin_conf, 1) != HAL_OK) return HAL_ERROR;
    int_pin_conf |= BIT_INT_RD_CLEAR;
    int_pin_conf |= BIT_LATCH_INT_EN;
    int_pin_conf &= ~BIT_INT_LEVEL;
    int_pin_conf &= ~BIT_INT_OPEN;
    if (MPU6050_Mem_Write(hmpu6050, REG_INT_PIN_CFG, &int_pin_conf, 1) != HAL_OK) return HAL_ERROR;

    uint8_t int_enable = 0;
    if (MPU6050_Mem_Read(hmpu6050, REG_INT_ENABLE, &int_enable, 1) != HAL_OK) return HAL_ERROR;
    int_enable |= BIT_DATA_RDY_EN;
    if (MPU6050_Mem_Write(hmpu6050, REG_INT_ENABLE, &int_enable, 1) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_I2C_Bypass_Enable(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    uint8_t int_pin_conf = 0;
    if (MPU6050_Mem_Read(hmpu6050, REG_INT_PIN_CFG, &int_pin_conf, 1) != HAL_OK) return HAL_ERROR;
    int_pin_conf |= BIT_I2C_BYPASS_EN;
    if (MPU6050_Mem_Write(hmpu6050, REG_INT_PIN_CFG, &int_pin_conf, 1) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

uint8_t MPU6050_Is_Data_Ready(MPU6050_HandleTypeDef *hmpu6050) {
    uint8_t int_stat = 0;
    MPU6050_Mem_Read(hmpu6050, REG_INT_STATUS, &int_stat, 1);
    return (int_stat & (1 << 0));
}

HAL_StatusTypeDef MPU6050_Accel_Calibrate_6pts(MPU6050_HandleTypeDef *hmpu6050, void (*waitbuttonFunc)(void), void (*printFunc)(const char*)) {

	if (hmpu6050 == NULL) return HAL_ERROR;
    if(waitbuttonFunc == NULL)	return HAL_ERROR;
    if(printFunc == NULL)	return HAL_ERROR;

    printFunc("Starting Calibration..\r\n");
    static const char *prompts[6] = {
        "1/6: Hold the sensor with the X-axis pointing UP (X+), then press the button.\r\n",
        "2/6: Hold the sensor with the X-axis pointing DOWN (X-), then press the button.\r\n",
        "3/6: Hold the sensor with the Y-axis pointing UP (Y+), then press the button.\r\n",
        "4/6: Hold the sensor with the Y-axis pointing DOWN (Y-), then press the button.\r\n",
        "5/6: Hold the sensor flat with the chip facing UP (Z+), then press the button.\r\n",
        "6/6: Hold the sensor upside down with the chip facing DOWN (Z-), then press the button.\r\n"
    };

    float readings[6];
    static const uint8_t axis_of_step[6] = {0, 0, 1, 1, 2, 2};

    for (uint8_t step = 0; step < 6; step++) {
        printFunc(prompts[step]);
        waitbuttonFunc();

        int32_t sum[3] = {0, 0, 0};
        uint16_t retry_count = 0;

        for (uint16_t i = 0; i < ACCEL_CAL_SAMPLE_PER_POS; i++) {
            if (Accel_Read_raw(hmpu6050) != HAL_OK) {
                if (++retry_count > 50) return HAL_ERROR;
                i--;
                continue;
            }
            retry_count = 0;
            for (uint8_t j = 0; j < 3; j++) {
                sum[j] += hmpu6050->Accel.axyz_raw[j];
            }
            HAL_Delay(2);
        }

        uint8_t axis = axis_of_step[step];
        readings[step] = (float)sum[axis] / (float)ACCEL_CAL_SAMPLE_PER_POS;
    }

    for (uint8_t i = 0; i < 3; i++) {
        hmpu6050->Accel.Cal.axyz_offset[i] = (readings[i * 2] + readings[i * 2 + 1]) / 2.0f;
        hmpu6050->Accel.Cal.axyz_scale[i]  = (readings[i * 2] - readings[i * 2 + 1]) / 2.0f;
    }

    if (hmpu6050->Accel.Cal.axyz_scale[0] <= 0 || hmpu6050->Accel.Cal.axyz_scale[1] <= 0 || hmpu6050->Accel.Cal.axyz_scale[2] <= 0) {
        printFunc("ERROR: Calibration is invalid, sensor might be held in wrong position.\n");
        return HAL_ERROR;
    }

    hmpu6050->Accel.Cal.is_calibrated = 1;

    return HAL_OK;
}

HAL_StatusTypeDef SaveAccelCalibrationToFlash(MPU6050_HandleTypeDef *hmpu6050, uint32_t Sector, uint32_t Address) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    MPU6050_FlashCal_t cal;
    cal.magic = CAL_MAGIC;
    for (uint8_t i = 0; i < 3; i++) {
        cal.accel_offset[i] = hmpu6050->Accel.Cal.axyz_offset[i];
        cal.accel_scale[i]  = hmpu6050->Accel.Cal.axyz_scale[i];
    }

    if (sizeof(cal) % 4 != 0) return HAL_ERROR;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector       = Sector;
    eraseInit.NbSectors    = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sectorError = 0;
    if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
         HAL_FLASH_Lock();
         return HAL_ERROR;
    }

    const uint32_t *src  = (const uint32_t *)&cal;
    uint32_t        numWords = sizeof(cal) / 4;

    for (uint32_t i = 0; i < numWords; i++) {

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address + (i * 4), src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();

    return HAL_OK;
}

HAL_StatusTypeDef LoadAccelCalibrationFromFlash(MPU6050_HandleTypeDef *hmpu6050, uint32_t Address) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    const MPU6050_FlashCal_t *cal = (const MPU6050_FlashCal_t *)Address;

    if (cal->magic != CAL_MAGIC) {
        return HAL_ERROR;
    }
    for (uint8_t i = 0; i < 3; i++) {
        hmpu6050->Accel.Cal.axyz_offset[i] = cal->accel_offset[i];
        hmpu6050->Accel.Cal.axyz_scale[i]  = cal->accel_scale[i];
    }

    hmpu6050->Accel.Cal.is_calibrated = 1;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_raw_DMA(MPU6050_HandleTypeDef *hmpu6050, uint8_t *rawDatas) {
    if (hmpu6050 == NULL) return HAL_ERROR;
    if (HAL_I2C_Mem_Read_DMA(hmpu6050->hi2c, hmpu6050->DeviceI2CAddress, REG_ACCEL_XOUT_H, MEM_ADDR_SIZE, rawDatas, 14) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Parse(MPU6050_HandleTypeDef *hmpu6050, const uint8_t *rawDatastoParse) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    /* Accelerometer Data */
    for (uint8_t i = 0; i < 3; i++) {
        hmpu6050->Accel.axyz_raw[i] = (int16_t)((rawDatastoParse[i * 2] << 8) | (rawDatastoParse[i * 2 + 1]));
    }

    /* Temperature Data */
    int16_t temp = (int16_t)((rawDatastoParse[6] << 8) | (rawDatastoParse[7]));
    hmpu6050->temp = ((float)temp / 340.0f) + 36.53f;

    /* Gyroscope Data */
    for (uint8_t i = 0; i < 3; i++) {
        hmpu6050->Gyro.gxyz_raw[i] = (int16_t)((rawDatastoParse[i * 2 + 8] << 8) | (rawDatastoParse[i * 2 + 8 + 1]));
    }
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Accel_raw_to_g(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    if (hmpu6050->Accel.Cal.is_calibrated != 1) return HAL_ERROR;

    for (uint8_t i = 0; i < 3; i++) {
        hmpu6050->Accel.axyz_g[i] = (float)(hmpu6050->Accel.axyz_raw[i] - hmpu6050->Accel.Cal.axyz_offset[i]) / hmpu6050->Accel.Cal.axyz_scale[i];
    }
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Gyro_Calibrate(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    int32_t xyz_sum[3] = {0, 0, 0};

    for (int i = 0; i < GYRO_CAL_SAMPLE; i++) {
        if (Gyro_Read_raw(hmpu6050) == HAL_OK) {
            for (uint8_t j = 0; j < 3; j++) {
                xyz_sum[j] += hmpu6050->Gyro.gxyz_raw[j];
            }
        }
        HAL_Delay(1);
    }

    for (int i = 0; i < 3; i++) {
        hmpu6050->Gyro.Cal.gxyz_offset[i] = (float)xyz_sum[i] / (float)GYRO_CAL_SAMPLE;
    }
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Gyro_raw_to_dps(MPU6050_HandleTypeDef *hmpu6050) {
    if (hmpu6050 == NULL) return HAL_ERROR;

    for (uint8_t i = 0; i < 3; i++) {
        hmpu6050->Gyro.xyz_dps[i] = (float)(hmpu6050->Gyro.gxyz_raw[i] - hmpu6050->Gyro.Cal.gxyz_offset[i]) / hmpu6050->Gyro.Cal.gyro_scale;
    }
    return HAL_OK;
}
