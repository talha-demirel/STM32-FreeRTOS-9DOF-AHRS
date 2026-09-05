/*
 * qmc5883p.c
 *
 *  Created on: 1 May 2026
 *      Author: Talha
 */

#include "qmc5883p.h"
#include <math.h>

/* -------------------------------------------------------------------------- */
/*                          PRIVATE DEFINES & MACROS                          */
/* -------------------------------------------------------------------------- */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MEM_ADDR_SIZE       I2C_MEMADD_SIZE_8BIT
#define TIMEOUT             100

/* Constants & Identifiers */
#define QMC5883P_CHIPID     (uint8_t)0x80
#define QMC5883P_I2C_ADDR   (uint8_t)(0x2C << 1)

/* Register Addresses */
#define REG_CHIPID          (uint8_t)0x00
#define REG_XOUT_LSB        (uint8_t)0x01
#define REG_XOUT_MSB        (uint8_t)0x02
#define REG_YOUT_LSB        (uint8_t)0x03
#define REG_YOUT_MSB        (uint8_t)0x04
#define REG_ZOUT_LSB        (uint8_t)0x05
#define REG_ZOUT_MSB        (uint8_t)0x06
#define REG_STATUS          (uint8_t)0x09
#define REG_CONTROL_1       (uint8_t)0x0A
#define REG_CONTROL_2       (uint8_t)0x0B
#define REG_SIGN            (uint8_t)0x29

/* Bit Definitions */
#define BIT_DRDY            (uint8_t)0x01

/* Mathematical Macros */
#define MAGNETIC_DECLINATION 6.1833f
#define RAD_TO_DEG           (180.0f / M_PI)
#define DEG_TO_RAD           (M_PI / 180.0f)

/* FLASH */
#define QMC_CAL_MAGIC        0x514D4331 // "QMC1"
#define QMC_CAL_FLASH_SECTOR FLASH_SECTOR_10
#define QMC_CAL_FLASH_ADDR   0x080C0000

/* -------------------------------------------------------------------------- */
/*                        PRIVATE FUNCTION PROTOTYPES                         */
/* -------------------------------------------------------------------------- */

static HAL_StatusTypeDef QMC5883P_Mem_Write(QMC5883P_HandleTypeDef *hqmc5883p, uint16_t regAddr, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef QMC5883P_Mem_Read(QMC5883P_HandleTypeDef *hqmc5883p, uint16_t regAddr, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef QMC5883P_Mem_Read_DMA(QMC5883P_HandleTypeDef *hqmc5883p, uint16_t regAddr, uint8_t *pData, uint16_t Size);
static void find_cal_vals(QMC5883P_Cal_t *Cal, int16_t *xyz_min, int16_t *xyz_max);

/* -------------------------------------------------------------------------- */
/*                           PRIVATE FUNCTIONS                                */
/* -------------------------------------------------------------------------- */

static HAL_StatusTypeDef QMC5883P_Mem_Write(QMC5883P_HandleTypeDef *hqmc5883p, uint16_t regAddr, uint8_t *pData, uint16_t Size) {
    if (hqmc5883p == NULL) return HAL_ERROR;
    if (HAL_I2C_Mem_Write(hqmc5883p->hi2c, hqmc5883p->DeviceI2CAddress, regAddr, MEM_ADDR_SIZE, pData, Size, TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef QMC5883P_Mem_Read(QMC5883P_HandleTypeDef *hqmc5883p, uint16_t regAddr, uint8_t *pData, uint16_t Size) {
    if (hqmc5883p == NULL) return HAL_ERROR;
    if (HAL_I2C_Mem_Read(hqmc5883p->hi2c, hqmc5883p->DeviceI2CAddress, regAddr, MEM_ADDR_SIZE, pData, Size, TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef QMC5883P_Mem_Read_DMA(QMC5883P_HandleTypeDef *hqmc5883p, uint16_t regAddr, uint8_t *pData, uint16_t Size) {
    if (hqmc5883p == NULL) return HAL_ERROR;
    if (HAL_I2C_Mem_Read_DMA(hqmc5883p->hi2c, hqmc5883p->DeviceI2CAddress, regAddr, MEM_ADDR_SIZE, pData, Size) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}



static void find_cal_vals(QMC5883P_Cal_t *Cal, int16_t *xyz_min, int16_t *xyz_max) {
    float xyz_range[3] = {0};
    float range_sum = 0, range_avg = 0;

    for (uint8_t i = 0; i < 3; i++) {
        Cal->xyz_offset[i] = (xyz_max[i] + xyz_min[i]) / 2.0f;
        xyz_range[i] = (xyz_max[i] - xyz_min[i]) / 2.0f;
        range_sum += xyz_range[i];
    }
    range_avg = range_sum / 3.0f;

    for (uint8_t i = 0; i < 3; i++) {
        Cal->xyz_scale[i] = range_avg / xyz_range[i];
    }
}

/* -------------------------------------------------------------------------- */
/*                            PUBLIC FUNCTIONS                                */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef QMC5883P_Init(QMC5883P_HandleTypeDef *hqmc5883p) {
    if (hqmc5883p == NULL) return HAL_ERROR;

    if (HAL_I2C_IsDeviceReady(hqmc5883p->hi2c, hqmc5883p->DeviceI2CAddress, 3, TIMEOUT) != HAL_OK) return HAL_ERROR;

    uint8_t check_id;
    if (QMC5883P_Mem_Read(hqmc5883p, REG_CHIPID, &check_id, 1) != HAL_OK) return HAL_ERROR;
    if (check_id != QMC5883P_CHIPID) return HAL_ERROR;

    uint8_t sign_val = 0x06;
    if (QMC5883P_Mem_Write(hqmc5883p, REG_SIGN, &sign_val, 1) != HAL_OK) return HAL_ERROR;

    uint8_t ctrl_reg2 = 0;
    if (QMC5883P_Mem_Read(hqmc5883p, REG_CONTROL_2, &ctrl_reg2, 1) != HAL_OK) return HAL_ERROR;
    ctrl_reg2 &= ~0x0F;
    ctrl_reg2 |= ((hqmc5883p->Init.Range << 2) | hqmc5883p->Init.Set_Reset_Mode);
    if (QMC5883P_Mem_Write(hqmc5883p, REG_CONTROL_2, &ctrl_reg2, 1) != HAL_OK) return HAL_ERROR;

    switch (hqmc5883p->Init.Range) {
        case QMC5883P_RNG_2G:  hqmc5883p->Cal.sensitivity = 15000.0f; break;
        case QMC5883P_RNG_8G:  hqmc5883p->Cal.sensitivity = 3750.0f;  break;
        case QMC5883P_RNG_12G: hqmc5883p->Cal.sensitivity = 2500.0f;  break;
        case QMC5883P_RNG_30G: hqmc5883p->Cal.sensitivity = 1000.0f;  break;
    }

    uint8_t ctrl_reg1 = (hqmc5883p->Init.OSR2 << 6) | (hqmc5883p->Init.OSR1 << 4) |
                        (hqmc5883p->Init.ODR << 2)  | (hqmc5883p->Init.Mode);
    if (QMC5883P_Mem_Write(hqmc5883p, REG_CONTROL_1, &ctrl_reg1, 1) != HAL_OK) return HAL_ERROR;

    HAL_Delay(10);
    return HAL_OK;
}

HAL_StatusTypeDef QMC5883P_Set_Default(QMC5883P_HandleTypeDef *hqmc5883p) {
    if (hqmc5883p == NULL) return HAL_ERROR;

    hqmc5883p->DeviceI2CAddress = QMC5883P_I2C_ADDR;
    hqmc5883p->Init.Mode           = QMC5883P_MODE_CONTINUOUS;
    hqmc5883p->Init.ODR            = QMC5883P_ODR_200HZ;
    hqmc5883p->Init.OSR1           = QMC5883P_OSR1_8;
    hqmc5883p->Init.OSR2           = QMC5883P_OSR2_8;
    hqmc5883p->Init.Range          = QMC5883P_RNG_8G;
    hqmc5883p->Init.Set_Reset_Mode = QMC5883P_SET_RESET_ON;

    return HAL_OK;
}

HAL_StatusTypeDef Mag_Read_raw(QMC5883P_HandleTypeDef *hqmc5883p) {
    if (hqmc5883p == NULL) return HAL_ERROR;

    uint8_t OutputBuffer[6] = {0};
    if (QMC5883P_Mem_Read(hqmc5883p, REG_XOUT_LSB, OutputBuffer, 6) != HAL_OK) return HAL_ERROR;

    for (uint8_t i = 0; i < 3; i++) {
        hqmc5883p->Data.xyz_raw[i] = (int16_t)((OutputBuffer[i * 2 + 1] << 8) | OutputBuffer[i * 2]);
    }
    return HAL_OK;
}

HAL_StatusTypeDef Mag_Read_raw_DMA(QMC5883P_HandleTypeDef *hqmc5883p, uint8_t *rawDatas) {
    if (hqmc5883p == NULL) return HAL_ERROR;
    if (QMC5883P_Mem_Read_DMA(hqmc5883p, REG_XOUT_LSB, rawDatas, 6) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef Mag_Parse(QMC5883P_HandleTypeDef *hqmc5883p, const uint8_t *rawDatas) {
    if (hqmc5883p == NULL) return HAL_ERROR;

    for (uint8_t i = 0; i < 3; i++) {
        hqmc5883p->Data.xyz_raw[i] = (int16_t)((rawDatas[i * 2 + 1] << 8) | rawDatas[i * 2]);
    }
    return HAL_OK;
}

HAL_StatusTypeDef Mag_raw_to_gauss(QMC5883P_HandleTypeDef *hqmc5883p) {
    if (hqmc5883p == NULL) return HAL_ERROR;

    if(hqmc5883p->Cal.is_calibrated == 0)	return HAL_ERROR;

    float xyz_count[3];
    for (uint8_t i = 0; i < 3; i++) {
        xyz_count[i] = (hqmc5883p->Data.xyz_raw[i] - hqmc5883p->Cal.xyz_offset[i]) * hqmc5883p->Cal.xyz_scale[i];
        hqmc5883p->Data.xyz_gauss[i] = xyz_count[i] / (float)hqmc5883p->Cal.sensitivity;
    }
    return HAL_OK;
}

HAL_StatusTypeDef QMC5883P_Calibrate(QMC5883P_HandleTypeDef *hqmc5883p, uint32_t Duration) {

	if (hqmc5883p == NULL) return HAL_ERROR;

    int16_t xyz_min[3] = {0};
    int16_t xyz_max[3] = {0};

    if (hqmc5883p->Cal.start_tick == 0) {
        hqmc5883p->Cal.start_tick = HAL_GetTick();
    }

    for (uint8_t i = 0; i < 3; i++) {
        xyz_min[i] = INT16_MAX;
        xyz_max[i] = INT16_MIN;
    }

    uint16_t retry_count = 0;
    while (HAL_GetTick() - hqmc5883p->Cal.start_tick <= Duration) {
    	if(QMC5883P_Is_Data_Ready(hqmc5883p)){
			if (Mag_Read_raw(hqmc5883p) != HAL_OK) {
				if (++retry_count > 50) return HAL_ERROR;
				continue;
			}
			retry_count = 0;

			for (uint8_t i = 0; i < 3; i++) {
				if (hqmc5883p->Data.xyz_raw[i] < xyz_min[i]) xyz_min[i] = hqmc5883p->Data.xyz_raw[i];
				if (hqmc5883p->Data.xyz_raw[i] > xyz_max[i]) xyz_max[i] = hqmc5883p->Data.xyz_raw[i];
			}
    	}
    }

    find_cal_vals(&hqmc5883p->Cal, xyz_min, xyz_max);
    hqmc5883p->Cal.start_tick = 0;
    hqmc5883p->Cal.is_calibrated = 1;

    return HAL_OK;
}
HAL_StatusTypeDef SaveMagCalibrationToFlash(QMC5883P_HandleTypeDef *hqmc5883p, uint32_t Sector, uint32_t Address) {
    if (hqmc5883p == NULL) return HAL_ERROR;

    QMC5883P_FlashCal_t cal;
    cal.magic = QMC_CAL_MAGIC;

    for (uint8_t i = 0; i < 3; i++) {
        cal.mag_offset[i] = hqmc5883p->Cal.xyz_offset[i];
        cal.mag_scale[i]  = hqmc5883p->Cal.xyz_scale[i];
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

HAL_StatusTypeDef LoadMagCalibrationFromFlash(QMC5883P_HandleTypeDef *hqmc5883p, uint32_t Address) {
    if (hqmc5883p == NULL) return HAL_ERROR;

    const QMC5883P_FlashCal_t *cal = (const QMC5883P_FlashCal_t *)Address;

    if (cal->magic != QMC_CAL_MAGIC) {
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i < 3; i++) {
        hqmc5883p->Cal.xyz_offset[i] = cal->mag_offset[i];
        hqmc5883p->Cal.xyz_scale[i]  = cal->mag_scale[i];
    }

    hqmc5883p->Cal.is_calibrated = 1;

    return HAL_OK;
}

uint8_t QMC5883P_Is_Data_Ready(QMC5883P_HandleTypeDef *hqmc5883p) {
    uint8_t stat_reg = 0;
    QMC5883P_Mem_Read(hqmc5883p, REG_STATUS, &stat_reg, 1);
    return (stat_reg & BIT_DRDY);
}
