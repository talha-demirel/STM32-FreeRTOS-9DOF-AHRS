/*
 * madgwick.c (veya madgwick.h)
 *
 * STM32 & FreeRTOS Adaptation by : Talha Demirel
 * Date                           : 15 Aug 2026
 *
 * ==============================================================================
 * ORIGINAL AUTHOR & COPYRIGHT NOTICE
 *
 * Implementation of Madgwick's IMU and AHRS algorithms (2011 version).
 * Original source: https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU
 *
 * Date          Author          Notes
 * 29/09/2011    SOH Madgwick    Initial release
 * 02/10/2011    SOH Madgwick    Optimised for reduced CPU load
 *
 * (C) Copyright 2011, x-io Technologies
 * ==============================================================================
 */

#ifndef INC_MADGWICK_H_
#define INC_MADGWICK_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float beta;
    float q0, q1, q2, q3;
} MadgwickFilter_t;

void MadgwickInit(MadgwickFilter_t *filter, float beta);


void MadgwickAHRSUpdate(MadgwickFilter_t *filter, float *gxyz_radps, float *axyz, float *mxyz, float dt);


void MadgwickAHRSUpdateIMU(MadgwickFilter_t *filter, float *gxyz_radps, float *axyz, float dt);


void MadgwickGetEulerAngles(const MadgwickFilter_t *filter,
                             float *rollDeg, float *pitchDeg, float *yawDeg);

void MadgwickGetQuaternions(const MadgwickFilter_t *filter, float *q0, float *q1, float *q2, float *q3);

#ifdef __cplusplus
}
#endif

#endif /* INC_MADGWICK_H_ */
