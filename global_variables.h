/******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product").
 * By including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing so
 * agrees to indemnify Cypress against all liability.
******************************************************************************/
/*
 * global_variables.h
 *
 *  Created on: 24.07.2024
 *      Author: schoeffmannc
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_

#include "DataTypes.h"
#include "user_defines.h"
#include "PI.h"
#include "Currents.h"

//--- Objects, variables,... ---
Currents i;
ADC_RAW_t ADC_RAW;				// [count] Q12
IFX_Q_ABC_t Iabc;				// [per-unit] Qxx
IFX_Q_ALPHA_BETA_t Ialphabeta;  // [per-unit] Qxx
IFX_Q_DQ_t Idq;                 // [per-unit] Qxx
uint16_t ThetaMeRaw_U16 = 0;	// [per-unit] Q1.15
uint16_t ThetaMeRawOld_U16 = 0;	// [per-unit] Q1.15
int16_t dThetaMeRaw_S16 = 0;	// [per-unit] Q1.15
int16_t SpdCtrlPrdFg = 1;		//

uint16_t ThetaReRaw_U16 = 0;	// [per-unit] Q1.15
q_t Cos_Q12 = IFX_Q(1.0f);
q_t Sin_Q12 = IFX_Q(0.0f);		// Q1.11, for trigonometric values
q_t RpmRef = 0, RpmFbk = 0;
q_t RpmRef_p125 = 0, RpmFbk_p125 = 0;
q_t RpmRefPu = 0, RpmFbkPu = 0;
q_t RpmRefFltdPu = 0, RpmFbkFltdPu = 0;

uint16_t POSIF_ValidInitSmplFg  = 0;
uint16_t POSIF_ValidInitSmplCnt = 0;

q_t RpmFbkMovAvgSum = 0;
q_t RpmFbkMovAvgBuf[ENCODER_MOV_AVG_SIZE] =  { 0 };
q_t RpmFbkMagMovAvg = 0;
int16_t rpm_fbk_head = 0;

q_t dThetaMeRawMovAvgSum = 0;
q_t dThetaMeRawMovAvgBuf[ENCODER_MOV_AVG_SIZE] =  { 0 };
q_t dThetaMeRawMagMovAvg = 0;
int16_t dThetaMeRaw_head = 0;
q_t dThetaMeRawFltd = 0;
q_t dThetaMeRawFltdErr = 0;
q_t OmegaMeFltd = 0;
q_t RpmFltd = 0;

PI_CTRL_t SpdCtrl;
PI_CTRL_t IdCtrl;
PI_CTRL_t IqCtrl;

q_t MIcmd = 0;
uint16_t ThetaPwm_U16 = 0;

uint16_t MechanicalAngle_initial = 0;
uint16_t ThetaReRaw_U16_init = 0;
uint16_t ThetaMeRaw_U16_init = 0;
uint32_t theta_init;

int32_t MechanicalAngle_temp = 0;
uint32_t loop_counter = 0;
uint16_t angle_counter = 0;

uint16_t ThetaPwm_ANGLE = 0;

uint8_t rotation_detect_flag = 0;
uint8_t encoder_calib_flag = 0;
uint8_t prepos_flag = 0;

#endif /* GLOBAL_VARIABLES_H_ */
