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
 * Currents.h
 *
 *  Created on: 24 Aug 2023
 *      Author: SchiestlMart
 */

#ifndef LIBRARIES_CURRENTS_H_
#define LIBRARIES_CURRENTS_H_

#include "xmc_common.h"
#include "../6EDL7141/6EDL_gateway.h"
#include "../Libraries/ADC_MEASUREMENT/adc_measurement.h"

#define ADC_TOTAL_STEPS 4096
#define VREF 5
#define AMPLIFICATION 64
#define SHUNT_RESISTANCE 0.01

#define I_OFFSET_CALIBRATION_CS_EN_DCCAL false
#define VADC_OFFSET_SAMPLES_NUM		1000		// Number of Samples used for initial DC Offset Calibration


typedef struct{
	float U;
	float V;
	float W;
	float alpha;
	float beta;
	float d;
	float q;
	float d_ref;
	float q_ref;
	float d_ramp;
	float q_ramp;
	float ADC_scaling;
	XMC_VADC_RESULT_SIZE_t U_offset;
	XMC_VADC_RESULT_SIZE_t V_offset;
	XMC_VADC_RESULT_SIZE_t W_offset;
}Currents;

void init_currents(Currents* i);
bool CS_EN_DCCAL_ManualCalibration(Currents* i);

#endif /* LIBRARIES_CURRENTS_H_ */
