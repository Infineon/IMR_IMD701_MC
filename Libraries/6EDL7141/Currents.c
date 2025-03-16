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
 * Currents.c
 *
 *  Created on: 24 Aug 2023
 *      Author: SchiestlMart
 */

#include "Currents.h"

void init_currents(Currents* i){

	if(I_OFFSET_CALIBRATION_CS_EN_DCCAL == false){
		/* load 6EDL7141 parameter from flash,
		 * set Edl7141Configured to 1 if load was succeed */
		EDL7141_FLASH_parameter_load();

		Edl7141Reg.SENSOR_CFG =
				HALL_DEGLITCH_640ns	<< SENSOR_CFG_HALL_DEGLITCH_Pos |
				OTEMP_PROT_DIS		<< SENSOR_CFG_OTS_DIS_Pos |
				CS_ACTIVE_ALWAYS	<< SENSOR_CFG_CS_TMODE_Pos;

		// Enable CSO Calibration
		Edl7141Reg.CSAMP_CFG =
				CS_GAIN_64V 		<< CSAMP_CFG_CS_GAIN_Pos |
				CS_GAIN_PROG_DIG 	<< CSAMP_CFG_CS_GAIN_ANA_Pos |
				CS_A_EN_B_EN_C_EN	<< CSAMP_CFG_CS_EN_Pos |
				CS_BLANK_500ns		<< CSAMP_CFG_CS_BLANK_Pos |
				CS_CALIB_EN			<< CSAMP_CFG_CS_EN_DCCAL_Pos |
				CS_DEGLITCH_8us		<< CSAMP_CFG_CS_OCP_DEGLITCH_Pos |
				OCP_FLT_TRIG_8		<< CSAMP_CFG_CS_OCPFLT_CFG_Pos;

		/* Initialize SPI interface with 6EDL7141, and 6EDL7141 related IO */
		EDL7141_Config_init();

		while(!CS_EN_DCCAL_ManualCalibration(i));
	}
	else{
		i->U_offset = ADC_TOTAL_STEPS/2;
		i->V_offset = ADC_TOTAL_STEPS/2;
		i->W_offset = ADC_TOTAL_STEPS/2;
	}

	i->d_ref = 0.0;
	i->d_ramp = 0.0;
	i->q_ramp = 0.0;

	i->ADC_scaling = 1/(ADC_TOTAL_STEPS/VREF*SHUNT_RESISTANCE*AMPLIFICATION);
}

bool CS_EN_DCCAL_ManualCalibration(Currents* i) {
	ADC_MEASUREMENT_StartConversion(&ADC_MEASUREMENT_0);
	static uint16_t Counter = 1;
	static float i_U_Avg = 0;
	static float i_V_Avg = 0;
	static float i_W_Avg = 0;

	i_U_Avg += ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Channel_A);
	i_V_Avg += ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Channel_B);
	i_W_Avg += ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Channel_C);

	if(Counter >= VADC_OFFSET_SAMPLES_NUM) {
		i->U_offset = i_U_Avg / Counter;
		i->V_offset = i_V_Avg / Counter;
		i->W_offset = i_W_Avg / Counter;
		return true;
	}

	Counter++;
	return false;
}
