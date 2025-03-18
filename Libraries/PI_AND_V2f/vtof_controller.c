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
 * vtof_controller.c
 *
 *  Created on: 24.07.2024
 *      Author: schoeffmannc
 */

/*
 * ifx_vtof_controller_f32.c
 *
 *  Created on: 15 May 2024
 *      Author: schoeffmannc
 */

/*
#include "vtof_controller.h"

void vtof_init(vtof_controller *vtof_controller)
{
	vtof_controller->control_rate = VTOF_RATE;
	vtof_controller->vtof_counter = 1;
	vtof_controller->rate_ratio = PWM_FREQ_HZ / VTOF_RATE;
	vtof_controller->vfratio = VDC_24/M / RPM_RATED;
	vtof_controller->corner_speed = 0.2f * RPM_RATED;
	vtof_controller->corner_voltage = vtof_controller->vfratio * vtof_controller->corner_speed;
	vtof_controller->target_speed = VTOF_TARGET_SPEED;
	vtof_controller->target_voltage = vtof_controller->vfratio * vtof_controller->target_speed;
	vtof_controller->rampup = VTOF_RAMPUP;
	vtof_controller->rampup_time = vtof_controller->target_speed / vtof_controller->rampup;
	vtof_controller->speed = 0.0f;
	vtof_controller->nr_cycles = vtof_controller->rampup_time * VTOF_RATE;
	vtof_controller->speed_increment_el = ((vtof_controller->rampup * POLE_PAIR) / 60.0f) / (VTOF_RATE);
}

void ifx_vtof_rampup(vtof_controller *vtof_controller, float *ref_angle_f32, float *ref_voltage_f32)
{
	float delta_angle_f32;

	if (vtof_controller->vtof_counter < vtof_controller->nr_cycles)
	{
		vtof_controller->speed += vtof_controller->speed_increment_el;

		delta_angle_f32 = vtof_controller->speed / (VTOF_RATE);

		*ref_angle_f32 += TWO_PI_F32*delta_angle_f32;
		*ref_voltage_f32 = vtof_controller->vfratio * (vtof_controller->speed * 60.0f / POLE_PAIR);

		if (*ref_angle_f32 >= TWO_PI_F32)
		{
			*ref_angle_f32 = 0.0f;
		}

		vtof_controller->vtof_counter += 1;

	}
	else
	{
		delta_angle_f32 = vtof_controller->speed / (VTOF_RATE);

		*ref_angle_f32 += TWO_PI_F32*delta_angle_f32;
		*ref_voltage_f32 = vtof_controller->vfratio * (vtof_controller->speed * 60.0f / POLE_PAIR);

		if (*ref_angle_f32 >= TWO_PI_F32)
		{
			*ref_angle_f32 = 0.0f;
		}
	}
}

*/
