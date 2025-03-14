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
 * Constants.h
 *
 *  Created on: 24 Aug 2023
 *      Author: SchiestlMart
 */

#ifndef LIBRARIES_CONSTANTS_H_
#define LIBRARIES_CONSTANTS_H_

//#define ENCODER_MODE_SELECTION			(0U)		// 0U ... Manual Speed Calc. w. Moving Average Filter;			1U ... App integrated PT1 Speed Calculation

#define VDC 					(24.0f) // DC-link voltage 1/2
#define SQRT_THREE_HALF 		(0.8660254038f)
#define TWO_INV_SQRT_THREE 		(1.154700538f)
#define SQRT_THREE_THIRD 		(0.57735f)
#define PI_DIV_180 				(0.0174533f)
#define RAD_TO_DEG 				(57.295779513082323f)
#define TWO_THIRD 				(0.666666666f)
#define ONE_THIRD 				(0.333333333f)
#define TWO_PI 					(6.2832f)

#endif /* LIBRARIES_CONSTANTS_H_ */
