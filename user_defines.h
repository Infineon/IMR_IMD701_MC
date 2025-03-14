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
 * user_defines.h
 *
 *  Created on: 24.07.2024
 *      Author: schoeffmannc
 */

#ifndef USER_DEFINES_H_
#define USER_DEFINES_H_

//--------------------------------------------------------------------------------
//--- HW Configuration:
//--------------------------------------------------------------------------------
//--- ADC ---
#define ADC_RESOLUTION				12						// [bit]
#define V_ADC_REF					5.0f					// [V]		ADC reference voltage

//--- OP-amp for current feedbacks ---
#define OP_AMP_IN_MAX				0.300f					// [V]		OP-Amp input voltage base
#define OP_AMP_GAIN					64 						//			Chimera OP-AMP gain

//--- Vdc divider and leg shunt ---
#define R_UPPER_VDC_DIV				56.2e3f					// [ohm]	Vdc divider upper side resistor
#define R_LOWER_VDC_DIV				2.61e3f					// [ohm]	Vdc divider lower side resistor
#define R_SHUNT						10.0e-3f				// [ohm]	Leg shunt register value

//--- Motor primary parameters and electrical specs ---
#define VDC_NOM						48.0f					// [V]		Nominal DC-link voltage: Refer to "https://store.tmotor.com/product/gl60-out-running-gimbal-motor.html"
#define RPM_RATED					300.0f					// [rpm]	GL60KV25 Motor Rated Speed
#define POLE_PAIR					14						// [pole-pair]	Motor Pole Pair: Refer to "https://store.tmotor.com/product/gl60-out-running-gimbal-motor.html"
#define RS_MTR_LL					5.5						// [ohm] 	GL60KV25 Motor Stator Resistance Line to Line: Refer to "https://store.tmotor.com/product/gl60-out-running-gimbal-motor.html"
#define LS_MTR_LL					2.72e-3f				// [H]		GL60KV25 Motor Stator Inductance Line to Line: Refer to "https://store.tmotor.com/product/gl60-out-running-gimbal-motor.html"

//--- Emulated Encoder ---
#define ENCODER_PPR_1X				4096					// [cnt/rev]	12bit Encoder PPR: Refer to "https://www.infineon.com/dgdl/Infineon-TLE5012B_Exxxx-DataSheet-v02_01-EN.pdf?fileId=db3a304334fac4c601350f31c43c433f"
#define ENCODER_PPR_4X				(ENCODER_PPR_1X*4)		// [cnt/rev] Quadratic value of the encoder ppr

//--------------------------------------------------------
//--- Base values for normalization
//--------------------------------------------------------
#define	V_BASE						VDC_MEAS_MAX		// [V]	Base Voltage VP_MEAS_MAX > VDC_MEAS_MAX*2.0f
#define I_BASE						10  //I_MEAS_MAX			// [A]	Base Current: select considering the motor peak current level
#define L_BASE						(10.0e-3f)			// [H]
#define RPM_BASE					(1000.0f)			// [rpm]	Select your base speed to cover the maximum intended operating speed of the motor
#define T_ACCEL_BASE				0.1f				// [s] base acceleration time
#define Z_BASE						(V_BASE/I_BASE)		// [ohm] 	Base Impedance
#define	OMEGA_ME_BASE				(RPM_BASE*PI/30.0f)	// [rad/s]	Base Speed for angular velocity
#define THETA_BASE					PI					// [rad]

#define TASK0_FREQ_HZ				20000.0f			// [Hz]
#define TASK1_FREQ_HZ				2000.0f				// [Hz]
#define VTOF_RATE					2000.0f

#define VREF_NOM_OUT_MAX				(2.0f/PI*VDC_NOM)			// [V] Nominal Vref output max
#define V_BASE_OVER_VREF_NOM_OUT_MAX	(V_BASE / VREF_NOM_OUT_MAX)	// [V] Nominal Vref output max

//---------------------------------------------------------
//--- Position/Speed Feedback Detection
//---------------------------------------------------------
#define T_POSIF_VALID				0.001f					// [s]	Time to validate the POSIF position after power on
#define ZERO_RPM_TH					0.125f					// [rpm] Speed relution. Below this value, it is considered 0[rpm]
#define POSIF_INIT_SMPL_CNT			(uint32_t)(T_POSIF_VALID * TASK0_FREQ_HZ)	// (= 1ms / Task0 Period) Initial number of POSIF sampling to validate the position feedback
#define T_POSIF_CLK					333.0e-9f				// [s]  POSIF peripheral base clock period
#define F_POSIF_CLK					3.0e6f					// [Hz]	POSIF peripheral base clock frequency
#define ZERO_RPM_TH_PU				IFX_Q(ZERO_RPM_TH / RPM_BASE)
#define N_POSIF_CLK_TO_p125_RPM		((60.0f/ENCODER_PPR_4X)*F_POSIF_CLK/ZERO_RPM_TH)	// [cnt/rpm] The number of POSIF Base Clock corresponding to 0.125[rpm]
#define	dThetaMeRaw_MOV_AVG_SIZE	32						// Moving average buffer size for dThetaMeRaw
#define N_SHFT_dThetaMeRaw_MOV_AVG	5						// log2(dThetaMeRaw_MOV_AVG_SIZE)
#define dTHETA_PU_2_OMEGA_PU		IFX_Q(THETA_BASE / (OMEGA_ME_BASE*Ts_SPD_CTRL))
#define ENCODER_MOV_AVG_SIZE		42					    // Averaging filter size
#define ENCODER_MECHANICAL_OFFSET	319


//----------------------------------------------
//--- Feedback Circuit Scaling Including ADC
//----------------------------------------------
#define ADC_OUT_MAX					((1 << ADC_RESOLUTION) - 1)

//--- Current Feedback Scaling as a function of Rsh -----
#define OP_AMP_OUT_MAX				(OP_AMP_IN_MAX * OP_AMP_GAIN)							// [V] i.e. 3.6[V]
#define I_SHUNT_MAX					(OP_AMP_IN_MAX / R_SHUNT)								// [A] i.e. 100.0 [A] for 3mOhm
#define I_MEAS_MAX					(I_SHUNT_MAX * 0.5f)									// [A] i.e. -50.0 ~ 50.0[A] for 3mOhm
#define ADC_OUT_MAX_AT_I_SHUNT_MAX	((int16_t)(ADC_OUT_MAX * (OP_AMP_OUT_MAX / V_ADC_REF)))	// [cnt]
#define ADC_TO_AMP					(I_MEAS_MAX / ADC_OUT_MAX_AT_I_SHUNT_MAX)				// [A/cnt]

//--- Vdc [V] feedback ---
#define VDC_DIV						(R_LOWER_VDC_DIV / (R_LOWER_VDC_DIV + R_UPPER_VDC_DIV))
#define VDC_MEAS_MAX				(V_ADC_REF / VDC_DIV)									// [V] i.e 52.65[V]
#define ADC_OUT_MAX_AT_VDC_MEAS_MAX	((float)ADC_OUT_MAX)									// [cnt]
#define ADC_TO_VDC					(VDC_MEAS_MAX / ADC_OUT_MAX_AT_VDC_MEAS_MAX)			// [V/cnt]

//--- Vp [V] Pole Voltage feedback ---
#define R_UPPER_VP_DIV				47.0e3f													// [ohm]
#define R_LOWER_VP_DIV				1.65e3f													// [ohm]
#define VP_DIV						(R_LOWER_VP_DIV/(R_LOWER_VP_DIV + R_UPPER_VP_DIV))
#define VP_MEAS_MAX					(V_ADC_REF / VP_DIV)									// [V] i.e. 147.42[V]
#define ADC_OUT_AT_VP_MEAS_MAX		((float)ADC_OUT_MAX)									// [cnt]
#define ADC_TO_VP					(VP_MEAS_MAX / ADC_OUT_AT_VP_MEAS_MAX)					// [V/cnt]

//--- ADC to PU conversion: Iuvw, Vpuvw, Vdc ---
#define ADC_I_MEAS_TO_PU			((q_t)((V_ADC_REF / (OP_AMP_OUT_MAX*0.5f)*(I_MEAS_MAX / I_BASE))*(0x1 << Qxx)))		// Coefficient to normalize the current ADC feedback
#define ADC_VDC_MEAS_TO_PU			((q_t)((VDC_MEAS_MAX / V_BASE            )*(0x1 << Qxx)))		// Coefficient to normalize the Vdc ADC feedback
#define ADC_VP_MEAS_TO_PU 			((q_t)(( VP_MEAS_MAX / V_BASE            )*(0x1 << Qxx)))		// Coefficient to normalize the Vp ADC feedback
#define PU_TO_ADC_I_MEAS			((q_t)((OP_AMP_OUT_MAX / V_ADC_REF       )*(0x1 << Qxx)))		// Qxx: xx = ADC_RESOLUTION

// PWM defines
#define PWM_BASE_CLK_FREQ_HZ		96.0e6f					// [Hz]
#define PWM_FREQ_HZ					20000.0f				// [Hz]
#define PWM_PRD_REG_VAL				(int32_t)((PWM_BASE_CLK_FREQ_HZ / PWM_FREQ_HZ) - 1)	// = 4799 [cnt]

#define MAX_INT                     65535
#define CPU_FREQ                    48000000.0f
#define DELAY_TIME                  0.5f
#define DELTA_ANGLE                 4
#define TRUE                        1
#define FALSE                       0

// Robot constant variables
#define R_WHEEL 				0.05 // [m]
#define VMAX_ROBOT				3.0  // [m/s]
#define SMAX_WHEEL				(VMAX_ROBOT/R_WHEEL) // [rad/s] theoretical maximum value for the wheel speed
#define _2_15					32768
#define RADPS2_15BIT			(_2_15/SMAX_WHEEL)

#endif /* USER_DEFINES_H_ */
