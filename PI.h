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
 * PI.h
 *
 *  Created on: May 30, 2024
 *      Author: leeyoung
 */

#ifndef PMSM_FOC_FOCLIB_PI_H_
#define PMSM_FOC_FOCLIB_PI_H_

#include "DataTypes.h"
#include "user_defines.h"

//------------------------------------------------------------------------------
//--- Motor electrical parameters. Magnetic parameter may be necessary later
//--- This is derived from the primary parameters
//------------------------------------------------------------------------------
#define RS							(RS_MTR_LL*0.5f)
#define LD_NOM						(LS_MTR_LL*0.5f)
#define LQ_NOM						(LS_MTR_LL*0.5f)


//---------------------------------------------------------
//--- Speed Control
//---------------------------------------------------------
#define Fs_SPD_CTRL					TASK1_FREQ_HZ			// [Hz] Speed controller frequency
#define TICK_SPD_CTRL_PRD_BY_TASK0	((int16_t)(TASK0_FREQ_HZ / Fs_SPD_CTRL))
#define Fc_SPD_CTRL					(25.0f)					// [Hz] Speed controller bandwidth
#define Ts_SPD_CTRL					(1.0f / Fs_SPD_CTRL)
#define RPM_MAX						300.0f					// [rpm] Maximum motor speed
#define OUT_SPD_CTRL_MAX			( 6.0f)					// [A] Maximum current reference from speed controller
#define OUT_SPD_CTRL_MIN			(-6.0f)					// [A] Minimum current reference from speed controller

#define Kp_SPD_CTRL					0.1f //0.2f					// [A/rad/s]
#define Ki_SPD_CTRL					(1.0f*Ts_SPD_CTRL) 	//(1.0f*Ts_SPD_CTRL)		// [A/rad/s]
#define Ka_SPD_CTRL					0.0f					// [A/rad/s]

#define Kp_SPD_CTRL_PU				IFX_Q(Kp_SPD_CTRL            *OMEGA_ME_BASE/I_BASE)
#define Ki_SPD_CTRL_PU				IFX_Q(Ki_SPD_CTRL*Ts_SPD_CTRL*OMEGA_ME_BASE/I_BASE)
#define Ka_SPD_CTRL_PU				IFX_Q(Ka_SPD_CTRL            *OMEGA_ME_BASE/I_BASE)

#define OUT_SPD_CTRL_MAX_PU			IFX_Q(OUT_SPD_CTRL_MAX / I_BASE)			// [pu] Maximum current reference from speed controller
#define OUT_SPD_CTRL_MIN_PU			IFX_Q(OUT_SPD_CTRL_MIN / I_BASE)			// [pu] Minimum current reference from speed controller

//---------------------------------------------------------
//--- Current Control
//---------------------------------------------------------
#define Fc_CRNT_CTRL				(250.0f)				// [Hz]
#define Ts_CRNT_CTRL				(1.0f / TASK0_FREQ_HZ)
#define I_MAX						6.0f					// [A]   Maximum motor current
#define	SVM_VREF_MAX				1.0f					// [pu] SVM max MI value
#define OUT_CRNT_CTRL_MAX			( SVM_VREF_MAX)			// [V] Maximum voltage reference from current controller
#define OUT_CRNT_CTRL_MIN			(-SVM_VREF_MAX)    		// [V] Minimum voltage reference from current controller

#define GpD							1.0f					// Scaling factor for KpD
#define GpQ							1.0f					// Scaling factor for KpQ
#define GiD							0.01f					// Scaling factor for KiD
#define GiQ							0.01f					// Scaling factor for KiQ

#define KpD							((GpD*TWO_PI*LD_NOM*Fc_CRNT_CTRL         ) * V_BASE_OVER_VREF_NOM_OUT_MAX)	// Gain normalized wrt Vbase / (2/pi*Vdc_nom)
#define KpQ							((GpQ*TWO_PI*LQ_NOM*Fc_CRNT_CTRL         ) * V_BASE_OVER_VREF_NOM_OUT_MAX)	//
#define KiD							((GiD*TWO_PI*RS*Fc_CRNT_CTRL*Ts_CRNT_CTRL) * V_BASE_OVER_VREF_NOM_OUT_MAX)	//
#define KiQ							((GiQ*TWO_PI*RS*Fc_CRNT_CTRL*Ts_CRNT_CTRL) * V_BASE_OVER_VREF_NOM_OUT_MAX)	//
#define KaD							(0.0f)					// [V]
#define KaQ							(0.0f)					// [V]

#define KpD_PU						IFX_Q(KpD / Z_BASE)		// Gain normalized wrt Zbase
#define KpQ_PU						IFX_Q(KpQ / Z_BASE) 	//
#define KiD_PU						IFX_Q(KiD / Z_BASE) 	//
#define KiQ_PU						IFX_Q(KiQ / Z_BASE) 	//
#define KaD_PU						IFX_Q(KaD / Z_BASE)		//
#define KaQ_PU						IFX_Q(KaQ / Z_BASE)		//

#define OUT_CRNT_CTRL_MAX_PU		IFX_Q(OUT_CRNT_CTRL_MAX)	// [pu] Maximum voltage reference from current controller
#define OUT_CRNT_CTRL_MIN_PU		IFX_Q(OUT_CRNT_CTRL_MIN)	// [pu] Minimum voltage reference from current controller
#define I_MAX_PU					IFX_Q(I_MAX / I_BASE)	// [pu]	Maximum motor current

//-----------------------------------------------------------------------------------------------------------
//---
//--- PI Controllers
//---
//-----------------------------------------------------------------------------------------------------------
typedef volatile struct {
	q_t ref;
	q_t fbk;
	q_t fbk1;
	q_t err;
	q_t err1;

	q_t	Kp;
	q_t Ki;
	q_t Ka;

	q_t Pterm;
	q_t Iterm;

	q_t PtermErr;
	q_t ItermErr;

	q_t Out_max;
	q_t Out_min;

	q_t Out;
} PI_CTRL_t;


typedef struct {
	q_t p_err_max;
	q_t p_err_min;
	q_t i_err_max;
	q_t i_err_min;
	q_t ref;
	q_t fbk;
	q_t err;
	q_t err1;

	q_t	Kp;
	q_t Ki;

	q_t Pterm;
	q_t Iterm;
	q_t Dterm;
	q_t Dcpl;
	q_t PtermErr;
	q_t ItermErr;

	q_t Out_max;
	q_t Out_min;

	q_t Out;
} CRNT_CTRL_t;


//--------------------------------------------
//--- PI Controller w/ 2 degree of freedom
//--------------------------------------------
#if 1
__STATIC_INLINE void PI_Ctrl_Inline(PI_CTRL_t *ptr)
{
	q_t out, i_temp, err;

	err = ptr->ref - ptr->fbk;

	i_temp = (ptr->Kp*err - ptr->Ka*ptr->fbk);									// Active Damping
	ptr->Pterm = i_temp >> Qxx;
	i_temp -= (ptr->Pterm << Qxx);

	#if 0
	i_temp = (ptr->Iterm << Qxx) + (err * ptr->Ki) + ptr->ItermErr + i_temp;	// Q30 to take care of the truncation error term from the fixed point integration
	out    = i_temp >> Qxx;														// Q15
	ptr->ItermErr = i_temp - (out << Qxx);										// Q30

	#else
	i_temp += err*ptr->Ki;														// Q30 to take care of the truncation error term from the fixed point integration
	out     = ptr->Iterm + (i_temp >> Qxx);
	#endif

	if 		(out > ptr->Out_max)	out = ptr->Out_max;							// Q15
	else if (out < ptr->Out_min)	out = ptr->Out_min;							// Q15
	ptr->Iterm = out;

	out += (ptr->Pterm = IFX_Q_mul(ptr->err, ptr->Kp));
	if 		(out > ptr->Out_max)	out = ptr->Out_max;
	else if (out < ptr->Out_min)	out = ptr->Out_min;

	ptr->err = err;
	ptr->Out = out;
}
#else
__STATIC_INLINE void PI_Ctrl_Inline(volatile PI_CTRL_t *ptr)
{
	q_t dout, i_temp;
	q_t derr, err, dfbk;

	err  = ptr->ref - ptr->fbk;
	derr = err - ptr->err1;
	dfbk = ptr->fbk - ptr->fbk1;
	ptr->err1 = err;
	ptr->fbk1 = ptr->fbk;

	dout = (derr*ptr->Kp) + (err*ptr->Ki) - (dfbk*ptr->Ka) + ptr->ItermErr;
	i_temp = dout >> Qxx;
	ptr->ItermErr = dout - (i_temp << Qxx);
	i_temp += ptr->Iterm;

	if 		(i_temp > ptr->Out_max)	i_temp = ptr->Out_max;
	else if (i_temp < ptr->Out_min)	i_temp = ptr->Out_min;
	ptr->Iterm = i_temp;
	ptr->Out = i_temp;
}
#endif

extern void PI_Ctrl(PI_CTRL_t *ptr);
extern void InitPI_Ctrl(PI_CTRL_t *ptr, q_t kp, q_t ki, q_t ka, q_t out_min, q_t out_max);
extern void PresetPI_Ctrl(PI_CTRL_t *ptr, q_t val);
extern void SetPI_CtrlGain(PI_CTRL_t *ptr, q_t kp, q_t ki, q_t ka, q_t out_min, q_t out_max, int16_t init_fg);

#endif /* PMSM_FOC_FOCLIB_PI_H_ */
