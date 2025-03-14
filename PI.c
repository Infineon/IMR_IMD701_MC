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
 * PI.c
 *
 *  Created on: May 30, 2024
 *      Author: leeyoung
 */

#include "PI.h"

//-----------------------------------------------------------------------------------------------------------
//---
//--- Speed PI Controllers
//---
//-----------------------------------------------------------------------------------------------------------
//-----------------------------------------
//--- Speed PI Controller
//-----------------------------------------
void PI_Ctrl(PI_CTRL_t *ptr)
{
	PI_Ctrl_Inline(ptr);
}

//-----------------------------------------
//--- Set PI controllers' gains
//-----------------------------------------
void SetPI_CtrlGain(PI_CTRL_t *ptr, q_t kp, q_t ki, q_t ka, q_t out_min, q_t out_max, int16_t init_fg)
{
	ptr->Kp = kp;
	ptr->Ki = ki;
	ptr->Ka = ka;
	ptr->Out_max = out_max;
	ptr->Out_min = out_min;
}

//-----------------------------------------
//--- Initialize PI controller
//-----------------------------------------
void InitPI_Ctrl(PI_CTRL_t *ptr, q_t kp, q_t ki, q_t ka, q_t out_min, q_t out_max)
{
	SetPI_CtrlGain(ptr, kp, ki, ka, out_min, out_max, 1);
	ptr->ref       = 0;
	ptr->fbk       = 0;
	ptr->fbk1      = 0;
	ptr->err       = 0;
	ptr->err1      = 0;
	ptr->Pterm     = 0;
	ptr->Iterm     = 0;
//	ptr->PtermErr  = 0;
	ptr->ItermErr  = 0;
	ptr->Out       = 0;
}

//-----------------------------------------
//--- Preset PI controller
//-----------------------------------------
void PresetPI_Ctrl(PI_CTRL_t *ptr, q_t val)
{
	ptr->ref    = 0;
	ptr->fbk    = 0;
	ptr->fbk1   = 0;
	ptr->err    = 0;
	ptr->err1   = 0;
	ptr->Pterm  = 0;
	ptr->Iterm  = val;
//	ptr->PtermErr = 0;
	ptr->ItermErr = 0;
	ptr->Out    = val;
}


//-----------------------------------------------------------------------------------------------------------
//---
//--- Current PI Controllers
//---
//-----------------------------------------------------------------------------------------------------------
__RAM_FUNC void CrntCtrl(volatile CRNT_CTRL_t *ptr)
{
	q_t dout, i_temp;
	q_t err, derr;

	err  = ptr->ref - ptr->fbk;
	derr = err - ptr->err1;
	ptr->err1 = err;

	dout = (derr*ptr->Kp) + (err*ptr->Ki) + ptr->ItermErr;
	i_temp = dout >> Qxx;
	ptr->ItermErr = dout - (i_temp << Qxx);
	i_temp += ptr->Iterm;

	if 		(i_temp > ptr->Out_max)	i_temp = ptr->Out_max;
	else if (i_temp < ptr->Out_min)	i_temp = ptr->Out_min;
	ptr->Iterm = i_temp;

	ptr->Out = i_temp + ptr->Dcpl;
}

void SetCrntCtrlGain(volatile CRNT_CTRL_t *ptr, q_t kp, q_t ki, q_t kd, q_t kcmplx, q_t krdamp, q_t out_min, q_t out_max, int16_t init_fg)
{
	ptr->Kp = kp;
	ptr->Ki = ki;
	ptr->Out_max   = out_max;
	ptr->Out_min   = out_min;
}

void InitCrntCtrl(volatile CRNT_CTRL_t *ptr, q_t kp, q_t ki, q_t kd, q_t kcmplx, q_t krdamp, q_t out_min, q_t out_max)
{
	SetCrntCtrlGain(ptr, kp, ki, kd, kcmplx, krdamp, out_min, out_max, 1);

	ptr->ref       = 0;
	ptr->fbk       = 0;
	ptr->err       = 0;
	ptr->err1      = 0;
	ptr->Pterm     = 0;
	ptr->Iterm     = 0;
	ptr->Dterm     = 0;
	ptr->Dcpl      = 0;
	ptr->PtermErr  = 0;
	ptr->ItermErr  = 0;
	ptr->Out       = 0;
}

void PresetCrntCtrl(volatile CRNT_CTRL_t *ptr, q_t val)
{
	ptr->ref      = 0;
	ptr->fbk      = 0;
	ptr->err      = 0;
	ptr->err1     = 0;
	ptr->Pterm    = 0;
	ptr->Iterm    = val;
	ptr->Dterm    = 0;
	ptr->Dcpl     = 0;
	ptr->PtermErr = 0;
	ptr->ItermErr = 0;
	ptr->Out      = val;
}
