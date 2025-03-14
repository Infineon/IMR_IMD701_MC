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
 * CORDIC_math.h
 *
 *  Created on: Mar 13, 2023
 *      Author: leeyoung
 */

#ifndef PMSM_FOC_FOCLIB_BMCLIB_CORDIC_MATH_H_
#define PMSM_FOC_FOCLIB_BMCLIB_CORDIC_MATH_H_

#ifndef __STATIC_INLINE
#define __STATIC_INLINE	static inline
#endif

#include "DataTypes.h"

#define CORDIC_USED			1		// 1: Use CORDIC HW Accelerator
#define CHECK_CORDIC_BUSY	0
#define CORDIC_SHIFT		14

#define MATH__abs(x)				((x < 0)? -x: x)
#define MATH__minmax(x, min, max)	((x < min)? min: ((x > max)? max: x))
#define MATH__min(x, min)			((x < min)? min: x)
#define MATH__max(x, max)			((x > max)? max: x)

__STATIC_INLINE void ABCToAlphaBeta(const q_t a, const q_t b, const q_t c, volatile q_t *alpha, volatile q_t *beta)
{
	*alpha = IFX_Q_mul_n( (a - ((b + c) >> 1)), IFX_Q14(2.0f / 3.0f), Q14);	// Qxx: Ialpha = (Iu - (Iv + Iw)/2) * 2/3
	*beta  = IFX_Q_mul_n(       (b - c),        IFX_Q14(   0.57735f), Q14);	// Qxx: Ibeta  =       (Iv - Iw)    /  √3
}

__STATIC_INLINE void AlphaBetaToABC(const q_t alpha, const q_t beta, volatile q_t *a, volatile q_t *b, volatile q_t *c)
{
	*a = alpha;
	*b = (beta*IFX_Q(0.866025f) - (alpha >> 1)) >> Q14;	// Qxx: IFX_Q_mul(beta, IFX_Q(0.866025f)) - IFX_Q_mul(alpha, IFX_Q(0.5f));
	*c = -(*a + *b);
}

__STATIC_INLINE void AlphaBetaToDQ(const q_t alpha, const q_t beta, const q_t cos_q12, const q_t sin_q12, volatile q_t *d, volatile q_t *q)
{
	*d = (alpha*cos_q12 +  beta*sin_q12) >> Q12;		// Qxx
	*q = ( beta*cos_q12 - alpha*sin_q12) >> Q12;		// Qxx
}

__STATIC_INLINE void DQToAlphaBeta(const q_t d, const q_t q, const q_t cos_q12, const q_t sin_q12, volatile q_t *alpha, volatile q_t *beta)
{
	*alpha = (d*cos_q12 - q*sin_q12) >> Q12;			// Qxx
	*beta  = (q*cos_q12 + d*sin_q12) >> Q12;			// Qxx
}


#define IS_CORDIC_BUSY                     (MATH->STATC & 0x01)	/*!< Returns 1 if CORDIC is busy */
#define IS_MATH_DIV_BUSY                   (MATH->DIVST & 0x01)	/*!< Returns 1 if DIVIDER is busy */

#define CORDIC_CIRCULAR_VECTORING_MODE     (0x62)          		/*!< CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default). */
#define CORDIC_CIRCULAR_MPS_BY_K_SCALED    (311)           		/*!< CORDIC MPS/K ->(2/1.64676)* 2^CORDIC_MPS_BY_K_SCALE */
#define CORDIC_MPS_BY_K_SCALE              (8)             		/*!< CORDIC MPS/K scaling factor */
#define CORDIC_HYPERBOLIC_VECTORING_MODE   (0x66)          		/*!< CORDIC: Hyperbolic Vectoring Mode. MPS: Divide by 2 (default).*/
#define CORDIC_HYPERBOLIC_MPS_BY_K_SCALED  (618)           		/* CORDIC MPS/K ->(2/0.828159360960)* 2^CORDIC_MPS_BY_K_SCALE */
#define CORDIC_MPS_BY_K_SCALE              (8)             		/*!< CORDIC MPS/K scaling factor */

#define nop10()	{	asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");	}	// nop = 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
#define nop1()	{	asm("nop");	}
#define nop_320ns	nop10
#define nop_32ns	nop1
#define nop_640ns()	{	nop_320ns();	nop_320ns();	}
#define nop_672ns()	{	nop_640ns();	nop_32ns();		}


__STATIC_INLINE void CORDIC_udiv_core(const uint32_t x, const uint32_t y, udiv32_t *result)
{
//	XMC_GPIO_SetOutputHigh(TP151);		// 42ns
//	XMC_GPIO_SetOutputLow(TP151);		// 42ns
//	XMC_GPIO_SetOutputHigh(TP151);		// 42ns
	MATH->DVD = x;						// dividend
	MATH->DVS = y;						// divisor and get the operation started: 35 Kernel Cycles (96MHz) = 364.58333ns
//	XMC_GPIO_SetOutputLow(TP151);		// 42ns
	#if (CHECK_CORDIC_BUSY)
	while (MATH->DIVST);				//
	#else
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	#endif
//	XMC_GPIO_SetOutputHigh(TP151);		// 42ns
	result->q = MATH->QUOT;				// = x / y
	result->r = MATH->RMD;				// = x - QUOT*y
//	XMC_GPIO_SetOutputLow(TP151);
}

__STATIC_INLINE void CORDIC_div_core(const int32_t x, const int32_t y, div32_t *result)
{
//	XMC_GPIO_SetOutputHigh(TP151);		// 42ns
//	XMC_GPIO_SetOutputLow(TP151);		// 42ns
//	XMC_GPIO_SetOutputHigh(TP151);		// 42ns
	MATH->DVD = x;						// dividend
	MATH->DVS = y;						// divisor and get the operation started: 35 Kernel Cycles (96MHz) = 364.58333ns
//	XMC_GPIO_SetOutputLow(TP151);		// 42ns
	#if (CHECK_CORDIC_BUSY)
	while (MATH->DIVST);				//
	#else
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	asm("nop");		// 32ns however ideal 20.8333ns per 48MHz: 1 CPU clock cycle
	#endif
//	XMC_GPIO_SetOutputHigh(TP151);		// 42ns
	result->q = (int32_t)MATH->QUOT;	// = x / y
	result->r = (int32_t)MATH->RMD;		// = x - QUOT*y
//	XMC_GPIO_SetOutputLow(TP151);
}

//--------------------------------------------------
//--- unsigned 16bit div by 16bit: 650ns
//--------------------------------------------------
__STATIC_INLINE void CORDIC_udiv_16by16(const uint16_t x, const uint16_t y, udiv32_t *result)
{
	//--- CORDIC Division takes about 1us ---
	MATH->DIVCON = (0x00008014 | (0 << 16UL) | (0 << 8UL));		// 0000 0000 000x xxxx 100x xxxx 0001 0100: 16 div by 16, unsigned, start with DVS writing,
	CORDIC_udiv_core(x, y, result);
}

//--------------------------------------------------
//--- unsigned 32bit div by 32bit: 650ns
//--------------------------------------------------
__STATIC_INLINE void CORDIC_udiv_32by32(const uint32_t x, const uint32_t y, udiv32_t *result)
{
	//--- CORDIC Division takes about 1us ---
	MATH->DIVCON = (0x00008004 | (0 << 16UL) | (0 << 8UL));		// 0000 0000 000x xxxx 100x xxxx 0000 0100: 32 div by 32, unsigned, start with DVS writing,
	CORDIC_udiv_core(x, y, result);
}

//--------------------------------------------------
//--- unsigned 32bit div by 16bit: 650ns
//--------------------------------------------------
__STATIC_INLINE void CORDIC_udiv_32by16(const uint32_t x, const uint16_t y, udiv32_t *result)
{
	//--- CORDIC Division takes about 1us ---
	MATH->DIVCON = (0x0000800C | (0 << 16UL) | (0 << 8UL));		// 0000 0000 000x xxxx 100x xxxx 0000 1100: 32 div by 16, unsigned, start with DVS writing,
	CORDIC_udiv_core(x, y, result);
}

//--------------------------------------------------
//--- signed 16bit div by 16bit: 650ns
//--------------------------------------------------
__STATIC_INLINE void CORDIC_div_16by16(const int16_t x, const int16_t y, div32_t *result)
{
	//--- CORDIC Division takes about 1us ---
	MATH->DIVCON = (0x00008010 | (0 << 16UL) | (0 << 8UL));		// 0000 0000 000x xxxx 100x xxxx 0001 0000: 16 div by 16, signed, start with DVS writing,
	CORDIC_div_core(x, y, result);
}

//--------------------------------------------------
//--- signed 32bit div by 32bit: 650ns
//--------------------------------------------------
__STATIC_INLINE void CORDIC_div_32by32(const int32_t x, const int32_t y, div32_t *result)
{
	//--- CORDIC Division takes about 1us ---
	MATH->DIVCON = (0x00008000 | (0 << 16UL) | (0 << 8UL));		// 0000 0000 000x xxxx 100x xxxx 0000 0000: 32 div by 32, signed, start with DVS writing,
	CORDIC_div_core(x, y, result);
}

//--------------------------------------------------
//--- signed 32bit div by 16bit: 650ns
//--------------------------------------------------
__STATIC_INLINE void CORDIC_div_32by16(const int32_t x, const int16_t y, div32_t *result)
{
	//--- CORDIC Division takes about 1us ---
	MATH->DIVCON = (0x00008008 | (0 << 16UL) | (0 << 8UL));		// 0000 0000 000x xxxx 100x xxxx 0000 1000: 32 div by 16, signed, start with DVS writing,
	CORDIC_div_core(x, y, result);
}

//--------------------------------------------------
//--- signed 32bit qdiv by 16bit: 650ns
//--------------------------------------------------
__STATIC_INLINE void CORDIC_qdiv_32by16(const int32_t x, const int16_t y, div32_t *result)		// for |x| < |y|
{
	//--- CORDIC Division takes about 1us ---
	MATH->DIVCON = (0x00008008 | (15 << 16UL) | (0 << 8UL));	// 0000 0000 000x xxxx 100x xxxx 0000 1000: 32 div by 16, signed, start with DVS writing,
	CORDIC_div_core(x, y, result);
}

__STATIC_INLINE __RAM_FUNC float FLOAT_Mul(const float x, const float y)
{
	float ret;

	ret = x*y;
	return ret;
}

__STATIC_INLINE __RAM_FUNC float FLOAT_Div(const float x, const float y)
{
	float ret;

	ret = x/y;
	return ret;
}

//--- sqrt(x^2 + y^2) & atan2 (y/x) ---
__STATIC_INLINE q_t CORDIC_Q_mag_atan2(const q_t x, const q_t y, q_t *theta)
{
	q_t mag;

	MATH->CON   = CORDIC_CIRCULAR_VECTORING_MODE;	/* General control of CORDIC Control Register */
	MATH->CORDZ = (*theta) << (Q31 - Qxx);			/* Z = φ. Θ = atan(Vq/Vd) + rotor angle φ, equivalent to Inv. Park Transform */
	MATH->CORDY = (uint32_t) (y << CORDIC_SHIFT); 	/* Y = Vq = PI_Torque.Uk */
	MATH->CORDX = (uint32_t) (x << CORDIC_SHIFT);	/* X = Vd = PMSM_FOC_FLUX_PI.Uk. Input CORDX data, and auto start of CORDIC calculation */

	#if (CHECK_CORDIC_BUSY)
	while (IS_CORDIC_BUSY);
	#endif

	*theta = (((q_t)MATH->CORRZ) >> (Q31 - Qxx));	// Q31: Angle addition by CORDIC directly, where Θ = atan(y/x), φ is rotor angle */

	mag = MATH->CORRX;								// (Qxx + CORDIC_SHIFT) Read CORDIC result |Vref| - 32-bit unsigned  and scale down to get real value */
	mag = (mag >> CORDIC_SHIFT);					/* Get real values by scaling down */
	mag = (uint32_t) ((mag * CORDIC_CIRCULAR_MPS_BY_K_SCALED) >> CORDIC_MPS_BY_K_SCALE); // x MPS/K.

	return mag;
}

//--- sqrt(x^2 + y^2) & atan2 (y/x) ---
__STATIC_INLINE q_t CORDIC_Q_sqrt_xy(const q_t x, const q_t y)
{
	q_t mag;

	MATH->CON = CORDIC_CIRCULAR_VECTORING_MODE;		/* General control of CORDIC Control Register */
	MATH->CORDZ = 0;								/* Z = φ. Θ = atan(Vq/Vd) + rotor angle φ, equivalent to Inv. Park Transform */
	MATH->CORDY = (uint32_t) (y << CORDIC_SHIFT); 	/* Y = Vq = PI_Torque.Uk */
	MATH->CORDX = (uint32_t) (x << CORDIC_SHIFT);	/* X = Vd = PMSM_FOC_FLUX_PI.Uk. Input CORDX data, and auto start of CORDIC calculation */

	#if (CHECK_CORDIC_BUSY)
	while (IS_CORDIC_BUSY);
	#endif

	mag = MATH->CORRX;								// (Qxx + CORDIC_SHIFT) Read CORDIC result |Vref| - 32-bit unsigned  and scale down to get real value */
	mag = (mag >> CORDIC_SHIFT);					/* Get real values by scaling down */
	mag = (uint32_t) ((mag * CORDIC_CIRCULAR_MPS_BY_K_SCALED) >> CORDIC_MPS_BY_K_SCALE); // x MPS/K.

	return mag;
}

//--- sqrt(x^2 - y^2) ---
__STATIC_INLINE q_t CORDIC_Q_sqrt_xsqr_ysqr(const q_t x, const q_t y)
{
	MATH->CON = CORDIC_HYPERBOLIC_VECTORING_MODE;	/* General control of CORDIC Control Register */
	MATH->CORDZ = 0;								/* Z = φ. Θ = atan(Vq/Vd) + rotor angle φ, equivalent to Inv. Park Transform */
	MATH->CORDY = (uint32_t) (y << CORDIC_SHIFT);	/* Y = Vq = PI_Torque.Uk */
	MATH->CORDX = (uint32_t) (x << CORDIC_SHIFT);	/* X = Vd = PMSM_FOC_FLUX_PI.Uk. Input CORDX data, and auto start of CORDIC calculation */

	uint32_t mag;
	#if (CHECK_CORDIC_BUSY)
	while (IS_CORDIC_BUSY);
	#endif

	mag = MATH->CORRX;								/* Read CORDIC result - 32-bit unsigned  and scale down to get real value */
	mag = (mag >> CORDIC_SHIFT);					/* Get real values by scaling down */
	mag = (uint32_t) ((mag * CORDIC_HYPERBOLIC_MPS_BY_K_SCALED) >> CORDIC_MPS_BY_K_SCALE); // x MPS/K.
	return mag;
}

//--- sqrt(s) ---
__STATIC_INLINE q_t CORDIC_Q_sqrt_x(const q_t x)	// a = Q15.Q15 * Q15.Q15 = Q1.Q30
{
	if (x < 0)
	{
		return 0;
	}
	MATH->CON = CORDIC_HYPERBOLIC_VECTORING_MODE;	/* General control of CORDIC Control Register */
	MATH->CORDZ = 0;								/* Z = φ. Θ = atan(Vq/Vd) + rotor angle φ, equivalent to Inv. Park Transform */
	MATH->CORDY = 0;								/* Y = Vq = PI_Torque.Uk */
	MATH->CORDX = (uint32_t) (x << CORDIC_SHIFT);	/* X = Vd = PMSM_FOC_FLUX_PI.Uk. Input CORDX data, and auto start of CORDIC calculation */

	uint32_t mag;
	#if (CHECK_CORDIC_BUSY)
	while (IS_CORDIC_BUSY);
	#endif

	mag = MATH->CORRX;								/* Read CORDIC result - 32-bit unsigned  and scale down to get real value */
	mag = (mag >> CORDIC_SHIFT);					/* Get real values by scaling down */
	mag = (uint32_t) ((mag * CORDIC_HYPERBOLIC_MPS_BY_K_SCALED) >> CORDIC_MPS_BY_K_SCALE); // x MPS/K.
	return mag;
}


#define COS_SIN_Q12_TBL_SZ		1024
#define COS_SIN_TBL_IDX_MASK 	0x3FF
extern const int16_t Cos_Sin_Q12[COS_SIN_Q12_TBL_SZ][2];

extern __RAM_FUNC void Get_CosSin_Q12(uint16_t ThetaU16, q_t *Cos, q_t *Sin);


#endif /* PMSM_FOC_FOCLIB_BMCLIB_CORDIC_MATH_H_ */
