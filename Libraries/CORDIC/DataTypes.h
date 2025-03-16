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
 * DataTypes.h
 *
 *  Created on: May 30, 2024
 *      Author: leeyoung
 */

#ifndef PMSM_FOC_FOCLIB_DATATYPES_H_
#define PMSM_FOC_FOCLIB_DATATYPES_H_

#include "xmc_common.h"
#include "stdint.h"

#define	q_t				signed long int

#define q8_t			signed long int
#define q9_t			signed long int
#define q10_t			signed long int
#define q11_t			signed long int
#define q12_t			signed long int
#define q13_t			signed long int
#define q14_t			signed long int
#define q15_t			signed long int

#define q16_t			signed long int
#define q17_t			signed long int
#define q18_t			signed long int
#define q19_t			signed long int
#define q20_t			signed long int
#define q21_t			signed long int
#define q22_t			signed long int
#define q23_t			signed long int

#define q24_t			signed long int
#define q25_t			signed long int
#define q26_t			signed long int
#define q27_t			signed long int
#define q28_t			signed long int
#define q29_t			signed long int
#define q30_t			signed long int
#define q31_t			signed long int

#define Q6				6
#define Q7				7
#define Q8				8
#define Q9				9
#define Q10				10
#define Q11				11
#define Q12				12
#define Q13				13
#define Q14				14
#define Q15				15

#define Q16				16
#define Q17				17
#define Q18				18
#define Q19				19
#define Q20				20
#define Q21				21
#define Q22				22
#define Q23				23

#define Q24				24
#define Q25				25
#define Q26				26
#define Q27				27
#define Q28				28
#define Q29				29
#define Q30				30
#define Q31				31

#define Qxx				Q15

#if 0
#define Q8_MAX			((1 <<  Q8) - 1)
#define Q9_MAX			((1 <<  Q9) - 1)
#define Q10_MAX			((1 << Q10) - 1)
#define Q11_MAX			((1 << Q11) - 1)
#define Q12_MAX			((1 << Q12) - 1)
#define Q13_MAX			((1 << Q13) - 1)
#define Q14_MAX			((1 << Q14) - 1)
#define Q15_MAX			((1 << Q15) - 1)

#define Q16_MAX			((1 << Q16) - 1)
#define Q17_MAX			((1 << Q17) - 1)
#define Q18_MAX			((1 << Q18) - 1)
#define Q19_MAX			((1 << Q19) - 1)
#define Q20_MAX			((1 << Q20) - 1)
#define Q21_MAX			((1 << Q21) - 1)
#define Q22_MAX			((1 << Q22) - 1)
#define Q23_MAX			((1 << Q23) - 1)

#define Q24_MAX			((1 << Q24) - 1)
#define Q25_MAX			((1 << Q25) - 1)
#define Q26_MAX			((1 << Q26) - 1)
#define Q27_MAX			((1 << Q27) - 1)
#define Q28_MAX			((1 << Q28) - 1)
#define Q29_MAX			((1 << Q29) - 1)
#define Q30_MAX			((1 << Q30) - 1)
#define Q31_MAX			((1 << Q31) - 1)

#define Qxx_MAX			((1 << Qxx) - 1)

#else
#define Q8_MAX			((1 <<  Q8))
#define Q9_MAX			((1 <<  Q9))
#define Q10_MAX			((1 << Q10))
#define Q11_MAX			((1 << Q11))
#define Q12_MAX			((1 << Q12))
#define Q13_MAX			((1 << Q13))
#define Q14_MAX			((1 << Q14))
#define Q15_MAX			((1 << Q15))

#define Q16_MAX			((1 << Q16))
#define Q17_MAX			((1 << Q17))
#define Q18_MAX			((1 << Q18))
#define Q19_MAX			((1 << Q19))
#define Q20_MAX			((1 << Q20))
#define Q21_MAX			((1 << Q21))
#define Q22_MAX			((1 << Q22))
#define Q23_MAX			((1 << Q23))

#define Q24_MAX			((1 << Q24))
#define Q25_MAX			((1 << Q25))
#define Q26_MAX			((1 << Q26))
#define Q27_MAX			((1 << Q27))
#define Q28_MAX			((1 << Q28))
#define Q29_MAX			((1 << Q29))
#define Q30_MAX			((1 << Q30))
#define Q31_MAX			((1 << Q31))

#define Qxx_MAX			((1 << Qxx))
#endif

#define Q8_MIN			(- Q8_MAX)
#define Q9_MIN			(- Q9_MAX)
#define Q10_MIN			(-Q10_MAX)
#define Q11_MIN			(-Q11_MAX)
#define Q12_MIN			(-Q12_MAX)
#define Q13_MIN			(-Q13_MAX)
#define Q14_MIN			(-Q14_MAX)
#define Q15_MIN			(-Q15_MAX)

#define Q16_MIN			(-Q16_MAX)
#define Q17_MIN			(-Q17_MAX)
#define Q18_MIN			(-Q18_MAX)
#define Q19_MIN			(-Q19_MAX)
#define Q20_MIN			(-Q20_MAX)
#define Q21_MIN			(-Q21_MAX)
#define Q22_MIN			(-Q22_MAX)
#define Q23_MIN			(-Q23_MAX)

#define Q24_MIN			(-Q24_MAX)
#define Q25_MIN			(-Q25_MAX)
#define Q26_MIN			(-Q26_MAX)
#define Q27_MIN			(-Q27_MAX)
#define Q28_MIN			(-Q28_MAX)
#define Q29_MIN			(-Q29_MAX)
#define Q30_MIN			(-Q30_MAX)
#define Q31_MIN			(-Q31_MAX)

#define Qxx_MIN			(-Qxx_MAX)

#define IFX_Q(x)							((q_t)((x)*Qxx_MAX))
#define IFX_Q_mul(x, y)						((q_t)(((x)*(y)) >> Qxx))
#define IFX_Q_mul_n(x, y, n)				((q_t)(((x)*(y)) >> n))
#define IFX_Q_div(x, y)						((((q_t)x) << Q15) / y)
#define IFX_Q_div_n(x, y, n)				((((q_t)x) << n) / y)
#define IFX_Q_mul_xyz(x, y, z)				((q_t)(((x+y)*(z)) >> Qxx))
#define IFX_Qxx_mul_n(x, y, n)				((q_t)(((x)*(y)) >> n))
#define IFX_Q_xy_sqr(x, y)					((x*x + y*y) >> Qxx)
#define IFX_Q_mul_ax_plus_by(a, x, b, y)	((a*x + b*y) >> Qxx)
#define IFX_Q_mul_ax_minus_by(a, x, b, y)	((a*x - b*y) >> Qxx)
#define IFX_Q_mul32(x, y) 					(((long long)x*y) >> Qxx)

#define IFX_Q8(x)		((q_t)((x)*Q8_MAX))
#define IFX_Q9(x)		((q_t)((x)*Q9_MAX))
#define IFX_Q10(x)		((q_t)((x)*Q10_MAX))
#define IFX_Q11(x)		((q_t)((x)*Q11_MAX))
#define IFX_Q12(x)		((q_t)((x)*Q12_MAX))
#define IFX_Q13(x)		((q_t)((x)*Q13_MAX))
#define IFX_Q14(x)		((q_t)((x)*Q14_MAX))
#define IFX_Q15(x)		((q_t)((x)*Q15_MAX))
#define IFX_Q16(x)		((q_t)((x)*Q16_MAX))
#define IFX_Q17(x)		((q_t)((x)*Q17_MAX))
#define IFX_Q18(x)		((q_t)((x)*Q18_MAX))
#define IFX_Q19(x)		((q_t)((x)*Q19_MAX))
#define IFX_Q20(x)		((q_t)((x)*Q20_MAX))
#define IFX_Q21(x)		((q_t)((x)*Q21_MAX))
#define IFX_Q22(x)		((q_t)((x)*Q22_MAX))
#define IFX_Q23(x)		((q_t)((x)*Q23_MAX))
#define IFX_Q24(x)		((q_t)((x)*Q24_MAX))
#define IFX_Q25(x)		((q_t)((x)*Q25_MAX))
#define IFX_Q26(x)		((q_t)((x)*Q26_MAX))
#define IFX_Q27(x)		((q_t)((x)*Q27_MAX))
#define IFX_Q28(x)		((q_t)((x)*Q28_MAX))
#define IFX_Q29(x)		((q_t)((x)*Q29_MAX))
#define IFX_Q30(x)		((q_t)((x)*Q30_MAX))
#define IFX_Q31(x)		((q_t)((x)*Q31_MAX))

#define IFX_Qa_To_Qb(x, a, b)	((q_t)(x >> (a - b))	// Must be 'a >= b'
#define IFX_Qb_To_Qa(x, b, a)	((q_t)(x << (a - b))	// Must be 'a >= b'

#if (Qxx >= Q8)
  #define IFX_Qxx_To_Q8(x)		((q_t)((x) >> (Qxx - Q8)))
#else
  #define IFX_Qxx_To_Q8(x)		((q_t)((x) << (Q8 - Qxx)))
#endif

#if (Qxx >= Q9)
  #define IFX_Qxx_To_Q9(x)		((q_t)((x) >> (Qxx - Q9)))
#else
  #define IFX_Qxx_To_Q9(x)		((q_t)((x) << (Q9 - Qxx)))
#endif

#if (Qxx >= Q10)
  #define IFX_Qxx_To_Q10(x)		((q_t)((x) >> (Qxx - Q10)))
#else
  #define IFX_Qxx_To_Q10(x)		((q_t)((x) << (Q10 - Qxx)))
#endif

#if (Qxx >= Q11)
  #define IFX_Qxx_To_Q11(x)		((q_t)((x) >> (Qxx - Q11)))
#else
  #define IFX_Qxx_To_Q11(x)		((q_t)((x) << (Q11 - Qxx)))
#endif

#if (Qxx >= Q12)
  #define IFX_Qxx_To_Q12(x)		((q_t)((x) >> (Qxx - Q12)))
#else
  #define IFX_Qxx_To_Q12(x)		((q_t)((x) << (Q12 - Qxx)))
#endif

#if (Qxx >= Q13)
  #define IFX_Qxx_To_Q13(x)		((q_t)((x) >> (Qxx - Q13)))
#else
  #define IFX_Qxx_To_Q13(x)		((q_t)((x) << (Q13 - Qxx)))
#endif

#if (Qxx >= Q14)
  #define IFX_Qxx_To_Q14(x)		((q_t)((x) >> (Qxx - Q14)))
#else
  #define IFX_Qxx_To_Q14(x)		((q_t)((x) << (Q14 - Qxx)))
#endif

#if (Qxx >= Q15)
  #define IFX_Qxx_To_Q15(x)		((q_t)((x) >> (Qxx - Q15)))
#else
  #define IFX_Qxx_To_Q15(x)		((q_t)((x) << (Q15 - Qxx)))
#endif

#if (Qxx >= Q16)
  #define IFX_Qxx_To_Q16(x)		((q_t)((x) >> (Qxx - Q16)))
#else
  #define IFX_Qxx_To_Q16(x)		((q_t)((x) << (Q16 - Qxx)))
#endif

#if (Qxx >= Q17)
  #define IFX_Qxx_To_Q17(x)		((q_t)((x) >> (Qxx - Q17)))
#else
  #define IFX_Qxx_To_Q17(x)		((q_t)((x) << (Q17 - Qxx)))
#endif

#if (Qxx >= Q18)
  #define IFX_Qxx_To_Q18(x)		((q_t)((x) >> (Qxx - Q18)))
#else
  #define IFX_Qxx_To_Q18(x)		((q_t)((x) << (Q18 - Qxx)))
#endif

#if (Qxx >= Q19)
  #define IFX_Qxx_To_Q19(x)		((q_t)((x) >> (Qxx - Q19)))
#else
  #define IFX_Qxx_To_Q19(x)		((q_t)((x) << (Q19 - Qxx)))
#endif

#if (Qxx >= Q20)
  #define IFX_Qxx_To_Q20(x)		((q_t)((x) >> (Qxx - Q20)))
#else
  #define IFX_Qxx_To_Q20(x)		((q_t)((x) << (Q20 - Qxx)))
#endif

#if (Qxx >= Q21)
  #define IFX_Qxx_To_Q21(x)		((q_t)((x) >> (Qxx - Q21)))
#else
  #define IFX_Qxx_To_Q21(x)		((q_t)((x) << (Q21 - Qxx)))
#endif

#if (Qxx >= Q22)
  #define IFX_Qxx_To_Q22(x)		((q_t)((x) >> (Qxx - Q22)))
#else
  #define IFX_Qxx_To_Q22(x)		((q_t)((x) << (Q22 - Qxx)))
#endif

#if (Qxx >= Q23)
  #define IFX_Qxx_To_Q23(x)		((q_t)((x) >> (Qxx - Q23)))
#else
  #define IFX_Qxx_To_Q23(x)		((q_t)((x) << (Q23 - Qxx)))
#endif

#if (Qxx >= Q24)
  #define IFX_Qxx_To_Q24(x)		((q_t)((x) >> (Qxx - Q24)))
#else
  #define IFX_Qxx_To_Q24(x)		((q_t)((x) << (Q24 - Qxx)))
#endif

#if (Qxx >= Q25)
  #define IFX_Qxx_To_Q25(x)		((q_t)((x) >> (Qxx - Q25)))
#else
  #define IFX_Qxx_To_Q25(x)		((q_t)((x) << (Q25 - Qxx)))
#endif

#if (Qxx >= Q26)
  #define IFX_Qxx_To_Q26(x)		((q_t)((x) >> (Qxx - Q26)))
#else
  #define IFX_Qxx_To_Q26(x)		((q_t)((x) << (Q26 - Qxx)))
#endif

#if (Qxx >= Q27)
  #define IFX_Qxx_To_Q27(x)		((q_t)((x) >> (Qxx - Q27)))
#else
  #define IFX_Qxx_To_Q27(x)		((q_t)((x) << (Q27 - Qxx)))
#endif

#if (Qxx >= Q28)
  #define IFX_Qxx_To_Q28(x)		((q_t)((x) >> (Qxx - Q28)))
#else
  #define IFX_Qxx_To_Q28(x)		((q_t)((x) << (Q28 - Qxx)))
#endif

#if (Qxx >= Q29)
  #define IFX_Qxx_To_Q29(x)		((q_t)((x) >> (Qxx - Q29)))
#else
  #define IFX_Qxx_To_Q29(x)		((q_t)((x) << (Q29 - Qxx)))
#endif

#if (Qxx >= Q30)
  #define IFX_Qxx_To_Q30(x)		((q_t)((x) >> (Qxx - Q30)))
#else
  #define IFX_Qxx_To_Q30(x)		((q_t)((x) << (Q30 - Qxx)))
#endif

#if (Qxx >= Q31)
  #define IFX_Qxx_To_Q31(x)		((q_t)((x) >> (Qxx - Q31)))
#else
  #define IFX_Qxx_To_Q31(x)		((q_t)((x) << (Q31 - Qxx)))
#endif

#if (Qxx >= Q8)
  #define IFX_Q8_To_Qxx(x)		((q_t)((x) << (Qxx - Q8)))
#else
  #define IFX_Q8_To_Qxx(x)		((q_t)((x) >> (Q8 - Qxx)))
#endif

#if (Qxx >= Q9)
  #define IFX_Q9_To_Qxx(x)		((q_t)((x) << (Qxx - Q9)))
#else
  #define IFX_Q9_To_Qxx(x)		((q_t)((x) >> (Q9 - Qxx)))
#endif

#if (Qxx >= Q10)
  #define IFX_Q10_To_Qxx(x)		((q_t)((x) << (Qxx - Q10)))
#else
  #define IFX_Q10_To_Qxx(x)		((q_t)((x) >> (Q10 - Qxx)))
#endif

#if (Qxx >= Q11)
  #define IFX_Q11_To_Qxx(x)		((q_t)((x) << (Qxx - Q11)))
#else
  #define IFX_Q11_To_Qxx(x)		((q_t)((x) >> (Q11 - Qxx)))
#endif

#if (Qxx >= Q12)
  #define IFX_Q12_To_Qxx(x)		((q_t)((x) << (Qxx - Q12)))
#else
  #define IFX_Q12_To_Qxx(x)		((q_t)((x) >> (Q12 - Qxx)))
#endif

#if (Qxx >= Q13)
  #define IFX_Q13_To_Qxx(x)		((q_t)((x) << (Qxx - Q13)))
#else
  #define IFX_Q13_To_Qxx(x)		((q_t)((x) >> (Q13 - Qxx)))
#endif

#if (Qxx >= Q14)
  #define IFX_Q14_To_Qxx(x)		((q_t)((x) << (Qxx - Q14)))
#else
  #define IFX_Q14_To_Qxx(x)		((q_t)((x) >> (Q14 - Qxx)))
#endif

#if (Qxx >= Q15)
  #define IFX_Q15_To_Qxx(x)		((q_t)((x) << (Qxx - Q15)))
#else
  #define IFX_Q15_To_Qxx(x)		((q_t)((x) >> (Q15 - Qxx)))
#endif

#if (Qxx >= Q16)
  #define IFX_Q16_To_Qxx(x)		((q_t)((x) << (Qxx - Q16)))
#else
  #define IFX_Q16_To_Qxx(x)		((q_t)((x) >> (Q16 - Qxx)))
#endif

#if (Qxx >= Q17)
  #define IFX_Q17_To_Qxx(x)		((q_t)((x) << (Qxx - Q17)))
#else
  #define IFX_Q17_To_Qxx(x)		((q_t)((x) >> (Q17 - Qxx)))
#endif

#if (Qxx >= Q18)
  #define IFX_Q18_To_Qxx(x)		((q_t)((x) << (Qxx - Q18)))
#else
  #define IFX_Q18_To_Qxx(x)		((q_t)((x) >> (Q18 - Qxx)))
#endif

#if (Qxx >= Q19)
  #define IFX_Q19_To_Qxx(x)		((q_t)((x) << (Qxx - Q19)))
#else
  #define IFX_Q19_To_Qxx(x)		((q_t)((x) >> (Q19 - Qxx)))
#endif

#if (Qxx >= Q20)
  #define IFX_Q20_To_Qxx(x)		((q_t)((x) << (Qxx - Q20)))
#else
  #define IFX_Q20_To_Qxx(x)		((q_t)((x) >> (Q20 - Qxx)))
#endif

#if (Qxx >= Q21)
  #define IFX_Q21_To_Qxx(x)		((q_t)((x) << (Qxx - Q21)))
#else
  #define IFX_Q21_To_Qxx(x)		((q_t)((x) >> (Q21 - Qxx)))
#endif

#if (Qxx >= Q22)
  #define IFX_Q22_To_Qxx(x)		((q_t)((x) << (Qxx - Q22)))
#else
  #define IFX_Q22_To_Qxx(x)		((q_t)((x) >> (Q22 - Qxx)))
#endif

#if (Qxx >= Q23)
  #define IFX_Q23_To_Qxx(x)		((q_t)((x) << (Qxx - Q23)))
#else
  #define IFX_Q23_To_Qxx(x)		((q_t)((x) >> (Q23 - Qxx)))
#endif

#if (Qxx >= Q24)
  #define IFX_Q24_To_Qxx(x)		((q_t)((x) << (Qxx - Q24)))
#else
  #define IFX_Q24_To_Qxx(x)		((q_t)((x) >> (Q24 - Qxx)))
#endif

#if (Qxx >= Q25)
  #define IFX_Q25_To_Qxx(x)		((q_t)((x) << (Qxx - Q25)))
#else
  #define IFX_Q25_To_Qxx(x)		((q_t)((x) >> (Q25 - Qxx)))
#endif

#if (Qxx >= Q26)
  #define IFX_Q26_To_Qxx(x)		((q_t)((x) << (Qxx - Q26)))
#else
  #define IFX_Q26_To_Qxx(x)		((q_t)((x) >> (Q26 - Qxx)))
#endif

#if (Qxx >= Q27)
  #define IFX_Q27_To_Qxx(x)		((q_t)((x) << (Qxx - Q27)))
#else
  #define IFX_Q27_To_Qxx(x)		((q_t)((x) >> (Q27 - Qxx)))
#endif

#if (Qxx >= Q28)
  #define IFX_Q28_To_Qxx(x)		((q_t)((x) << (Qxx - Q28)))
#else
  #define IFX_Q28_To_Qxx(x)		((q_t)((x) >> (Q28 - Qxx)))
#endif

#if (Qxx >= Q29)
  #define IFX_Q29_To_Qxx(x)		((q_t)((x) << (Qxx - Q29)))
#else
  #define IFX_Q29_To_Qxx(x)		((q_t)((x) >> (Q29 - Qxx)))
#endif

#if (Qxx >= Q30)
  #define IFX_Q30_To_Qxx(x)		((q_t)((x) << (Qxx - Q30)))
#else
  #define IFX_Q30_To_Qxx(x)		((q_t)((x) >> (Q30 - Qxx)))
#endif

#if (Qxx >= Q31)
  #define IFX_Q31_To_Qxx(x)		((q_t)((x) << (Qxx - Q31)))
#else
  #define IFX_Q31_To_Qxx(x)		((q_t)((x) >> (Q31 - Qxx)))
#endif

typedef struct {
	unsigned long int q;
	unsigned long int r;
} udiv32_t;

typedef struct {
	signed long int q;
	signed long int r;
} div32_t;

typedef struct
{
	q_t a;
	q_t b;
	q_t c;
} IFX_Q_ABC_t;

typedef struct
{
	q_t alpha;
	q_t beta;
} IFX_Q_ALPHA_BETA_t;

typedef struct
{
	q_t d;
	q_t q;
} IFX_Q_DQ_t;

typedef struct
{
	int16_t angle;
	q_t radius;
} CORDIC_ATAN2SQRT_TYPE;


// mod by Christian
typedef struct {
	int16_t Iu;
	int16_t Iv;
	int16_t Iw;
	int16_t Iu_ofst;
	int16_t Iv_ofst;
	int16_t Iw_ofst;
} ADC_RAW_t;



#define PI					(3.1415926536f)
#ifndef TWO_PI
#define TWO_PI				(2.0f * PI)
#endif
#define TWO_PI_OVER_3		(TWO_PI / 3.0f)
#define FOUR_PI_OVER_3		(TWO_PI*2.0 / 3.0f)
#define PI_OVER_3			(PI / 3.0f)


#endif /* PMSM_FOC_FOCLIB_DATATYPES_H_ */
