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
 * CAN.h
 *
 *  Created on: 20 June 2024
 *      Author: Schmidt Michael (IFAT PSS AIS SAE)
 *  Last change: 2025-01-20, S. Detzel
*/

#include "IMR_CAN.h"
#include "IMR_CAN_GLOBAL.h"
#include "user_defines.h"
#include "DataTypes.h"

bool CAN_TimeOut = false;
int16_t CAN_speed_ref = 0;
uint16_t MechanicalAngle = 0;
int16_t EncoderSpeed = 0;

uint8_t can_calibration_flag = 0;

/*****************************************************************************/

void IRQ_NUMBER_CAN_TX_TIMER_HANDLER(void) {
	// Read and Set CAN Identifier from DIP Switch
	uint32_t GPIO_OFFSET = 0;
	GPIO_OFFSET |= (uint8_t)(!XMC_GPIO_GetInput(CAN_ID1_PORT, CAN_ID1_PIN) << 0);
	GPIO_OFFSET |= (uint8_t)(!XMC_GPIO_GetInput(CAN_ID2_PORT, CAN_ID2_PIN) << 1);
	uint32_t CAN_MSG_ID = MOT_FL_ENCODER_DATA | GPIO_OFFSET;
	CAN_TX_Request(CAN_MSG_ID,
			(uint8_t[]) {(EncoderSpeed >> 8 & 0xFF),
						 (EncoderSpeed & 0xFF),
						 (MechanicalAngle >> 8 & 0xFF),
						 (MechanicalAngle & 0xFF)}, 4);
}

/*****************************************************************************/

void IRQ_NUMBER_CAN_TIMEOUT_TIMER_HANDLER(void) {
	// Check if TimeOut is still True after last Check
	if (CAN_TimeOut == true)
		CAN_speed_ref = 0;

	CAN_TimeOut = false;
}

/*****************************************************************************/

void CAN_IRQ_RX_CALIBRATION_MESSAGE_HANDLER(void) {
	// Receive motor calibration request
	XMC_CAN_MO_Receive(&CAN_NODE_CALIBRATION_LMO_NAME);
	can_calibration_flag = true;
}

/*****************************************************************************/

void CAN_IRQ_RX_MESSAGE_HANDLER(void) {
	CAN_TimeOut = false;

	// Receive data from CAN Node and transfer into CAN data structure
	// intended for setting the motor speed
	XMC_CAN_MO_Receive(&CAN_NODE_RECEIVE_LMO_NAME);

	uint32_t id = XMC_CAN_MO_GetIdentifier(&CAN_NODE_RECEIVE_LMO_NAME);
    uint8_t *data = CAN_NODE_RECEIVE_LMO_NAME.can_data_byte;

    // Read and Set CAN Identifier from DIP Switch
	uint32_t GPIO_OFFSET = 0;
	GPIO_OFFSET |= (uint8_t)(!XMC_GPIO_GetInput(CAN_ID1_PORT, CAN_ID1_PIN) << 0);
	GPIO_OFFSET |= (uint8_t)(!XMC_GPIO_GetInput(CAN_ID2_PORT, CAN_ID2_PIN) << 1);

	// conversion from 16 Bit signed integer to rad/s, then to RPM.
	if (id == (MOT_FL_SPEED_COMMAND | GPIO_OFFSET)) {
		CAN_speed_ref = (int16_t)((data[0] << 8) + data[1]) /
						RADPS2_15BIT * 60.0 / (2 * PI);
	}
	// Toggle CAN RX LED to indicate that a message has been received
	#if (CAN_NODE_RECEIVE_LED_ENABLE)
		XMC_GPIO_ToggleOutput(CAN_RX_LED_PIN_PORT_NAME, CAN_RX_LED_PIN_PIN_NAME);
	#endif
}

/*****************************************************************************/

void CAN_Initialize(void) {
	// Read and Set CAN Identifier from DIP Switch
	uint32_t GPIO_OFFSET = 0;
	GPIO_OFFSET |= (uint8_t)(!XMC_GPIO_GetInput(CAN_ID1_PORT, CAN_ID1_PIN) << 0);
	GPIO_OFFSET |= (uint8_t)(!XMC_GPIO_GetInput(CAN_ID2_PORT, CAN_ID2_PIN) << 1);

	XMC_CAN_MO_SetIdentifier(&CAN_NODE_RECEIVE_LMO_NAME,
			(uint32_t)(MOT_FL_SPEED_COMMAND | GPIO_OFFSET));
	XMC_CAN_MO_SetIdentifier(&CAN_NODE_CALIBRATION_LMO_NAME,
			(uint32_t)CALIBRATION_REQUEST_ALL_MOTORS);

	// Enable NVIC IRQ with correct IRQ number - see Reference Manual
	NVIC_EnableIRQ(IRQ_NUMBER_CAN_TIMEOUT_TIMER);
	NVIC_EnableIRQ(IRQ_NUMBER_CAN_TX_TIMER);
	NVIC_EnableIRQ(CAN_IRQ_RX_NUMBER);
	NVIC_EnableIRQ(CAN_IRQ_RX_CALIBRATION_NUMBER);

	/* Change Interrupt event source of channel 24 (TX Timer) &
	 * 16 (TimeOut Timer) & channel 3 (CAN) ...
	 * see XMC1400 Reference Manual Table 5-1 */
	WR_REG(SCU_GENERAL->INTCR0, SCU_GENERAL_INTCR0_INTSEL3_Msk,
			SCU_GENERAL_INTCR0_INTSEL3_Pos, 0x02);	// CAN RX
	WR_REG(SCU_GENERAL->INTCR0, SCU_GENERAL_INTCR0_INTSEL4_Msk,
			SCU_GENERAL_INTCR0_INTSEL4_Pos, 0x02);
	WR_REG(SCU_GENERAL->INTCR1, SCU_GENERAL_INTCR1_INTSEL16_Msk,
			SCU_GENERAL_INTCR1_INTSEL16_Pos, 0x02);	// CAN TimeOut Timer
	WR_REG(SCU_GENERAL->INTCR1, SCU_GENERAL_INTCR1_INTSEL21_Msk,
			SCU_GENERAL_INTCR1_INTSEL21_Pos, 0x01);	// CAN TX Timer

	XMC_CCU4_SLICE_StartTimer(CAN_NODE_RX_TIMEOUT_TIMER_NAME);
	XMC_CCU4_SLICE_StartTimer(CAN_NODE_TX_TIMER_NAME);
}
