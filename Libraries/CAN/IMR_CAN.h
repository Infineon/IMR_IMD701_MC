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

#ifndef LIBRARIES_IMR_CAN_H_
#define LIBRARIES_IMR_CAN_H_

#include "IMR_CAN_GLOBAL.h"
#include "cybsp.h"
#include "cy_utils.h"

#define CAT(x, y) CAT_(x, y)
#define CAT_(x, y) x ## y
#define CAN_TX_TIMEOUT	0x400	// Timeout counter value ... 1024

extern bool CAN_TimeOut;
extern int16_t CAN_speed_ref;
extern uint16_t MechanicalAngle;
extern int16_t EncoderSpeed;
extern uint8_t can_calibration_flag;

void CAN_Initialize(void);
uint32_t XMC_CAN_MO_Busy(XMC_CAN_MO_t *mo_ptr);

void CAN_IRQ_RX_MESSAGE_HANDLER(void);
void CAN_IRQ_RX_CALIBRATION_MESSAGE_HANDLER(void);

// Select the CAN_Node name chosen in the MTB Device Configurator;
#define CAN_NODE_CONFIGURATOR_NAME				CAN_NODE
// Select the CAN_Node number chosen in the MTB Device Configurator;
// CAN Node 0 ... CAN_NODE0		CAN Node 1 ... CAN_NODE1
#define CAN_NODE_CONFIGURATOR_CHANNEL			CAN_NODE1

/* CAN Interrupt Number Setting
 * XMC1404 = IRQ3_IRQn; 		XMC4700 = CAN0_0_IRQn */
#define CAN_IRQ_RX_NUMBER               		IRQ3_IRQn
/* CAN Interrupt Handler Setting
 * XMC1404 = IRQ3_Handler;	XMC4700 = IRQ_Hdlr_76 */
#define CAN_IRQ_RX_MESSAGE_HANDLER         		IRQ3_Handler
#define CAN_IRQ_RX_CALIBRATION_NUMBER			IRQ4_IRQn
#define CAN_IRQ_RX_CALIBRATION_MESSAGE_HANDLER	IRQ4_Handler

#define CAN_STATUS_NODE_BUSY		( 2U )

#define CAN_NODE_GLOBAL_HW_NAME			CAT(CAN_NODE_CONFIGURATOR_NAME, _HW)
#define CAN_NODE_RECEIVE_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_0)
#define CAN_NODE_TRANSMIT_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_1)
#define CAN_NODE_CALIBRATION_LMO_NAME	CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_2)

/************************************************************************/
/******************* INVERTER BOARD - IMPORTANT NOTICE ******************/
/************************************************************************/

/* This library is designed to work with the IMR CAN identification in mind;
 * boards are identified using DIP switches
 * according to the selected board type
 *
 * Make sure to deactivate the option to "Store Config in Flash"
 *
 * Activate both Capture Compare Units CCU4_0 and CCU 4_1
 * 	Activate Slice 0 for CCU4_0
 * 		Set the desired Timer Frequency
 * 		(Prescaler Initial Value - 2048 and Timer Period Value - 5860 for 4Hz)
 * 		Activate "Period Match while Counting Up" and
 * 		set "Period Match Up Service Request" to Service Request 2
 * 		Deactivate "Start After Initialization"
 *
 * 	Activate Slice 0 for CCU4_1
 * 		Set the desired Timer Frequency
 * 		(Prescaler Initial Value - 64 and Timer Period Value - 15000 for 100Hz)
 * 		Activate "Period Match while Counting Up" and
 * 		set "Period Match Up Service Request" to Service Request 0
 * 		Deactivate "Start After Initialization"
 */

/************************************************************************/
/****************** NOT CHANGE SETTINGS ABOVE THIS LINE *****************/
/************************************************************************/

// Select the CAN_Node RX timeout timer name
// chosen in the MTB Device Configurator (CCU4);
#define CAN_NODE_RX_TIMEOUT_CCU4_NAME	CAN_TIMEOUT
// Select the CAN_Node TX timer name
// chosen in the MTB Device Configurator (CCU4);
#define CAN_NODE_TX_TIMER_CCU4_NAME		CAN_TX_TIMER

// Select if the board is supposed to receive CAN messages;
// 0U ... CAN Receive NOT used; 1U ... CAN Receive used;
#define CAN_NODE_RECEIVE_ENABLE			(1U)
// Select if the board is supposed to transmit CAN messages;
// 0U ... CAN Transmit NOT used; 1U ... CAN Transmit used;
#define CAN_NODE_TRANSMIT_ENABLE		(1U)
// Select if a specific LED should indicate the receive of a CAN message;
// 0U ... CAN RX LED NOT used; 1U ... CAN RX LED used;
#define CAN_NODE_RECEIVE_LED_ENABLE		(1U)

#if (CAN_NODE_RECEIVE_LED_ENABLE)
// Select the CAN RX LED Pin name chosen in the MTB Device Configurator;
#define CAN_RX_LED_PIN_CONFIGURATOR_NAME	LED_RED
#endif

/************************************************************************/
/************** DO NOT CHANGE SETTINGS BELOW THIS LINE ******************/
/************************************************************************/

#define CAN_NODE_RX_TIMEOUT_TIMER_NAME	CAT(CAN_NODE_RX_TIMEOUT_CCU4_NAME, _HW)
#define CAN_NODE_TX_TIMER_NAME			CAT(CAN_NODE_TX_TIMER_CCU4_NAME, _HW)

#if (CAN_NODE_RECEIVE_LED_ENABLE)
#define CAN_RX_LED_PIN_PORT_NAME	CAT(CAN_RX_LED_PIN_CONFIGURATOR_NAME, _PORT)
#define CAN_RX_LED_PIN_PIN_NAME		CAT(CAN_RX_LED_PIN_CONFIGURATOR_NAME, _PIN)
#endif

/* IRQ Event Source Names for XMC1404
 * see XMC1400 Reference Manual Table 5-1 */
/* Defines IRQ number of the period match event interrupt */
#define IRQ_NUMBER_CAN_TIMEOUT_TIMER			IRQ16_IRQn
/* Defines handler of the period match event interrupt */
#define IRQ_NUMBER_CAN_TIMEOUT_TIMER_HANDLER	IRQ16_Handler
/* Defines IRQ number of the period match event interrupt */
#define IRQ_NUMBER_CAN_TX_TIMER					IRQ21_IRQn
/* Defines handler of the period match event interrupt */
#define IRQ_NUMBER_CAN_TX_TIMER_HANDLER			IRQ21_Handler

#endif /* LIBRARIES_IMR_CAN_H_ */
