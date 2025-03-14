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

#include "IMR_CAN.h"

uint32_t XMC_CAN_MO_Busy(XMC_CAN_MO_t* mo_ptr) {
	return (XMC_CAN_MO_GetStatus(mo_ptr) & CAN_MO_MOSTAT_TXRQ_Msk) >> CAN_MO_MOSTAT_TXRQ_Pos;
}

CAN_STATUS_t CAN_TX_Request(uint32_t CAN_ID, uint8_t* Target_Data, uint8_t Target_Data_Length) {
	uint32_t CAN_TimeOut = 0;			// Implement TimeOut feature to prevent CAN messages getting stuck on the CAN bus with no recipient - TimeOut can be defined in IMR2_CAN.h
	while (XMC_CAN_MO_Busy(&CAN_NODE_TRANSMIT_LMO_NAME)) {
		CAN_TimeOut++;

		if (CAN_TimeOut >= CAN_TX_TIMEOUT) {
			XMC_CAN_Enable(CAN_NODE_GLOBAL_HW_NAME);
			return XMC_CAN_STATUS_BUSY;
		}
	}

	XMC_CAN_MO_SetIdentifier(&CAN_NODE_TRANSMIT_LMO_NAME, CAN_ID);				// Set CAN ID of message recipient

	XMC_CAN_MO_SetDataLengthCode(&CAN_NODE_TRANSMIT_LMO_NAME, Target_Data_Length);		// Set Data Length Code (DLC) to amount of transmit bytes
	CAN_NODE_TRANSMIT_LMO_NAME.can_data_length = Target_Data_Length;
	for (uint16_t i = 0; i < Target_Data_Length; i++)								// Transfer transmit message bytes into CAN data structure
		CAN_NODE_TRANSMIT_LMO_NAME.can_data_byte[i] = Target_Data[i];

	CAN_STATUS_t status = CAN_SUCCESS;
	status = XMC_CAN_MO_UpdateData(&CAN_NODE_TRANSMIT_LMO_NAME);    		// Update CAN Node data with bytes from CAN data structure
	if (status != XMC_CAN_STATUS_SUCCESS) return status;					// Check for success on data update

	status = XMC_CAN_MO_Transmit(&CAN_NODE_TRANSMIT_LMO_NAME);    			// Send previously updated data using designed CAN node object
	if (status != XMC_CAN_STATUS_SUCCESS) return status;					// Check for success on data transmit

	return CAN_SUCCESS;
}

/***************************************************************************************************************/

