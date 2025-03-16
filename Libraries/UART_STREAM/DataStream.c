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
 * DataStream.c
 *
 *  Created on: 18 Jan 2024
 *      Author: wattenberg
 */

//#include "DAVE.h"
#include <cybsp.h>
#include "DataStream.h"

dataStream_t dataStream;
volatile bool dataIsBeingPrinted;

static inline void DataStream_ClearBufferPos()
{
	uint8_t i;
	for(i = 0; i<DATASTREAM_NUMBER_OF_CHANNELS; i++)
	{
		dataStream.pos[i] = 0;
	}
}

void ISR_EndOfTransmission()
{
	uint8_t i;
	for(i = 0; i<DATASTREAM_NUMBER_OF_CHANNELS; i++)
	{
		dataStream.pos[i] = 0;
	}

	dataIsBeingPrinted = false;
}

void DataStream_AddData(uint8_t channel, DATASTREAM_TYPE data)
{
	if(dataStream.pos[channel] < DATASTREAM_NUMBER_OF_ELEMENTS)
	{
		uint8_t* ptr = dataStream.buffer;
		//ptr = ptr + sizeof(float) * (DATASTREAM_NUMBER_OF_CHANNELS * dataStream.pos[channel] + channel) + 1;
		ptr = ptr + sizeof(DATASTREAM_TYPE) * (DATASTREAM_NUMBER_OF_CHANNELS * dataStream.pos[channel] + channel);
		ptr = ptr + (dataStream.pos[channel] + 1);

//		*((float*)(ptr)) = data;
		memcpy(ptr,&data,sizeof(DATASTREAM_TYPE));
		dataStream.pos[channel]++;
	}
}

void DataStream_PrintData()
{
		if(DATASTREAM_BUFFER_SIZE > 4095)
			//We have to do several transmissions
			//This is a problem for the future
			;
		else
			for (uint16_t i = 0; i < DATASTREAM_BUFFER_SIZE; i++)
			{
				XMC_UART_CH_Transmit(UART0_HW, (uint16_t)dataStream.buffer[i]);
			}
			//UART_Transmit(&UART_0, (uint8_t*)dataStream.buffer, DATASTREAM_BUFFER_SIZE);

		DataStream_ClearBufferPos();
}

bool DataStream_BufferIsFull()
{
	if(dataStream.pos[0] == DATASTREAM_NUMBER_OF_ELEMENTS)
		return true;
	else
		return false;
}



void DataStream_Init()
{

	XMC_UART_CH_SetBaudrate(UART0_HW, DATASTREAM_BAUDRATE, 4);
	//UART_SetBaudrate(&UART_0, DATASTREAM_BAUDRATE, 4);

	DataStream_ClearBufferPos();
	uint16_t i;

	uint8_t* ptr = dataStream.buffer;
	for(i = 0; i < DATASTREAM_NUMBER_OF_CHANNELS * DATASTREAM_NUMBER_OF_ELEMENTS; i++)
	{
		*ptr = DATASTREAM_FRAME_START;
		ptr = ptr + (1 + DATASTREAM_NUMBER_OF_CHANNELS * sizeof(DATASTREAM_TYPE));
	}
}
