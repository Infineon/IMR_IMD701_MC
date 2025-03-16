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
 * DataStream.h
 *
 *  Created on: 11 Apr 2024
 *      Author: wattenberg
 */

#ifndef DATASTREAM_H_
#define DATASTREAM_H_

#include <stdbool.h>

#define DATASTREAM_MODE_CONTINOUS

//How many different variables we want to stream
#define DATASTREAM_NUMBER_OF_CHANNELS 3//Define how many elements we want to hold in the buffer and stream at once
//Streaming large chunks of data a once is a bit more efficient than streaming element by element
#define DATASTREAM_TYPE signed long int
#if defined(DATASTREAM_MODE_CONTINOUS)
	#define DATASTREAM_NUMBER_OF_ELEMENTS 1
#elif defined(DATASTREAM_MODE_BULK)
	#define DATASTREAM_NUMBER_OF_ELEMENTS 100
#endif
//Code word for SerialPlot to look for
#define DATASTREAM_FRAME_START 0xAA

#define DATASTREAM_BUFFER_SIZE (DATASTREAM_NUMBER_OF_CHANNELS * DATASTREAM_NUMBER_OF_ELEMENTS *sizeof(DATASTREAM_TYPE) + DATASTREAM_NUMBER_OF_ELEMENTS)
//#define DATASTREAM_BAUDRATE 3000000UL
#define DATASTREAM_BAUDRATE 115200UL

typedef struct dataSteam_t
{
	uint16_t pos[DATASTREAM_NUMBER_OF_CHANNELS];
	uint8_t buffer[DATASTREAM_BUFFER_SIZE]; //reserve just some random memory on the heap
} dataStream_t;


void DataStream_AddData(uint8_t channel, DATASTREAM_TYPE data);
void DataStream_PrintData();
void DataStream_Init();
bool DataStream_BufferIsFull();


#endif /* DATASTREAM_H_ */
