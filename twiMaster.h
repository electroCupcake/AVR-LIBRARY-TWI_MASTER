/*
 * twiMaster.h
 *
 *  Created on: Aug 12, 2015
 *      Author: jconvertino
 
    Copyright (C) 2015 John Convertino

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef TWIMASTER_H_
#define TWIMASTER_H_

#include <stdlib.h>
#include <util/twi.h>
#include <inttypes.h>

//speed for twi SCL line
#define LOW_SCL 50000UL
#define MED_SCL 100000UL
#define HIGH_SCL 400000UL

//exit status
#define EXIT_DATA_LRG	-6
#define EXIT_BUF_EMP 	-5
#define EXIT_BUF_FUL 	-4
#define EXIT_INV_DATA  	-3
#define EXIT_BUSY    	-2
#define EXIT_TWI_ERR 	-1
#define EXIT_SUCCESS  	 0

//response status
#define TWI_ACK    2
#define TWI_NACK   1
#define TWI_STOP   0
#define TWI_READY  -1

//buffer Size
#define MAX_BUFFER_SIZE 256

typedef enum {disabled, enabled} flag;

//handler function pointer typedefs
typedef int (*consumer)(volatile uint8_t *);
typedef int (*producer)(volatile uint8_t *);

//setup twi at a specified scl speed, allows handles to be use fnptr_consumer for receiving data from twi,
//and fnptr_producer for sending data on the twi. Use NULL for no handlers.
void twiInit(uint32_t scl_speed, consumer fnptr_consumer, producer fnptr_producer);

//enable twi pullups
void twiPullups();

//create a buffer with data, and send that buffer to the address.
int twiSend(uint8_t address, uint8_t *data, size_t size);

//create buffer, send address, copy received data to buffer, then copy buffer data to buffer.
int twiRecv(uint8_t address, uint8_t *data, size_t size);

//begin transmission using handler
void twiBeginHandlerTrans(uint8_t address, uint8_t RW);

//end a transmission using handler
void twiStopHandlerTrans();

//check if twi is busy (if interrupt enable is on, twi busy)
uint8_t twiBusyCk();

//get last status when ISR was jumped to
uint8_t twiGetLastStatus();

//get last error ISR branched to.
uint8_t twiGetLastError();




#endif /* TWIMASTER_H_ */
