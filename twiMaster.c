/*
 * twiMaster.c
 *
 *  Created on: Aug 12, 2015
 *      Author: jconvertino
 * 
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#include "twiMaster.h"

//idea for state machine
//typedef enum {idle, start, rep_start, sla_r, sla_w, send, recv, stop, error} twiStates;

//struct for containing all data members for twi driver
struct
{
	uint8_t lastStatus;
	uint8_t lastError;
	uint8_t bufferSize;
	uint8_t bufferIndex;
	uint8_t bufferData[MAX_BUFFER_SIZE];
	//union to hold address, allow access rw directly
	union
	{
		uint8_t allBits;
		struct
		{
			uint8_t RW:1;
			uint8_t address:7;
		} bits;
	} sla;
	flag handlerEnable;
	//function pointers
	consumer fnptr_consumer;
	producer fnptr_producer;
} twi;

//functions for twi control register setup and clearing interrupt
//generate start condition
static inline void twiStart()
{
	TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
}

//generate stop condition
static inline void twiStop()
{
	TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

//clear interrupt and keep enabled
static inline void twiReady()
{
	TWCR =  (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
}

//send ack pulse
static inline void twiACK()
{
	TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
}

//send nack pulse
static inline void twiNACK()
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
}

//reset twi in case or error
static inline void twiRST()
{
	TWCR = (1 << TWINT) |(1 << TWEN) | (1 << TWIE) | (1 << TWEA);
}

//check return value and setup twi action based on value.
static inline void twiAction(int value)
{
	switch(value)
	{
	case TWI_READY:
		twiReady();
		break;
	case TWI_NACK:
		twiNACK();
		break;
	case TWI_STOP:
		twiStop();
		break;
	default:
		twiACK();
		break;
	}
}


//generic method for setting up address and settings read/write bit, also trigger start condition
void twiBeginTrans(uint8_t address, uint8_t RW)
{
	twi.sla.bits.address = address;

	twi.sla.bits.RW = RW;

	twiStart();
}

//init twi info, calculate scl_speed for TWBR, also enable interrupts
void twiInit(uint32_t scl_speed, consumer fnptr_consumer, producer fnptr_producer)
{
	TWSR = 0;
	TWCR = 0;
	TWBR = (F_CPU / scl_speed - 16) / 2;

	twi.lastStatus = 0;
	twi.lastError = 0;
	twi.bufferIndex = 0;
	twi.bufferSize = 0;
	twi.sla.allBits = 0;
	twi.handlerEnable = disabled;
	twi.fnptr_consumer = fnptr_consumer;
	twi.fnptr_producer = fnptr_producer;

	memset(twi.bufferData, 0, MAX_BUFFER_SIZE);

	sei();
}

void twiPullups()
{
	PORTC |= (1 << PORTC5) | (1 << PORTC4);
}

//write to buffer for twi data send
int twiSend(uint8_t address, uint8_t *data, size_t size)
{

	if(twiBusyCk())
	{
		return EXIT_BUSY;
	}

	if(data == NULL)
	{
		return EXIT_INV_DATA;
	}

	if(size > MAX_BUFFER_SIZE)
	{
		return EXIT_DATA_LRG;
	}

	//deep copy data
	memcpy(twi.bufferData, data, size);

	twi.bufferSize = size;

	twi.bufferIndex = 0;

	twiBeginTrans(address, TW_WRITE);

	return EXIT_SUCCESS;
}

//read from buffer after twi has sent address and built up data
int twiRecv(uint8_t address, uint8_t *data, size_t size)
{
	if(twiBusyCk())
	{
		return EXIT_BUSY;
	}

	if(data == NULL)
	{
		return EXIT_INV_DATA;
	}

	if(size > MAX_BUFFER_SIZE)
	{
		return EXIT_DATA_LRG;
	}

	twi.bufferSize = size;

	twiBeginTrans(address, TW_READ);

	//wait till read is done, as we have to wait to do the deep copy
	while(twiBusyCk());

	if(twi.bufferIndex != twi.bufferSize)
	{
		return EXIT_TWI_ERR;
	}

	//deep copy
	memcpy(data, twi.bufferData, size);

	return EXIT_SUCCESS;
}

//begin transmission using handler
void twiBeginHandlerTrans(uint8_t address, uint8_t RW)
{
	twi.handlerEnable = enabled;

	twiBeginTrans(address, RW);
}

//stop handler, allow ISR to end transmission correctly
void twiStopHandlerTrans()
{
	twi.handlerEnable = disabled;
}

//check if the the interrupt is enabled, if so, ISR in progress and were busy
uint8_t twiBusyCk()
{
	return (TWCR & (1 << TWIE));
}

//return last status from ISR vector jump
uint8_t twiGetLastStatus()
{
	return twi.lastStatus;
}

//return last error caught by ISR
uint8_t twiGetLastError()
{
	return twi.lastError;
}

//ISR will activate when TWINT is 1, must be reset manually
ISR(TWI_vect)
{
	twi.lastStatus = TW_STATUS;
	twi.lastError = 0;

	switch(TW_STATUS)
	{
		//start conditions, send address
		case TW_START:
		case TW_REP_START:
			TWDR = twi.sla.allBits;
			twi.bufferIndex = 0;
			twiReady();
			break;
		//slave+w or data acknowledged
		case TW_MT_SLA_ACK:
		case TW_MT_DATA_ACK:
			//handler enabled, and there is a valid handler use it.
			if((twi.fnptr_producer != NULL) && (twi.handlerEnable == enabled))
			{
				twiAction(twi.fnptr_producer(&TWDR));
			}
			//other wise use twiWrite routine if there is room and a valid buffer is available
			else if(twi.bufferIndex < twi.bufferSize)
			{
				TWDR = twi.bufferData[twi.bufferIndex];
				twi.bufferIndex++;
				twiReady();
			}
			//nothing else to do but stop
			else
			{
				twiStop();
			}
			break;
		//slave+r has been sent, send ack just sets up state on next receive to be DATA_ACK
		//this assumes we will always receive a byte, will need to be changed if there is a case where this is not true.
		case TW_MR_SLA_ACK:
			twiACK();
			break;
		//setting TWEA before the next received byte decides the ACK or NACK response, so NACK must be sent before the last byte.
		case TW_MR_DATA_ACK:
		case TW_MR_DATA_NACK:
			//valid handler and its enabled, use it
			if((twi.fnptr_consumer != NULL) && (twi.handlerEnable == enabled))
			{
				//decides on either a nack, ack, or stop
				twiAction(twi.fnptr_consumer(&TWDR));
			}
			//else use twiRecv, if buffer is allocated and there is room
			else if(twi.bufferIndex < twi.bufferSize)
			{
				twi.bufferData[twi.bufferIndex] = TWDR;
				twi.bufferIndex++;

				//devices based on bytes left for nack, ack or stop, 1 byte left NACK, 0 stop, anything greater than 1 ACK
				twiAction(twi.bufferSize - twi.bufferIndex);
			}
			//stop transmission if all other conditions fail and we have for some reason gotten to this state.
			else
			{
				twiStop();
			}
			break;
		//error states, store state, reset twi and then free buffer if needed
		case TW_MT_SLA_NACK:
		case TW_MT_ARB_LOST:
			twi.lastError = twi.lastStatus;
			twiStart();
			break;
		case TW_MT_DATA_NACK:
			twi.lastError = twi.lastStatus;
			twiStop();
			break;
		case TW_MR_SLA_NACK:
		default:
			twi.lastError = twi.lastStatus;
			twiRST();
			break;
	}
}
