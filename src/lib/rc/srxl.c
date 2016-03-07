/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file srxl.h
 * RC protocol definition for Multiplex SRXL
 */

#include <stdbool.h>
#include <stdio.h>
#include "srxl.h"


enum SRXL_DECODE_STATE {
	SRXL_DECODE_STATE_UNSYNCED = 0,
	SRXL_DECODE_STATE_GOT_HEADER,
	SRXL_DECODE_STATE_GOT_DATA,
	SRXL_DECODE_STATE_GOT_CRC
};


/* define range mapping here:
 * 0x000..0xFFF -> 800..2200
 * -100%..+100% -> 950..2050 */
#define SRXL_RANGE_MIN 0.0f
#define SRXL_RANGE_MAX 4095.0f

#define SRXL_TARGET_MIN 800.0f
#define SRXL_TARGET_MAX 2200.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SRXL_SCALE_FACTOR ((SRXL_TARGET_MAX - SRXL_TARGET_MIN) / (SRXL_RANGE_MAX - SRXL_RANGE_MIN))
#define SRXL_SCALE_OFFSET (int)(SRXL_TARGET_MIN - (SRXL_SCALE_FACTOR * SRXL_RANGE_MIN + 0.5f))

static enum SRXL_DECODE_STATE _decode_state = SRXL_DECODE_STATE_UNSYNCED;
static uint8_t   _data_len = 0;
static uint8_t   _ver      = 0;
static uint16_t  _crc16    = 0x0000;
static bool      _crcOK    = false;
static bool      _debug    = false;

static ReceiverFcPacketSXRL _rxpacket;


uint16_t srxl_crc16(uint16_t crc, uint8_t value)
{
	uint8_t i;
	crc = crc ^ (int16_t)value<<8;
	for(i = 0; i < 8; i++) {
		if(crc & 0x8000)
			crc = crc << 1^0x1021;
		else
			crc = crc << 1;
	}
	return crc;
}

int srxl_decode(uint8_t byte, uint8_t *rssi, uint8_t *rx_count, uint16_t *channel_count, uint16_t *channels,
			uint16_t max_chan_count)
{

	int ret = 1;
	switch (_decode_state) {
	case SRXL_DECODE_STATE_UNSYNCED:
		if(_debug)
			printf( " SRXL_DECODE_STATE_UNSYNCED \n") ;
			
		switch (byte) {
		case SRXL_HEADER_V1:
			_rxpacket.header = byte;
			_ver   = 1;
			_data_len = 0;
			_rxpacket.length = SRXL_V1_MAX_CHANNELS;
			_crc16 = 0x0000;
			_crcOK = false;
			_crc16 = srxl_crc16(_crc16, byte);
			_decode_state = SRXL_DECODE_STATE_GOT_HEADER;
			if(_debug)
				printf( " SRXL_DECODE_STATE_GOT_HEADER: %x \n", byte);
			break;
		case SRXL_HEADER_V2:
			_rxpacket.header = byte;
			_ver   = 2;
			_data_len = 0;
			_rxpacket.length = SRXL_V2_MAX_CHANNELS;
			_crc16 = 0x0000;
			_crcOK = false;
			_crc16 = srxl_crc16(_crc16, byte);
			_decode_state = SRXL_DECODE_STATE_GOT_HEADER;
			if(_debug)
				printf( " SRXL_DECODE_STATE_GOT_HEADER: %x \n", byte);
			break;
		default:
			ret = 2;
		}

		break;

	case SRXL_DECODE_STATE_GOT_HEADER:
		_rxpacket.srxl_data[_data_len] = byte;
		_crc16 = srxl_crc16(_crc16, byte);
		_data_len++;
		
		if (_data_len < ((_rxpacket.length*2) )) {
			if(_debug)
				printf( " SRXL_DECODE_STATE_GOT_DATA[%d]: %x\n", _data_len, byte);
		} else {
			_decode_state = SRXL_DECODE_STATE_GOT_DATA;
			if(_debug)
				printf( " SRXL_DECODE_STATE_GOT_DATA[%d]: %x\n", _data_len, byte);
			if(_debug)
				printf( " SRXL_DECODE_STATE_GOT_DATA -- finish --\n");
		}

		break;

	case SRXL_DECODE_STATE_GOT_DATA:
		_rxpacket.crc16_high = byte;
		if(_debug)
			printf( " SRXL_DECODE_STATE_GOT_CRC16[1]: %x   [%x]\n", byte, ((_crc16 >> 8) & 0xff));
		_decode_state = SRXL_DECODE_STATE_GOT_CRC;

		break;
		
	case SRXL_DECODE_STATE_GOT_CRC:
		_rxpacket.crc16_low = byte;
		if(_debug)
			printf( " SRXL_DECODE_STATE_GOT_CRC[2]: %x   [%x]\n\n", byte, (_crc16 & 0xff)) ;
			
		if (_crc16 == (uint16_t)(_rxpacket.crc16_high<<8)+_rxpacket.crc16_low) {
			_crcOK = true;
		}
		
		if (_crcOK) {
			if(_debug)
				printf( " CRC - OK \n") ;

			if(_debug)
				printf( " Got valid SRXL Packet\n") ;
			
			if(_debug)
				printf( " Data Length: %d  [Channels: %d] \n\n", _data_len, (_data_len)/2) ;

			ret = 0;

			(*rx_count)++;      

			/* received Channels */
			if ((uint16_t)_rxpacket.length > max_chan_count) {
				_rxpacket.length = (uint8_t) max_chan_count;
			}
			*channel_count = (uint16_t)_rxpacket.length;

			*rssi = 100;

			/* Multiplex SRXL channel order is:
			*  Roll, Nick, Yaw, Pitch, Throttle, Gyro, Aux, etc..
			**/
			uint8_t chMap[] = {0, 3, 4, 2, 1, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
	
			for (uint8_t i = 0; i<_rxpacket.length; i++) {
				channels[chMap[i]] = (uint16_t)((_rxpacket.srxl_data[chMap[i]*2]<<8) | _rxpacket.srxl_data[chMap[i]*2+1]);
				if(_debug)
					printf( "ch[%d] : %x %x [ %x  ", chMap[i], _rxpacket.srxl_data[chMap[i]*2], _rxpacket.srxl_data[chMap[i]*2+1], channels[chMap[i]]);
				/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
				channels[chMap[i]] = (uint16_t)(channels[chMap[i]] * SRXL_SCALE_FACTOR + 0.5f) + SRXL_SCALE_OFFSET;
				if(_debug)
					printf( "  %d ]\n", channels[chMap[i]]);
			}

		} else {
			/* decoding failed */
			ret = 4;
			if(_debug)
				printf( " CRC - fail \n") ;
		}

		_decode_state = SRXL_DECODE_STATE_UNSYNCED;
		break;
	}

	return ret;
}
