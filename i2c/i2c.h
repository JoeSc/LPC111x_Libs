/*
 i2c.h - I2C/I2C library for Wiring & Arduino
 Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef i2c_h
#define i2c_h

#include "lpc111x.h"
#include "sysdefs.h"



#define I2C_BUSERROR              0x00
#define I2C_STARTTX               0x08
#define I2C_REPEATEDSTARTTX       0x10
#define I2C_SLAWTX_ACKRX          0x18
#define I2C_SLAWTX_NACKRX         0x20
#define I2C_DATTX_ACKRX           0x28
#define I2C_DATTX_NACKRX          0x30
#define I2C_ARBLOST               0x38
#define I2C_SLARTX_ACKRX          0x40
#define I2C_SLARTX_NACKRX         0x48
#define I2C_DATRX_ACKTX           0x50
#define I2C_DATRX_NACKTX          0x58
#define I2C_NOINFO                0xf8


#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH 32
#endif

#define I2C_READY 0
#define I2C_MRX   1
#define I2C_MTX   2
#define I2C_SRX   3
#define I2C_STX   4

uint8_t i2c_txBuffer[I2C_BUFFER_LENGTH];
//uint8_t i2c_rxBuffer[I2C_BUFFER_LENGTH];


static volatile uint8_t i2c_state;
static volatile uint8_t i2c_slarw;


   //static uint8_t* i2c_masterBuffer;
uint8_t i2c_masterBuffer[I2C_BUFFER_LENGTH];
static volatile uint8_t i2c_masterBufferIndex;
static volatile uint8_t i2c_masterBufferLength;

   //static uint8_t* i2c_txBuffer;
static volatile uint8_t i2c_txBufferIndex;
static volatile uint8_t i2c_txBufferLength;

   //static uint8_t* i2c_rxBuffer;

//static volatile uint8_t i2c_rxBufferIndex;

static volatile uint8_t i2c_error;





void i2cInit(void);
void i2c_setAddress(uint8_t address);
uint8_t i2c_readFrom(uint8_t address, uint8_t* data, uint8_t length);
uint8_t i2c_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait);
uint8_t i2c_transmit(uint8_t* data, uint8_t length);
void i2c_reply(uint8_t ack);
void i2c_stop(void);
void i2c_releaseBus(void);

#endif

