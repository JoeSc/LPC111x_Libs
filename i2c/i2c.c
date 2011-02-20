/*
   i2c.c - I2C/I2C library for Wiring & Arduino
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

#include "i2c.h"

#define TW_READ 1
#define TW_WRITE 0




/* 
 * Function i2c_init
 * Desc     readys i2c pins and sets i2c bitrate
 * Input    none
 * Output   none
 */
void i2c_init(void)
{
    // initialize state
    i2c_state = I2C_READY;

   
   SCB_PRESETCTRL |= (0x1<<1);
      // Enable I2C clock
   SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_I2C);
   
      // Configure pin 0.4 for SCL
   IOCON_PIO0_4 &= ~(IOCON_PIO0_4_FUNC_MASK | IOCON_PIO0_4_I2CMODE_MASK);
   IOCON_PIO0_4 |= (IOCON_PIO0_4_FUNC_I2CSCL);
   
      // Configure pin 0.5 for SDA
   IOCON_PIO0_5 &= ~(IOCON_PIO0_5_FUNC_MASK | IOCON_PIO0_5_I2CMODE_MASK);
   IOCON_PIO0_5 |= IOCON_PIO0_5_FUNC_I2CSDA;
   
      // Clear flags
   I2C_I2CCONCLR = I2C_I2CCONCLR_AAC | 
   I2C_I2CCONCLR_SIC | 
   I2C_I2CCONCLR_STAC | 
   I2C_I2CCONCLR_I2ENC;
   
   I2C_I2CSCLL   = 60;
   I2C_I2CSCLH   = 60;

    //---------------------------------------------------------
    // INIT THE PORTS
    //---------------------------------------------------------

    //---------------------------------------------------------
    // INIT THE FREQ REGS
    //---------------------------------------------------------
   
   
   /* Enable the I2C Interrupt */
   NVIC_EnableIRQ(I2C_IRQn);
   I2C_I2CCONSET = I2C_I2CCONSET_I2EN;
   
   
    // enable i2c module, acks, and i2c interrupt
    //TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);

    // allocate buffers
    //i2c_masterBuffer = (uint8_t*) calloc(I2C_BUFFER_LENGTH, sizeof(uint8_t));
    //i2c_txBuffer = (uint8_t*) calloc(I2C_BUFFER_LENGTH, sizeof(uint8_t));
    //i2c_rxBuffer = (uint8_t*) calloc(I2C_BUFFER_LENGTH, sizeof(uint8_t));
}

/* 
 * Function i2c_slaveInit
 * Desc     sets slave address and enables interrupt
 * Input    none
 * Output   none
 */
void i2c_setAddress(uint8_t address)
{
    // set i2c slave address (skip over TWGCE bit)
    I2C_I2CADR0 = address << 1;
}

/* 
 * Function i2c_readFrom
 * Desc     attempts to become i2c bus master and read a
 *          series of bytes from a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes to read into array
 * Output   number of bytes read
 */
uint8_t i2c_readFrom(uint8_t address, uint8_t* data, uint8_t length)
{
    uint8_t i;

    // ensure data will fit into buffer
    if(I2C_BUFFER_LENGTH < length){
        return 0;
    }

    // wait until i2c is ready, become master receiver
    while(I2C_READY != i2c_state){
        continue;
    }
    i2c_state = I2C_MRX;
    // reset error state (0xFF.. no error occured)
    i2c_error = 0xFF;

    // initialize buffer iteration vars
    i2c_masterBufferIndex = 0;
    i2c_masterBufferLength = length-1;  // This is not intuitive, read on...
    // On receive, the previously configured ACK/NACK setting is transmitted in
    // response to the received byte before the interrupt is signalled. 
    // Therefor we must actually set NACK when the _next_ to last byte is
    // received, causing that NACK to be sent in response to receiving the last
    // expected byte of data.

    // build sla+w, slave device address + w bit
    i2c_slarw = TW_READ;
    i2c_slarw |= address << 1;

    // send start condition
    //TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
    I2C_I2CCONSET = I2C_I2CCONSET_STA;

    // wait for read operation to complete
    while(I2C_MRX == i2c_state){
        continue;
    }

    if (i2c_masterBufferIndex < length)
        length = i2c_masterBufferIndex;

    // copy i2c buffer to data
    for(i = 0; i < length; ++i){
        data[i] = i2c_masterBuffer[i];
    }
   
      //printf("I2C_ERROR = 0x%x\n",i2c_error);
   
   if (i2c_error == 0xFF)
      return 0;
   
   return 1;
}

/* 
 * Function i2c_writeTo
 * Desc     attempts to become i2c bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other i2c error (lost bus arbitration, bus error, ..)
 */
uint8_t i2c_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait)
{
    uint8_t i;

    // ensure data will fit into buffer
   //printf("in write\n");
    if(I2C_BUFFER_LENGTH < length){
        return 1;
    }

    // wait until i2c is ready, become master transmitter
    while(I2C_READY != i2c_state){
        continue;
    }
    i2c_state = I2C_MTX;
    // reset error state (0xFF.. no error occured)
    i2c_error = 0xFF;

    // initialize buffer iteration vars
    i2c_masterBufferIndex = 0;
    i2c_masterBufferLength = length;

    // copy data to i2c buffer
    for(i = 0; i < length; ++i){
        i2c_masterBuffer[i] = data[i];
    }

    // build sla+w, slave device address + w bit
    i2c_slarw = TW_WRITE;
    i2c_slarw |= address << 1;

    // send start condition
//    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
// I2C_I2CCONCLR = I2C_I2CCONCLR_AAC | I2C_I2CCONCLR_SIC;
    I2C_I2CCONSET = I2C_I2CCONSET_STA;

    // wait for write operation to complete
    while(wait && (I2C_MTX == i2c_state)){
        continue;
    }

    if (i2c_error == 0xFF)
        return 0;    // success
    else if (i2c_error == I2C_SLAWTX_NACKRX)
        return 2;    // error: address send, nack received
    else if (i2c_error == I2C_DATTX_NACKRX)
        return 3;    // error: data send, nack received
    else
        return 4;    // other i2c error
}

/* 
 * Function i2c_transmit
 * Desc     fills slave tx buffer with data
 *          must be called in slave tx event callback
 * Input    data: pointer to byte array
 *          length: number of bytes in array
 * Output   1 length too long for buffer
 *          2 not slave transmitter
 *          0 ok
 */
uint8_t i2c_transmit(uint8_t* data, uint8_t length)
{
    uint8_t i;

    // ensure data will fit into buffer
    if(I2C_BUFFER_LENGTH < length){
        return 1;
    }

    // ensure we are currently a slave transmitter
    if(I2C_STX != i2c_state){
        return 2;
    }

    // set length and copy data into tx buffer
    i2c_txBufferLength = length;
    for(i = 0; i < length; ++i){
        i2c_txBuffer[i] = data[i];
    }

    return 0;
}


/* 
 * Function i2c_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
void i2c_reply(uint8_t ack)
{
    // transmit master read ready signal, with or without ack
    if(ack){
        //TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
        I2C_I2CCONSET = I2C_I2CCONSET_I2EN | I2C_I2CCONSET_AA;
      I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;
        
    }else{
        //TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
        I2C_I2CCONCLR = I2C_I2CCONCLR_AAC;
      I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;
    }
}

/* 
 * Function i2c_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
void i2c_stop(void)
{
    // send stop condition
    //TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);
    I2C_I2CCONSET =  I2C_I2CCONSET_STO;
   I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;
    
    // wait for stop condition to be exectued on bus
    // TWINT is not set after a stop condition!
    while(I2C_I2CCONSET & (I2C_I2CCONSET_STO)){
        continue;
    }

    // update i2c state
    i2c_state = I2C_READY;
}

/* 
 * Function i2c_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
void i2c_releaseBus(void)
{
    // release bus
    //TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
    I2C_I2CCONSET = I2C_I2CCONSET_I2EN | I2C_I2CCONSET_AA | I2C_I2CCONSET_STO;

    // update i2c state
    i2c_state = I2C_READY;
}

void I2C_IRQHandler(void) 
{
      //printf("0x%x\n" , I2C_I2CSTAT);
    switch(I2C_I2CSTAT){
        // All Master
        case I2C_STARTTX:     // sent start condition
            {
               // sent repeated start condition
               // copy device address and r/w bit to output register and ack
            I2C_I2CDAT = i2c_slarw;
               //i2c_reply(1);
            I2C_I2CCONCLR = (I2C_I2CCONCLR_SIC | I2C_I2CCONCLR_STAC);
            break;
            }
        case I2C_REPEATEDSTARTTX: 
            {
            // sent repeated start condition
            // copy device address and r/w bit to output register and ack
            I2C_I2CDAT = i2c_slarw;
               //i2c_reply(1);
            I2C_I2CCONCLR = (I2C_I2CCONCLR_SIC | I2C_I2CCONCLR_STAC);
            break;
            }
            // Master Transmitter
        case I2C_SLAWTX_ACKRX:  // slave receiver acked address
        case I2C_DATTX_ACKRX: // slave receiver acked data
            // if there is data to send, send it, otherwise stop 
            if(i2c_masterBufferIndex < i2c_masterBufferLength){
                // copy data to output register and ack
                I2C_I2CDAT = i2c_masterBuffer[i2c_masterBufferIndex++];
               //i2c_reply(1);
            I2C_I2CCONCLR = (I2C_I2CCONCLR_SIC | I2C_I2CCONCLR_STAC);
            }else{
                i2c_stop();
            }
            break;
        case I2C_SLAWTX_NACKRX:  // address sent, nack received
            i2c_error = I2C_SLAWTX_NACKRX;
            i2c_stop();
            break;
        case I2C_DATTX_NACKRX: // data sent, nack received
            i2c_error = I2C_DATTX_NACKRX;
            i2c_stop();
            break;
        case I2C_ARBLOST: // lost bus arbitration
            i2c_error = I2C_ARBLOST;
            i2c_stop();
            break;

            // Master Receiver
        case I2C_DATRX_ACKTX: // data received, ack sent
            // put byte into buffer
            i2c_masterBuffer[i2c_masterBufferIndex++] = I2C_I2CDAT;
        case I2C_SLARTX_ACKRX:  // address sent, ack received
            // ack if more bytes are expected, otherwise nack
            if(i2c_masterBufferIndex < i2c_masterBufferLength){
               //i2c_reply(1);
            I2C_I2CCONSET = I2C_I2CCONSET_AA;
            I2C_I2CCONCLR = (I2C_I2CCONCLR_SIC);

            }else{
               //i2c_reply(0);
            I2C_I2CCONCLR = I2C_I2CCONCLR_AAC | I2C_I2CCONCLR_SIC;
            }
            break;
        case I2C_DATRX_NACKTX: // data received, nack sent
            // put final byte into buffer
            i2c_masterBuffer[i2c_masterBufferIndex++] = I2C_I2CDAT;
        case I2C_SLARTX_NACKRX: // address sent, nack received
            i2c_stop();
            break;
            // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

            // All
        case I2C_NOINFO:   // no state information
            break;
        case I2C_BUSERROR: // bus error, illegal stop/start
            i2c_error = I2C_BUSERROR;
            i2c_stop();
            break;
	}
}

