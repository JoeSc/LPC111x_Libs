/*
 *  uart.h
 *  TEST
 *
 *  Created by Joe  Schaack on 8/13/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "lpc111x.h"
#include "sysdefs.h"

#ifndef _uart_H_
#define _uart_H_

#define TX_BUFFER_SIZE 32
#define RX_BUFFER_SIZE 32

void uartInit(uint32_t baudrate);
void uartSend(char data);
uint8_t uartDataAvailable();
uint8_t uartRead();


#endif

