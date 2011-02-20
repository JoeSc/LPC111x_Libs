/*
 *  RC.h
 *  RC_PWM
 *
 *  Created by Joe  Schaack on 12/2/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _RC_PWM_H_
#define _RC_PWM_H_

#include "lpc111x.h"


#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

#define MIN_PULSEWIDTH 900
#define MAX_PULSEWIDTH 2100

#define NUM_CHANNELS 6

volatile unsigned int timer_old;
volatile unsigned char PPM_Counter;
volatile unsigned int PWM_RAW[NUM_CHANNELS];
volatile unsigned char radio_status;


/* Init all the timers */
void rcinit();

/* set the channels to a specific pwm*/
void setRC(uint8_t ch , int pwm);

/* get value received from the PPM channel */
unsigned int inputRC(uint8_t ch);

#endif
