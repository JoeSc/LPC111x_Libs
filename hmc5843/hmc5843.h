/*
 *  HMC5843.h
 *  I2C
 *
 *  Created by Joe  Schaack on 11/18/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef HMC5843_H_
#define HMC5843_H_

#include "i2c.h"
#include "fixmath.h"

   
#define HMC5843_ADDR 0x1E

#define HMC5843_CFGA    0x00
#define HMC5843_CFGB    0x01
#define HMC5843_MR      0x02
#define HMC5843_MAGX_H  0x03
#define HMC5843_MAGX_L  0x04
#define HMC5843_MAGY_H  0x05
#define HMC5843_MAGY_L  0x06
#define HMC5843_MAGZ_H  0x07
#define HMC5843_MAGZ_L  0x08

int16_t hmc5843_mag_x;
int16_t hmc5843_mag_y;
int16_t hmc5843_mag_z;
fix16_t hmc5843_head_x;
fix16_t hmc5843_head_y;
fix16_t hmc5843_heading;


uint8_t init_hmc5843();
void update_hmc5843();
void calculate_heading_hmc5843( fix16_t roll, fix16_t pitch);

   
   
#endif 

   
   //
