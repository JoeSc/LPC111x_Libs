/*
 *  HMC5843.c
 *  I2C
 *
 *  Created by Joe  Schaack on 11/18/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "HMC5843.h"

uint8_t init_hmc5843()
{
   uint8_t data[] = {0,0};
   
   data[0] = HMC5843_MR;
   data[1] = 0x00;
   if(i2c_writeTo(HMC5843_ADDR, data, 2, 1))
      return 1;
   
   data[0] = HMC5843_CFGA;
   data[1] = 0x10;  // 10HZ data rate
   i2c_writeTo(HMC5843_ADDR, data, 2, 1);
   
   data[0] = HMC5843_CFGB;
   data[1] = 0x20;  // Gain of 1
   i2c_writeTo(HMC5843_ADDR, data, 2, 1);
   
   
   return 0;   
}


void update_hmc5843( )
{
   uint8_t data[6];
   data[0] = HMC5843_MAGX_H;
   i2c_writeTo(HMC5843_ADDR, data, 1, 1);
   
   
   i2c_readFrom(HMC5843_ADDR, data, 6);
   
  // printf("Read   ");
//   for (i=0; i<6; i++) {
//   printf("0x%x  ",data[i]);
//   }
//   printf("\n");
   
   
   hmc5843_mag_x = (data[0] << 8 ) | data[1];
   hmc5843_mag_y = (data[2] << 8 ) | data[3];
   hmc5843_mag_z = (data[4] << 8 ) | data[5];
   
}

void calculate_heading_hmc5843( fix16_t roll, fix16_t pitch)
{
//   float Head_X;
//   float Head_Y;
//   float cos_roll;
//   float sin_roll;
//   float cos_pitch;
//   float sin_pitch;
//  
//
//   cos_roll = cos(roll);  // Optimizacion, se puede sacar esto de la matriz DCM?
//   sin_roll = sin(roll);
//   cos_pitch = cos(pitch);
//   sin_pitch = sin(pitch);
//      // Tilt compensated Magnetic field X component:
//   Head_X = hmc5843_mag_x*cos_pitch+
//      hmc5843_mag_y*sin_roll*sin_pitch+
//      hmc5843_mag_z*cos_roll*sin_pitch;
//      // Tilt compensated Magnetic field Y component:
//   Head_Y = hmc5843_mag_y*cos_roll-
//      hmc5843_mag_z*sin_roll;
//      // Magnetic Heading
//   //hmc_5843_heading = atan2(-Head_Y,Head_X);
   
    fix16_t cos_roll  = fix16_cos(roll);
    fix16_t sin_roll  = fix16_sin(roll);
    fix16_t cos_pitch = fix16_cos(pitch);
    fix16_t sin_pitch = fix16_sin(pitch);

    hmc5843_head_x = fix16_sadd(fix16_mul(fix16_from_int(hmc5843_mag_x),cos_pitch),
           fix16_sadd(fix16_mul(fix16_from_int(hmc5843_mag_y),fix16_mul(sin_roll,sin_pitch)),
           fix16_mul(fix16_from_int(hmc5843_mag_z),fix16_mul(cos_roll,sin_pitch))));

    hmc5843_head_y = fix16_sadd(fix16_mul(fix16_from_int(hmc5843_mag_y),cos_roll),
            -fix16_mul(fix16_from_int(hmc5843_mag_z),sin_roll));
    
    hmc5843_heading = fix16_atan2(-hmc5843_head_y,hmc5843_head_x);

}

   //
