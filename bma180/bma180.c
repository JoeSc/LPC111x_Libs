/*
 *  BMA180.c
 *  I2C
 *
 *  Created by Joe  Schaack on 11/16/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "bma180.h"

uint8_t init_bma180( uint8_t range, uint8_t bw)
{
   uint8_t data[] = {0,0};
   uint8_t temp = 0x00;
   data[0] = BMA180_ID;
   if(i2c_writeTo(BMA180_ADDR, data, 1, 1))
      return 1;
   
   if (i2c_readFrom(BMA180_ADDR, data, 1))
      return 1;

   if(data[0] != 3)
      return 1;
   
   data[0] = BMA180_CTRLREG0;
   i2c_writeTo(BMA180_ADDR, data, 1, 1);
   i2c_readFrom(BMA180_ADDR, data, 1);
   data[1] = data[0] | 0x10;
   data[0] = BMA180_ADDR;
   i2c_writeTo(BMA180_ADDR, data, 2, 1);
   
   
   data[0] = BMA180_BWTCS;
   i2c_writeTo(BMA180_ADDR, data, 1, 1);
   i2c_readFrom(BMA180_ADDR, data, 1);
   temp = bw << 4;
   temp &= (~BMA180_BWMASK);
   data[1] = data[0] | temp;
   data[0] = BMA180_ADDR;
   i2c_writeTo(BMA180_ADDR, data, 2, 1);
   
   data[0] = BMA180_OLSB1;
   i2c_writeTo(BMA180_ADDR, data, 1, 1);
   i2c_readFrom(BMA180_ADDR, data, 1);
   temp = range << BMA180_RANGESHIFT;
   temp &= (~BMA180_RANGEMASK);
   data[1] = data[0] | temp;
   data[0] = BMA180_ADDR;
   i2c_writeTo(BMA180_ADDR, data, 2, 1);
   
   return 0;
   
}

void update_bma180( )
{
   uint8_t data[7];
   data[0] = BMA180_ACCXLSB;
   i2c_writeTo(BMA180_ADDR, data, 1, 1);
   
   i2c_readFrom(BMA180_ADDR, data, 7);
//   printf("Read   ");
//   for (i=0; i<7; i++) {
//      printf("0x%x  ",data[i]);
//   }
//   printf("\n");
   bma180_acc_x = ((data[1] << 8 ) | data[0]);
   bma180_acc_y = ((data[3] << 8 ) | data[2]);
   bma180_acc_z = ((data[5] << 8 ) | data[4]);
   
   bma180_acc_x = bma180_acc_x >> 2;
   bma180_acc_y = bma180_acc_y >> 2;
   bma180_acc_z = bma180_acc_z >> 2;

   bma180_raw_temp = data[6];   
   
}

int get_temperature_bma180()
{
   return 24 + bma180_raw_temp/2;
}









   //
