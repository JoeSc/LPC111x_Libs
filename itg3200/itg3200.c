/*
 *  itg-3200.c
 *  I2C
 *
 *  Created by Joe  Schaack on 11/18/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "ITG3200.h"

uint8_t init_itg3200()
{
   uint8_t data[] = {0,0};
   
   data[0] = ITG3200_PWR_M;
   data[1] = ITG3200_PWR_M_RST;
   if(i2c_writeTo(ITG3200_ADDR, data, 2, 1))
      return 1;
   
   data[0] = ITG3200_SMPL;
   data[1] = 0x00;         // sample fastest
   i2c_writeTo(ITG3200_ADDR, data, 2, 1);
   
   data[0] = ITG3200_DLPF;
   data[1] = 0x18;         //
   i2c_writeTo(ITG3200_ADDR, data, 2, 1);
   
   data[0] = ITG3200_PWR_M;
   data[1] = 0x00;         // 
   i2c_writeTo(ITG3200_ADDR, data, 2, 1);
   
   return 1;
}


void update_itg3200( )
{
   uint8_t data[8];
   data[0] = ITG3200_TMP_H;
   i2c_writeTo(ITG3200_ADDR, data, 1, 1);
   
   i2c_readFrom(ITG3200_ADDR, data, 8);
//   printf("Read   ");
//   for (i=0; i<8; i++) {
//      printf("0x%x  ",data[i]);
//   }
//   printf("\n");
   
   itg3200_raw_temp = (data[0] << 8 ) | data[1];
   itg3200_gyro_x = (data[2] << 8 ) | data[3];
   itg3200_gyro_y = (data[4] << 8 ) | data[5];
   itg3200_gyro_z = (data[6] << 8 ) | data[7];  
   
}

int get_itg3200_temperature()
{
   return      35 + ((itg3200_raw_temp + 13200) /280);
}












   //
