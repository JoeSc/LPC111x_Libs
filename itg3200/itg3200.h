/*
 *  itg-3200.h
 *  I2C
 *
 *  Created by Joe  Schaack on 11/18/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef ITG3200_H_
#define ITG3200_H_

#include "i2c.h"

#define ITG3200_GAIN (1/14.375)
#define ITG3200_ADDR 0x69
   // ITG3200 Register Defines
#define ITG3200_WHO		0x00
#define	ITG3200_SMPL	0x15
#define ITG3200_DLPF	0x16
#define ITG3200_INT_C	0x17
#define ITG3200_INT_S	0x1A
#define	ITG3200_TMP_H	0x1B
#define	ITG3200_TMP_L	0x1C
#define	ITG3200_GX_H	0x1D
#define	ITG3200_GX_L	0x1E
#define	ITG3200_GY_H	0x1F
#define	ITG3200_GY_L	0x20
#define ITG3200_GZ_H	0x21
#define ITG3200_GZ_L	0x22
#define ITG3200_PWR_M	0x3E
#define ITG3200_PWR_M_RST 0x80

int16_t itg3200_raw_temp;
int16_t itg3200_gyro_x;
int16_t itg3200_gyro_y;
int16_t itg3200_gyro_z;


uint8_t init_itg3200();
void update_itg3200( );
int get_itg3200_temperature();


#endif 






   //
