/*
 *  DCM.h
 *  I2C
 *
 *  Created by Joe  Schaack on 11/18/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef DCM_H_
#define DCM_H_

#include <math.h>

   #define Kp_ROLLPITCH  0.0014
   #define Ki_ROLLPITCH  0.00000015
#define Kp_YAW  1.0
#define Ki_YAW  0.00002

   //float Kp_ROLLPITCH;// = 0.0014;
   //float Ki_ROLLPITCH;// = 0.00000015;

#define Gyro_Scaled_X(x) x*ToRad((1.0/14.375)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad((1.0/14.375)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad((1.0/14.375)) //Return the scaled ADC raw data of the gyro in radians for second

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float G_Dt;
float roll ;
float pitch ;
float yaw ;



void Normalize(void);
void Drift_correction(void);
void Accel_adjust(void);
void Matrix_update( int gx , int gy , int gz , int ax , int ay ,  int az );
void Euler_angles(void);
float Vector_Dot_Product(float vector1[3],float vector2[3]);
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3]);
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2);
void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]);
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);




#endif
   //