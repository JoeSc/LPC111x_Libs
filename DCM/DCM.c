/*
 *  DCM.c
 *  I2C
 *
 *  Created by Joe  Schaack on 11/18/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "DCM.h"
#define OUTPUTMODE 1
#define GRAVITY 4096





   //float G_Dt=0.02;                  // Integration time for the gyros (DCM algorithm)
float Accel_Vector[3]= {0, 0, 0}; //Store the acceleration in a vector
float Accel_Vector_unfiltered[3]= {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0};  //Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0};      //Omega Proportional correction
float Omega_I[3]= {0, 0, 0};      //Omega Integrator
float Omega[3]= {0, 0, 0};
   //float Accel_magnitude;
   //float Accel_weight;

float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float errorCourse = 0;
float COGX = 0; //Course overground X axis
float COGY = 1; //Course overground Y axis

// roll = 0;
// pitch = 0;
// yaw = 0;

unsigned int counter = 0;

float DCM_Matrix[3][3]= {
   { 1,0,0 },
   { 0,1,0 },
   { 0,0,1 }}; 
float Update_Matrix[3][3]={
   { 0,1,2 },
   { 3,4,5 },
   { 6,7,8 }}; //Gyros here

float Temporary_Matrix[3][3]={
   { 0,0,0 },
   { 0,0,0 },
   { 0,0,0 }};



/*
 ArduCopter v1.3 - August 2010
 www.ArduCopter.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 This program is free software: you can redistribute it and/or modify 
 it under the terms of the GNU General Public License as published by 
 the Free Software Foundation, either version 3 of the License, or 
 (at your option) any later version. 
 
 This program is distributed in the hope that it will be useful, 
 but WITHOUT ANY WARRANTY; without even the implied warranty of 
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 GNU General Public License for more details. 
 
 You should have received a copy of the GNU General Public License 
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */




/* ******* DCM IMU functions ********************* */
/**************************************************/
void Normalize(void)
{
	float error=0;
	float temporary[3][3];
	float renorm=0;
   
	error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19
   
	Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
	Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
   
	Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
	Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
   
	Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
   
	renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
	Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
   
	renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
	Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
   
	renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
	Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void)
{
      //Compensation the Roll, Pitch and Yaw drift. 
	float errorCourse;
	static float Scaled_Omega_P[3];
	static float Scaled_Omega_I[3];
   
      //*****Roll and Pitch***************
   
	Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
                                                                                // errorRollPitch are in Accel ADC units
                                                                                // Limit max errorRollPitch to limit max Omega_P and Omega_I
	errorRollPitch[0] = constrain(errorRollPitch[0],-1000,1000);
	errorRollPitch[1] = constrain(errorRollPitch[1],-1000,1000);
	errorRollPitch[2] = constrain(errorRollPitch[2],-1000,1000);
   
      //printf("ERROR ROLL PITCH = %d,%d,%d\n",(int)(1000*errorRollPitch[0]),(int)(1000*errorRollPitch[1]),(int)(1000*errorRollPitch[2]));
      //printf("%d\n",(int)(1000*errorRollPitch[0]));
	Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH);
   
	Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH);
	Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);
   
      //*****YAW***************
      // We make the gyro YAW drift correction based on compass magnetic heading
#ifdef IsMAG
	if (MAGNETOMETER == 1) {
		errorCourse= (DCM_Matrix[0][0]*APM_Compass.Heading_Y) - (DCM_Matrix[1][0]*APM_Compass.Heading_X);  //Calculating YAW error
		Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
      
		Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
		Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
      
         // Limit max errorYaw to limit max Omega_I
		errorYaw[0] = constrain(errorYaw[0],-50,50);
		errorYaw[1] = constrain(errorYaw[1],-50,50);
		errorYaw[2] = constrain(errorYaw[2],-50,50);
      
		Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
		Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
	}
#endif
   
}
 


void Matrix_update( int gx , int gy , int gz , int ax , int ay ,  int az )
{
      //printf("ACCEL Analog = %d,%d,%d\n",ax,ay,az);
	Gyro_Vector[0]=Gyro_Scaled_X(((float)(gx))); //gyro x roll
	Gyro_Vector[1]=Gyro_Scaled_Y(((float)gy)); //gyro y pitch
	Gyro_Vector[2]=Gyro_Scaled_Z(((float)(gz))); //gyro Z yaw
   
      //Accel_Vector[0]=read_adc(3); // acc x
      //Accel_Vector[1]=read_adc(4); // acc y
      //Accel_Vector[2]=read_adc(5); // acc z
   
      // Low pass filter on accelerometer data (to filter vibrations)
	Accel_Vector[0]=Accel_Vector[0]*0.6 + ((float)ax)*0.4; // acc x
	Accel_Vector[1]=Accel_Vector[1]*0.6 + ((float)ay)*0.4; // acc y
	Accel_Vector[2]=Accel_Vector[2]*0.6 + ((float)az)*0.4; // acc z
   
      //printf("Gyro VECTOR = %d,%d,%d\n",(int)(1000*Gyro_Vector[0]),(int)(1000*Gyro_Vector[1]),(int)(1000*Gyro_Vector[2]));
   
	Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);//adding integrator
	Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]);//adding proportional
   
      //Accel_adjust();//adjusting centrifugal acceleration. // Not used for quadcopter
   
#if OUTPUTMODE==1 // corrected mode
	Update_Matrix[0][0]=0;
	Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
	Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
	Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
	Update_Matrix[1][1]=0;
	Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
	Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
	Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
	Update_Matrix[2][2]=0;
#endif
#if OUTPUTMODE==0 // uncorrected data of the gyros (with drift)
	Update_Matrix[0][0]=0;
	Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
	Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
	Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
	Update_Matrix[1][1]=0;
	Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
	Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
	Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
	Update_Matrix[2][2]=0;
#endif
   
	Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c
   int x;
   int y;
	for( x=0; x<3; x++)  //Matrix Addition (update)
	{
		for(y=0; y<3; y++)
		{
			DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
		} 
	}
}
 
 

void Euler_angles(void)
{
#if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
	roll = atan2(Accel_Vector[1],Accel_Vector[2]);   // atan2(acc_y,acc_z)
	pitch = -asin((Accel_Vector[0])/(float)GRAVITY); // asin(acc_x)
	yaw = 0;
#else        // Euler angles from DCM matrix
	pitch = asin(-DCM_Matrix[2][0]);
	roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
	yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
#endif
}


   // VECTOR FUNCTIONS
   //Computes the dot product of two vectors

float Vector_Dot_Product(float vector1[3],float vector2[3])
{
	float op=0;
   int c;
	for( c=0; c<3; c++)
	{
		op+=vector1[c]*vector2[c];
	}
   
	return op; 
}

   //Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
	vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
	vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
	vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

   //Multiply the vector by a scalar. 
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
   int c;
	for(c=0; c<3; c++)
	{
		vectorOut[c]=vectorIn[c]*scale2; 
	}
}

void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
   int c;
	for(c=0; c<3; c++)
	{
		vectorOut[c]=vectorIn1[c]+vectorIn2[c];
	}
}

/********* MATRIX FUNCTIONS *****************************************/
   //Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 

void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
	float op[3]; 
   int x,y,w;
	for( x=0; x<3; x++)
	{
		for( y=0; y<3; y++)
		{
			for( w=0; w<3; w++)
			{
				op[w]=a[x][w]*b[w][y];
			} 
			mat[x][y]=0;
			mat[x][y]=op[0]+op[1]+op[2];
         
			float test=mat[x][y];
		}
	}
}






   //