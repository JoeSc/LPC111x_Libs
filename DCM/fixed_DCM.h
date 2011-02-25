
#ifndef FIXED_DCM_H_
#define FIXED_DCM_H_

#include <fixmath.h>



#define Kp_ROLLPITCH  const_fix16_from_dbl(0.0014)
#define Ki_ROLLPITCH  const_fix16_from_dbl(0.00000015)
#define Kp_YAW  const_fix16_from_dbl(1.0)
#define Ki_YAW  const_fix16_from_dbl(0.00002)


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define OUTPUTMODE 1

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

#define const_fix16_from_dbl(x) ((fix16_t)(x*65536.0))
#define Gyro_Scaled_X(x) fix16_mul(x, const_fix16_from_dbl(ToRad((1.0/14.375))))
#define Gyro_Scaled_Y(x) fix16_mul(x, const_fix16_from_dbl(ToRad((1.0/14.375))))
#define Gyro_Scaled_Y(x) fix16_mul(x, const_fix16_from_dbl(ToRad((1.0/14.375))))


fix16_t G_Dt;
fix16_t roll ;
fix16_t pitch ;
fix16_t yaw ;

void Normalize(void);
void Drift_correction(fix16_t head_x, fix16_t head_y);
void Accel_adjust(void);
void Matrix_update( int gx , int gy , int gz , int ax , int ay ,  int az );
void Euler_angles(void);
fix16_t Vector_Dot_Product(fix16_t vector1[3],fix16_t vector2[3]);
void Vector_Cross_Product(fix16_t vectorOut[3], fix16_t v1[3],fix16_t v2[3]);
void Vector_Scale(fix16_t vectorOut[3],fix16_t vectorIn[3], fix16_t scale2);
void Vector_Add(fix16_t vectorOut[3],fix16_t vectorIn1[3], fix16_t vectorIn2[3]);
void Matrix_Multiply(fix16_t a[3][3], fix16_t b[3][3],fix16_t mat[3][3]);
void Matrix_Addto(fix16_t matrixOut[3][3],fix16_t a[3][3]);



#endif

