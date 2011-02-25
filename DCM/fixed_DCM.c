
#include "fixed_DCM.h"

#include <stdio.h>


//G_Dt = const_fix16_from_dbl(.02);G_Dt needs to be set before calling anything

fix16_t Gyro_Vector[3]= {0, 0, 0};  //Store the gyros rutn rate in a vector
fix16_t Accel_Vector[3]= {0, 0, 0}; //Store the acceleration in a vector
fix16_t Omega_Vector[3]= {0, 0, 0}; //Corrected Gyro_Vector data
fix16_t Omega_P[3]= {0, 0, 0};      //Omega Proportional correction
fix16_t Omega_I[3]= {0, 0, 0};      //Omega Integrator
fix16_t Omega[3]= {0, 0, 0};

fix16_t errorRollPitch[3] = {0, 0, 0};
fix16_t errorYaw[3] = {0, 0, 0};
fix16_t errorCourse = 0;

fix16_t DCM_Matrix[3][3]= {
    { 65536,0,0 },
    { 0,65536,0 },
    { 0,0,65536 }}; 
fix16_t Update_Matrix[3][3]={
    { 0,1*65536,2*65536 },
    { 3*65536,4*65536,5*65536 },
    { 6*65536,7*65536,8*65536 }}; //Gyros here

fix16_t Temporary_Matrix[3][3]={
    { 0,0,0 },
    { 0,0,0 },
    { 0,0,0 }};


void Normalize(void)
{
    fix16_t error = 0;
    fix16_t temporary[3][3];
    fix16_t renorm = 0;

    error = -fix16_mul(Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0]),
            const_fix16_from_dbl(.5));

    Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
    Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

    Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
    Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

    Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
    
    renorm = fix16_mul(const_fix16_from_dbl(.5), fix16_sadd(fix16_from_int(3),
                -Vector_Dot_Product(&temporary[0][0],&temporary[0][0])));
    Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

    renorm = fix16_mul(const_fix16_from_dbl(.5), fix16_sadd(fix16_from_int(3),
                -Vector_Dot_Product(&temporary[1][0],&temporary[1][0])));
    Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);


    renorm = fix16_mul(const_fix16_from_dbl(.5), fix16_sadd(fix16_from_int(3),
               - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])));
    Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

void Drift_correction(void)
{
    fix16_t errorCourse;
    static fix16_t Scaled_Omega_P[3];
    static fix16_t Scaled_Omega_I[3];


    Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]);

    errorRollPitch[0] = constrain(errorRollPitch[0],-fix16_from_int(1000),fix16_from_int(1000));
    errorRollPitch[1] = constrain(errorRollPitch[1],-fix16_from_int(1000),fix16_from_int(1000));
    errorRollPitch[2] = constrain(errorRollPitch[2],-fix16_from_int(1000),fix16_from_int(1000));

    Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH);
    
    Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);

#ifdef IsMAG
    
    errorCourse= fix16_sadd(fix16_mul(DCM_Matrix[0][0],/*TODO HEADING Y*/), 
                 fix16_mul(DCM_Matrix[1][0],/*TODO HEADING X */)); 
    
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); 

    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.


    errorYaw[0] = constrain(errorYaw[0],-fix16_from_int(50),fix16_from_int(50));
    errorYaw[1] = constrain(errorYaw[1],-fix16_from_int(50),fix16_from_int(50));
    errorYaw[2] = constrain(errorYaw[2],-fix16_from_int(50),fix16_from_int(50));

    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
    
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
#endif

}




void Matrix_update( int gx , int gy , int gz , int ax , int ay ,  int az )
{
    Gyro_Vector[0] = Gyro_Scaled_X(fix16_from_int(gx));
    Gyro_Vector[1] = Gyro_Scaled_X(fix16_from_int(gy));
    Gyro_Vector[2] = Gyro_Scaled_X(fix16_from_int(gz));

/*
    Accel_Vector[0]=fix16_sadd(fix16_mul(Accel_Vector[0],const_fix16_from_dbl(0.6)), 
            fix16_mul(fix16_from_int(ax),const_fix16_from_dbl(0.4)));

    Accel_Vector[1]=fix16_sadd(fix16_mul(Accel_Vector[0],const_fix16_from_dbl(0.6)), 
            fix16_mul(fix16_from_int(ay),const_fix16_from_dbl(0.4)));

    Accel_Vector[2]=fix16_sadd(fix16_mul(Accel_Vector[0],const_fix16_from_dbl(0.6)), 
            fix16_mul(fix16_from_int(az),const_fix16_from_dbl(0.4)));
*/
    /* NO LOW PASS FILTER */
    Accel_Vector[0] = fix16_from_int(ax);
    Accel_Vector[1] = fix16_from_int(ay);
    Accel_Vector[2] = fix16_from_int(az);
    

    Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);//adding integrator
    Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]);//adding proportional

#if OUTPUTMODE==1 // corrected mode
    Update_Matrix[0][0] =  0;
    Update_Matrix[0][1] = -fix16_mul(G_Dt, Omega_Vector[2]);//-z
    Update_Matrix[0][2] =  fix16_mul(G_Dt, Omega_Vector[1]);//y
    Update_Matrix[1][0] =  fix16_mul(G_Dt, Omega_Vector[2]);//z
    Update_Matrix[1][1] =  0;
    Update_Matrix[1][2] = -fix16_mul(G_Dt, Omega_Vector[0]);//-x
    Update_Matrix[2][0] = -fix16_mul(G_Dt, Omega_Vector[1]);//-y
    Update_Matrix[2][1] =  fix16_mul(G_Dt, Omega_Vector[0]);//x
    Update_Matrix[2][2] =  0;
#endif

    Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix);

    Matrix_Addto(DCM_Matrix, Temporary_Matrix);

}
#define GRAVITY 4096

void Euler_angles(void)
{
//    printf("%f,%f,%f\n",fix16_to_dbl(-DCM_Matrix[2][0]),fix16_to_dbl(DCM_Matrix[2][1]),fix16_to_dbl(DCM_Matrix[2][2]));
#if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
    roll = fix16_atan2(Accel_Vector[1],Accel_Vector[2]);   // atan2(acc_y,acc_z)
    pitch = fix16_atan2((Accel_Vector[0]),Accel_Vector[2]); // asin(acc_x)
    yaw = 0;
#else        // Euler angles from DCM matrix
    pitch = fix16_asin(-DCM_Matrix[2][0]);
    roll = fix16_atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    yaw = fix16_atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
#endif
}





void Matrix_Addto(fix16_t matrixOut[3][3],fix16_t a[3][3])
{
    int x;
    int y;
    for( x=0; x<3; x++)  //Matrix Addition (update)
    {
        for(y=0; y<3; y++)
        {
            matrixOut[x][y] = fix16_sadd(a[x][y], matrixOut[x][y]);
            //printf("%f ",fix16_to_dbl(DCM_Matrix[x][y]));
        } 
        //printf("\n");
    }


}

fix16_t Vector_Dot_Product(fix16_t vector1[3],fix16_t vector2[3])
{
    fix16_t op=0;
    int c;
    for( c=0; c<3; c++)
    {
        op = fix16_sadd(op,fix16_mul(vector1[c],vector2[c]));
    }

    return op; 
}


void Vector_Cross_Product(fix16_t vectorOut[3], fix16_t v1[3],fix16_t v2[3])
{
    vectorOut[0]= fix16_sadd(fix16_mul(v1[1],v2[2]), -fix16_mul(v1[2],v2[1]));
    vectorOut[1]= fix16_sadd(fix16_mul(v1[2],v2[0]), -fix16_mul(v1[0],v2[2]));
    vectorOut[2]= fix16_sadd(fix16_mul(v1[0],v2[1]), -fix16_mul(v1[1],v2[0]));
}


void Vector_Scale(fix16_t vectorOut[3],fix16_t vectorIn[3], fix16_t scale2)
{
    int c;
    for(c=0; c<3; c++)
    {
        vectorOut[c]=fix16_mul(vectorIn[c],scale2);
    }
}


void Vector_Add(fix16_t vectorOut[3],fix16_t vectorIn1[3], fix16_t vectorIn2[3])
{
    int c;
    for(c=0; c<3; c++)
    {
        vectorOut[c]=fix16_sadd(vectorIn1[c],vectorIn2[c]);
    }
}


void Matrix_Multiply(fix16_t a[3][3], fix16_t b[3][3],fix16_t mat[3][3])
{
    fix16_t op[3]; 
    int x,y,w;
    for( x=0; x<3; x++)
    {
        for( y=0; y<3; y++)
        {
            for( w=0; w<3; w++)
            {
                op[w] = fix16_mul(a[x][w],b[w][y]);
            } 
            mat[x][y]=0;
            mat[x][y]=fix16_sadd(op[0],fix16_sadd(op[1],op[2]));
        }
    }
}

