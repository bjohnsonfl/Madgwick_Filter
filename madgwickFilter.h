//
//  madgwickFilter.h
//  madgwickFilter
//
//  Created by Blake Johnson on 4/28/20.
//  Copyright © 2020 Blake Johnson. All rights reserved.
//

#ifndef madgwickFilter_h
#define madgwickFilter_h

#define deltaT 0.01 // 100Hz sampling frequency


#include <math.h>
#include <stdio.h>



struct quaternion{
    float q1;
    float q2;
    float q3;
    float q4;
};

//global variables
extern struct quaternion q_est;

//Multiply two quaternions and return a copy of the result
struct quaternion quat_mult (struct quaternion q_L, struct quaternion q_R);

//Multiply a reference of a quaternion by a scalar
static inline void quat_scalar(struct quaternion * q, float scalar){
    q -> q1 *= scalar;
    q -> q2 *= scalar;
    q -> q3 *= scalar;
    q -> q4 *= scalar;
}

//Adds two quaternions together and the sum is the pointer to another quaternion
static inline void quat_add(struct quaternion * Sum, struct quaternion L, struct quaternion R){
    Sum -> q1 = L.q1 + R.q1;
    Sum -> q2 = L.q2 + R.q2;
    Sum -> q3 = L.q3 + R.q3;
    Sum -> q4 = L.q4 + R.q4;
}

// the conjugate of a quaternion is it's imaginary component sign changed
static inline struct quaternion conjugate(struct quaternion q){
    q.q2 = -q.q2;
    q.q3 = -q.q3;
    q.q4 = -q.q4;
    return q;
}

// norm of a quaternion is the same as a complex number
// sqrt( q1^2 + q2^2 + q3^2 + q4^2)
// the norm is also the sqrt(q * conjugate(q)), but thats a lot of operations in the quaternion multiplication
static inline float quaternion_Norm (struct quaternion q)
{
    return sqrt(q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3 +q.q4*q.q4);
}

static inline void printQuaternion (struct quaternion q){
    printf("%f %f %f %f\n", q.q1, q.q2, q.q3, q.q4);
}


//IMU consists of a Gyroscope plus Accelerometer sensor fusion
void imu_filter(float ax, float ay, float az, float gx, float gy, float gz);

void marg_filter(void);




#endif /* madgwickFilter_h */
