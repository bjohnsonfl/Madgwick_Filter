//
//  madgwickFilter.h
//  madgwickFilter
//
//  Created by Blake Johnson on 4/28/20.
//

#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

// Include a hardware specific header file to redefine these predetermined values
#ifndef DELTA_T
    #define DELTA_T 0.01f // 100Hz sampling frequency
#endif

#ifndef PI  
    #define PI 3.14159265358979f
#endif

#ifndef GYRO_MEAN_ERROR
    #define GYRO_MEAN_ERROR PI * (5.0f / 180.0f) // 5 deg/s gyroscope measurement error (in rad/s)  *from paper*
#endif

#ifndef BETA
    #define BETA sqrt(3.0f/4.0f) * GYRO_MEAN_ERROR    //*from paper*
#endif

#include <math.h>
#include <stdio.h>

struct quaternion{
    float q1;
    float q2;
    float q3;
    float q4;
};

// global variables
extern struct quaternion q_est;

// Multiply two quaternions and return a copy of the result, prod = L * R
struct quaternion quat_mult (struct quaternion q_L, struct quaternion q_R);

// Multiply a reference of a quaternion by a scalar, q = s*q
static inline void quat_scalar(struct quaternion * q, float scalar){
    q -> q1 *= scalar;
    q -> q2 *= scalar;
    q -> q3 *= scalar;
    q -> q4 *= scalar;
}

// Adds two quaternions together and the sum is the pointer to another quaternion, Sum = L + R
static inline void quat_add(struct quaternion * Sum, struct quaternion L, struct quaternion R){
    Sum -> q1 = L.q1 + R.q1;
    Sum -> q2 = L.q2 + R.q2;
    Sum -> q3 = L.q3 + R.q3;
    Sum -> q4 = L.q4 + R.q4;
}

// Subtracts two quaternions together and the sum is the pointer to another quaternion, sum = L - R
static inline void quat_sub(struct quaternion * Sum, struct quaternion L, struct quaternion R){
    Sum -> q1 = L.q1 - R.q1;
    Sum -> q2 = L.q2 - R.q2;
    Sum -> q3 = L.q3 - R.q3;
    Sum -> q4 = L.q4 - R.q4;
}


// the conjugate of a quaternion is it's imaginary component sign changed  q* = [s, -v] if q = [s, v]
static inline struct quaternion quat_conjugate(struct quaternion q){
    q.q2 = -q.q2;
    q.q3 = -q.q3;
    q.q4 = -q.q4;
    return q;
}

// norm of a quaternion is the same as a complex number
// sqrt( q1^2 + q2^2 + q3^2 + q4^2)
// the norm is also the sqrt(q * conjugate(q)), but thats a lot of operations in the quaternion multiplication
static inline float quat_Norm (struct quaternion q)
{
    return sqrt(q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3 +q.q4*q.q4);
}

// Normalizes pointer q by calling quat_Norm(q),
static inline void quat_Normalization(struct quaternion * q){
    float norm = quat_Norm(*q);
    q -> q1 /= norm;
    q -> q2 /= norm;
    q -> q3 /= norm;
    q -> q4 /= norm;
}

static inline void printQuaternion (struct quaternion q){
    printf("%f %f %f %f\n", q.q1, q.q2, q.q3, q.q4);
}


// IMU consists of a Gyroscope plus Accelerometer sensor fusion
void imu_filter(float ax, float ay, float az, float gx, float gy, float gz);

// void marg_filter(void); for future


void eulerAngles(struct quaternion q, float* roll, float* pitch, float* yaw);



#endif /* MADGWICK_FILTER_H */
