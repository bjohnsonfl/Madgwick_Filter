//
//  madgwickFilter.h
//  madgwickFilter
//
//  Created by Blake Johnson on 4/28/20.
//  Copyright © 2020 Blake Johnson. All rights reserved.
//

#ifndef madgwickFilter_h
#define madgwickFilter_h

#include <stdio.h>
struct quaternion{
    float q1;
    float q2;
    float q3;
    float q4;
};

extern struct quaternion q_est;

//Multiply two quaternions and return a copy of the result
struct quaternion quat_mult (struct quaternion q_L, struct quaternion q_R);

//IMU consists of a Gyroscope plus Accelerometer sensor fusion
void imu_filter(float ax, float ay, float az, float gx, float gy, float gz);

void marg_filter(void);




#endif /* madgwickFilter_h */
