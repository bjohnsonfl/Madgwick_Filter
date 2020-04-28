//
//  madgwickFilter.c
//  madgwickFilter
//
//  Created by Blake Johnson on 4/28/20.
//  Copyright © 2020 Blake Johnson. All rights reserved.
//

#include "madgwickFilter.h"

struct quaternion q_est;


struct quaternion quat_mult (struct quaternion L, struct quaternion R){
    
    
    struct quaternion product;
    product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
    product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
    product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
    product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);
    
    return product;
}



void imu_filter(float ax, float ay, float az, float gx, float gy, float gz){
    
    // Store q_est_t into q_est_t-1
    struct quaternion q_est_prev = q_est;
    
    printf("%f %f %f %f", q_est_prev.q1, q_est_prev.q2, q_est_prev.q3, q_est_prev.q4);
    // Integrate angluar velocity to obtain position in angles.
    
    
    
    
}

