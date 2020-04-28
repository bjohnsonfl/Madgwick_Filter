//
//  madgwickFilter.c
//  madgwickFilter
//
//  Created by Blake Johnson on 4/28/20.
//  Copyright © 2020 Blake Johnson. All rights reserved.
//

#include "madgwickFilter.h"

struct quaternion q_est = {0};



struct quaternion quat_mult (struct quaternion L, struct quaternion R){
    
    
    struct quaternion product;
    product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
    product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
    product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
    product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);
    
    return product;
}



void imu_filter(float ax, float ay, float az, float gx, float gy, float gz){
    printQuaternion(q_est);
    // Store q_est_t into q_est_t-1
    struct quaternion q_est_prev = q_est;
    
    /* Integrate angluar velocity to obtain position in angles. */
    struct quaternion q_w;                   // equation (10), places gyroscope readings in a quaternion
    q_w.q1 = 0;                              // the real component is zero, which the Madgwick uses to simplfy quat. mult.
    q_w.q2 = gx;
    q_w.q3 = gy;
    q_w.q4 = gz;
    
    quat_scalar(&q_w, 0.5);                  // the 0.5 comes from the equation P = Po + 1/2(deltaV) * t I believe
    
    printQuaternion(q_w);
    
    q_w = quat_mult(q_est_prev, q_w);        // equation (12)
    
    printQuaternion(q_w);
    quat_scalar(&q_w, deltaT);               // equation (13) integrates the angles velocity to position starting with time multiplication
  printQuaternion(q_w);
    quat_add(&q_w, q_w, q_est_prev);         // addition part of equation (13)
    printQuaternion(q_w);
    
    
    
    
    
    
    
    
}

