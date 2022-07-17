//
//  main.c
//  madgwickFilter
//
//  Created by Blake Johnson on 4/28/20.
//

#include <stdio.h>
#include "madgwickFilter.h"

int main(int argc, const char * argv[]) {
    float roll = 0.0, pitch = 0.0, yaw = 0.0;

    // flat upright position to resolve the graident descent problem
    printf("\n\n Upright to solve gradient descent problem\n");
    for(float i = 0; i<1000; i++){
        imu_filter(0.05, 0.05, 0.9, 0, 0, 0);
        eulerAngles(q_est, &roll, &pitch, &yaw);
        printf("Time (s): %.3f, roll: %f, pitch: %f, yaw: %f\n", i * DELTA_T, roll, pitch, yaw);
    }

    // Angular Rotation around Z axis (yaw) at 100 degrees per second
    printf("\n\nYAW 100 Degrees per Second\n");
    for(float i = 0; i<1000; i++){
        imu_filter(0, 0, 1, 0, 0, -100 * PI / 180);
        eulerAngles(q_est, &roll, &pitch, &yaw);
        printf("Time (s): %.3f, roll: %f, pitch: %f, yaw: %f\n", i * DELTA_T, roll, pitch, yaw);
    }

    // Angular Rotation around Z axis (yaw) at -20 degrees per second
    printf("\n\nYAW -20 Degrees per Second\n");
    for(float i = 0; i<1000; i++){
        imu_filter(0, 0, 1, 0, 0, 20 * PI / 180);
        eulerAngles(q_est, &roll, &pitch, &yaw);
        printf("Time (s): %.3f, roll: %f, pitch: %f, yaw: %f\n", i * DELTA_T, roll, pitch, yaw);
    }
    
    return 0;
}
