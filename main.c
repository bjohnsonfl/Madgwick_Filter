//
//  main.c
//  madgwickFilter
//
//  Created by Blake Johnson on 4/28/20.
//  Copyright © 2020 Blake Johnson. All rights reserved.
//

#include <stdio.h>
#include "madgwickFilter.h"
int main(int argc, const char * argv[]) {
    
    float roll = 0.0, pitch = 0.0, yaw = 0.0;
    for(int i = 0; i<100; i++){
        imu_filter(0, 0.5, 0.5, 0, 0, 0);
        eulerAngles(q_est, &roll, &pitch, &yaw);
        printf("roll: %f, pitch: %f, yaw: %f\n", roll, pitch, yaw);
    }
    return 0;
}
