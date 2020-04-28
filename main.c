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
   
    q_est.q1 = 5;
    q_est.q2 = 6;
    q_est.q3 = 7;
    q_est.q4 = 8;
    struct quaternion q = {0};//{ 2,4,5,6};
    printQuaternion(q);
    
    imu_filter(0, 0, 1, 2, -2, 2);
    
    
    
    return 0;
}
