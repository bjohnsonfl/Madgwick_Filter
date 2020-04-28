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
    
    imu_filter(0, 0, 1, 0, 0, 0);
    
    
    
    return 0;
}
