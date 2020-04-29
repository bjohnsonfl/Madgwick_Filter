# Madgwick Filter
The paper can be found [here](https://courses.cs.washington.edu/courses/cse474/17wi/labs/l4/madgwick_internal_report.pdf).
My explanation of the filter can be found on my website [here](https://blakejohnsonuf.com/project/#IMU), just scroll down to the Madgwick Filter section.

## Brief Description
An IMU is a sensor suite complete with an accelerometer and gyroscope. A MARG contains an IMU plus a Magnetometer.
All three of these sensors measure physical qualities of Earth's fields or orientation due to angular momentum. Alone, these sensors have faults thats that the other sensors can make up for. One problem is when a sensor has an axis aligned with Earth's field which prevents using trig functions to determine orientation due to tan(90) being undefined.
The fusion of sensors is critical for accuracy of orientation and for a complete transformation from the sensor's inertial frame to the Earth frame. 

The Madgwick Filter fuses the IMU and optonally the MARG. It does this by using gradient descent to optimize a Quaternion that orients accelerometer data to a known reference of gravity. This quaternion is weighted and integrated with the gyroscope quaternion and previous orientation. This result is normalized and and converted to Euler angles.

A Quaternion is a 4 dimensional number, an extension of the complex plane. It has a real component and 3 imaginary components. Rather than rotate about 3 axes, the quaternion rotates a certain amount of degrees about a vector. The math is ridiculously simple (in quaternion form) compared to euler rotation matrices. My website link above goes into quaternions to a deeper level.

## Whats next

This filter will be used in my IMU project, also on github, and run on an embedded MCU. I also intend to implement the MARG filter.  
 
