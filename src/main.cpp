#include <iostream>
#include "extended_kalman_filter.h"

//find pareticle filter algorithm
//https://www.youtube.com/watch?v=3JH4w8Qkx2M
//write a program to implement the algorithm

//1. Initialize particles
//2. Predict the state of the system
//3. Update the weights of the particles
//4. Resample the particles
//5. Estimate the state of the system
//6. Repeat from step 2

int main() {
    for(int seq = 0; seq < 22; seq++) {
        std::cout << "Processing sequence " << seq << std::endl;
        ExtendedKalmanFilter ekf;
        
    }
}