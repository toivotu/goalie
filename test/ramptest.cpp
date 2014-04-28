#include <iostream>

#include "ramp.h"

int main()
{
    RampState ramp;
    
    RAMPSetParams(&ramp, 20, 30, 80);
    
    std::cout << ramp.accStepCount << " " << ramp.decStepCount << std::endl;
    std::cout << ramp.acceleration << " " << ramp.deceleration << std::endl;
    std::cout << ramp.maxSpeed << std::endl;
    
    uint32_t speed;
    uint32_t position = 0;
    uint32_t targetPosition = 100;
    float time = 0;
    
    
    
    std::cout << "Time" << "," << "Speed" << "," << "Position" << std::endl;
    do {
        speed = RAMPGetSpeed(&ramp, position, targetPosition);
        
        if (speed > 0) {
            time += 1.f / float(speed);
        }
        std::cout << time << "," << speed << "," << position << std::endl;
    
        position += 1;
    } while (position != targetPosition);
    
    

    return 0;
}