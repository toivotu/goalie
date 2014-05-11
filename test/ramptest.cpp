#include <iostream>

#include "ramp.h"

int main()
{
    RampState ramp;
    
    RAMPSetParams(&ramp, 20, 30, 10, 120);
    
    std::cout << ramp.accStepCount << " " << ramp.decStepCount << std::endl;
    std::cout << ramp.acceleration << " " << ramp.deceleration << std::endl;
    std::cout << ramp.maxSpeed << std::endl;
    
    int32_t speed;
    uint32_t position = 2001;
    uint32_t targetPosition = 0;
    float time = 0;
    RAMPInit(&ramp);
    
    ramp.speed = 120;

    std::cout << "Time" << "," << "Speed" << "," << "Position" << std::endl;
    do {
        speed = RAMPGetSpeed(&ramp, position, targetPosition);
        
        if (speed > 0) {
            time += 1.f / float(fabsf(speed));
        }
        std::cout << time << "," << speed << "," << position << std::endl;
    
        position += speed > 0 ? 1 : -1;
    } while (position != targetPosition);
    
    

    return 0;
}
