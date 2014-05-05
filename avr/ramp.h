/*
 * ramp.h
 *
 *  Created on: Apr 23, 2014
 *      Author: Mafioso
 */

#ifndef RAMP_H
#define RAMP_H
 
#include <math.h>
#include <inttypes.h>

typedef struct {
    uint32_t accStepCount;
    uint32_t decStepCount;
    uint32_t acceleration;
    uint32_t deceleration;
    float speed;
    uint32_t minSpeed;
    float maxSpeed;
} RampState;

extern void RAMPSetAcceleration(RampState* state, float acceleration);
extern void RAMPSetDeceleration(RampState* state, float deceleration);
extern void RAMPSetMaxSpeed(RampState* state, float maxSpeed);

extern void RAMPSetParams(
    RampState* state,
    uint32_t acceleration,
    uint32_t deceleration,
    uint32_t minSpeed,
    uint32_t maxSpeed);
    
extern uint32_t RAMPGetSpeed(
    RampState* state,
    uint32_t position,
    uint32_t targetPosition);

#endif /* RAMP_H */
