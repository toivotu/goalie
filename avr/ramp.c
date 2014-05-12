/*
 * ramp.c
 *
 *  Created on: Apr 23, 2014
 *      Author: Mafioso
 */
 
#include "ramp.h"

#if 1
#define DEBUG_OUT(x,y)
#else
#include <stdio.h>
#define DEBUG_OUT(x,y) printf(x, y);
#endif

/**
 * @param state
 * @param acceleration [steps / s^2]
 * @param decelration [steps / s^2]
 * @param minSpeed [steps / s]
 * @param maxSpeed [steps / s]
 */
void RAMPSetParams(
    RampState* state,
    uint32_t acceleration,
    uint32_t deceleration,
    uint32_t minSpeed,
    uint32_t maxSpeed)
{
    /* Ramp steps:
     *  1/2 * a * t^2
     *  where t = maxSpeed / a
     */

    float accelerationTime = (float)maxSpeed / (float)acceleration;
    float decelerationTime = (float)maxSpeed / (float)deceleration;

    state->accStepCount = 0.5f * acceleration * accelerationTime * accelerationTime;
    state->decStepCount = 0.5f * deceleration * decelerationTime * decelerationTime;
    
    state->minSpeed = minSpeed;
    state->maxSpeed = maxSpeed;
    state->acceleration = acceleration;
    state->deceleration = deceleration;
}

void RAMPInit(RampState* ramp)
{
    ramp->speed = 0;
}

void RAMPSetAcceleration(RampState* state, float acceleration)
{
    RAMPSetParams(state, acceleration, state->deceleration, state->minSpeed, state->maxSpeed);
}

void RAMPSetDeceleration(RampState* state, float deceleration)
{
    RAMPSetParams(state, state->acceleration, deceleration, state->minSpeed, state->maxSpeed);
}

void RAMPSetMaxSpeed(RampState* state, float maxSpeed)
{
    RAMPSetParams(state, state->acceleration, state->deceleration, state->minSpeed, maxSpeed);
}

static uint32_t DecelerationSpeedAt(const RampState* state, uint32_t distance)
{
    return sqrt(distance * 2.f * state->deceleration);
}

static float Speed(float speed, int32_t diff)
{
    return diff > 0 ? speed : -speed;
}

static void Decelerate(RampState* state, int32_t diff)
{
    float step = (float)state->deceleration / state->speed;

    if (fabsf(state->speed) > fabsf(step) + state->minSpeed) {
        state->speed -= step;
    } else {
        state->speed = Speed(state->minSpeed, diff);
    }
}

static bool_t Decelerating(const RampState* state, int32_t diff)
{
    uint32_t absDiff = diff > 0 ? diff : -diff;

    return (diff > 0 && state->speed < 0) ||
           (diff < 0 && state->speed > 0) ||
           ((absDiff <= state->decStepCount) && (fabsf(state->speed) > DecelerationSpeedAt(state, absDiff)));

}

int32_t RAMPGetSpeed(RampState* state, uint32_t position, uint32_t targetPosition)
{
    int32_t diff = targetPosition - position;

    if (diff == 0) {
    
        /* Stopped */
        DEBUG_OUT("%s", "stopped ");
        state->speed = 0.f;
        
    } else if (Decelerating(state, diff)) {
    
        /* Decelerating */
        DEBUG_OUT("%s", "declerating ");
        Decelerate(state, diff);
        
    } else if (fabsf(state->speed) < state->maxSpeed) {
    
        /* Accelerating */
        DEBUG_OUT("%s", "accelerating ");
        if (fabsf(state->speed) > 0) {
            state->speed += (float)state->acceleration / state->speed;
        } else {
            state->speed = Speed(state->minSpeed, diff);
        }
            
        if (fabsf(state->maxSpeed) > state->maxSpeed) {
            state->speed = Speed(state->maxSpeed, diff);
        }
        
    }  else {
    
        /* Running at max speed */
        DEBUG_OUT("%s", "maxSpeed ");
        state->speed = Speed(state->maxSpeed, diff);
    }
        
    return state->speed;
}
