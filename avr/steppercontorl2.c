/*
 * steppercontorl.c
 *
 *  Created on: Nov 14, 2013
 *      Author: Mafioso
 */

#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>

//#include <math.h>

#include "uart.h"

#define DDR_EN      DDRC
#define PORT_EN     PORTC
#define PIN_EN      PC1

#define DDR_CW      DDRC
#define PORT_CW     PORTC
#define PIN_CW      PC2

#define DDR_CLK     DDRC
#define PORT_CLK    PORTC
#define PIN_CLK     PC3



#define DEAD_ZONE 10

void Init(void)
{
    DDR_EN |= (1 << PIN_EN);
    DDR_CW |= (1 << PIN_CW);
    DDR_CLK |= (1 << PIN_CLK);

    // Ground potentiometer side
    DDRC |= 1 << PC4;
    PORTC &=  ~(1 << PC4);

    UARTInit();
}

typedef enum {
    DIR_RIGHT,
    DIR_LEFT
} Direction;

static bool_t AtRightEnd(void)
{
    /* TODO */

    return true;
}

static bool_t AtLeftEnd(void)
{
    /* TODO */
    return true;
}

static void StandardDelay(void)
{
    _delay_ms(10);
}

static double GetDelay(uint32_t currentPosition, uint32_t targetPosition)
{
    return 10000.f;
}

static void Clock(void)
{
    /* Clock out, 30us minimum as per TB6560 specs */
    PORT_CLK |= (1 << PIN_CLK);
    _delay_us(30);
    PORT_CLK &= ~(1 << PIN_CLK);
}

static int8_t Step(Direction dir)
{
    uint8_t step = 0;

    if (dir == DIR_RIGHT && !AtRightEnd()) {
        step = -1;
        PORT_CW &= ~(1 << PIN_CW);
        Clock();
    } else if (dir == DIR_LEFT && !AtLeftEnd()) {
        step = 1;
        PORT_CW |= (1 << PIN_CW);
        Clock();
    }

    return step;
}

void Update(void)
{

}

static uint32_t FindMaxPosition(void)
{
    uint32_t maxPosition = 0;

    while (!AtRightEnd()) {
        Step(DIR_RIGHT);
        StandardDelay();
    }

    while (!AtRightEnd()) {
        Step(DIR_LEFT);
        StandardDelay();
        ++maxPosition;
    }

    return maxPosition;
}

static Direction GetDirection(uint32_t position, uint32_t target)
{
    return position < target ? DIR_LEFT : DIR_RIGHT;
}

static uint32_t GetTargetPosition(float target, uint32_t maxPosition)
{
    uint32_t center = maxPosition / 2;

    return center + center * target;
}

void InitTimer(void)
{
    /* Clock / 64  -> 4us / bit @ 16MHz */
    TCCR1B = (1 << CS11) | (1 << CS10);
}

/* Delay from last return of this function */
void SynchronizedDelay(uint32_t delay_us)
{
    static uint16_t prevCounter = 0;

    while ((TCNT1 - prevCounter) < delay_us / 4);

    prevCounter = TCNT1;
}


int main(void)
{
    Init();

    _delay_ms(1000);

    uint32_t maxPosition = FindMaxPosition();
    uint32_t position = 0;
    uint32_t targetPosition = 0;

    PORT_EN &= ~(1 << PIN_EN);

    float normalizedTarget = 0.f;


    targetPosition = GetTargetPosition(normalizedTarget, maxPosition);

    while (1)
    {
        if (position != targetPosition) {
            Direction dir = GetDirection(position, targetPosition);
            SynchronizedDelay(GetDelay(position, targetPosition));
            position = Step(dir);
        }
    }
}
