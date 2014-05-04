/*
 * steppercontorl.c
 *
 *  Created on: Nov 14, 2013
 *      Author: Mafioso
 */

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include <stdlib.h>

#include "uart.h"
#include "ramp.h"

#define DDR_EN      DDRC
#define PORT_EN     PORTC
#define PIN_EN      PC1

#define DDR_CW      DDRC
#define PORT_CW     PORTC
#define PIN_CW      PC2

#define DDR_CLK     DDRC
#define PORT_CLK    PORTC
#define PIN_CLK     PC3

#define DDR_SW_R    DDRC
#define PORT_SW_R   PORTC
#define INPORT_SW_R PINC
#define PIN_SW_R    PC4

#define DDR_SW_L    DDRC
#define PORT_SW_L   PORTC
#define INPORT_SW_L PINC
#define PIN_SW_L    PC5


void InitTimer(void)
{
    /* Clock / 64  -> 4us / bit @ 16MHz */
    TCCR1B = (1 << CS11) | (1 << CS10);
}

/* Synchronized delay
 *
 * @param Delay from last return of this function
 */
void SynchronizedDelay(uint32_t delay_us)
{
    static volatile uint16_t prevCounter = 0;

    delay_us /= 4;

    if (delay_us > 65500) {
        delay_us = 65500;
    }

    while (TCNT1 - prevCounter < delay_us) {}

    prevCounter = TCNT1;
}


typedef enum {
    DIR_RIGHT,
    DIR_LEFT
} Direction;

static bool_t AtRightEnd(void)
{
    return (INPORT_SW_R & (1 << PIN_SW_R)) != 0;
}

static bool_t AtLeftEnd(void)
{
    return (INPORT_SW_L & (1 << PIN_SW_L)) != 0;
}

static void StandardDelay(void)
{
    _delay_ms(10);
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
        step = 1;
        PORT_CW &= ~(1 << PIN_CW);
        Clock();
    } else if (dir == DIR_LEFT && !AtLeftEnd()) {
        step = -1;
        PORT_CW |= (1 << PIN_CW);
        Clock();
    }

    return step;
}

static uint32_t FindMaxPosition(void)
{
    uint32_t maxPosition = 0;

    while (Step(DIR_LEFT) != 0) {
        StandardDelay();
    }

    while (Step(DIR_RIGHT) != 0) {
        StandardDelay();
        ++maxPosition;
    }

    return maxPosition;
}

static Direction GetDirection(uint32_t position, uint32_t target)
{
    return position > target ? DIR_LEFT : DIR_RIGHT;
}

static uint32_t GetTargetPosition(float target, uint32_t maxPosition)
{
    uint32_t center = maxPosition / 2;

    return center + center * target;
}

/* Progmem data must be declared globally */
char helloString[] PROGMEM = "Stepper controller for raspi goalie!\r";

void PrintPosition(uint32_t position)
{
    char buffer [12];
    uint8_t pos = 12;

    do {
        buffer[--pos] = position % 10 + '0';
        position /= 10;
    } while (position > 0);

    UARTSend(&buffer[pos], 12 - pos);
    UARTSendChar('\r');
}

typedef enum {
    ENABLE_INACTIVE,
    ENABLE_ACTIVE
} EnableStatus;

void ControlEnable(EnableStatus status)
{
    if (status == ENABLE_ACTIVE) {
        PORT_EN &= ~(1 << PIN_EN);
    } else {
        PORT_EN |= (1 << PIN_EN);
    }
}

void Init(void)
{
    DDR_EN |= (1 << PIN_EN);
    DDR_CW |= (1 << PIN_CW);
    DDR_CLK |= (1 << PIN_CLK);

    /* Pull-ups */
    PORT_SW_R |= (1 << PIN_SW_R);
    PORT_SW_L |= (1 << PIN_SW_L);

    UARTInit();
    InitTimer();
}

int main(void)
{
    uint32_t maxPosition;
    uint32_t position = 0;
    uint32_t targetPosition = 0;
    float normalizedTarget = 0.f;

    char buffer[32];
    uint8_t bufferPos = 0;

    RampState ramp;
    RAMPSetParams(&ramp, 400, 600, 50, 1000);

    Init();
    _delay_ms(1000);

    UARTSendString_P(helloString);

    ControlEnable(ENABLE_ACTIVE);

    /* 5 steps of slack to each end */
    maxPosition = FindMaxPosition();
    position = maxPosition + 5;
    maxPosition -= 10;

    targetPosition = GetTargetPosition(normalizedTarget, maxPosition);

    while (1)
    {
        if (position != targetPosition) {
            Direction dir = GetDirection(position, targetPosition);
            SynchronizedDelay(1000000u / RAMPGetSpeed(&ramp, position, targetPosition));
            ControlEnable(ENABLE_ACTIVE);
            position += Step(dir);
        } else {
            /* No hold current to decrease power consumption */
            ControlEnable(ENABLE_INACTIVE);
        }

        if (UARTDataReady()) {
            buffer[bufferPos] = UARTRead();
            UARTSendChar(buffer[bufferPos]);

           if (buffer[bufferPos] == '\r') {

                float pos;
                buffer[bufferPos] = 0;
                pos = atof(buffer);

                if (pos > 1.f) {
                    pos = 1.f;
                } else if (pos < -1.f) {
                    pos = -1.f;
                }

                targetPosition = GetTargetPosition(pos, maxPosition);
                bufferPos = 0;

                PrintPosition(targetPosition);

            } else if (bufferPos < 31) {
                ++bufferPos;
            }
        }
    }
}

