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
#include "types.h"

#define ABS(x) (x > 0 ? x : -x)

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

typedef enum {
    ENABLE_INACTIVE,
    ENABLE_ACTIVE
} EnableStatus;

typedef enum {
    DIR_RIGHT,
    DIR_LEFT,
    DIR_NONE
} Direction;

/* Progmem data must be declared globally */
char helloString[] PROGMEM = "Stepper controller for raspi goalie!\r";
char posStr[] PROGMEM = "Position ";
char accStr[] PROGMEM = "Acceleration ";
char decStr[] PROGMEM = "Deceleration ";
char maxStr[] PROGMEM = "Max speed ";
char spdStr[] PROGMEM = "Speed ";


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
    _delay_ms(500);
    while (Step(DIR_RIGHT) != 0) {
        StandardDelay();
        ++maxPosition;
    }

    return maxPosition;
}

static Direction GetDirection(int32_t speed)
{
    return speed > 0 ? DIR_RIGHT : DIR_LEFT;
}

static uint32_t GetTargetPosition(float target, uint32_t maxPosition)
{
    uint32_t center = maxPosition / 2;

    return center + center * target;
}

void PrintValue(int32_t position)
{
    char buffer [12];
    uint8_t pos = 12;

    if (position < 0) {
        UARTSendChar('-');
    }

    do {
        buffer[--pos] = position % 10 + '0';
        position /= 10;
    } while (position > 0);


    UARTSend(&buffer[pos], 12 - pos);
    UARTSendChar('\r');
}

void SetEnableState(EnableStatus status)
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

uint32_t UpdatePosition(uint32_t position, uint32_t targetPosition, RampState* ramp)
{
    if (position != targetPosition) {
        int32_t speed = RAMPGetSpeed(ramp, position, targetPosition);
        Direction dir = GetDirection(speed);
        SynchronizedDelay(1000000u / ABS(speed));

        SetEnableState(ENABLE_ACTIVE);
        position += Step(dir);
    } else {
        /* No hold current to decrease power consumption */
        SetEnableState(ENABLE_INACTIVE);
    }

    return position;
}

typedef struct {
    enum {
        COMMAND_POSITION,
        COMMAND_ACCELERATION,
        COMMAND_DECELERATION,
        COMMAND_MAX_SPEED,
        COMMAND_GET_POSITION,
        COMMAND_GET_ACCELERATION,
        COMMAND_GET_DECELERATION,
        COMMAND_GET_MAX_SPEED,
        COMMAND_GET_SPEED,
        COMMAND_NONE
    } command;
    float arg;
} Command;

bool_t CommandReady(Command* command)
{
    static char buffer[32];
    static uint8_t bufferPos = 0;

    command->command = COMMAND_NONE;

    if (UARTDataReady()) {
        buffer[bufferPos] = UARTRead();
        UARTSendChar(buffer[bufferPos]);

       if (buffer[bufferPos] == '\r') {

           if (buffer[0] == 'p') {
               if (buffer[1] == '?') {
                   command->command = COMMAND_GET_POSITION;
               } else {
                   command->command = COMMAND_POSITION;
               }
           } else if (buffer[0] == 'a') {
               if (buffer[1] == '?') {
                   command->command = COMMAND_GET_ACCELERATION;
               } else {
                   command->command = COMMAND_ACCELERATION;
               }
           } else if (buffer[0] == 'd') {
               if (buffer[1] == '?') {
                   command->command = COMMAND_GET_DECELERATION;
               } else {
                   command->command = COMMAND_DECELERATION;
               }
           } else if (buffer[0] == 'm') {
               if (buffer[1] == '?') {
                   command->command = COMMAND_GET_MAX_SPEED;
               } else {
                   command->command = COMMAND_MAX_SPEED;
               }
           }

           else if (buffer[0] == 's') {
              if (buffer[1] == '?') {
                  command->command = COMMAND_GET_SPEED;
              }
          }

           buffer[bufferPos] = 0;
           command->arg = atof(&buffer[1]);

           bufferPos = 0;

        } else if (bufferPos < 31) {
            ++bufferPos;
        }
    }

    return command->command != COMMAND_NONE;
}

float LimitValue(float value, float min, float max)
{
    return value < min ? min :
            value > max ? max :
                value;
}

int main(void)
{
    uint32_t maxPosition;
    uint32_t position = 0;
    uint32_t targetPosition = 0;

    RampState ramp;
    RAMPSetParams(&ramp, 400, 600, 50, 1000);
    RAMPInit(&ramp);

    Init();

    UARTSendString_P(helloString);

    SetEnableState(ENABLE_ACTIVE);

    /* 5 steps of slack to each end */
    maxPosition = FindMaxPosition();
    position = maxPosition + 5;
    maxPosition = maxPosition > 10 ? maxPosition - 10 : maxPosition;

    targetPosition = GetTargetPosition(0.f, maxPosition);

    while (1)
    {
        Command command;
        position = UpdatePosition(position, targetPosition, &ramp);

        if (CommandReady(&command)) {
            switch (command.command)
            {
            case COMMAND_POSITION:
                command.arg = LimitValue(command.arg, -1.f, 1.f);
                targetPosition = GetTargetPosition(command.arg, maxPosition);
                UARTSendString_P(posStr);
                PrintValue(targetPosition);
                break;

            case COMMAND_ACCELERATION:
                command.arg = LimitValue(command.arg, 0, 1000);
                RAMPSetAcceleration(&ramp, command.arg);
                UARTSendString_P(accStr);
                PrintValue(command.arg);
                break;

            case COMMAND_DECELERATION:
                command.arg = LimitValue(command.arg, 0, 1000);
                RAMPSetDeceleration(&ramp, command.arg);
                UARTSendString_P(decStr);
                PrintValue(command.arg);
                break;

            case COMMAND_MAX_SPEED:
                command.arg = LimitValue(command.arg, 0, 5000);
                RAMPSetMaxSpeed(&ramp, command.arg);
                UARTSendString_P(maxStr);
                PrintValue(command.arg);
                break;

            case COMMAND_GET_POSITION:
                UARTSendString_P(posStr);
                PrintValue(position);
                break;

            case COMMAND_GET_ACCELERATION:
                UARTSendString_P(accStr);
                PrintValue(ramp.acceleration);
                break;

            case COMMAND_GET_DECELERATION:
                UARTSendString_P(decStr);
                PrintValue(ramp.deceleration);
                break;

            case COMMAND_GET_MAX_SPEED:
                UARTSendString_P(maxStr);
                PrintValue(ramp.maxSpeed);
                break;

            case COMMAND_GET_SPEED:
                UARTSendString_P(spdStr);
                PrintValue(ramp.speed);
                break;

            default:
                break;
            }
        }

    }
}

