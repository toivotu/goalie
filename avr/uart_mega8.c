/*
 * uart.c
 *
 *  Created on: 16.3.2009
 *      Author: Mafioso
 */

#include <avr/io.h>

#include "uart.h"

void UARTInit(void)
{
    /* 38400 bps @ 16MHz */
    UBRR0H = 0;
    UBRR0L = 51;

    UCSR0A |= (1 << U2X0);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}

void UARTSendByte(char dataByte)
{
    while (~UCSR0A & (1 << UDRE0));
    UDR0 = dataByte;
}

void UARTSend(const char* data, uint8_t dataCount)
{
    uint8_t i;

    for (i = 0; i < dataCount; ++i) {
        UARTSendByte(data[i]);
    }
}

void UARTSendString(const char* str)
{
    while (*str != 0) {
        UARTSendByte(*str++);
    }
}

/** Blocking read function */
uint8_t UARTRead(void)
{
    while(!(UCSR0A & (1 << RXC0)));

    return UDR0;
}

bool_t UARTDataReady(void)
{
    return (UCSR0A & (1 << RXC0)) != 0;
}
