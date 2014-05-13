/*
 * uart.c
 *
 *  Created on: 16.3.2009
 *      Author: Mafioso
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "uart.h"

#define BUFFER_SIZE 64

static char rxBuffer[BUFFER_SIZE];
static uint8_t rxHead = 0;
static uint8_t rxTail = 0;

static char txBuffer[BUFFER_SIZE];
static uint8_t txHead = 0;
static uint8_t txTail = 0;

SIGNAL(USART_RX_vect)
{
    /* No overrun detection */
    rxBuffer[rxHead] = UDR0;
    rxHead += 1;
    rxHead %= BUFFER_SIZE;
}

SIGNAL(USART_TX_vect)
{
    /* No overrun detection */
    if (txHead != txTail) {
        UDR0 = txBuffer[txTail];
        txTail += 1;
        txTail %= BUFFER_SIZE;
    }
}

void UARTInit(void)
{
    /* 38400 bps @ 16MHz
    UBRR0H = 0;
    UBRR0L = 51; */

    /* 57600 bps @ 16MHz */
    UBRR0H = 0;
    UBRR0L = 34;

    UCSR0A |= (1 << U2X0);
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXCIE0) | (1 << TXEN0);
}

void UARTSendChar(char dataByte)
{
    cli();
    if (txHead == txTail && (UCSR0A & (1 << UDRE0)) != 0) {
        UDR0 = dataByte;
    } else {
        txBuffer[txHead] = dataByte;
        txHead += 1;
        txHead %= BUFFER_SIZE;
    }
    sei();
}

void UARTSend(const char* data, uint8_t dataCount)
{
    uint8_t i;

    for (i = 0; i < dataCount; ++i) {
        UARTSendChar(data[i]);
    }
}

void UARTSendString_P(const char* str)
{
    char dataByte = pgm_read_byte(str++);

    while (dataByte) {
        UARTSendChar(dataByte);
        dataByte = pgm_read_byte(str++);
    }
}

void UARTSendString(const char* str)
{
    while (*str != 0) {
        UARTSendChar(*str++);
    }
}

uint8_t UARTRead(void)
{
    char data = 0;

    if (rxHead != rxTail) {
        data = rxBuffer[rxTail];
        rxTail += 1;
        rxTail %= BUFFER_SIZE;
    }

    return data;
}

bool_t UARTDataReady(void)
{
    return rxHead != rxTail;
}
