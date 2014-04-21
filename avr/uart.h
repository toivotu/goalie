/*
 * uart.h
 *
 *  Created on: 16.3.2009
 *      Author: Mafioso
 */

#ifndef UART_H_
#define UART_H_

#include "types.h"

extern void UARTInit(void);

extern void UARTSendByte(char dataByte);
extern void UARTSend(const char* data, uint8_t dataCount);
extern void UARTSendString(const char* str);

extern uint8_t UARTRead(void);
extern bool_t UARTDataReady(void);

#endif /* UART_H_ */
