/*
 * UART.h
 *
 *  Created on: 29 ago. 2023
 *      Author: pablo
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "board.h"
#include "pin_mux.h"
#include "fsl_uart.h"

#define UART_CLK_FREQ   CLOCK_GetFreq(SYS_CLK)

uint8_t UART_SendString(uint8_t * pStr);
uint8_t UART_SendByte(uint8_t byte);
uint8_t UART_Initialization(void);

#endif /* UART_H_ */
