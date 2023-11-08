/*
 * UART.c
 *
 *  Created on: 29 ago. 2023
 *      Author: pablo
 */

#include <QueueMine.h>
#include "UART.h"
#include "NVIC.h"

#define PORTB_MASK_ALT3_UART_TX  (1 << 8) | (1 << 9)
#define UART_BAUD_RATE	9600U

uint8_t * handler;
Queue_type myQueue;

void BOARD_UART_IRQ_HANDLER(void)
{
	/* Read UART0->S1 to clear the flags. */
	(void)UART0->S1;
	UART0->D = *(handler++);

	if('\0' == *handler)
	{
		// Dequeue the string that was already sent.
		// Dequeue the string from the queue of pointers
		myQueue.queue[myQueue.front] = 0;

		// If front position is equal to the rear position, disable interrupts, no more strings
		if(myQueue.front == myQueue.rear || 0 == handler)
		{
			// Queue_is_empty, disable UART TX and Interrupts
			UART0->C2 &= ~((1 << 6) | (1 << 7) | (1 << 3)); //TRANSMISSION COMPLETE REQUESTS DISABLED
			// Restart front and rear to point to the start of the queue.
			myQueue.front = 0;
			myQueue.rear = 0;
		}
		else
		{
			// Assign to handler the next string in the front from Queue
			handler = myQueue.queue[++myQueue.front];
		}
	}
}

uint8_t UART_SendString(uint8_t * pStr)
{
	if(myQueue.front == myQueue.rear)
	{
		if( Queue_Enqueue(&myQueue, pStr) && 0 == *handler )
		{
			handler = myQueue.queue[myQueue.front];
		}

	    UART0->C2 |= UART_C2_TE_MASK; // ENABLE UART0 TRANSMITTER UNTIL HANDLER HAS A MESSAGE TO BE SENT
		NVIC_enable_interrupt_and_priority(UART0_RX_TX_IRQ, PRIORITY_1); // ENABLE NVIC INTERRUPT
		UART0->C2 |= (1 << 6 | 1 << 7); // ENABLE UART0 TRANSMITTER INTERRUPTS
	}
	else
	{
		if( Queue_Enqueue(&myQueue, pStr) )
		{
			// If Enqueued was successful, is not full
		}
		else
		{
			// Do a quick sweep to see where the new string can be allocated
			for( uint8_t i = 0; i < QUEUE_SIZE; i++ )
			{
				if( 0 == myQueue.queue[i] )
				{
					myQueue.queue[i] = pStr;
				}
			}
		}
	}

	return myQueue.front;
}


uint8_t UART_SendByte(uint8_t byte)
{
	uint8_t* newString = (uint8_t*)malloc(4 * sizeof(uint8_t)); // Allocate memory for the byte and null terminator
	newString[0] = byte;
	newString[1] = '\r';
	newString[2] = '\n';
	newString[3] = '\0'; // Null-terminate the string

	if(myQueue.front == myQueue.rear)
	{
		if( Queue_Enqueue(&myQueue, newString) && 0 == *handler )
		{
			handler = myQueue.queue[myQueue.front];
		}

		UART0->C2 |= UART_C2_TE_MASK; // ENABLE UART0 TRANSMITTER UNTIL HANDLER HAS A MESSAGE TO BE SENT
		NVIC_enable_interrupt_and_priority(UART0_RX_TX_IRQ, PRIORITY_1); // ENABLE NVIC INTERRUPT
		UART0->C2 |= (1 << 6 | 1 << 7); // ENABLE UART0 TRANSMITTER INTERRUPTS
	}
	else
	{
		if( Queue_Enqueue(&myQueue, newString) )
		{
			// If Enqueued was successful, is not full
		}
		else
		{
			// Do a quick sweep to see where the new string can be allocated
			for( uint8_t i = 0; i < QUEUE_SIZE; i++ )
			{
				if( 0 == myQueue.queue[i] )
				{
					myQueue.queue[i] = newString;
				}
			}
		}
	}

	return myQueue.front;
}

uint8_t UART_Initialization(void)
{
    status_t status;
    uint32_t sbr = 0;

	SIM->SCGC5 |= (1 << 10); //ENABLE CLOCK FOR PORTB
	SIM->SCGC4 |= (1 << 10); //ENABLE CLOCK FOR UART0
	PORTB->PCR[17] |= PORTB_MASK_ALT3_UART_TX; //CONFIG MUX OF PIN 17 TO BE AS UART0 TX

	SIM->SOPT5 = ((SIM->SOPT5 &
				   /* Mask bits to zero which are setting */
				   (~(SIM_SOPT5_UART0TXSRC_MASK)))

				  /* UART 0 transmit data source select: UART0_TX pin. */
				  | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX));

	CLOCK_SetSimSafeDivs();

	/****** Setup of BaudRate ******/

    /* Calculate the baud rate modulo divisor, sbr*/
    sbr = UART_CLK_FREQ / (UART_BAUD_RATE * 16U);

    /* Write the sbr value to the BDH and BDL registers*/
    UART0->BDH = (UART0->BDH & ~(uint8_t)UART_BDH_SBR_MASK) | (uint8_t)(sbr >> 8);
    UART0->BDL = (uint8_t)sbr;

	status = true;

    return status;
}
