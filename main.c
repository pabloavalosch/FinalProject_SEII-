/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    main.c
 * @brief   Application entry point.
 */
/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

/* Demo includes. */
#include "UART.h"
#include "QueueMine.h"
#include "Strings.h"
#include "FTM.h"
#include "NVIC.h"

//#define mainONE_SHOT_TIMER_PERIOD		( pdMS_TO_TICKS( 51.82 / 1000 ) )
//#define mainAUTO_RELOAD_TIMER_PERIOD	( pdMS_TO_TICKS( 103.64 / 1000 ) )

#define mainONE_SHOT_TIMER_PERIOD		( pdMS_TO_TICKS( 500 ) )
#define mainAUTO_RELOAD_TIMER_PERIOD	( pdMS_TO_TICKS( 1000 ) )


// Queue to hold random numbers
QueueHandle_t dataQueue;

// Semaphore to protect shared resources
SemaphoreHandle_t sharedSemaphore;

// Software timer handle
TimerHandle_t xOneShotTimer;
TimerHandle_t xAutoReloadTimer;

// Variable to take the bits received from UART
uint16_t bit_count = 0;
uint16_t current_byte = 0;
uint16_t read_bit;

uint8_t * str1 = {"Received Number: \r\n"};
uint8_t * str2 = {"Auto-Reload Timer Executing\r\n"};
uint8_t * str3 = {"One-Shot Timer Executed, about to start a new timer\r\n"};
uint8_t * str4 = {"bitReceived "};

// Timer callback function
void prvOneShotTimerCallback(TimerHandle_t xTimer)
{
	/* Configure PORTD1 as GPIO as input and Alt. 1 to do bit reading */
	PORTD->PCR[0] = (kPORT_MuxAsGpio << 8) | (kPORT_PullUp);
	GPIOD->PDDR |= 0x01; /* Place a 1 on bit 0 to behave as output */
//	GPIOD->PTOR |= 0x01; /* Place a 1 on bit 0 to toggle that GPIO bit */
    // Send data to the queue when the timer expires
//    int data = 42; // Example data to be sent
//    UART_SendString(str3);
//    if (xQueueSend(dataQueue, &data, 0) != pdPASS) {
//        // Handle queue full error
//    }

}

void prvAutoReloadTimerCallback(TimerHandle_t xTimer)
{
//	uint8_t read_bit;
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    if (xTimerStopFromISR(xOneShotTimer, &xHigherPriorityTaskWoken) != pdPASS) {
//        // Handle error if timer start fails
//    }
	/* Configure PORTD1 as GPIO as input and Alt. 1 to do bit reading */
//	PORTD->PCR[1] = kPORT_MuxAsGpio;
//	GPIOD->PDDR &= ~(1 << 1); /* Place a 0 on bit 1 to behave as input */
//	read_bit = GPIO_PinRead(GPIOD, 1U); /* Read value from GPIO */;
//	UART_SendByte(read_bit);
	GPIOD->PTOR |= 0x01; /* Place a 1 on bit 0 to toggle that GPIO bit */

//	UART_SendString(str4);
}


// ISR from External Hardware Interrupt
void FTM_INPUT_CAPTURE_HANDLER(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	/* Clear interrupt flag.*/
	FTM_ClearStatusFlags(BOARD_FTM_BASEADDR, FTM_CHANNEL_FLAG);

    // Start the software timer from ISR (use xTimerStartFromISR)
    if (xTimerStartFromISR(xOneShotTimer, &xHigherPriorityTaskWoken) != pdPASS) {
        // Handle error if timer start fails
    }

    // Release the semaphore to notify a task
    xSemaphoreGiveFromISR(sharedSemaphore, &xHigherPriorityTaskWoken);

    // Request a context switch if required
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task that processes data from the queue
void uartTask(void *pvParameters)
{
    int receivedData;
    while (1)
    {
        // Wait for the semaphore to be available
        if (xSemaphoreTake(sharedSemaphore, portMAX_DELAY) == pdTRUE)
        {
        	// Create and start a new timer
			xAutoReloadTimer = xTimerCreate("Auto-Reload_Timer",
											mainAUTO_RELOAD_TIMER_PERIOD,
											pdTRUE,
											(void *)0,
											prvAutoReloadTimerCallback);
			if (xAutoReloadTimer != NULL)
			{
				if (xTimerStart(xAutoReloadTimer, 0) != pdPASS)
				{
					// Handle timer start error
				}
			}
//            // Receive data from the queue
//            if (xQueueReceive(dataQueue, &receivedData, portMAX_DELAY) == pdTRUE)
//            {
//                // Process received data
//                UART_SendString(str1);
//                UART_SendString(tick_to_ascii(receivedData));
//            }

        }
        else
        {
        	//	Estamos en la lectura del byte
			if (bit_count  > 0 && bit_count <= 9)
			{
				 // recorremos a la izquierda para agregar el proximo bit
				 current_byte <<= 1;
				 // agregamos el bit
				 current_byte |= read_bit;
			}
			// START bit
			else if (bit_count == 0 && read_bit == 0)
			{
				bit_count += 1;
			}
			// STOP bit, idealmente checar si el bit leido es 1
			else if (bit_count == 9 && read_bit == 1)
			{
				bit_count = 0;
			}
			else
			{

			}
        }

	}
}

int main(void)
{
	uint8_t byte = 0x62;
	Queue_type myQueue;
	UART_Initialization();
	Queue_Init(myQueue);

	UART_SendByte(byte);
    // Initialize the hardware and other peripherals
	FTM_Initialization();
	// Create a software timer with a 2-second period, using pdFALSE for one-shot timer
    xOneShotTimer = xTimerCreate("One-Shot_Timer",
    							 mainONE_SHOT_TIMER_PERIOD,
								 pdFALSE,
								 NULL,
								 prvOneShotTimerCallback);

    // Create a queue to hold data
    dataQueue = xQueueCreate(10, sizeof(int)); // Queue can hold up to 10 integers

    // Create a semaphore for synchronization of input capture and one-shot timer
    sharedSemaphore = xSemaphoreCreateBinary();

    // Check if the timer, queue, and semaphore were created successfully
    if (xOneShotTimer != NULL && dataQueue != NULL && sharedSemaphore != NULL) {
        // Configure and enable the external interrupt
        // For example, configure GPIO interrupt, NVIC settings, etc.
    	NVIC_enable_interrupt_and_priority(FTM3_IRQ, PRIORITY_2);
        // Create data processing task
        xTaskCreate(uartTask, "UartTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    }

	// Start the FreeRTOS scheduler
	vTaskStartScheduler();

	/* The following line should never be reached because vTaskStartScheduler()
	will only return if there was not enough FreeRTOS heap memory available to
	create the Idle and (if configured) Timer tasks.  Heap management, and
	techniques for trapping heap exhaustion, are described in the book text. */
    for (;;)
    {

    }

    return 0;
}

