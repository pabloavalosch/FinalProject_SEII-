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
#include "task.h"
#include "semphr.h"
#include "timers.h"

/* Demo includes. */
#include "UART.h"
#include "QueueMine.h"
#include "Strings.h"
#include "FTM.h"
#include "NVIC.h"

// Queue to hold random numbers
QueueHandle_t dataQueue;

// Semaphore to protect shared resources
SemaphoreHandle_t sharedSemaphore;

// Software timer handle
TimerHandle_t xOneShotTimer;

uint8_t * str1 = {"Received Number: \r\n"};
uint8_t * str3 = {"Timer Callback: Performing Periodic Task\r\n"};

// Timer callback function
void prvOneShotTimerCallback(TimerHandle_t xTimer)
{
    // Send data to the queue when the timer expires
    int data = 42; // Example data to be sent
    UART_SendString(str3);
    if (xQueueSend(dataQueue, &data, 0) != pdPASS) {
        // Handle queue full error
    }
}

// ISR Simulating an External Hardware Interrupt
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
    while (1) {
        // Wait for the semaphore to be available
        if (xSemaphoreTake(sharedSemaphore, portMAX_DELAY) == pdTRUE) {
            // Receive data from the queue
            if (xQueueReceive(dataQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
                // Process received data
                UART_SendString(str1);
                UART_SendString(tick_to_ascii(receivedData));
            }
        }
    }
}

int main(void)
{

	Queue_type myQueue;
	UART_Initialization();
	Queue_Init(myQueue);

    // Initialize the hardware and other peripherals
	FTM_Initialization();
	// Create a software timer with a 2-second period, using pdFALSE for one-shot timer
    xOneShotTimer = xTimerCreate("One-Shot_Timer",
    							 pdMS_TO_TICKS(2000),
								 pdFALSE,
								 (void *)0,
								 prvOneShotTimerCallback);

    // Create a queue to hold data
    dataQueue = xQueueCreate(10, sizeof(int)); // Queue can hold up to 10 integers

    // Create a semaphore for synchronization
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

