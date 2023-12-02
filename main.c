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
/* TODO: insert other include files here. */

//#include "UART.h"
//#include "QueueMine.h"
//#include "Strings.h"
//#include "FTM.h"
#include "NVIC.h"
#include "GPIO.h"
/* TODO: insert other definitions and declarations here. */

// Define the virtual UART pin
#define UART_TX_PIN  4U

// Define the Ticks that Auto-Reload Timer is going to count for timeout
#define mainAUTO_RELOAD_TIMER_PERIOD				( (TickType_t) 100 ) // Ticks (as tick rate is 30000 ticks in 1 second, and I want 0.003175 sec.
#define NO_TICKS_TO_WAIT							(0U)

// Global variable to store the value to be transmitted
uint8_t transmitValue = 50;

// Variable to take the current count of bit where the UART transmission goes
uint8_t bit_count = 0;

// Semaphore to protect shared resources between AutoReloadTimer and uartTransmissionTask
SemaphoreHandle_t sharedSemaphore;

// Creation of variable to handle the timer
TimerHandle_t xAutoReloadTimer;

/*
 * @brief   This API generates the virtual UART transmission in different
 * 			times and for 4 different UARTs.
 */


void prvAutoReloadTimerCallback(TimerHandle_t xTimer)
{
	static uint8_t ucLocalTickCount = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(9 >= ucLocalTickCount)
	{
		// Release the semaphore to notify a task that time bit has passed
		xSemaphoreGiveFromISR(sharedSemaphore, &xHigherPriorityTaskWoken);
	}
	else
	{
		// xAutoReloadTimer is deleted as it is not needed anymore until the next Start bit
		xTimerDelete(xAutoReloadTimer, NO_TICKS_TO_WAIT);
	}

	 ucLocalTickCount++;

    // Request a context switch if required
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}


// Bit-banging transmission task
void uartTransmissionTask(void *pvParameters)
{
    static TickType_t lastWakeTime = 0;
    static uint8_t bit = 0;
    TickType_t currentWakeTime;

    currentWakeTime = xTaskGetTickCount();

    while (1)
    {
        // Bit-banging implementation here
        // Transmit the bits of the current transmitValue

        /* Transmit data bits (LSB first) each time the Auto-Reload Timer callback give the semaphore from ISR. */

    	// Wait for the semaphore to be available
		if (xSemaphoreTake(sharedSemaphore, portMAX_DELAY) == pdTRUE)
		{
	    	// If lastWakeTime is 0, is because it is the start bit and we forced to do the fallingEdge one time only
	    	if(0 == lastWakeTime)
	    	{
				// Transmit a start bit and wait
	    		GPIO_PinWrite(GPIOB, UART_TX_PIN, 0U);
				lastWakeTime = currentWakeTime;
	    	}
	    	else
	    	{
		    	// Check if 8 data bits had passed (bit_count from 0 to 8) to send the stop bit
				if(8 <= bit_count)
				{
					// Transmit a stop bit
					GPIO_PinWrite(GPIOB, UART_TX_PIN, 1U);
					// Restart the bit_count to be ready for next frame to be sent
					bit_count = 0;
					lastWakeTime = 0;
					// Suspend tasks to be waiting for the next start bit that wanted to be sent
					vTaskSuspendAll();
					// Update transmitValue to send another bit frame
				    // Generate a random value between 0 and 100 (inclusive)
				    transmitValue = rand() % 101;
				}
				else
				{
					// Check if current bit is 1 or 0 to be assigned to bit variable and then send it
					bit = (transmitValue >> bit_count) & 0x01;

					// Transmit the bit (bit-banging)
					GPIO_PinWrite(GPIOB, UART_TX_PIN, bit);
					// Increment bit_count each time a bit is being sent
					bit_count++;
				}
	    	}


		}
    }
}

//void updateTransmitValue(TimerHandle_t xTimer) {
//    // Generate a random value between 0 and 100 (inclusive)
//    transmitValue = rand() % 101;
//}

int main()
{
	GPIO_Initialization();

    // Create a timer for updating the transmit value
	/* Re-configure timer with the time bit to do the samples each timeout*/
	xAutoReloadTimer = xTimerCreate("Auto-Reload_Timer",
									mainAUTO_RELOAD_TIMER_PERIOD,
									pdTRUE, // pdTRUE to be an Auto-Reload Timer
									(void *)0,
									prvAutoReloadTimerCallback);

    if (xAutoReloadTimer != NULL)
    {
    	/* Start the timer once it is aligned to the bit rate from UART */
        xTimerStart(xAutoReloadTimer, NO_TICKS_TO_WAIT);
    }

    // Create a semaphore for synchronization between Auto-Reload Timer and uartTransmissionTask
    sharedSemaphore = xSemaphoreCreateBinary();

    // Create the UART transmission task
    xTaskCreate(uartTransmissionTask, "UARTTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // The code should never reach here
    while (1) {
        // Main application loop
        // Handle other tasks or events
    }

    return 0;
}

