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
#include "PWM.h"

//#define mainONE_SHOT_TIMER_PERIOD		( pdMS_TO_TICKS( 51.82 / 1000 ) )
//#define mainAUTO_RELOAD_TIMER_PERIOD	( pdMS_TO_TICKS( 103.64 / 1000 ) )

//#define mainONE_SHOT_TIMER_PERIOD					( (TickType_t) 144 )
#define TICK_RATE_US								( 33.33333333333333 )
#define mainAUTO_RELOAD_TIMER_PERIOD				( (TickType_t) 97 ) // Ticks (as tick rate is 30000 ticks in 1 second, and I want 0.003175 sec.
#define mainAUTO_RELOAD_TIMER_PERIOD_FOR_START_BIT	( (TickType_t) 48 ) // Ticks (as tick rate is 30000 ticks in 1 second, and I want 0.0015875 sec.
#define NO_TICKS_TO_WAIT							(0U)
#define MAX_DUTYCYCLE								(100U)
#define PWM_OUTPUT_PERIOD_TICKS					 	( PWM_OUTPUT_PERIOD_US / TICK_RATE_US )

#define VIRTUAL_UART0_PIN							(1U)

//#define VIRTUAL_UART1_PIN							(1U)
//#define VIRTUAL_UART2_PIN							(1U)
//#define VIRTUAL_UART3_PIN							(1U)

// Queue to hold random numbers
QueueHandle_t dataQueue;

// Semaphore to protect shared resources
SemaphoreHandle_t sharedSemaphore;
SemaphoreHandle_t timerSemaphore;
SemaphoreHandle_t xMutex;

// Software timer handle
TimerHandle_t xOneShotTimer;
TimerHandle_t xAutoReloadTimer;

// Variable to take the bits received from UART
uint8_t bit_count = 0;
uint8_t current_byte = 0;
uint8_t pin_value_read;

enum{
	UART_1,
	UART_2,
	UART_3,
	UART_4
};

// Define a structure to hold PWM data and its identifier
typedef struct {
    uint8_t updatedDutycycle[4]; // Array to save the received Duty Cycle from different UARTs
    uint8_t uart_num;  // Identifier for the PWM output
    uint8_t onTimeinTicks[4];
} PwmData_t;

PwmData_t g_uartRxPwmData;

uint8_t * str1 = {"Input Capture Current Tick: \r\n"};
uint8_t * str2 = {"Auto-Reload Timer Executing\r\n"};
uint8_t * str3 = {"One-Shot Timer Executed, about to start a new timer\r\n"};
uint8_t * str4 = {"bitReceived \r\n"};

// Timer callback function

void prvAutoReloadTimerCallback(TimerHandle_t xTimer)
{
	static uint8_t ucLocalTickCount = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Do the toogle of pin to know when this callback is entering (just for debugging) */
	GPIOC->PTOR |= (1 << 1); /* Place a 1 on bit 1 to toggle that GPIO bit */


    if(!ucLocalTickCount)
    {
    	// We delete timer, to create it again with new configuration timeout period
    	xTimerDelete(xAutoReloadTimer, NO_TICKS_TO_WAIT);
    	// Release the semaphore to notify a task that half of bit time has passed
    	xSemaphoreGiveFromISR(sharedSemaphore, &xHigherPriorityTaskWoken);
    }
    else if(9 >= ucLocalTickCount)
    {
    	// Release the semaphore of timer to notify bitBangingTask that needs to capture bit
    	xSemaphoreGiveFromISR(timerSemaphore, &xHigherPriorityTaskWoken);
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


// ISR from External Hardware Interrupt
void FTM_INPUT_CAPTURE_HANDLER(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	/* Clear interrupt flag.*/
	FTM_ClearStatusFlags(BOARD_FTM_BASEADDR, FTM_CHANNEL_FLAG);

    // Start the software timer from ISR (use xTimerStartFromISR)
    if (xTimerStartFromISR(xAutoReloadTimer, &xHigherPriorityTaskWoken) != pdPASS) {
        // Handle error if timer start fails
    }

    DisableIRQ(FTM_INTERRUPT_NUMBER);


}

// Task that processes timer creation for second time with the time bit, after this was deleted
void uartTask(void *pvParameters)
{
    while (1)
    {
        // Wait for the semaphore to be available
        if (xSemaphoreTake(sharedSemaphore, portMAX_DELAY) == pdTRUE)
        {
        	/* Re-configure timer with the time bit to do the samples each timeout*/
        	xAutoReloadTimer = xTimerCreate("Auto-Reload_Timer",
        									mainAUTO_RELOAD_TIMER_PERIOD,
        									pdTRUE,
        									(void *)0,
        									prvAutoReloadTimerCallback);
        	/* Start the timer once it is aligned to the bit rate from UART*/
        	xTimerStart(xAutoReloadTimer, NO_TICKS_TO_WAIT);

        }

	}
}

void showPwmSignalTask(void *pvParameters)
{
	TickType_t xLastExecutionTime;

	/* Initialize the variable used by the call to vTaskDelayUntil(). */
	xLastExecutionTime = xTaskGetTickCount();

	while(1)
	{
		/* Check if is different to 0, because if dutyCycle is 0, only low state has to be activated. */
		if(0 != g_uartRxPwmData.updatedDutycycle[UART_1])
		{
			// Write a 1 to pin to generate PWM signal
			GPIO_PinWrite(GPIOB, PWM0_PIN, 1U);
			/* This is a periodic task. Block until it is time to run again. The task
			will execute after onTime has passed. */
			vTaskDelayUntil( &xLastExecutionTime, g_uartRxPwmData.onTimeinTicks[UART_1] );
		}

		/* Check if is different to 100, because if dutyCycle is 100, only high state has to be activated. */
		if(100 != g_uartRxPwmData.updatedDutycycle[UART_1])
		{
			// Write a 0 to pin for offTime
			GPIO_PinWrite(GPIOB, PWM0_PIN, 0U);
			// Wait for the rest of the period the offTime
			vTaskDelayUntil( &xLastExecutionTime, (PWM_OUTPUT_PERIOD_TICKS - g_uartRxPwmData.onTimeinTicks[UART_1]) );
		}
	}
}

void pwmTask(void * pvParameters)
{
	// pvParameters is the dataQueue
	QueueHandle_t pwmLocalDataQueue = (QueueHandle_t)pvParameters;
	PwmData_t uartRxLocalPwmData;

	while(1)
	{
		// Wait for data in the queue
		if (xQueueReceive(pwmLocalDataQueue, &uartRxLocalPwmData, portMAX_DELAY))
		{
			// Generate PWM signal based on pwmData.pwmValue
			//

			// Identify the PWM output using pwmData.pwmId
			// ...

			switch(uartRxLocalPwmData.uart_num)
			{
				case UART_1:
					// Wait for the on-time based on the duty cycle
					g_uartRxPwmData.updatedDutycycle[UART_1] = uartRxLocalPwmData.updatedDutycycle[UART_1];
					g_uartRxPwmData.onTimeinTicks[UART_1] = ( (uartRxLocalPwmData.updatedDutycycle[UART_1] * PWM_OUTPUT_PERIOD_US) / MAX_DUTYCYCLE ) / TICK_RATE_US;
//					g_uartRxPwmData.offTimeinTicks[UART_1] = ( ( (100 - uartRxLocalPwmData.updatedDutycycle[UART_1]) * PWM_OUTPUT_PERIOD_US) / MAX_DUTYCYCLE ) / TICK_RATE_US;
					// Create task that will be handling the PWM output with the updatedDutycycle received
					xTaskCreate(showPwmSignalTask, "ShowPwmSignalTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
					break;
				case UART_2:
					break;
				case UART_3:
					break;
				case UART_4:
					break;
			}
		}


	}
}

void bitBangingTask(void * pvParameters)
{
	 // Create an instance of the structure and initialize its members
	PwmData_t *pwmLocalDataStruct = (PwmData_t *)pvPortMalloc(sizeof(pwmLocalDataStruct));
	while(1)
	{
        if(xSemaphoreTake(timerSemaphore, mainAUTO_RELOAD_TIMER_PERIOD) == pdTRUE)
        {
        	// We read the bit from the GPIO
        	pin_value_read = GPIO_PinRead(GPIOD, VIRTUAL_UART0_PIN);

        	/* If it is the first bit to be read bit_count is 0*/
			if (0 <= bit_count && 8 > bit_count)
			{
				 // Bit is shift the bit_count number to make sure current_byte is being armed*/
				 current_byte |= (pin_value_read << bit_count);
				 // Increase the bit_count
				 bit_count++;
			}
			// STOP bit has to be a 1, so we check it to be sure
			else if (8 == bit_count && 1 == pin_value_read)
			{
				// bit_count++ to avoid entering to these conditions again
				bit_count++;

				g_uartRxPwmData.updatedDutycycle[UART_1] = current_byte;
				g_uartRxPwmData.uart_num = UART_1;

				// Send the updatedDutycycle to a queue
				xQueueSend(dataQueue, &g_uartRxPwmData, portMAX_DELAY);

				// Create task that will be handling the PWM output with the updatedDutycycle received
				xTaskCreate(pwmTask, "PwmTask", configMINIMAL_STACK_SIZE, dataQueue, tskIDLE_PRIORITY + 2, NULL);

				// Enable FTM input capture interrupt again to wait for the next UART frame
				EnableIRQ(FTM_INTERRUPT_NUMBER);

			}
			else
			{

			}

        }
        else
        {

        }

	}
}


int main(void)
{
	uint8_t * startString = {"Starting..\r\n"};
	Queue_type myQueue;
	UART_Initialization();
	Queue_Init(myQueue);

	/**** Configure pin from PortC to toogle and debugging *****/
    CLOCK_EnableClock(kCLOCK_PortC);
	/* Configure PORTC1 as GPIO as output and Alt. 1 */
	PORTC->PCR[1] |= (kPORT_MuxAsGpio << 8) | (kPORT_FastSlewRate << 2);
	GPIOC->PDOR |= ~(1 << 1); /* Assign a safe value (0) in output before configuring as output */
	GPIOC->PDDR |= (1 << 1); /* Place a 1 on bit 1 to behave as output */

	UART_SendString(startString);
    // Initialize the hardware and other peripherals
	FTM_Initialization();
	PWM_PinInitialization();

	// Create and start a new timer
	xAutoReloadTimer = xTimerCreate("Auto-Reload_Timer",
									mainAUTO_RELOAD_TIMER_PERIOD_FOR_START_BIT,
									pdTRUE,
									(void *)0,
									prvAutoReloadTimerCallback);

    // Create a queue to hold data
    dataQueue = xQueueCreate(4, sizeof(PwmData_t)); // Queue is going to hold up 4 structures corresponding to each PWM

    // Create a semaphore for synchronization of input capture and one-shot timer
    sharedSemaphore = xSemaphoreCreateBinary();
    timerSemaphore = xSemaphoreCreateBinary();
    xMutex = xSemaphoreCreateMutex(); // This mutex semaphore is going to help bitBangingTask and pwmTask to interact



	// Configure and enable the external interrupt
	// For example, configure GPIO interrupt, NVIC settings, etc.
	NVIC_enable_interrupt_and_priority(FTM3_IRQ, PRIORITY_2);
	// Create data processing task
	xTaskCreate(uartTask, "UartTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(bitBangingTask, "BitBangingTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);


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

