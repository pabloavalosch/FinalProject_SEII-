/*
 * GPIO.c
 *
 *  Created on: 2 dic. 2023
 *      Author: pablo
 */


#include "GPIO.h"

void GPIO_Initialization(void)
{
	CLOCK_EnableClock(kCLOCK_PortB);

	PORTB->PCR[2] |= (kPORT_MuxAsGpio << 8);
	GPIOB->PDOR |= (1 << 2); // Assign a safe value (1) before configuring it as output and because UART idle state is 1
	GPIOB->PDDR |= (1 << 2); /* Place a 1 on bit 2 to behave as output */

	return;

}
