/*
 * PWM.c
 *
 *  Created on: 3 dic. 2023
 *      Author: pablo
 */


#include "PWM.h"

void PWM_PinInitialization(void)
{
	CLOCK_EnableClock(kCLOCK_PortB);

	PORTB->PCR[PWM0_PIN] |= (kPORT_MuxAsGpio << 8);
	GPIOB->PDOR |= ~(1 << PWM0_PIN); // Assign a safe value (0) before configuring it as output
	GPIOB->PDDR |= (1 << PWM0_PIN); /* Place a 1 on PWM0_PIN to behave as output */

	return;

}
