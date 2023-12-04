/*
 * PWM.h
 *
 *  Created on: 3 dic. 2023
 *      Author: pablo
 */

#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include <fsl_clock.h>
#include <fsl_port.h>
#include <fsl_gpio.h>

// Assume that all PWM signals are going to be at 1KHz
#define PWM_OUTPUT_PERIOD_US						(10000U) // 10000 usec that are equivalent to the 1/100Hz

#define PWM0_PIN									(4U) // PWM0 pin is with duty cycle received from VIRTUAL UART0
#define PWM1_PIN									(5U) // PWM1 pin is with duty cycle received from VIRTUAL UART1
#define PWM2_PIN									(6U) // PWM2 pin is with duty cycle received from VIRTUAL UART2
#define PWM3_PIN									(7U) // PWM3 pin is with duty cycle received from VIRTUAL UART3

void PWM_PinInitialization(void);

#endif /* PWM_H_ */
