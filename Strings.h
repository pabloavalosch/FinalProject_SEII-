/*
 * String.h
 *
 *  Created on: 30 oct. 2023
 *      Author: pablo
 */

#ifndef STRINGS_H_
#define STRINGS_H_

#include <stdint.h>
#include <FreeRTOS.h>

uint8_t* tick_to_ascii(TickType_t ticks);

#endif /* STRINGS_H_ */
