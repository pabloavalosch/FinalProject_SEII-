/*
 * String.c
 *
 *  Created on: 30 oct. 2023
 *      Author: pablo
 */

#include "Strings.h"
#include <stdlib.h>

uint8_t* tick_to_ascii(TickType_t ticks)
{
	uint8_t i;
	uint8_t numDigits = 0;
	uint32_t temp = ticks;

	while (temp > 0)
	{
		temp /= 10;
		numDigits++;
	}

	// Allocate memory for the string (numDigits + 3 for alignment, line break and null terminator)
	uint8_t* newString = (uint8_t*)malloc(numDigits + 3);

	// Convert the number to string from left to right
	for (i = numDigits; i > 0; i--)
	{
		newString[i-1] = ticks % 10 + '0';
		ticks /= 10;
	}

	newString[numDigits] = '\r';
	newString[numDigits+1] = '\n';
	newString[numDigits+2] = '\0'; // Null-terminate the string

	return newString;

}
