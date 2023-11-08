/*
 * FTM.h
 *
 *  Created on: 7 nov. 2023
 *      Author: pablo
 */

#ifndef FTM_H_
#define FTM_H_

#include <stdint.h>
#include "board.h"
#include "pin_mux.h"
#include "fsl_ftm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_FTM_BASEADDR FTM3

/* FTM channel used for input capture */
#define BOARD_FTM_INPUT_CAPTURE_CHANNEL kFTM_Chnl_1

/* Interrupt number and interrupt handler for the FTM base address used */
#define FTM_INTERRUPT_NUMBER      FTM3_IRQn
#define FTM_INPUT_CAPTURE_HANDLER FTM3_IRQHandler

/* Interrupt to enable and flag to read */
#define FTM_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl1InterruptEnable
#define FTM_CHANNEL_FLAG             kFTM_Chnl1Flag

void FTM_Initialization(void);
uint32_t FTM_Capture(void);

#endif /* FTM_H_ */
