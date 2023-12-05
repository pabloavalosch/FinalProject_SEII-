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
#define BOARD_FTM_BASEADDR 			FTM3

/* The Flextimer instance/channel used for board generation of PWM channels */
#define BOARD_FTM_BASEADDR_FOR_PWM	FTM0
#define BOARD_FIRST_FTM_CHANNEL  4U
#define BOARD_SECOND_FTM_CHANNEL 6U
#define BOARD_THIRD_FTM_CHANNEL  1U
#define BOARD_FOURTH_FTM_CHANNEL 3U

/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#ifndef FTM_PWM_ON_LEVEL
#define FTM_PWM_ON_LEVEL kFTM_HighTrue
#endif
#ifndef DEMO_PWM_FREQUENCY
#define DEMO_PWM_FREQUENCY (24000U)
#endif

/* FTM channel used for input capture */
#define BOARD_FTM_INPUT_CAPTURE_CHANNEL 	kFTM_Chnl_1
#define BOARD_FTM_INPUT_CAPTURE_CHANNEL2 	kFTM_Chnl_2
#define BOARD_FTM_INPUT_CAPTURE_CHANNEL3 	kFTM_Chnl_3
#define BOARD_FTM_INPUT_CAPTURE_CHANNEL4 	kFTM_Chnl_4

/* Interrupt number and interrupt handler for the FTM base address used */
#define FTM_INTERRUPT_NUMBER      FTM3_IRQn
#define FTM_INPUT_CAPTURE_HANDLER FTM3_IRQHandler

/* Interrupt to enable and flag to read */
#define FTM_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl1InterruptEnable
#define FTM_CHANNEL_FLAG             kFTM_Chnl1Flag
/* Interrupt to enable and flag to read */
#define FTM_CHANNEL_INTERRUPT_ENABLE2 kFTM_Chnl2InterruptEnable
#define FTM_CHANNEL_FLAG2             kFTM_Chnl2Flag
/* Interrupt to enable and flag to read */
#define FTM_CHANNEL_INTERRUPT_ENABLE3 kFTM_Chnl3InterruptEnable
#define FTM_CHANNEL_FLAG3             kFTM_Chnl3Flag
/* Interrupt to enable and flag to read */
#define FTM_CHANNEL_INTERRUPT_ENABLE4 kFTM_Chnl4InterruptEnable
#define FTM_CHANNEL_FLAG4             kFTM_Chnl4Flag

void FTM_Initialization(void);
//uint32_t FTM_Capture(void);

#endif /* FTM_H_ */
