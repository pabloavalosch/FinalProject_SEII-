/*
 * FTM.c
 *
 *  Created on: 7 nov. 2023
 *      Author: pablo
 */

#include "FTM.h"
#include "Strings.h"
#include "UART.h"


/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool ftmIsrFlag = false;
//uint32_t captureVal;
//uint8_t * inputCaptureMsg = {"\r\nCapture value C(n)V=%x\r\n"};

/*******************************************************************************
 * Code
 ******************************************************************************/


void FTM_Initialization(void)
{
	ftm_config_t ftmInfo;


	// PORTD_1 for FTM3_CH1 CONFIG
    FTM_GetDefaultConfig(&ftmInfo);


    CLOCK_EnableClock(kCLOCK_PortD);
    /* PORTD1 (pin D4) is configured as FTM3_CH1 */
    PORT_SetPinMux(PORTD, 1U, kPORT_MuxAlt4);

    /* Configuration for FTM3 input capture channels 2, 3, 4 */
    /* Port E Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortE);

	/* PORTA4 is configured as FTM0_CH1 */
	PORT_SetPinMux(PORTE, 7U, kPORT_MuxAlt6);

	/* PORTA6 is configured as FTM0_CH3 */
	PORT_SetPinMux(PORTE, 8U, kPORT_MuxAlt6);

	/* PORTA1 is configured as FTM0_CH6 */
	PORT_SetPinMux(PORTE, 9U, kPORT_MuxAlt6);

    /* Initialize FTM module for input Capture */
    FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);

    /* Calculate the clock division based on the PWM frequency to be obtained */
	ftmInfo.prescale = FTM_CalculateCounterClkDiv(BOARD_FTM_BASEADDR, DEMO_PWM_FREQUENCY, FTM_SOURCE_CLOCK);
	/* Initialize FTM module for PWM channels */
	FTM_Init(BOARD_FTM_BASEADDR_FOR_PWM, &ftmInfo);

    /* Setup dual-edge capture on a FTM channel pair */
    FTM_SetupInputCapture(BOARD_FTM_BASEADDR, BOARD_FTM_INPUT_CAPTURE_CHANNEL, kFTM_FallingEdge, 0);
    /* Setup dual-edge capture on a FTM channel pair */
    FTM_SetupInputCapture(BOARD_FTM_BASEADDR, BOARD_FTM_INPUT_CAPTURE_CHANNEL2, kFTM_FallingEdge, 0);
    /* Setup dual-edge capture on a FTM channel pair */
    FTM_SetupInputCapture(BOARD_FTM_BASEADDR, BOARD_FTM_INPUT_CAPTURE_CHANNEL3, kFTM_FallingEdge, 0);
    /* Setup dual-edge capture on a FTM channel pair */
    FTM_SetupInputCapture(BOARD_FTM_BASEADDR, BOARD_FTM_INPUT_CAPTURE_CHANNEL4, kFTM_FallingEdge, 0);

    /* Set the timer to be in free-running mode */
    FTM_SetTimerPeriod(BOARD_FTM_BASEADDR, 0xFFFF);

    /* Enable channel interrupt when the second edge is detected */
    FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE);
    /* Enable channel interrupt when the second edge is detected */
    FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE2);
    /* Enable channel interrupt when the second edge is detected */
    FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE3);
    /* Enable channel interrupt when the second edge is detected */
    FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE4);

    /* Enable at the NVIC */
    EnableIRQ(FTM_INTERRUPT_NUMBER);

    FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);

}

//uint32_t FTM_Capture(void)
//{
//    captureVal = FTM_GetInputCaptureValue(BOARD_FTM_BASEADDR, BOARD_FTM_INPUT_CAPTURE_CHANNEL);
//    UART_SendString( inputCaptureMsg );
//    UART_SendString( tick_to_ascii(captureVal) );
//
//    return captureVal;
//
//}
