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
	ftm_chnl_pwm_signal_param_t ftmParam[4];


	// PORTD_1 for FTM3_CH1 CONFIG
    FTM_GetDefaultConfig(&ftmInfo);


    CLOCK_EnableClock(kCLOCK_PortD);
    /* PORTD1 (pin D4) is configured as FTM3_CH1 */
    PORT_SetPinMux(PORTD, 1U, kPORT_MuxAlt4);

    /* Configuration for FTM0 PWM channels 1, 3, 4 and 6 */
    /* Port A Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortA);

	/* PORTA4 is configured as FTM0_CH1 */
	PORT_SetPinMux(PORTA, 4U, kPORT_MuxAlt3);

	/* PORTA6 is configured as FTM0_CH3 */
	PORT_SetPinMux(PORTA, 6U, kPORT_MuxAlt3);

	/* PORTA1 is configured as FTM0_CH6 */
	PORT_SetPinMux(PORTA, 1U, kPORT_MuxAlt3);

	/* PORTA7 is configured as FTM0_CH4 */
	PORT_SetPinMux(PORTA, 7U, kPORT_MuxAlt3);

    /* Initialize FTM module for input Capture */
    FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);

    /* Calculate the clock division based on the PWM frequency to be obtained */
	ftmInfo.prescale = FTM_CalculateCounterClkDiv(BOARD_FTM_BASEADDR, DEMO_PWM_FREQUENCY, FTM_SOURCE_CLOCK);
	/* Initialize FTM module for PWM channels */
	FTM_Init(BOARD_FTM_BASEADDR_FOR_PWM, &ftmInfo);

	/* Configure ftm params with frequency 24kHZ */
	ftmParam[0].chnlNumber            = (ftm_chnl_t)BOARD_FIRST_FTM_CHANNEL;
	ftmParam[0].level                 = FTM_PWM_ON_LEVEL;
	ftmParam[0].dutyCyclePercent      = 0U;
	ftmParam[0].firstEdgeDelayPercent = 0U;
	ftmParam[0].enableComplementary   = false;
	ftmParam[0].enableDeadtime        = false;

	ftmParam[1].chnlNumber            = (ftm_chnl_t)BOARD_SECOND_FTM_CHANNEL;
	ftmParam[1].level                 = FTM_PWM_ON_LEVEL;
	ftmParam[1].dutyCyclePercent      = 0U;
	ftmParam[1].firstEdgeDelayPercent = 0U;
	ftmParam[1].enableComplementary   = false;
	ftmParam[1].enableDeadtime        = false;

	ftmParam[2].chnlNumber            = (ftm_chnl_t)BOARD_THIRD_FTM_CHANNEL;
	ftmParam[2].level                 = FTM_PWM_ON_LEVEL;
	ftmParam[2].dutyCyclePercent      = 0U;
	ftmParam[2].firstEdgeDelayPercent = 0U;
	ftmParam[2].enableComplementary   = false;
	ftmParam[2].enableDeadtime        = false;

	ftmParam[3].chnlNumber            = (ftm_chnl_t)BOARD_FOURTH_FTM_CHANNEL;
	ftmParam[3].level                 = FTM_PWM_ON_LEVEL;
	ftmParam[3].dutyCyclePercent      = 0U;
	ftmParam[3].firstEdgeDelayPercent = 0U;
	ftmParam[3].enableComplementary   = false;
	ftmParam[3].enableDeadtime        = false;

    /* Setup dual-edge capture on a FTM channel pair */
    FTM_SetupInputCapture(BOARD_FTM_BASEADDR, BOARD_FTM_INPUT_CAPTURE_CHANNEL, kFTM_FallingEdge, 0);

    /* Setup PWM configuration on a FTM channel pair */
    FTM_SetupPwm(BOARD_FTM_BASEADDR_FOR_PWM, ftmParam, 4U, kFTM_EdgeAlignedPwm, DEMO_PWM_FREQUENCY, FTM_SOURCE_CLOCK);

    /* Set the timer to be in free-running mode */
    FTM_SetTimerPeriod(BOARD_FTM_BASEADDR, 0xFFFF);

    /* Start timer for PWM signals */
//    FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
//    FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR_FOR_PWM, (ftm_chnl_t)BOARD_FIRST_FTM_CHANNEL, kFTM_EdgeAlignedPwm, updatedDutycycle);
//    FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR_FOR_PWM, (ftm_chnl_t)BOARD_SECOND_FTM_CHANNEL, kFTM_EdgeAlignedPwm, updatedDutycycle);
//    FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR_FOR_PWM, (ftm_chnl_t)BOARD_THIRD_FTM_CHANNEL, kFTM_EdgeAlignedPwm, updatedDutycycle);
//    FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR_FOR_PWM, (ftm_chnl_t)BOARD_FOURTH_FTM_CHANNEL, kFTM_EdgeAlignedPwm, updatedDutycycle);

    /* Software trigger to update registers */
//    FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR_FOR_PWM, true);

    /* Enable channel interrupt when the second edge is detected */
    FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE);

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
