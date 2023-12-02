/*
 * NVIC.h
 *
 *  Created on: 3 sep. 2023
 *      Author: pablo
 */

#ifndef NVIC_H_
#define NVIC_H_

#include <stdint.h>
#include "MK64F12.h"

#define NVIC_global_enable_interrupts __enable_irq()
#define NVIC_disable_interrupts __disable_irq()

/** enum type that defines the priority levels for the NVIC.
 * The highest priority is PRIORITY_0 and the lowest PRIORITY_15 */
typedef enum {PRIORITY_0, PRIORITY_1, PRIORITY_2, PRIORITY_3, PRIORITY_4, PRIORITY_5, PRIORITY_6,
			  PRIORITY_7, PRIORITY_8, PRIORITY_9, PRIORITY_10, PRIORITY_11, PRIORITY_12, PRIORITY_13,
			  PRIORITY_14, PRIORITY_15 } priority_level_t;

/** enum type that defines the IRQs for the NVIC.*/
typedef enum {
	  DMA0_DMA16_IRQ,                             /**< DMA Channel 0, 16 Transfer Complete */
	  DMA1_DMA17_IRQ,                             /**< DMA Channel 1, 17 Transfer Complete */
	  DMA2_DMA18_IRQ,                             /**< DMA Channel 2, 18 Transfer Complete */
	  DMA3_DMA19_IRQ,                             /**< DMA Channel 3, 19 Transfer Complete */
	  DMA4_DMA20_IRQ,                             /**< DMA Channel 4, 20 Transfer Complete */
	  DMA5_DMA21_IRQ,                             /**< DMA Channel 5, 21 Transfer Complete */
	  DMA6_DMA22_IRQ,                             /**< DMA Channel 6, 22 Transfer Complete */
	  DMA7_DMA23_IRQ,                             /**< DMA Channel 7, 23 Transfer Complete */
	  DMA8_DMA24_IRQ,                             /**< DMA Channel 8, 24 Transfer Complete */
	  DMA9_DMA25_IRQ,                             /**< DMA Channel 9, 25 Transfer Complete */
	  DMA10_DMA26_IRQ,                            /**< DMA Channel 10, 26 Transfer Complete */
	  DMA11_DMA27_IRQ,                            /**< DMA Channel 11, 27 Transfer Complete */
	  DMA12_DMA28_IRQ,                            /**< DMA Channel 12, 28 Transfer Complete */
	  DMA13_DMA29_IRQ,                            /**< DMA Channel 13, 29 Transfer Complete */
	  DMA14_DMA30_IRQ,                            /**< DMA Channel 14, 30 Transfer Complete */
	  DMA15_DMA31_IRQ,                            /**< DMA Channel 15, 31 Transfer Complete */
	  DMA_Error_IRQ,                              /**< DMA Error Interrupt */
	  MCM_IRQ,                                    /**< Normal Interrupt */
	  FTFE_IRQ,                                   /**< FTFE Command complete interrupt */
	  Read_Collision_IRQ,                         /**< Read Collision Interrupt */
	  LVD_LVW_IRQ,                                /**< Low Voltage Detect, Low Voltage Warning */
	  LLWU_IRQ,                                   /**< Low Leakage Wakeup Unit */
	  WDOG_EWM_IRQ,                               /**< WDOG Interrupt */
	  RNG_IRQ,                                    /**< RNG Interrupt */
	  I2C0_IRQ,                                   /**< I2C0 interrupt */
	  I2C1_IRQ,                                   /**< I2C1 interrupt */
	  SPI0_IRQ,                                   /**< SPI0 Interrupt */
	  SPI1_IRQ,                                   /**< SPI1 Interrupt */
	  I2S0_Tx_IRQ,                                /**< I2S0 transmit interrupt */
	  I2S0_Rx_IRQ,                                /**< I2S0 receive interrupt */
	  Reserved46_IRQ,                             /**< Reserved interrupt 46 */
	  UART0_RX_TX_IRQ,                            /**< UART0 Receive/Transmit interrupt */
	  UART0_ERR_IRQ,                              /**< UART0 Error interrupt */
	  UART1_RX_TX_IRQ,                            /**< UART1 Receive/Transmit interrupt */
	  UART1_ERR_IRQ,                              /**< UART1 Error interrupt */
	  UART2_RX_TX_IRQ,                            /**< UART2 Receive/Transmit interrupt */
	  UART2_ERR_IRQ,                              /**< UART2 Error interrupt */
	  UART3_RX_TX_IRQ,                            /**< UART3 Receive/Transmit interrupt */
	  UART3_ERR_IRQ,                              /**< UART3 Error interrupt */
	  ADC0_IRQ,                                   /**< ADC0 interrupt */
	  CMP0_IRQ,                                   /**< CMP0 interrupt */
	  CMP1_IRQ,                                   /**< CMP1 interrupt */
	  FTM0_IRQ,                                   /**< FTM0 fault, overflow and channels interrupt */
	  FTM1_IRQ,                                   /**< FTM1 fault, overflow and channels interrupt */
	  FTM2_IRQ,                                   /**< FTM2 fault, overflow and channels interrupt */
	  CMT_IRQ,                                    /**< CMT interrupt */
	  RTC_IRQ,                                    /**< RTC interrupt */
	  RTC_Seconds_IRQ,                            /**< RTC seconds interrupt */
	  PIT0_IRQ,                                   /**< PIT timer channel 0 interrupt */
	  PIT1_IRQ,                                   /**< PIT timer channel 1 interrupt */
	  PIT2_IRQ,                                   /**< PIT timer channel 2 interrupt */
	  PIT3_IRQ,                                   /**< PIT timer channel 3 interrupt */
	  PDB0_IRQ,                                   /**< PDB0 Interrupt */
	  USB0_IRQ,                                   /**< USB0 interrupt */
	  USBDCD_IRQ,                                 /**< USBDCD Interrupt */
	  Reserved71_IRQ,                             /**< Reserved interrupt 71 */
	  DAC0_IRQ,                                   /**< DAC0 interrupt */
	  MCG_IRQ,                                    /**< MCG Interrupt */
	  LPTMR0_IRQ,                                 /**< LPTimer interrupt */
	  PORTA_IRQ,                                  /**< Port A interrupt */
	  PORTB_IRQ,                                  /**< Port B interrupt */
	  PORTC_IRQ,                                  /**< Port C interrupt */
	  PORTD_IRQ,                                  /**< Port D interrupt */
	  PORTE_IRQ,                                  /**< Port E interrupt */
	  SWI_IRQ,                                    /**< Software interrupt */
	  SPI2_IRQ,                                   /**< SPI2 Interrupt */
	  UART4_RX_TX_IRQ,                            /**< UART4 Receive/Transmit interrupt */
	  UART4_ERR_IRQ,                              /**< UART4 Error interrupt */
	  Reserved84_IRQ,                             /**< Reserved interrupt 84 */
	  Reserved85_IRQ,                             /**< Reserved interrupt 85 */
	  CMP2_IRQ,                                   /**< CMP2 interrupt */
	  FTM3_IRQ,                                   /**< FTM3 fault, overflow and channels interrupt */
	  DAC1_IRQ,                                   /**< DAC1 interrupt */
	  ADC1_IRQ,                                   /**< ADC1 interrupt */
	  I2C2_IRQ,                                   /**< I2C2 interrupt */
	  CAN0_ORed_Message_buffer_IRQ,               /**< CAN0 OR'd message buffers interrupt */
	  CAN0_Bus_Off_IRQ,                           /**< CAN0 bus off interrupt */
	  CAN0_Error_IRQ,                             /**< CAN0 error interrupt */
	  CAN0_Tx_Warning_IRQ,                        /**< CAN0 Tx warning interrupt */
	  CAN0_Rx_Warning_IRQ,                        /**< CAN0 Rx warning interrupt */
	  CAN0_Wake_Up_IRQ,                           /**< CAN0 wake up interrupt */
	  SDHC_IRQ,                                   /**< SDHC interrupt */
	  ENET_1588_Timer_IRQ,                        /**< Ethernet MAC IEEE 1588 Timer Interrupt */
	  ENET_Transmit_IRQ,                          /**< Ethernet MAC Transmit Interrupt */
	  ENET_Receive_IRQ,                           /**< Ethernet MAC Receive Interrupt */
	  ENET_Error_IRQ,                             /**< Ethernet MAC Error and miscelaneous Interrupt */
	  LPUART0_IRQ,                                /**< LPUART0 status/error interrupt */
	  TSI0_IRQ,                                   /**< TSI0 interrupt */
	  TPM1_IRQ,                                   /**< TPM1 fault, overflow and channels interrupt */
	  TPM2_IRQ,                                   /**< TPM2 fault, overflow and channels interrupt */
	  USBHSDCD_IRQ,                               /**< USBHSDCD, USBHS Phy Interrupt */
	  I2C3_IRQ,                                   /**< I2C3 interrupt */
	  CMP3_IRQ,                                   /**< CMP3 interrupt */
	  USBHS_IRQ,                                  /**< USB high speed OTG interrupt */
	  CAN1_ORed_Message_buffer_IRQ,               /**< CAN1 OR'd message buffers interrupt */
	  CAN1_Bus_Off_IRQ,                           /**< CAN1 bus off interrupt */
	  CAN1_Error_IRQ,                             /**< CAN1 error interrupt */
	  CAN1_Tx_Warning_IRQ,                        /**< CAN1 Tx warning interrupt */
	  CAN1_Rx_Warning_IRQ,                        /**< CAN1 Rx warning interrupt */
	  CAN1_Wake_Up_IRQ                            /**< CAN1 wake up interrupt */
} interrupt_t;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function enables a IRQ in the NVIC and establishes its priority.

 	 \param[in] interruptNumber is the desired IRQ to be activivated.
 	 \param[in] priority establishes the priority of the IRQ
 	 \return void
 */
void NVIC_enable_interrupt_and_priority(interrupt_t interrupt_number, priority_level_t priority);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function establishes the threshold level to interrupt the MCU.

 	 \param[in]  priority threshold to be established.
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void NVIC_set_basepri_threshold(priority_level_t priority);

#endif /* NVIC_H_ */
