/**
 * \file Configuration.h
 * \brief Global configuration
 *
 * \version iLLD_Demos_0_1_0_11
 * \copyright Copyright (c) 2014 Infineon Technologies AG. All rights reserved.
 *
 *
 *                                 IMPORTANT NOTICE
 *
 *
 * Infineon Technologies AG (Infineon) is supplying this file for use
 * exclusively with Infineon's microcontroller products. This file can be freely
 * distributed within development tools that are supporting such microcontroller
 * products.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * \defgroup IfxLld_Demo_QspiDmaDemo_SrcDoc_Config Application configuration
 * \ingroup IfxLld_Demo_QspiDmaDemo_SrcDoc
 *
 *
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H
/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "Ifx_Cfg.h"
#include <_PinMap/IfxQspi_PinMap.h>
#include <_PinMap/IfxPort_PinMap.h>
#include <_PinMap/IfxScu_PinMap.h>
#include <_PinMap/IfxGtm_PinMap.h>
#include "Qspi0.h"
#include "Qspi1.h"
#include "Qspi2.h"
#include "Qspi3.h"
#include "Qspi4.h"
#include "ConfigurationIsr.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
/* set here the used pins for QSPI4 not used on board */
#define QSPI4_MAX_BAUDRATE          50000000  // maximum baudrate which is possible to get a small time quantum
#define QSPI4_SCLK_PIN	            IfxQspi4_SCLK_P22_3_OUT
#define QSPI4_MTSR_PIN	            IfxQspi4_MTSR_P22_0_OUT
#define QSPI4_MRST_PIN	            IfxQspi4_MRSTB_P22_1_IN
//#define QSPI4_USE_DMA  // uncomment line for using DMA

/* set here the used pins for QSPI3 not used on board */
#define QSPI3_MAX_BAUDRATE          50000000  // maximum baudrate which is possible to get a small time quantum
#define QSPI3_SCLK_PIN	            IfxQspi3_SCLK_P02_7_OUT
#define QSPI3_MTSR_PIN	            IfxQspi3_MTSR_P02_6_OUT
#define QSPI3_MRST_PIN	            IfxQspi3_MRSTA_P02_5_IN
//#define QSPI3_USE_DMA  // uncomment line for using DMA
/* next values are used for TC23X/TC22X, because TC23X/TC22X don't has I2C */
//#define DMA_CH_QSPI3_TX             RTC_DMA_CH_TXBUFF_TO_TXFIFO
//#define DMA_CH_QSPI3_RX             RTC_DMA_CH_RXBUFF_FROM_RXFIFO
//#define RTC_QSPI_INIT				qspi3_init
//#define RTC_USE_CHIPSELECT          IfxQspi3_SLSO13_P23_1_OUT
/* next values are not used for TC23X/TC22X, because TC23X/TC22X don't has I2C */
#define RTC_SCL_PIN                 IfxI2c0_SCL_P15_4_INOUT
#define RTC_SDA_PIN                 IfxI2c0_SDA_P15_5_INOUT
#define RTC_ALARM_PIN_INPUT         IfxScu_REQ8_P33_7_IN
#define RTC_ALARM_SRC               &MODULE_SRC.SCU.SCU.ERU[0]

/* set here the used pins for QSPI2 */
#define QSPI2_MAX_BAUDRATE          50000000  // maximum baudrate which is possible to get a small time quantum
#define QSPI2_SCLK_PIN	            IfxQspi2_SCLK_P15_3_OUT
//#define QSPI2_MTSR_PIN	            IfxQspi2_MTSR_P15_5_OUT  // for Application Kit TC234L
#define QSPI2_MTSR_PIN	            IfxQspi2_MTSR_P15_6_OUT      // for Application Kit TC2X7
#define QSPI2_MRST_PIN	            IfxQspi2_MRSTB_P15_7_IN
//#define QSPI2_USE_DMA  // uncomment line for using DMA
//#define DMA_CH_QSPI2_TX             TLF_DMA_CH_TXBUFF_TO_TXFIFO
//#define DMA_CH_QSPI2_RX             TLF_DMA_CH_RXBUFF_FROM_RXFIFO

#define TLF_QSPI_INIT				qspi2_init
#define TLF_USE_CHIPSELECT          IfxQspi2_SLSO1_P14_2_OUT

/* set here the used pins for QSPI1 */
#define QSPI1_MAX_BAUDRATE          50000000  // maximum baudrate which is possible to get a small time quantum
#define QSPI1_SCLK_PIN	            IfxQspi1_SCLK_P10_2_OUT
#define QSPI1_MTSR_PIN	            IfxQspi1_MTSR_P10_3_OUT
#define QSPI1_MRST_PIN	            IfxQspi1_MRSTA_P10_1_IN
#define QSPI1_USE_DMA  // comment line for not using DMA
#define DMA_CH_QSPI1_TX             SDCARD_DMA_CH_TXBUFF_TO_TXFIFO
#define DMA_CH_QSPI1_RX             SDCARD_DMA_CH_RXBUFF_FROM_RXFIFO

#define SDCARD_QSPI_INIT			qspi1_init
#define SDCARD_USE_CHIPSELECT       IfxQspi1_SLSO9_P10_5_OUT
#define SDCARD_USE_CD               IfxPort_P15_8
/* ECON for 25MHz (A=3, B=3, C=0, TQ=2) with 300MHz fbaud2 */
#define SDCARD_FCLK_FAST_VALUE      0x0381

/* set here the used pins for QSPI0 */
#define QSPI0_MAX_BAUDRATE          50000000  // maximum baudrate which is possible to get a small time quantum
#define QSPI0_SCLK_PIN	            IfxQspi0_SCLK_P20_11_OUT
#define QSPI0_MTSR_PIN	            IfxQspi0_MTSR_P20_14_OUT
#define QSPI0_MRST_PIN	            IfxQspi0_MRSTA_P20_12_IN
#define QSPI0_USE_DMA  // comment line for not using DMA
#define DMA_CH_QSPI0_TX             TFT_DMA_CH_TXBUFF_TO_TXFIFO
#define DMA_CH_QSPI0_RX             TFT_DMA_CH_RXBUFF_FROM_RXFIFO
#define QSPI0_TRANSMIT_CALLBACK     tft_transmit_callback

#define TFT_QSPI_INIT			    qspi0_init
#define TFT_USE_CHIPSELECT          IfxQspi0_SLSO8_P20_6_OUT
#define TFT_USE_SCLK                QSPI0_SCLK_PIN

#define TFT_UPDATE_IRQ              MODULE_SRC.GPSR.GPSR[0].SR0

#define TOUCH_QSPI_INIT			    qspi0_init
#define TOUCH_USE_CHIPSELECT        IfxQspi0_SLSO9_P20_3_OUT
#define TOUCH_USE_INT               IfxPort_P20_0

#define BACKGROUND_LIGHT            IfxGtm_TOM1_1_TOUT69_P20_13_OUT

#define BEEPER                      IfxGtm_TOM0_4_TOUT22_P33_0_OUT

#define PERFORMANCE_MEASURE         IfxGtm_TOM0_0_TOUT76_P15_5_OUT // we can use P15.5 because no output needed and P15.5 maybe used by I2C

#define CAN_TXD_USE                 IfxPort_P20_8
#define CAN_RXD_USE                 IfxPort_P20_7

#define LIN_TXD_USE                 IfxPort_P15_0
#define LIN_RXD_USE                 IfxPort_P15_1

#define ASC_TXD_USE                 IfxPort_P14_0
#define ASC_RXD_USE                 IfxPort_P14_1

#define LED0_PORT                   IfxPort_P13_0
#define LED1_PORT                   IfxPort_P13_1
#define LED2_PORT                   IfxPort_P13_2
#define LED3_PORT                   IfxPort_P13_3

#define LIFEHOLD_LED                LED3_PORT

/* we define here the pins for ethernet phy RMII connection */
#define ETH_CRSDIV_PIN              IfxEth_CRSDVA_P11_11_IN
#define ETH_REFCLK_PIN              IfxEth_REFCLK_P11_12_IN
#define ETH_RXD0_PIN                IfxEth_RXD0_P11_10_IN
#define ETH_RXD1_PIN                IfxEth_RXD1_P11_9_IN
#define ETH_MDC_PIN                 IfxEth_MDC_P21_2_OUT
#define ETH_MDIO_PIN                IfxEth_MDIOD_P21_3_INOUT
#define ETH_TXD0_PIN                IfxEth_TXD0_P11_3_OUT
#define ETH_TXD1_PIN                IfxEth_TXD1_P11_2_OUT
#define ETH_TXEN_PIN                IfxEth_TXEN_P11_6_OUT


/* set the values for systemtimer */
#define IFX_CFG_STM_TICKS_PER_MS (100000) /**< \brief Number of STM ticks per millisecond */

/* define the CPU which holds the variable for different parts */
#define TFT_DISPLAY_VAR_LOCATION    2
#define TOUCH_VAR_LOCATION          2
#define SDCARD_VAR_LOCATION         0
#define RTC_VAR_LOCATION            1
#define TLF_VAR_LOCATION            0
#define LWIP_VAR_LOCATION           0

#define USE_TFT
//#define TFT_OVER_DAS


#define REFRESH_TFT 50		// Refresh rate [ms]; 1x refresh ~0,8 % CPU load; for graphic-out: 4 colors ~ 1.2 % CPU load, 16 colors ~ 0.8 % CPU load
							// Max refresh rate ~ 40 ms due to QSPI-load

#define REFRESH_MEASUREMENT 50		// Refresh rate [ms] for temperature and voltage measurement

#define VERSION_TEXT        "Application Kit TC297TF SW V1.1..."

#endif
