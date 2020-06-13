/**
 * \file 
 * \brief QSPI1 Driver Initialisation
 *
 * This Driver manage the global initialisation of QSPI1.
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <Cpu/Std/Ifx_Types.h>
#include "Configuration.h"
#include "Qspi1.h"


/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
#if ISR_PROVIDER_QSPI1 == 0
	#if defined(__GNUC__)
	#pragma section ".bss_cpu0" awc0
	#endif
	#if defined(__TASKING__)
	#pragma section farbss "bss_cpu0"
	#pragma section fardata "data_cpu0"
	#endif
	#if defined(__DCC__)
	#pragma section DATA ".data_cpu0" ".bss_cpu0" far-absolute RW
	#endif
#elif ISR_PROVIDER_QSPI1 == 1
	#if defined(__GNUC__)
	#pragma section ".bss_cpu1" awc1
	#endif
	#if defined(__TASKING__)
	#pragma section farbss "bss_cpu1"
	#pragma section fardata "data_cpu1"
	#endif
	#if defined(__DCC__)
	#pragma section DATA ".data_cpu1" ".bss_cpu1" far-absolute RW
	#endif
#elif ISR_PROVIDER_QSPI1 == 2
	#if defined(__GNUC__)
	#pragma section ".bss_cpu2" awc2
	#endif
	#if defined(__TASKING__)
	#pragma section farbss "bss_cpu2"
	#pragma section fardata "data_cpu2"
	#endif
	#if defined(__DCC__)
	#pragma section DATA ".data_cpu2" ".bss_cpu2" far-absolute RW
	#endif
#else
#error "Set ISR_PROVIDER_QSPI1 to a valid value!"
#endif

IfxQspi_SpiMaster spi1Master={{ 0, 0, 0, 0, 0, { 0, 0, 0, 0, 0}}, 0,{{ 0, 0, 0},{ 0, 0, 0}, 0, 0, 0}}; /**< \brief Qspi1 global data */

#if defined(__GNUC__)
#pragma section
#endif
#if defined(__TASKING__)
#pragma section farbss restore
#pragma section fardata restore
#endif
#if defined(__DCC__)
#pragma section DATA RW
#endif


/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
#ifdef QSPI1_TRANSMIT_CALLBACK
    extern void QSPI1_TRANSMIT_CALLBACK(void);
#endif
#ifdef QSPI1_RECEIVE_CALLBACK
    extern void QSPI1_RECEIVE_CALLBACK(void);
#endif

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/* pin configuration */
const IfxQspi_SpiMaster_Pins qspi1_pins = {&QSPI1_SCLK_PIN, IfxPort_OutputMode_pushPull,  /* SCLK */
                                           &QSPI1_MTSR_PIN,  IfxPort_OutputMode_pushPull, /* MTSR */
                                           &QSPI1_MRST_PIN,  IfxPort_InputMode_pullDown,  /* MRST */
                                           IfxPort_PadDriver_ttlSpeed1		              /* pad driver mode */
};

/******************************************************************************/
/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
/** \addtogroup IfxLld_Demo_QspiCpu_SrcDoc_Main_Interrupt
 * \{ */

/** \name Interrupts for Qspi Master (QSPI1)
 * \{ */
IFX_INTERRUPT(ISR_qspi1_Tx, 0, ISR_PRIORITY_QSPI1_TX);
IFX_INTERRUPT(ISR_qspi1_Rx, 0, ISR_PRIORITY_QSPI1_RX);
IFX_INTERRUPT(ISR_qspi1_Er, 0, ISR_PRIORITY_QSPI1_ER);

/** \} */

/** \} */

/** \brief Handle qspi1_Tx interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI1
 * \isrPriority \ref ISR_PRIORITY_QSPI1
 *
 */
void ISR_qspi1_Tx(void)
{
    IfxCpu_enableInterrupts();
#ifdef QSPI1_USE_DMA
    IfxQspi_SpiMaster_isrDmaTransmit(&spi1Master);
#else
    IfxQspi_SpiMaster_isrTransmit(&spi1Master);
#endif

#ifdef QSPI1_TRANSMIT_CALLBACK
    QSPI1_TRANSMIT_CALLBACK();
#endif
}


/** \brief Handle qspi1_Rx interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI1
 * \isrPriority \ref ISR_PRIORITY_QSPI1
 *
 */
void ISR_qspi1_Rx(void)
{
    IfxCpu_enableInterrupts();
#ifdef QSPI1_USE_DMA
    IfxQspi_SpiMaster_isrDmaReceive(&spi1Master);
#else
    IfxQspi_SpiMaster_isrReceive(&spi1Master);
#endif
#ifdef QSPI1_RECEIVE_CALLBACK
    QSPI1_RECEIVE_CALLBACK();
#endif
}

/** \brief Handle qspi1_Er interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI1
 * \isrPriority \ref ISR_PRIORITY_QSPI1
 *
 */
void ISR_qspi1_Er(void)
{
    IfxCpu_enableInterrupts();
    IfxQspi_SpiMaster_isrError(&spi1Master);
}

/** \brief QSPI1 initialisation
 *
 * This function initialises the Qspi1 in master mode.
 */
IfxQspi_SpiMaster *qspi1_init (void)
{

    /* disable interrupts */
    boolean interruptState = IfxCpu_disableInterrupts();

    IfxQspi_SpiMaster_Config        spiMasterConfig;

    if (spi1Master.qspi == 0)
    {
        /* create module config */
        IfxQspi_SpiMaster_initModuleConfig(&spiMasterConfig, &MODULE_QSPI1);

        /* set the maximum baudrate */
        spiMasterConfig.base.maximumBaudrate = QSPI1_MAX_BAUDRATE;

        /* ISR priorities and interrupt target */
        spiMasterConfig.base.txPriority  = ISR_PRIORITY_QSPI1_TX;
        spiMasterConfig.base.rxPriority  = ISR_PRIORITY_QSPI1_RX;
        spiMasterConfig.base.erPriority  = ISR_PRIORITY_QSPI1_ER;
        spiMasterConfig.base.isrProvider = ISR_PROVIDER_QSPI1;

#ifdef QSPI1_USE_DMA
        // DMA configuration
        spiMasterConfig.dma.txDmaChannelId = DMA_CH_QSPI1_TX;
        spiMasterConfig.dma.rxDmaChannelId = DMA_CH_QSPI1_RX;
        spiMasterConfig.dma.useDma = 1;
#endif
        spiMasterConfig.pins = &qspi1_pins;

        /* initialize module */
        IfxQspi_SpiMaster_initModule(&spi1Master, &spiMasterConfig);
        /* set the MRST_input also to the selected pad driver mode if needed */
        if (qspi1_pins.mrst != NULL_PTR)
            IfxPort_setPinPadDriver(qspi1_pins.mrst->pin.port, qspi1_pins.mrst->pin.pinIndex, qspi1_pins.pinDriver);
    }

    /* enable interrupts again */
    IfxCpu_restoreInterrupts(interruptState);

  return (&spi1Master);
}
