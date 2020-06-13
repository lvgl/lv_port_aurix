/**
 * \file 
 * \brief QSPI3 Driver Initialisation
 *
 * This Driver manage the global initialisation of QSPI3.
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <Cpu/Std/Ifx_Types.h>
#include "Configuration.h"
#include "Qspi3.h"


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
#if ISR_PROVIDER_QSPI3 == 0
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
#elif ISR_PROVIDER_QSPI3 == 1
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
#elif ISR_PROVIDER_QSPI3 == 2
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
#error "Set ISR_PROVIDER_QSPI3 to a valid value!"
#endif

IfxQspi_SpiMaster spi3Master={{ 0, 0, 0, 0, 0, { 0, 0, 0, 0, 0}}, 0,{{ 0, 0, 0},{ 0, 0, 0}, 0, 0, 0}}; /**< \brief Qspi3 global data */

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
#ifdef QSPI3_TRANSMIT_CALLBACK
    extern void QSPI3_TRANSMIT_CALLBACK(void);
#endif
#ifdef QSPI3_RECEIVE_CALLBACK
    extern void QSPI3_RECEIVE_CALLBACK(void);
#endif

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/* pin configuration */
const IfxQspi_SpiMaster_Pins qspi3_pins = {&QSPI3_SCLK_PIN, IfxPort_OutputMode_pushPull,  /* SCLK */
                                           &QSPI3_MTSR_PIN,  IfxPort_OutputMode_pushPull, /* MTSR */
                                           &QSPI3_MRST_PIN,  IfxPort_InputMode_pullDown,  /* MRST */
                                           IfxPort_PadDriver_ttlSpeed1		              /* pad driver mode */
};

/******************************************************************************/
/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
/** \addtogroup IfxLld_Demo_QspiCpu_SrcDoc_Main_Interrupt
 * \{ */

/** \name Interrupts for Qspi Master (QSPI3)
 * \{ */
IFX_INTERRUPT(ISR_qspi3_Tx, 0, ISR_PRIORITY_QSPI3_TX);
IFX_INTERRUPT(ISR_qspi3_Rx, 0, ISR_PRIORITY_QSPI3_RX);
IFX_INTERRUPT(ISR_qspi3_Er, 0, ISR_PRIORITY_QSPI3_ER);

/** \} */

/** \} */

/** \brief Handle qspi3_Tx interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI3
 * \isrPriority \ref ISR_PRIORITY_QSPI3
 *
 */
void ISR_qspi3_Tx(void)
{
    IfxCpu_enableInterrupts();
#ifdef QSPI3_USE_DMA
    IfxQspi_SpiMaster_isrDmaTransmit(&spi3Master);
#else
    IfxQspi_SpiMaster_isrTransmit(&spi3Master);
#endif

#ifdef QSPI3_TRANSMIT_CALLBACK
    QSPI3_TRANSMIT_CALLBACK();
#endif
}


/** \brief Handle qspi3_Rx interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI3
 * \isrPriority \ref ISR_PRIORITY_QSPI3
 *
 */
void ISR_qspi3_Rx(void)
{
    IfxCpu_enableInterrupts();
#ifdef QSPI3_USE_DMA
    IfxQspi_SpiMaster_isrDmaReceive(&spi3Master);
#else
    IfxQspi_SpiMaster_isrReceive(&spi3Master);
#endif
#ifdef QSPI3_RECEIVE_CALLBACK
    QSPI3_RECEIVE_CALLBACK();
#endif
}

/** \brief Handle qspi3_Er interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI3
 * \isrPriority \ref ISR_PRIORITY_QSPI3
 *
 */
void ISR_qspi3_Er(void)
{
    IfxCpu_enableInterrupts();
    IfxQspi_SpiMaster_isrError(&spi3Master);
}

/** \brief QSPI3 initialisation
 *
 * This function initialises the Qspi3 in master mode.
 */
IfxQspi_SpiMaster *qspi3_init (void)
{

    /* disable interrupts */
    boolean interruptState = IfxCpu_disableInterrupts();

    IfxQspi_SpiMaster_Config        spiMasterConfig;

    if (spi3Master.qspi == 0)
    {
        /* create module config */
        IfxQspi_SpiMaster_initModuleConfig(&spiMasterConfig, &MODULE_QSPI3);

        /* set the maximum baudrate */
        spiMasterConfig.base.maximumBaudrate = QSPI3_MAX_BAUDRATE;

        /* ISR priorities and interrupt target */
        spiMasterConfig.base.txPriority  = ISR_PRIORITY_QSPI3_TX;
        spiMasterConfig.base.rxPriority  = ISR_PRIORITY_QSPI3_RX;
        spiMasterConfig.base.erPriority  = ISR_PRIORITY_QSPI3_ER;
        spiMasterConfig.base.isrProvider = ISR_PROVIDER_QSPI3;

#ifdef QSPI3_USE_DMA
        // DMA configuration
        spiMasterConfig.dma.txDmaChannelId = DMA_CH_QSPI3_TX;
        spiMasterConfig.dma.rxDmaChannelId = DMA_CH_QSPI3_RX;
        spiMasterConfig.dma.useDma = 1;
#endif
        spiMasterConfig.pins = &qspi3_pins;

        /* initialize module */
        IfxQspi_SpiMaster_initModule(&spi3Master, &spiMasterConfig);
        /* set the MRST_input also to the selected pad driver mode if needed */
        if (qspi3_pins.mrst != NULL_PTR)
            IfxPort_setPinPadDriver(qspi3_pins.mrst->pin.port, qspi3_pins.mrst->pin.pinIndex, qspi3_pins.pinDriver);
    }

    /* enable interrupts again */
    IfxCpu_restoreInterrupts(interruptState);

  return (&spi3Master);
}
