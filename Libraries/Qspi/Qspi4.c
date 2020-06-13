/**
 * \file 
 * \brief QSPI4 Driver Initialisation
 *
 * This Driver manage the global initialisation of QSPI4.
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <Cpu/Std/Ifx_Types.h>
#include "Configuration.h"
#include "Qspi4.h"


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
#if ISR_PROVIDER_QSPI4 == 0
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
#elif ISR_PROVIDER_QSPI4 == 1
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
#elif ISR_PROVIDER_QSPI4 == 2
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
#error "Set ISR_PROVIDER_QSPI4 to a valid value!"
#endif

IfxQspi_SpiMaster spi4Master={{ 0, 0, 0, 0, 0, { 0, 0, 0, 0, 0}}, 0,{{ 0, 0, 0},{ 0, 0, 0}, 0, 0, 0}}; /**< \brief Qspi4 global data */

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
#ifdef QSPI4_TRANSMIT_CALLBACK
    extern void QSPI4_TRANSMIT_CALLBACK(void);
#endif
#ifdef QSPI4_RECEIVE_CALLBACK
    extern void QSPI4_RECEIVE_CALLBACK(void);
#endif

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/* pin configuration */
const IfxQspi_SpiMaster_Pins qspi4_pins = {&QSPI4_SCLK_PIN, IfxPort_OutputMode_pushPull,  /* SCLK */
                                           &QSPI4_MTSR_PIN,  IfxPort_OutputMode_pushPull, /* MTSR */
                                           &QSPI4_MRST_PIN,  IfxPort_InputMode_pullDown,  /* MRST */
                                           IfxPort_PadDriver_ttlSpeed1		              /* pad driver mode */
};

/******************************************************************************/
/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
/** \addtogroup IfxLld_Demo_QspiCpu_SrcDoc_Main_Interrupt
 * \{ */

/** \name Interrupts for Qspi Master (QSPI4)
 * \{ */
IFX_INTERRUPT(ISR_qspi4_Tx, 0, ISR_PRIORITY_QSPI4_TX);
IFX_INTERRUPT(ISR_qspi4_Rx, 0, ISR_PRIORITY_QSPI4_RX);
IFX_INTERRUPT(ISR_qspi4_Er, 0, ISR_PRIORITY_QSPI4_ER);

/** \} */

/** \} */

/** \brief Handle qspi4_Tx interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI4
 * \isrPriority \ref ISR_PRIORITY_QSPI4
 *
 */
void ISR_qspi4_Tx(void)
{
    IfxCpu_enableInterrupts();
#ifdef QSPI4_USE_DMA
    IfxQspi_SpiMaster_isrDmaTransmit(&spi4Master);
#else
    IfxQspi_SpiMaster_isrTransmit(&spi4Master);
#endif

#ifdef QSPI4_TRANSMIT_CALLBACK
    QSPI4_TRANSMIT_CALLBACK();
#endif
}


/** \brief Handle qspi4_Rx interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI4
 * \isrPriority \ref ISR_PRIORITY_QSPI4
 *
 */
void ISR_qspi4_Rx(void)
{
    IfxCpu_enableInterrupts();
#ifdef QSPI4_USE_DMA
    IfxQspi_SpiMaster_isrDmaReceive(&spi4Master);
#else
    IfxQspi_SpiMaster_isrReceive(&spi4Master);
#endif
#ifdef QSPI4_RECEIVE_CALLBACK
    QSPI4_RECEIVE_CALLBACK();
#endif
}

/** \brief Handle qspi4_Er interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI4
 * \isrPriority \ref ISR_PRIORITY_QSPI4
 *
 */
void ISR_qspi4_Er(void)
{
    IfxCpu_enableInterrupts();
    IfxQspi_SpiMaster_isrError(&spi4Master);
}

/** \brief QSPI4 initialisation
 *
 * This function initialises the Qspi4 in master mode.
 */
IfxQspi_SpiMaster *qspi4_init (void)
{

    /* disable interrupts */
    boolean interruptState = IfxCpu_disableInterrupts();

    IfxQspi_SpiMaster_Config        spiMasterConfig;

    if (spi4Master.qspi == 0)
    {
        /* create module config */
        IfxQspi_SpiMaster_initModuleConfig(&spiMasterConfig, &MODULE_QSPI4);

        /* set the maximum baudrate */
        spiMasterConfig.base.maximumBaudrate = QSPI4_MAX_BAUDRATE;

        /* ISR priorities and interrupt target */
        spiMasterConfig.base.txPriority  = ISR_PRIORITY_QSPI4_TX;
        spiMasterConfig.base.rxPriority  = ISR_PRIORITY_QSPI4_RX;
        spiMasterConfig.base.erPriority  = ISR_PRIORITY_QSPI4_ER;
        spiMasterConfig.base.isrProvider = ISR_PROVIDER_QSPI4;

#ifdef QSPI4_USE_DMA
        // DMA configuration
        spiMasterConfig.dma.txDmaChannelId = DMA_CH_QSPI4_TX;
        spiMasterConfig.dma.rxDmaChannelId = DMA_CH_QSPI4_RX;
        spiMasterConfig.dma.useDma = 1;
#endif
        spiMasterConfig.pins = &qspi4_pins;

        /* initialize module */
        IfxQspi_SpiMaster_initModule(&spi4Master, &spiMasterConfig);
        /* set the MRST_input also to the selected pad driver mode if needed */
        if (qspi4_pins.mrst != NULL_PTR)
            IfxPort_setPinPadDriver(qspi4_pins.mrst->pin.port, qspi4_pins.mrst->pin.pinIndex, qspi4_pins.pinDriver);
    }

    /* enable interrupts again */
    IfxCpu_restoreInterrupts(interruptState);

  return (&spi4Master);
}
