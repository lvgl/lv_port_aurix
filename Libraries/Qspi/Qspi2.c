/**
 * \file 
 * \brief QSPI2 Driver Initialisation
 *
 * This Driver manage the global initialisation of QSPI2.
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <Cpu/Std/Ifx_Types.h>
#include "Configuration.h"
#include "Qspi2.h"


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
#if ISR_PROVIDER_QSPI2 == 0
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
#elif ISR_PROVIDER_QSPI2 == 1
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
#elif ISR_PROVIDER_QSPI2 == 2
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
#error "Set ISR_PROVIDER_QSPI2 to a valid value!"
#endif

IfxQspi_SpiMaster spi2Master={{ 0, 0, 0, 0, 0, { 0, 0, 0, 0, 0}}, 0,{{ 0, 0, 0},{ 0, 0, 0}, 0, 0, 0}}; /**< \brief Qspi2 global data */

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
#ifdef QSPI2_TRANSMIT_CALLBACK
    extern void QSPI2_TRANSMIT_CALLBACK(void);
#endif
#ifdef QSPI2_RECEIVE_CALLBACK
    extern void QSPI2_RECEIVE_CALLBACK(void);
#endif

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/* pin configuration */
const IfxQspi_SpiMaster_Pins qspi2_pins = {&QSPI2_SCLK_PIN, IfxPort_OutputMode_pushPull,  /* SCLK */
                                           &QSPI2_MTSR_PIN,  IfxPort_OutputMode_pushPull, /* MTSR */
                                           &QSPI2_MRST_PIN,  IfxPort_InputMode_pullDown,  /* MRST */
                                           IfxPort_PadDriver_ttlSpeed1		              /* pad driver mode */
};

/******************************************************************************/
/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
/** \addtogroup IfxLld_Demo_QspiCpu_SrcDoc_Main_Interrupt
 * \{ */

/** \name Interrupts for Qspi Master (QSPI2)
 * \{ */
IFX_INTERRUPT(ISR_qspi2_Tx, 0, ISR_PRIORITY_QSPI2_TX);
IFX_INTERRUPT(ISR_qspi2_Rx, 0, ISR_PRIORITY_QSPI2_RX);
IFX_INTERRUPT(ISR_qspi2_Er, 0, ISR_PRIORITY_QSPI2_ER);

/** \} */

/** \} */

/** \brief Handle qspi2_Tx interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI2
 * \isrPriority \ref ISR_PRIORITY_QSPI2
 *
 */
void ISR_qspi2_Tx(void)
{
    IfxCpu_enableInterrupts();
#ifdef QSPI2_USE_DMA
    IfxQspi_SpiMaster_isrDmaTransmit(&spi2Master);
#else
    IfxQspi_SpiMaster_isrTransmit(&spi2Master);
#endif

#ifdef QSPI2_TRANSMIT_CALLBACK
    QSPI2_TRANSMIT_CALLBACK();
#endif
}


/** \brief Handle qspi2_Rx interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI2
 * \isrPriority \ref ISR_PRIORITY_QSPI2
 *
 */
void ISR_qspi2_Rx(void)
{
    IfxCpu_enableInterrupts();
#ifdef QSPI2_USE_DMA
    IfxQspi_SpiMaster_isrDmaReceive(&spi2Master);
#else
    IfxQspi_SpiMaster_isrReceive(&spi2Master);
#endif
#ifdef QSPI2_RECEIVE_CALLBACK
    QSPI2_RECEIVE_CALLBACK();
#endif
}

/** \brief Handle qspi2_Er interrupt.
 *
 * \isrProvider \ref ISR_PROVIDER_QSPI2
 * \isrPriority \ref ISR_PRIORITY_QSPI2
 *
 */
void ISR_qspi2_Er(void)
{
    IfxCpu_enableInterrupts();
    IfxQspi_SpiMaster_isrError(&spi2Master);
}

/** \brief QSPI2 initialisation
 *
 * This function initialises the Qspi2 in master mode.
 */
IfxQspi_SpiMaster *qspi2_init (void)
{

    /* disable interrupts */
    boolean interruptState = IfxCpu_disableInterrupts();

    IfxQspi_SpiMaster_Config        spiMasterConfig;

    if (spi2Master.qspi == 0)
    {
        /* create module config */
        IfxQspi_SpiMaster_initModuleConfig(&spiMasterConfig, &MODULE_QSPI2);

        /* set the maximum baudrate */
        spiMasterConfig.base.maximumBaudrate = QSPI2_MAX_BAUDRATE;

        /* ISR priorities and interrupt target */
        spiMasterConfig.base.txPriority  = ISR_PRIORITY_QSPI2_TX;
        spiMasterConfig.base.rxPriority  = ISR_PRIORITY_QSPI2_RX;
        spiMasterConfig.base.erPriority  = ISR_PRIORITY_QSPI2_ER;
        spiMasterConfig.base.isrProvider = ISR_PROVIDER_QSPI2;

#ifdef QSPI2_USE_DMA
        // DMA configuration
        spiMasterConfig.dma.txDmaChannelId = DMA_CH_QSPI2_TX;
        spiMasterConfig.dma.rxDmaChannelId = DMA_CH_QSPI2_RX;
        spiMasterConfig.dma.useDma = 1;
#endif
        spiMasterConfig.pins = &qspi2_pins;

        /* initialize module */
        IfxQspi_SpiMaster_initModule(&spi2Master, &spiMasterConfig);
        /* set the MRST_input also to the selected pad driver mode if needed */
        if (qspi2_pins.mrst != NULL_PTR)
            IfxPort_setPinPadDriver(qspi2_pins.mrst->pin.port, qspi2_pins.mrst->pin.pinIndex, qspi2_pins.pinDriver);
    }

    /* enable interrupts again */
    IfxCpu_restoreInterrupts(interruptState);

  return (&spi2Master);
}
