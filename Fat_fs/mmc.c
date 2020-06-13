/*------------------------------------------------------------------------*/
/* MMCv3/SDv1/SDv2 (SPI mode) control module                              */
/*------------------------------------------------------------------------*/
/*
/  Copyright (C) 2010, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/


#include <Cpu/Std/Ifx_Types.h>
#include "Configuration.h"
#include "diskio.h"

#define	_USE_CD		1	/* Use card detect switch */
#define _USE_WP		0	/* Use write protect switch */


/* MMC/SD command */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */

/* MMC control port and function controls  (Platform dependent) */
#define CS_LOW()	{ IfxPort_setPinLow(SDCARD_USE_CHIPSELECT.pin.port, SDCARD_USE_CHIPSELECT.pin.pinIndex);	}   /* MMC CS = L */
#define	CS_HIGH()	{ IfxPort_setPinHigh(SDCARD_USE_CHIPSELECT.pin.port, SDCARD_USE_CHIPSELECT.pin.pinIndex);   }	/* MMC CS = H */
#define SOCKPORT SDCARD_USE_CD.port->IN.U
#define SOCKWP 0 //not used
#define SOCKINS 0x1<<SDCARD_USE_CD.pinIndex

/* Set slow clock (100k-400k) */
#define	FCLK_SLOW()	{	SpiIf_ChConfig chConfig; \
	                    chConfig.baudrate = 300000; \
	                    chConfig.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge; \
	                    chConfig.mode.clockPolarity = SpiIf_ClockPolarity_idleLow; \
	                    chConfig.mode.parityCheck = 0; \
	                    IfxQspi_SpiMaster_writeExtendedConfiguration(&g_Qspi_Mmc.drivers.spiMasterChannel, IfxQspi_calculateExtendedConfigurationValue(g_Qspi_Mmc.drivers.spiMaster->qspi, 0, &chConfig));}
/* Set fast clock (MMC:20MHz max, SD:25MHz max) */
#define	FCLK_FAST()	{	/* we need a special value here because calculation will not result in the needed value */ \
	                    /* defined in configuration.h dependant of the fbaud2 */ \
	                    IfxQspi_SpiMaster_writeExtendedConfiguration(&g_Qspi_Mmc.drivers.spiMasterChannel, SDCARD_FCLK_FAST_VALUE);}

typedef struct
{
    struct
    {
        IfxQspi_SpiMaster         *spiMaster;            /**< \brief Pointer to spi Master handle */
        IfxQspi_SpiMaster_Channel spiMasterChannel;      /**< \brief Spi Master Channel handle */
    }drivers;
}  App_Qspi_Mmc;


/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/
#if SDCARD_VAR_LOCATION == 0
	#if defined(__GNUC__)
    #pragma section
	#pragma section ".bss_cpu0" awc0
	#endif
	#if defined(__TASKING__)
	#pragma section farbss "bss_cpu0"
	#endif
	#if defined(__DCC__)
	#pragma section DATA ".data_cpu0" ".bss_cpu0" far-absolute RW
	#endif
#elif SDCARD_VAR_LOCATION == 1
	#if defined(__GNUC__)
	#pragma section ".bss_cpu1" awc1
	#endif
	#if defined(__TASKING__)
	#pragma section farbss "bss_cpu1"
	#endif
	#if defined(__DCC__)
	#pragma section DATA ".data_cpu1" ".bss_cpu1" far-absolute RW
	#endif
#elif SDCARD_VAR_LOCATION == 2
	#if defined(__GNUC__)
	#pragma section ".bss_cpu2" awc2
	#endif
	#if defined(__TASKING__)
	#pragma section farbss "bss_cpu2"
	#endif
	#if defined(__DCC__)
	#pragma section DATA ".data_cpu2" ".bss_cpu2" far-absolute RW
	#endif
#else
#error "Set SDCARD_VAR_LOCATION to a valid value!"
#endif

// in case of changing FONT_YSIZE or TFT_XSIZE recalculate the alignment value in power of 2
App_Qspi_Mmc g_Qspi_Mmc;

volatile
DSTATUS Stat = STA_NOINIT;	/* Physical drive status */

volatile
WORD Timer1, Timer2;	/* 20Hz decrement timer stopped at zero (disk_timerproc()) */

BYTE CardType;			/* Card type flags */

#if defined(__GNUC__)
#pragma section
#endif
#if defined(__TASKING__)
#pragma section farbss restore
#endif
#if defined(__DCC__)
#pragma section DATA RW
#endif

/*-----------------------------------------------------------------------*/
/* Send a byte to MMC  (Platform dependent)                              */
/*-----------------------------------------------------------------------*/

/* Send a byte */
void xmit_spi (
	BYTE dat	/* Data to send */
)
{
    uint8 tx_data;
    /* we can use stack variables for exchange because we are waiting until the transfer is finished */

    /* wait until Spi is no longer busy (should not busy here) */
    while (IfxQspi_SpiMaster_getStatus(&g_Qspi_Mmc.drivers.spiMasterChannel) == SpiIf_Status_busy) {};

    tx_data = dat;

    IfxQspi_SpiMaster_exchange(&g_Qspi_Mmc.drivers.spiMasterChannel, &tx_data, 0, 1);

    /* wait until our datas are valid */
    while (IfxQspi_SpiMaster_getStatus(&g_Qspi_Mmc.drivers.spiMasterChannel) == SpiIf_Status_busy) {};
}

/* Send multiple byte */
void xmit_spi_multi (
	const BYTE *buff,	/* Pointer to the data */
	UINT btx			/* Number of bytes to send */
)
{
    /* wait until Spi is no longer busy (should not busy here) */
    while (IfxQspi_SpiMaster_getStatus(&g_Qspi_Mmc.drivers.spiMasterChannel) == SpiIf_Status_busy) {};

    IfxQspi_SpiMaster_exchange(&g_Qspi_Mmc.drivers.spiMasterChannel, buff, 0, (Ifx_SizeT)btx);

    /* wait don't wait our next call to xmit_spi will wait for us if needed */
}



/*-----------------------------------------------------------------------*/
/* Receive data from MMC  (Platform dependent)                           */
/*-----------------------------------------------------------------------*/

/* Receive a byte */
BYTE rcvr_spi (void)
{
    uint8 rx_data;
    /* we can use stack variables for exchange because we are waiting until the transfer is finished */

    /* wait until Spi is no longer busy (should not busy here) */
    while (IfxQspi_SpiMaster_getStatus(&g_Qspi_Mmc.drivers.spiMasterChannel) == SpiIf_Status_busy) {};

    /* we don't have explizit send values, our driver will send 0xFF in this case (what we need) */
    IfxQspi_SpiMaster_exchange(&g_Qspi_Mmc.drivers.spiMasterChannel, 0, &rx_data, 1);

    /* wait until our datas are valid */
    while (IfxQspi_SpiMaster_getStatus(&g_Qspi_Mmc.drivers.spiMasterChannel) == SpiIf_Status_busy) {};

    return(rx_data);
}

/* Receive multiple byte */
void rcvr_spi_multi (
	BYTE *buff,		/* Pointer to data buffer */
	UINT btr		/* Number of bytes to receive */
)
{
    /* wait until Spi is no longer busy (should not busy here) */
    while (IfxQspi_SpiMaster_getStatus(&g_Qspi_Mmc.drivers.spiMasterChannel) == SpiIf_Status_busy) {};

    /* we don't have explizit send values, our driver will send 0xFF in this case (what we need) */
    IfxQspi_SpiMaster_exchange(&g_Qspi_Mmc.drivers.spiMasterChannel, 0, buff, (Ifx_SizeT)btr);

    /* wait don't wait our next call to rcvr_spi will wait for us if needed */
}




/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

int wait_ready (	/* 1:Ready, 0:Timeout */
	UINT wt			/* Timeout [ms] */
)
{
	// our timer2 is incremented each 10ms
	Timer2 = (WORD)(wt/10);

	do {
		if (rcvr_spi() == 0xFF) return 1;	/* Card goes ready */
		/* This loop takes a time. Insert rot_rdq() here for multitask envilonment. */
	} while (Timer2);	/* Wait until card goes ready or timeout */

	return 0;	/* Timeout occured */
}



/*-----------------------------------------------------------------------*/
/* Deselect card and release SPI                                         */
/*-----------------------------------------------------------------------*/

void deselect (void)
{
	CS_HIGH();		/* CS = H */
	rcvr_spi();		/* Dummy clock (force DO hi-z for multiple slave SPI */
}



/*-----------------------------------------------------------------------*/
/* Select card and wait for ready                                        */
/*-----------------------------------------------------------------------*/
int select (void)	/* 1:OK, 0:Timeout */
{
	CS_LOW();		/* CS = L */
	rcvr_spi();		/* Dummy clock (force DO enabled) */

	if (wait_ready(500)) return 1;	/* OK */

	deselect();
	return 0;	/* Timeout */
}



/*-----------------------------------------------------------------------*/
/* Control SPI module (Platform dependent)                               */
/*-----------------------------------------------------------------------*/
void power_on (void)	/* Enable SPI */
{
    IfxQspi_SpiMaster *spiMaster;
	/* Attach SPIx module to I/O pads */
    /* disable interrupts */
    boolean interruptState = IfxCpu_disableInterrupts();

    spiMaster = SDCARD_QSPI_INIT();

    if (g_Qspi_Mmc.drivers.spiMaster != spiMaster)
    {
        /* this is the first call, we initialize the channel */
    	g_Qspi_Mmc.drivers.spiMaster = spiMaster;
    	IfxQspi_SpiMaster_ChannelConfig spiMasterChannelConfig;
        {
            /* create channel config */
#if defined(__DCC__)
    		// bug on DCC not all bits in mode are cleared
    		memset(&spiMasterChannelConfig, 0, sizeof(spiMasterChannelConfig));
#endif
            IfxQspi_SpiMaster_initChannelConfig(&spiMasterChannelConfig,
                g_Qspi_Mmc.drivers.spiMaster);

            /* set the baudrate for this channel */
            spiMasterChannelConfig.base.baudrate = 50000000;

            /* set the transfer data width */
            spiMasterChannelConfig.base.mode.dataWidth = 8;
            spiMasterChannelConfig.base.mode.csLeadDelay = SpiIf_SlsoTiming_0;
            spiMasterChannelConfig.base.mode.csTrailDelay = SpiIf_SlsoTiming_0;
            spiMasterChannelConfig.base.mode.csInactiveDelay = SpiIf_SlsoTiming_0;
            spiMasterChannelConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;

            const IfxQspi_SpiMaster_Output slsOutput = {&SDCARD_USE_CHIPSELECT,
                                                        IfxPort_OutputMode_pushPull,
                                                        IfxPort_PadDriver_cmosAutomotiveSpeed1};

            spiMasterChannelConfig.sls.output.pin    = slsOutput.pin;
            spiMasterChannelConfig.sls.output.mode   = slsOutput.mode;
            spiMasterChannelConfig.sls.output.driver = slsOutput.driver;

            /* initialize channel */
            IfxQspi_SpiMaster_initChannel(&g_Qspi_Mmc.drivers.spiMasterChannel,
                &spiMasterChannelConfig);
            /** - Override the SLSO manually as general-purpose output */
            IfxPort_setPinHigh(slsOutput.pin->pin.port, slsOutput.pin->pin.pinIndex);
            IfxQspi_initSlso(slsOutput.pin, slsOutput.mode, slsOutput.driver, IfxPort_OutputIdx_general);

        }
    }

    /* enable interrupts again */
    IfxCpu_restoreInterrupts(interruptState);

    if (_USE_CD) { }	/* MMC_CD */
	if (_USE_WP) { }	/* MMC_WP */

}


void power_off (void)	/* Disable SPI function */
{
	select();				/* Wait for card ready */
	deselect();
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from the MMC                                    */
/*-----------------------------------------------------------------------*/
int rcvr_datablock (	/* 1:OK, 0:Error */
	BYTE *buff,			/* Data buffer */
	UINT btr			/* Data block length (byte) */
)
{
	BYTE token;


	Timer1 = 20;
	do {							/* Wait for DataStart token in timeout of 200ms */
		token = rcvr_spi();
		/* This loop will take a time. Insert rot_rdq() here for multitask envilonment. */
	} while ((token == 0xFF) && Timer1);
	if(token != 0xFE) return 0;		/* Function fails if invalid DataStart token or timeout */

	rcvr_spi_multi(buff, btr);		/* Store trailing data to the buffer */
	rcvr_spi(); rcvr_spi();			/* Discard CRC */

	return 1;						/* Function succeeded */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to the MMC                                         */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
int xmit_datablock (	/* 1:OK, 0:Failed */
	const BYTE *buff,	/* Ponter to 512 byte data to be sent */
	BYTE token			/* Token */
)
{
	BYTE resp;


	if (!wait_ready(500)) return 0;		/* Wait for card ready */

	xmit_spi(token);					/* Send token */
	if (token != 0xFD) {				/* Send data if token is other than StopTran */
		xmit_spi_multi(buff, 512);		/* Data */
		xmit_spi(0xFF); xmit_spi(0xFF);	/* Dummy CRC */

		resp = rcvr_spi();				/* Receive data resp */
		if ((resp & 0x1F) != 0x05)		/* Function fails if the data packet was not accepted */
			return 0;
	}
	return 1;
}
#endif /* _USE_WRITE */



/*-----------------------------------------------------------------------*/
/* Send a command packet to the MMC                                      */
/*-----------------------------------------------------------------------*/
BYTE send_cmd (		/* Return value: R1 resp (bit7==1:Failed to send) */
	BYTE cmd,		/* Command index */
	DWORD arg		/* Argument */
)
{
	BYTE n, res;


	if (cmd & 0x80) {	/* Send a CMD55 prior to ACMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select card */
	deselect();
	if (!select()) return 0xFF;

	/* Send command packet */
	xmit_spi(0x40 | cmd);				/* Start + command index */
	xmit_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xmit_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xmit_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xmit_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive command resp */
	if (cmd == CMD12) rcvr_spi();		/* Diacard following one byte when CMD12 */
	n = 10;								/* Wait for response (10 bytes max) */
	do
		res = rcvr_spi();
	while ((res & 0x80) && --n);

	return res;							/* Return received response */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE drv		/* Physical drive number (0) */
)
{
	BYTE n, cmd, ty, ocr[4];


	if (drv) return STA_NOINIT;			/* Supports only drive 0 */
	if (Stat & STA_NODISK) return Stat;	/* Is card existing in the socket? */

	power_on();							/* Initialize SPI */
	FCLK_SLOW();						/* Set slow clock */
	for (n = 10; n; n--)
		{ rcvr_spi();	/* Send 80 dummy clocks */

		}
	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Put the card SPI/Idle state */
		Timer1 = 100;						/* Initialization timeout = 1 sec */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
			for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();		/* Get 32 bit return value of R7 resp */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* Is the card supports vcc of 2.7-3.6V? */
				while (Timer1 && send_cmd(ACMD41, 1UL << 30)) ;	/* Wait for end of initialization with ACMD41(HCS) */
				if (Timer1 && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* Card id SDv2 */
				}
			}
		} else {	/* Not SDv2 card */
			if (send_cmd(ACMD41, 0) <= 1) 	{	/* SDv1 or MMC? */
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 (ACMD41(0)) */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 (CMD1(0)) */
			}
			while (Timer1 && send_cmd(cmd, 0)) ;		/* Wait for end of initialization */
			if (!Timer1 || send_cmd(CMD16, 512) != 0)	/* Set block length: 512 */
				ty = 0;
		}
	}
	CardType = ty;	/* Card type */
	deselect();

	if (ty) {			/* OK */
		FCLK_FAST();			/* Set fast clock */
		Stat &= ~STA_NOINIT;	/* Clear STA_NOINIT flag */
	} else {			/* Failed */
		power_off();
		Stat = STA_NOINIT;
	}

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE drv		/* Physical drive number (0) */
)
{
	if (drv) return STA_NOINIT;		/* Supports only drive 0 */

	return Stat;	/* Return disk status */
}



/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,		/* Physical drive number (0) */
	BYTE *buff,		/* Pointer to the data buffer to store read data */
	DWORD sector,	/* Start sector number (LBA) */
	BYTE count		/* Number of sectors to read (1..128) */
)
{
	if (drv || !count) return RES_PARERR;		/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check if drive is ready */

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* LBA ot BA conversion (byte addressing cards) */

	if (count == 1) {	/* Single sector read */
		if ((send_cmd(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
			&& rcvr_datablock(buff, 512))
			count = 0;
	}
	else {				/* Multiple sector read */
		if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512)) break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;	/* Return result */
}



/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE drv,			/* Physical drive number (0) */
	const BYTE *buff,	/* Ponter to the data to write */
	DWORD sector,		/* Start sector number (LBA) */
	BYTE count			/* Number of sectors to write (1..128) */
)
{
	if (drv || !count) return RES_PARERR;		/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check drive status */
	if (Stat & STA_PROTECT) return RES_WRPRT;	/* Check write protect */

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* LBA ==> BA conversion (byte addressing cards) */

	if (count == 1) {	/* Single sector write */
		if ((send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple sector write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;	/* Return result */
}
#endif /* _USE_WRITE */



/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive number (0) */
	BYTE ctrl,		/* Control command code */
	void *buff		/* Pointer to the conrtol data */
)
{
	DRESULT res;
	BYTE n, csd[16], *ptr = buff;
	DWORD *dp, st, ed, csz;


	if (drv) return RES_PARERR;					/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check if drive is ready */

	res = RES_ERROR;

	switch (ctrl) {
	case CTRL_SYNC :		/* Wait for end of internal write process of the drive */
		if (select()) {
			deselect();
			res = RES_OK;
		}
		break;

	case GET_SECTOR_COUNT :	/* Get drive capacity in unit of sector (DWORD) */
		if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
			if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
				csz = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
				*(DWORD*)buff = csz << 10;
			} else {					/* SDC ver 1.XX or MMC ver 3 */
				n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
				csz = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
				*(DWORD*)buff = csz << (n - 9);
			}
			res = RES_OK;
		}
		break;

	case GET_SECTOR_SIZE :	/* Get sector size in unit of byte (WORD) */
		*(WORD*)buff = 512;
		res = RES_OK;
		break;

	case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
		if (CardType & CT_SD2) {	/* SDC ver 2.00 */
			if (send_cmd(ACMD13, 0) == 0) {	/* Read SD status */
				rcvr_spi();
				if (rcvr_datablock(csd, 16)) {				/* Read partial block */
					for (n = 64 - 16; n; n--) rcvr_spi();	/* Purge trailing data */
					*(DWORD*)buff = 16UL << (csd[10] >> 4);
					res = RES_OK;
				}
			}
		} else {					/* SDC ver 1.XX or MMC */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
				if (CardType & CT_SD1) {	/* SDC ver 1.XX */
					*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
				} else {					/* MMC */
					*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
				}
				res = RES_OK;
			}
		}
		break;

	case CTRL_ERASE_SECTOR :	/* Erase a block of sectors (used when _USE_ERASE == 1) */
		if (!(CardType & CT_SDC)) break;				/* Check if the card is SDC */
		if (disk_ioctl(drv, MMC_GET_CSD, csd)) break;	/* Get CSD */
		if (!(csd[0] >> 6) && !(csd[10] & 0x40)) break;	/* Check if sector erase can be applied to the card */
		dp = buff; st = dp[0]; ed = dp[1];				/* Load sector block */
		if (!(CardType & CT_BLOCK)) {
			st *= 512; ed *= 512;
		}
		if (send_cmd(CMD32, st) == 0 && send_cmd(CMD33, ed) == 0 && send_cmd(CMD38, 0) == 0 && wait_ready(30000))	/* Erase sector block */
			res = RES_OK;
		break;

	/* Following command are not used by FatFs module */

	case MMC_GET_TYPE :		/* Get MMC/SDC type (BYTE) */
		*ptr = CardType;
		res = RES_OK;
		break;

	case MMC_GET_CSD :		/* Read CSD (16 bytes) */
		if (send_cmd(CMD9, 0) == 0		/* READ_CSD */
			&& rcvr_datablock(ptr, 16))
			res = RES_OK;
		break;

	case MMC_GET_CID :		/* Read CID (16 bytes) */
		if (send_cmd(CMD10, 0) == 0		/* READ_CID */
			&& rcvr_datablock(ptr, 16))
			res = RES_OK;
		break;

	case MMC_GET_OCR :		/* Read OCR (4 bytes) */
		if (send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
			for (n = 4; n; n--) *ptr++ = rcvr_spi();
			res = RES_OK;
		}
		break;

	case MMC_GET_SDSTAT :	/* Read SD status (64 bytes) */
		if (send_cmd(ACMD13, 0) == 0) {	/* SD_STATUS */
			rcvr_spi();
			if (rcvr_datablock(ptr, 64))
				res = RES_OK;
		}
		break;

	default:
		res = RES_PARERR;
	}

	deselect();

	return res;
}
#endif /* _USE_IOCTL */


/*-----------------------------------------------------------------------*/
/* Device timer function  (Platform dependent)                           */
/*-----------------------------------------------------------------------*/
/* This function must be called from timer interrupt routine in period
/  of 10 ms to generate card control timing.
*/

void disk_timerproc (void)
{
	static WORD pv;
	WORD n;
	BYTE s;


	n = Timer1;						/* 100Hz decrement timer stopped at 0 */
	if (n) Timer1 = --n;
	n = Timer2;
	if (n) Timer2 = --n;

	n = pv;
	pv = SOCKPORT & (SOCKWP | SOCKINS);	/* Sample socket status */

	if (n == pv) {						/* Has contact stabled? */
		s = Stat;
		if (_USE_WP && (pv & SOCKWP))	/* WP == H (Write protected) */
			s |= STA_PROTECT;
		else							/* WP == L (Write enabled) */
			s &= ~STA_PROTECT;

		if (_USE_CD && (pv & SOCKINS))	/* CD == H (No card) */
			s |= (STA_NODISK | STA_NOINIT);
		else							/* CD == L (Card is exist) */
			s &= ~STA_NODISK;

		Stat = s;
	}
}

