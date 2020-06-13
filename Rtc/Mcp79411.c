/**
 * \file 
 * \brief RTC Driver
 *
 * The RTC Driver gets times, alarms, Unique ID from a MCP79411 via I2C.
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <Cpu/Std/Ifx_Types.h>
#include <Scu/Std/IfxScuEru.h>
#include <I2c/I2c/IfxI2c_I2c.h>
#include <Gtm/Std/IfxGtm_Cmu.h>
#include <Gtm/Std/IfxGtm_Tim.h>
#include "Configuration.h"
#include "RTC.h"


/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define RTC_BUFFER_SIZE 12   /**< \brief Tx/Rx Buffer size */

#define SECONDS         0        //On position 0 in Rx-buffer-array
#define MINUTES         1
#define HOURS           2
#define DAY             3
#define DATE            4
#define MONTH           5
#define YEAR            6
#define CONTROL         7

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
typedef struct
{
    uint8 i2cTxBuffer[RTC_BUFFER_SIZE];                  /**< \brief I2c Transmit buffer */
    uint8 i2cRxBuffer[RTC_BUFFER_SIZE];                  /**< \brief I2c receive buffer */
} AppI2c_RTC_Buffer;

/** \brief I2cCpu global data */
typedef struct
{
    AppI2c_RTC_Buffer i2cBuffer;                       /**< \brief I2c buffer */
    struct
    {
    	IfxI2c_I2c               i2cHandle;            /**< \brief I2c handle */
    	IfxI2c_I2c_Device        i2cDevRtc;            /**< \brief I2c Device handle to RTC  */
    	IfxI2c_I2c_Device        i2cDevEeprom;         /**< \brief I2c Device handle to Eeprom */
    }drivers;
} App_I2c_RTC_Cpu;


/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
#if RTC_VAR_LOCATION == 0
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
#elif RTC_VAR_LOCATION == 1
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
#elif RTC_VAR_LOCATION == 2
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
#error "Set RTC_VAR_LOCATION to a valid value!"
#endif

App_I2c_RTC_Cpu g_I2c_RTC_Cpu; /**< \brief I2c global data */
boolean rtc_calibration_finished;
boolean rtc_cal_sign;
uint8  saved_control_register, saved_cal_register, saved_hours_register;

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

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/
/* pin configuration */
const IfxI2c_Pins rtc_pins = {&RTC_SCL_PIN,                                   /* SCL */
                              &RTC_SDA_PIN,                                   /* SDA */
                              IfxPort_PadDriver_ttlSpeed1		              /* pad driver mode */
};
/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
/**
 * @brief Interrupt service routine if the alarm is activated
 */
IFX_INTERRUPT(Interrupt_from_RTC, 0, ISR_PRIORITY_RTC_ALARM)
{
    //conio_driver.dialogmode = SHOWALARMON;
}

/** \brief RTC initialization
 *
 * This function initializes RTC via I2c in master mode.
 */
sint32 rtc_init (void)
{
    /* disable interrupts */
    boolean interruptState = IfxCpu_disableInterrupts();

    // create config structure
    IfxI2c_I2c_Config i2cConfig;
    // fill structure with default values and Module address
    IfxI2c_I2c_initConfig(&i2cConfig, &MODULE_I2C0);
    // configure pins
    i2cConfig.pins = &rtc_pins;
    i2cConfig.baudrate = 400000; // 200 kHz
    // initialize module
    IfxI2c_I2c_initModule(&g_I2c_RTC_Cpu.drivers.i2cHandle, &i2cConfig);

    {
    	// create device config
    	IfxI2c_I2c_deviceConfig i2cDeviceConfig;
    	// fill structure with default values and i2c Handler
    	IfxI2c_I2c_initDeviceConfig(&i2cDeviceConfig, &g_I2c_RTC_Cpu.drivers.i2cHandle);
    	// set device specific values.
    	// configure Slave address as 8-bit value. in case of 7 bit address left shift it by 1.
    	// E.g in case of EEPROM,slave address is 7 bit represented as 0x50, after left shifting it by 1, it will be 0xa0
    	i2cDeviceConfig.deviceAddress = 0x6F<<1; // device / slave address, RTCC and SRA;
    	// initialize the i2c device handle
    	IfxI2c_I2c_initDevice(&g_I2c_RTC_Cpu.drivers.i2cDevRtc, &i2cDeviceConfig);

    	i2cDeviceConfig.deviceAddress = 0x57<<1; // device / slave address, eeprom
    	// initialize the i2c device handle
    	IfxI2c_I2c_initDevice(&g_I2c_RTC_Cpu.drivers.i2cDevEeprom, &i2cDeviceConfig);

        // we setup the ERU0 interrupt
        IfxSrc_init(RTC_ALARM_SRC, ISR_PROVIDER_RTC_ALARM, ISR_PRIORITY_RTC_ALARM);
        IfxSrc_enable(RTC_ALARM_SRC);
        // initialization of ERU
        IfxScuEru_initReqPin(&RTC_ALARM_PIN_INPUT, IfxPort_InputMode_pullDown);
        IfxScuEru_enableRisingEdgeDetection(RTC_ALARM_PIN_INPUT.channelId);
        // we use the OGU0
        IfxScuEru_connectTrigger(RTC_ALARM_PIN_INPUT.channelId, IfxScuEru_InputNodePointer_0);
        IfxScuEru_enableTriggerPulse(RTC_ALARM_PIN_INPUT.channelId);
        IfxScuEru_setInterruptGatingPattern(IfxScuEru_OutputChannel_0, IfxScuEru_InterruptGatingPattern_alwaysActive);

    }


    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0] = 0;
    g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[0] = 0;

    rtc_calibration_finished = FALSE;
    rtc_cal_sign = FALSE;
    /* enable interrupts again */
    IfxCpu_restoreInterrupts(interruptState);

  return (0);
}

/*!
 * \brief   gets the actual time 
 *
 * ...
 */

sint32
rtc_gettime (TIME *pActTime)
{
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0] =  SECONDS;     //address of SECONDS register

	// write data to device as soon as it is ready
	while(IfxI2c_I2c_write(&g_I2c_RTC_Cpu.drivers.i2cDevRtc, &g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0], 1) == IfxI2c_I2c_Status_nak);

    // read the time registers
	while(IfxI2c_I2c_read(&g_I2c_RTC_Cpu.drivers.i2cDevRtc, &g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[0], 8) == IfxI2c_I2c_Status_nak);

    if(g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[DAY] & 0x20)
        pActTime->flags.B.running = 1;
    else
        pActTime->flags.B.running = 0;

    pActTime->seconds    = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[SECONDS] & 0x7F;
    pActTime->minutes    = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[MINUTES] & 0x7F;

    // if 12 hour clock
    if(g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[HOURS] & 0x40)
    {
        pActTime->flags.B.hour_12 = 1;
        pActTime->hours        = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[HOURS] & 0x1F;

        // check if PM
        if(g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[HOURS] & 0x20)
        {
            pActTime->flags.B.pm_set = 1;
        }
    }

    // 24 hour clock
    else
    {
        pActTime->hours        = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[HOURS] & 0x3F;
        pActTime->flags.B.hour_12 = 0;
        pActTime->flags.B.pm_set = 0;

    }

    pActTime->day        = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[DAY] & 0x7;
    pActTime->date        = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[DATE] & 0x3F;
    pActTime->month        = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[MONTH] & 0x1F;
    pActTime->year        = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[YEAR];

    // check for leap year
    if((g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[MONTH] & 0x20) == 0x20)
        pActTime->flags.B.leap_year = 1;
    else
        pActTime->flags.B.leap_year = 0;

    //set/reset alarm-enable status-flags
    if(g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[CONTROL] & 0x10)
        pActTime->flags.B.alarm0 = 1;
    else
        pActTime->flags.B.alarm0 = 0;

    if(g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[CONTROL] & 0x20)
        pActTime->flags.B.alarm1 = 1;
    else
        pActTime->flags.B.alarm1 = 0;

    return (0);
}

/*!
 * \brief   gets the alarm time (0 or 1)
 *
 * ...
 */

sint32
rtc_getalarm (TIME *pActTime, uint8 ucAlarmNumber)
{

    if (ucAlarmNumber > 1) return (1);

    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0] =  0x0A + ucAlarmNumber*7;        //Register = seconds of alarm[ucAlarmNumber]
    // read the alarm registers
	while(IfxI2c_I2c_write(&g_I2c_RTC_Cpu.drivers.i2cDevRtc, &g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0], 1) == IfxI2c_I2c_Status_nak);

    // read the alarm registers
	while(IfxI2c_I2c_read(&g_I2c_RTC_Cpu.drivers.i2cDevRtc, &g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[0], 6) == IfxI2c_I2c_Status_nak);

    pActTime->seconds    = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[SECONDS] & 0x7F;
    pActTime->minutes    = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[MINUTES] & 0x7F;

    // if 12 hour clock
    if(g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[HOURS] & 0x40)
    {
        pActTime->flags.B.hour_12 = 1;
        pActTime->hours        = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[HOURS] & 0x1F;

        // check if PM
        if(g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[HOURS] & 0x20)
        {
            pActTime->flags.B.pm_set = 1;
        }
    }
    // 24 hour clock
    else
    {
        pActTime->hours        = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[HOURS] & 0x3F;
        pActTime->flags.B.hour_12 = 0;
        pActTime->flags.B.pm_set = 0;

    }

    pActTime->day        = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[DAY] & 0x7;
    pActTime->date       = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[DATE] & 0x3F;
    pActTime->month      = g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[MONTH] & 0x1F;


    return (0);
}


/*!
 * \brief   sets the actual time and flags
 *
 * ...
 */

sint32
rtc_settime (TIME *pActTime)
{

    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0] =  0x00;     //address of SECONDS register

    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[1] =  pActTime->seconds | 0x80; // also set the oscillator start bit, otherwise oscillator will be stopped
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[2] =  pActTime->minutes;
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[3] =  pActTime->hours | ((pActTime->flags.B.pm_set << 5) | (pActTime->flags.B.hour_12 << 6));
    if (rtc_cal_sign == TRUE) g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[HOURS] |= 0x80;
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[4] =  pActTime->day | 0x8; //Enable supply over external battery
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[5] =  pActTime->date;
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[6] =  pActTime->month | (pActTime->flags.B.leap_year << 5);
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[7] =  pActTime->year;
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[8] =  (pActTime->flags.B.alarm0 << 4) | (pActTime->flags.B.alarm1 << 5) | 0x80; // control register, MFP high if not used

	// write data to device as soon as it is ready
	while(IfxI2c_I2c_write(&g_I2c_RTC_Cpu.drivers.i2cDevRtc, &g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0], 9) == IfxI2c_I2c_Status_nak);

    return (0);
}

/*!
 * \brief   sets an alarm time (0 or 1)
 *
 * ...
 */

sint32
rtc_setalarm (TIME *pAlarmTime, uint8 ucAlarmNumber, uint8 ucAlarmMatch)
{
    if (ucAlarmNumber > 1) return (1);

    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0] =  0x0A + ucAlarmNumber*7;        //Register = seconds of alarm[ucAlarmNumber]

    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[1] =  pAlarmTime->seconds;
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[2] =  pAlarmTime->minutes;
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[3] =  pAlarmTime->hours | ((pAlarmTime->flags.B.pm_set << 5) | (pAlarmTime->flags.B.hour_12 << 6));
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[4] =  pAlarmTime->day | (ucAlarmMatch << 4);
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[5] =  pAlarmTime->date;
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[6] =  pAlarmTime->month;

	// write data to device as soon as it is ready
	while(IfxI2c_I2c_write(&g_I2c_RTC_Cpu.drivers.i2cDevRtc, &g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0], 7) == IfxI2c_I2c_Status_nak);

	return (0);
}

/*!
 * \brief   read a specific rtc register
 *
 * ...
 */
uint8 rtc_read_register(uint8 register_number)
{
	// we read any register
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0] =  register_number;   //Register
	// write data to device as soon as it is ready
	while(IfxI2c_I2c_write(&g_I2c_RTC_Cpu.drivers.i2cDevRtc, &g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0], 1) == IfxI2c_I2c_Status_nak);
    // read the register
	while(IfxI2c_I2c_read(&g_I2c_RTC_Cpu.drivers.i2cDevRtc, &g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[0], 1) == IfxI2c_I2c_Status_nak);

    return g_I2c_RTC_Cpu.i2cBuffer.i2cRxBuffer[0];
}

/*!
 * \brief   write a specific rtc register
 *
 * ...
 */
void rtc_write_register(uint8 register_number, uint8 value)
{
	// we write any register
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0] =  register_number;   //Register
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[1] =  value;             //value
	// write data to device as soon as it is ready
	while(IfxI2c_I2c_write(&g_I2c_RTC_Cpu.drivers.i2cDevRtc, &g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0], 2) == IfxI2c_I2c_Status_nak);
}

/*!
 * \brief   activate or deactivate an alarm (0 or 1)
 *
 * ...
 */

sint32
rtc_alarm_onoff (uint8 ucAlarmNumber, uint8 ucOn)
{
    uint8 alarm_state;

    if (ucAlarmNumber > 1) return (1);

    uint8 alarm;

    alarm = rtc_read_register(0x07);
    // we remove the SQWE if set
    alarm &= ~0x40;

    alarm_state = alarm >> (4 + ucAlarmNumber);    // mask out current alarm_state
    alarm_state &= 1;

    // check if states are different
    if(alarm_state != ucOn)
    {
        if(ucOn == 1)
            //enable alarm
            rtc_write_register( 0x07, alarm | (1 << (4 + ucAlarmNumber)));
        else
            //disable alarm
            rtc_write_register( 0x07, alarm & (uint8)(~(1 << (4 + ucAlarmNumber))));
    }
    else
    {
        // we are here and no alarm set, we write back to remove the SQWE
        rtc_write_register( 0x07, alarm);
    }

    return (0);
}

/*!
 * \brief   reset the alarm flags (0 or 1) 
 *
 * ...
 */

sint32
rtc_reset_alarmflag (uint8 ucAlarmNumber)
{
    uint8 alarmflags;

    if (ucAlarmNumber > 1) return (1);

    alarmflags = rtc_read_register(0x0D + ucAlarmNumber * 7);        //Register = alarm day register

    if((alarmflags & 0x8) == 0x8)
    {
        alarmflags = alarmflags & 0x07;

        //send the flags to reset of flags
        rtc_write_register(0x0D + ucAlarmNumber * 7, alarmflags);
    }

    return (0);
}

/*!
 * \brief   reset the alarm flags (0 or 1)
 *
 * ...
 */

sint32
rtc_get_unique_id (uint8 *pUniqueId)
{
    g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0] =  0xF0;     //address of unique ID location

	// write data to device as soon as it is ready
	while(IfxI2c_I2c_write(&g_I2c_RTC_Cpu.drivers.i2cDevEeprom, &g_I2c_RTC_Cpu.i2cBuffer.i2cTxBuffer[0], 1) == IfxI2c_I2c_Status_nak);

    // read the unique id value
	while(IfxI2c_I2c_read(&g_I2c_RTC_Cpu.drivers.i2cDevEeprom, pUniqueId, 8) == IfxI2c_I2c_Status_nak);

    return (0);
}

/*!
 * \brief   measurement from rtc calibration finished
 *
 * ...
 */
IFX_INTERRUPT(Calibration_measurement_finished, 0, ISR_PRIORITY_RTC_CAL_F)
{
    float32  rtc_freq;
    Ifx_GTM  *psGTM;
    uint32   actualPeriod;
    sint8    newCalValue;

    psGTM = &MODULE_GTM;
    __enable();

    if (rtc_calibration_finished != TRUE)
    {
        // reset the newval flag
        psGTM->TIM[0].CH3.IRQ_NOTIFY.B.NEWVAL = 1;

        actualPeriod = psGTM->TIM[0].CH3.GPR1.B.GPR1;

        // calculate the actual frequency
        rtc_freq = (float32)actualPeriod/(10000000.0f/64)*32768.0f;

        // calculate new calibaration value (with sign)
        newCalValue = (sint8)__roundf((32768.0f-rtc_freq)/2.0f);
        rtc_calibration_finished = TRUE;      // we finished
    }
    else
    {
        // measurement was aborted we set the old cal_value
    	newCalValue = saved_cal_register;
    }
    // write the new cal_value
    rtc_write_register( 0x08, (uint8)newCalValue);

    // restore the control register with SQWE removed
    rtc_write_register( 0x07, saved_control_register & ~0x40);
    //wait until we are sure that the value is written

    // Disable the TIM0_3 interrupt
    IfxSrc_disable(&MODULE_SRC.GTM.GTM[0].TIM[0][3]);
    // Disable CMU_CLK1
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKDIS_CLK1);
    // we clear any request and reenable the interrupt from the RTC
    IfxSrc_clearRequest(RTC_ALARM_SRC);
    IfxSrc_enable(RTC_ALARM_SRC);
}

/*!
 * \brief   calibrate the RTC
 *
 * ...
 */
sint32
rtc_calibration (void)
{
    Ifx_GTM *psGTM;

    psGTM = &MODULE_GTM;
    // first we switch off our interrupt input to avoid the interrupt
    IfxSrc_disable(RTC_ALARM_SRC);
    // check for running device
    saved_control_register = rtc_read_register(0x03);
    // check that the osc is running
    if ((saved_control_register & 0x20) != 0x20) rtc_write_register(0x00, rtc_read_register(0x00) | 0x80); // enable the osc if disabled
    // save our registers
    saved_control_register = rtc_read_register(0x07);
    saved_cal_register = rtc_read_register(0x08);

    // setup rtc to have 1 Hz on interrupt pin (SQE=1, RS2=1, RS1=1, RS0=1)
    rtc_write_register( 0x07, 0x47);
    // write make the measurement without calibration value
    rtc_write_register( 0x08, 0);
    //wait until we are sure that the value is written

    // set CMU1 frequency to 10MHz as timebase for our pwm measurement
    IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_1, 10000000.0f);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK1);
    // we setup our tim0_3 to measure the PWM from RTC (if the pin is changed this must be changed by hand
    psGTM->INOUTSEL.TIM[0].INSEL.B.CH3SEL = 0x9;
    psGTM->TIM[0].CH3.CTRL.B.TIM_MODE = IfxGtm_Tim_Mode_pwmMeasurement;  // set TIM to TPWM
    psGTM->TIM[0].CH3.CTRL.B.GPR0_SEL = IfxGtm_Tim_GprSel_cnts;          // write CNTS to GPR0
    psGTM->TIM[0].CH3.CTRL.B.GPR1_SEL = IfxGtm_Tim_GprSel_cnts;          // write CNT to GPR0
    psGTM->TIM[0].CH3.CTRL.B.CLK_SEL = 1;                                // use CMU_CLK1
    psGTM->TIM[0].CH3.CTRL.B.OSM = 1;                                    // make one shot
    psGTM->TIM[0].CH3.IRQ_EN.B.NEWVAL_IRQ_EN = 1;                        // enable newval irq

    IfxSrc_init(&MODULE_SRC.GTM.GTM[0].TIM[0][3], ISR_PROVIDER_RTC_ALARM, ISR_PRIORITY_RTC_CAL_F);
    IfxSrc_enable(&MODULE_SRC.GTM.GTM[0].TIM[0][3]);

    // reset the newval flag
    psGTM->TIM[0].CH3.IRQ_NOTIFY.B.NEWVAL = 1;
    // start the measurement
    psGTM->TIM[0].CH3.CTRL.B.TIM_EN = 1;
    return (0);
}

