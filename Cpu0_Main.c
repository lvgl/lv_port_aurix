


#include<stdio.h>
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxPort.h"
#include "Qspi0.h"
#include "Bsp.h"
#include "GPT12_Timer_Interrupt.h"
#include "lvgl.h"
#include "lv_init.h"
#include "lv_examples.h"
#include "RTC.h"
#include "ff.h"

IfxCpu_syncEvent g_cpuSyncEvent = 0;
#define BG_LED    &MODULE_P20,13
#define BUTTON &MODULE_P14,4    /* Port pin for the button  */
BYTE writeToEndOfFile(FATFS* fs, FIL* fp, const TCHAR* path, const void *buff, UINT btw);

extern uint32 touchTimeUp;
TIME curTime;

int core0_main(void)
{
    IfxCpu_enableInterrupts();
    
    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs and service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    
    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    // all our ports are tristate, we enable the pull-up on some pins


    IfxPort_setPinModeOutput(BG_LED, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinMode(BUTTON, IfxPort_Mode_inputPullUp);
    IfxPort_setPinHigh(BG_LED);

    waitTime(2000000);
    rtc_init();
    curTime.hours=22;
    curTime.minutes=0;
    curTime.seconds=0;
    curTime.day=07;
    curTime.month=6;
    curTime.year=20;
    rtc_gettime (&curTime);
    /* correct the date and month if needed */
    if (curTime.date == 0) {curTime.date = 0x01;rtc_settime (&curTime);}
    if (curTime.month == 0){curTime.month = 0x01;rtc_settime (&curTime);}
    /* clear all flags if it is not running */
    if (curTime.flags.B.running == 0) curTime.flags.U = 0x0;

    lv_configure();
    initGpt12Timer();
    lv_demo_widgets();

    /*
     // Code part to test rtc and sd card
     lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);
     char timeStr[32];
     FATFS FatFs;
     FIL Fil;
     rtc_gettime (&curTime);
     sprintf(timeStr,"%02d:%02d:%02d %02d-%02d-%02d \n",curTime.hours,curTime.minutes,curTime.seconds,curTime.day,curTime.month,curTime.year);
     writeToEndOfFile(&FatFs, &Fil,"Erhan.txt", timeStr,strlen(timeStr));
    */
    while(1)
    {


        waitTime(200000);

        if(IfxPort_getPinState(BUTTON) == 0)
        {

        }
        if(touchTimeUp)
        {
            touchTimeUp=0;
            //lv_task_handler();

        }
        if ((touch_driver.touchmode & MASK_TOUCH_DOWN) != 0 || (touch_driver.touchmode & MASK_TOUCH_MOVE) != 0)
        {
            /*
            touch_driver.touchmode &= ~MASK_TOUCH_DOWN;   //clear
            touch_driver.touchmode &= ~MASK_TOUCH_MOVE;   //clear
            tft_display_setxy(touch_driver.xdisp,touch_driver.ydisp,touch_driver.xdisp,touch_driver.ydisp);
            uint16 buff[4]={0xf100,0xf100,0xf100,0xf100};
            tft_flush_row_buff(4, &buff[0]);
            */
        }


    }
    return (1);
}

BYTE writeToEndOfFile(FATFS* fs, FIL* fp, const TCHAR* path, const void *buff, UINT btw)
{
    UINT bw;

    if(f_mount(0,fs) == FR_OK)
    {
        if (f_open(fp, path, FA_OPEN_ALWAYS | FA_WRITE) == FR_OK)
        {
            if(f_lseek(fp, f_size(fp)) == FR_OK)
            {
                if(f_write(fp, buff, btw, &bw) == FR_OK)
                {
                    if(f_close(fp) == FR_OK)
                    {
                        return 0;
                    }
                }
            }
        }
    }
    f_close(fp);
    return 1;
}
