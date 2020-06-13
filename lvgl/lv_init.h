#include "Ifx_Types.h"
#include "tfthw.h"
#include "lvgl.h"
#include "touch.h"




void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
static boolean touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);


void lv_configure(void)
{
    tft_init();
    lv_init();
    /*A static or global variable to store the buffers*/
    static lv_disp_buf_t disp_buf;
    /*Static or global buffer(s). The second buffer is optional*/
    static lv_color_t buf_1[320 * 40];
    //static lv_color_t buf_2[320 * 10];
    /*Initialize `disp_buf` with the buffer(s) */
    lv_disp_buf_init(&disp_buf, buf_1, NULL, 320*40);
    lv_disp_drv_t disp_drv; /*A variable to hold the drivers. Can belocal variable*/
    lv_disp_drv_init(&disp_drv); /*Basic initialization*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = 320;
    disp_drv.ver_res = 240;

    disp_drv.buffer = &disp_buf; /*Set an initialized buffer*/
    disp_drv.flush_cb = my_flush_cb; /*Set a flush callback to draw to thedisplay*/
    lv_disp_t * disp;
    disp = lv_disp_drv_register(&disp_drv);


    // Touc init
    touch_init();
    lv_indev_t * indev_touchpad;
    lv_indev_drv_t indev_drv;
    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    indev_touchpad = lv_indev_drv_register(&indev_drv);
}



void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    /*The most simple case (but also the slowest) to put all pixels to the screen oneby-one*/
    uint32 numberOfPixel;
    if((area->x2 >= area->x1) && area->y2 >= area->y1)
    {
      numberOfPixel = ((area->x2+1) - area->x1) * ((area->y2+1) - area->y1);
      tft_display_setxy(area->x1, area->y1, area->x2, area->y2);
      tft_flush_row_buff(numberOfPixel,color_p);
    }

/* IMPORTANT!!!
* Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}


/* Will be called by the library to read the touchpad */
static boolean touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    /*Save the pressed coordinates and the state*/

    if ((touch_driver.touchmode & MASK_TOUCH_DOWN) != 0 || (touch_driver.touchmode & MASK_TOUCH_MOVE) != 0)
    {

        touch_driver.touchmode &= ~MASK_TOUCH_DOWN;   //clear
        touch_driver.touchmode &= ~MASK_TOUCH_MOVE;   //clear
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touch_driver.xdisp;
        data->point.y = touch_driver.ydisp;
    }
    else
    {
       data->state = LV_INDEV_STATE_REL;

    }


    /*Return `false` because we are not buffering and no more data to read*/
    return false;
}
