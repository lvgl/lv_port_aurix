/*
 * tfthw.h
 *
 *  Created on: 06.02.2015
 *      Author: dienst
 */
#ifndef TFTHW_H
#define TFTHW_H

#define TFT_XSIZE 320             //!< x dimension of tft display
#define TFT_YSIZE 240             //!< y dimension of tft display


extern uint16 Row_Buff[];
extern volatile uint32 tft_status;

//specific entries tfthw.c
void tft_drvinit (void);
void tft_init (void);
// flush the actual row buff and callback pFunc if finished
void tft_flush_row_buff(uint32 numberOfPixel, const void * buff);
// set the pixel datapointer to x,y location
void tft_display_setxy (uint32 x1,uint32 y1,uint32 x2, uint32 y2);

#endif /* TFTHW_H */
