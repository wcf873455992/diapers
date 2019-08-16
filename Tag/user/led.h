#ifndef __led_H__
#define __led_H__
#include <stdint.h>
#include "config.h"

#define	ON	0
#define	OFF	1

#if	(BOARD == IKMSIK)
#define LED1	P00
#define LED2	P01
enum LEDS {
    led1,
    led2,
};
#else
	#define LED1	P15
	#define LED2	P16
	#define LED3	P00
enum LEDS {
    led1,
    led2,
    led3,
};
#endif

void led_init();
void led_flash(uint8_t led);
void led_on(uint8_t led);
void led_off(uint8_t led);
#endif