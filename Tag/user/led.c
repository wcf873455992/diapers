#include <reg24le1.h>
#include "led.h"
#include "hal_delay.h"
#include <string.h>

void led_init() {
#if	(BOARD == IKMSIK)
    P0DIR &= ~0x03;//P00,P01Êä³ö
#else	(BOARD == DIAPER)
    P0DIR &= ~0x03;//P00,P01Êä³ö
#endif
}

void led_flash(uint8_t led) {
    switch(led) {
    case led1:
        LED1 = ON;
        delay_us(100);
        LED1 = OFF;
        break;
    case led2:
        LED2	=	ON;
        delay_us(100);
        LED2 = OFF;
        break;
    default:
        break;
    }
}
void led_on(uint8_t led) {
    switch(led) {
    case led1:
        LED1 = ON;
        break;
    case led2:
        LED2	=	ON;
        break;
    default:
        break;
    }
}
void led_off(uint8_t led) {
    switch(led) {
    case led1:
        LED1 = OFF;
        break;
    case led2:
        LED2	=	OFF;
        break;
    default:
        break;
    }
}
/*********************************END FILE*************************************/