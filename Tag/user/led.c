#include <reg24le1.h>
#include "led.h"
#include "hal_delay.h"
#include <string.h>

void led_init() {
#if	(BOARD == IKMSIK)
    P0DIR &= ~0x03;//P00,P01Êä³ö
#else	(BOARD == DIAPER)
    P0DIR &= ~0x01;//P00,
    P1DIR &= ~0x60;//P16,P15Êä³ö
#endif
}

void led_flash(uint8_t led) {
    switch(led) {
    case led1:
        LED1 = ON;
        delay_us(200);
        LED1 = OFF;
        break;
    case led2:
        LED2	=	ON;
        delay_us(200);
        LED2 = OFF;
        break;
#if	(BOARD == DIAPER)
    case led3:
        LED3	=	ON;
        delay_us(200);
        LED3 = OFF;
        break;
#endif
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
		
#if	(BOARD == DIAPER)
    case led3:
        LED3	=	ON;
        break;
#endif
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
#if	(BOARD == DIAPER)
    case led3:
        LED3	=	OFF;
        break;
#endif
    default:
        break;
    }
}
/*********************************END FILE*************************************/