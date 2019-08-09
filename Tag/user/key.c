#include <reg24le1.h>
#include "key.h"
#include "hal_delay.h"
#include <string.h>

void key_init() {
#if	(BOARD == IKMSIK)
    P1DIR |= 0x06;//0-P12,P13Êä³ö
#else	(BOARD == DIAPER)
    P1DIR &= ~0x03;//P00,P01Êä³ö
#endif
}

/*********************************END FILE*************************************/