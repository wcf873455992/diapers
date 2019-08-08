#ifndef __DHT11_H__
#define __DHT11_H__
#include <stdint.h>

#define PIN_DATA_OUT   (P0DIR &= ~0x04);   //P0.2ÉèÖÃÎªÊäÈë
#define PIN_DATA_IN    (P0DIR |= 0x04);    //P0.2ÉèÖÃÎªÊä³ö


typedef struct DHT11_VALUE 
{
    uint16_t tempH;  //ÎÂ¶È¸ß×Ö½Ú£¨Ê®Î»£©
    uint8_t  tempL;  //ÎÂ¶ÈµÍÎ»½Ú£¨¸öÎ»£©
		uint8_t  tempD;  //ÎÂ¶ÈµÍÎ»½Ú£¨¸öÎ»£©
    uint8_t  humyH;   //Êª¶È¸ß×Ö½Ú£¨Ê®Î»£©
    uint8_t  humyL;  //Êª¶È¸ß×Ö½Ú£¨¸öÎ»£
		uint8_t  humyD;  //Êª¶È¸ß×Ö½Ú£¨¸öÎ»£©©

}DHT11VALUE;

extern   DHT11VALUE DHT11Value;




extern void COM(void);          // ÎÂÊªĞ´Èë
extern void DHT11(void);        //ÎÂÊª¶È´«¸ĞÆ÷Æô¶¯

extern uint8_t temp[2]; 
extern uint8_t temp1[5];
extern uint8_t humidity[2];
extern uint8_t humidity1[9];

#endif 