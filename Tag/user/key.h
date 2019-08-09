#ifndef __key_H__
#define __key_H__
#include <stdint.h>
#include "config.h"

#define	ON	0
#define	OFF	1

enum KEYS{
	key1,
	key2,
};
#if	(BOARD == IKMSIK)
	#define KEY1	P12
	#define KEY2	P13

#endif

void key_init();
#endif 