#ifndef _SBUS_H_
#define _SBUS_H_

#include "stm32f1xx_hal.h"

#define TEST25

//#define UART1_
#define UART2_DMA


//attention:though the sbus data byte has only 22u,head,flag and end byte should be include(25u at a time)




#define SBUS_FRAME_LENGH	25u
#define _sbusHeader 		0x0F
#define _sbusFooter 		0x00
#define _sbusLostFrame 		0x20
#define _sbusFailSafe 		0x10



void RXEncoder(void);
void TXEncoder(void);






#endif
