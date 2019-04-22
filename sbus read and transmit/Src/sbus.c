#include "sbus.h"

#include "stm32f1xx_hal.h"
#include "usart.h"


#ifdef TEST22
 uint8_t data[22];  
#endif

#ifdef TEST23
 uint8_t data[23];  
#endif

#ifdef TEST25
 uint8_t data[25];  
#endif



//uint8_t data[22];  
struct RC
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch5;
	int16_t ch6;
	int16_t ch7;
	int16_t ch8;
	int16_t ch9;
	int16_t ch10;
	int16_t ch11;
	int16_t ch12;
	int16_t ch13;
	int16_t ch14;
	int16_t ch15;
}rc;

uint16_t rcdata[16];
uint8_t txdata[25];

void RXEncoder(void)
{
	
#ifdef TEST22
			rc.ch0  = (uint16_t) ((data[0]    |data[1] <<8)                     & 0x07FF);
			rc.ch1  = (uint16_t) ((data[1]>>3 |data[2] <<5)                     & 0x07FF);
			rc.ch2  = (uint16_t) ((data[2]>>6 |data[3] <<2 |data[4]<<10)  	 & 0x07FF);
		    rc.ch3  = (uint16_t) ((data[4]>>1 |data[5] <<7)                     & 0x07FF);
			rc.ch4  = (uint16_t) ((data[5]>>4 |data[6] <<4)                     & 0x07FF);
			rc.ch5  = (uint16_t) ((data[6]>>7 |data[7] <<1 |data[8]<<9)   	 & 0x07FF);
			rc.ch6  = (uint16_t) ((data[8]>>2 |data[9] <<6)                     & 0x07FF);
			rc.ch7  = (uint16_t) ((data[9]>>5 |data[10]<<3)                     & 0x07FF);
			rc.ch8  = (uint16_t) ((data[11]   |data[12]<<8)                     & 0x07FF);
			rc.ch9  = (uint16_t) ((data[12]>>3|data[13]<<5)                     & 0x07FF);
			rc.ch10  = (uint16_t) ((data[13]>>6|data[14]<<2 |data[15]<<10) 	 & 0x07FF);
			rc.ch11  = (uint16_t) ((data[15]>>1|data[16]<<7)                     & 0x07FF);
			rc.ch12  = (uint16_t) ((data[16]>>4|data[17]<<4)                     & 0x07FF);
			rc.ch13  = (uint16_t) ((data[17]>>7|data[18]<<1 |data[19]<<9)  	 & 0x07FF);
			rc.ch14  = (uint16_t) ((data[19]>>2|data[20]<<6)                     & 0x07FF);
			rc.ch15  = (uint16_t) ((data[20]>>5|data[21]<<3)                     & 0x07FF);

#endif

#ifdef TEST23
			rc.ch0  = (uint16_t) ((data[1]    |data[2] <<8)                     & 0x07FF);
			rc.ch1  = (uint16_t) ((data[2]>>3 |data[3] <<5)                     & 0x07FF);
			rc.ch2  = (uint16_t) ((data[3]>>6 |data[4] <<2 |data[4]<<10)  	 & 0x07FF);
		    rc.ch3  = (uint16_t) ((data[5]>>1 |data[6] <<7)                     & 0x07FF);
			rc.ch4  = (uint16_t) ((data[6]>>4 |data[7] <<4)                     & 0x07FF);
			rc.ch5  = (uint16_t) ((data[7]>>7 |data[8] <<1 |data[9]<<9)   	 & 0x07FF);
			rc.ch6  = (uint16_t) ((data[9]>>2 |data[10] <<6)                     & 0x07FF);
			rc.ch7  = (uint16_t) ((data[10]>>5 |data[11]<<3)                     & 0x07FF);
			rc.ch8  = (uint16_t) ((data[12]   |data[13]<<8)                     & 0x07FF);
			rc.ch9  = (uint16_t) ((data[13]>>3|data[14]<<5)                     & 0x07FF);
			rc.ch10  = (uint16_t) ((data[14]>>6|data[15]<<2 |data[16]<<10) 	 & 0x07FF);
			rc.ch11  = (uint16_t) ((data[16]>>1|data[17]<<7)                     & 0x07FF);
			rc.ch12  = (uint16_t) ((data[17]>>4|data[18]<<4)                     & 0x07FF);
			rc.ch13  = (uint16_t) ((data[18]>>7|data[19]<<1 |data[20]<<9)  	 & 0x07FF);
			rc.ch14  = (uint16_t) ((data[20]>>2|data[21]<<6)                     & 0x07FF);
			rc.ch15  = (uint16_t) ((data[21]>>5|data[22]<<3)                     & 0x07FF);

#endif

#ifdef TEST25
//the first byte is start byte0x0f,so encoder process should begin with the second byte:data[1] rather data[0]

//            rc.ch0  = (uint16_t) ((data[1]    |data[2] <<8)                     & 0x07FF);
//			rc.ch1  = (uint16_t) ((data[2]>>3 |data[3] <<5)                     & 0x07FF);
//			rc.ch2  = (uint16_t) ((data[3]>>6 |data[4] <<2 |data[4]<<10)  	 & 0x07FF);
//		    rc.ch3  = (uint16_t) ((data[5]>>1 |data[6] <<7)                     & 0x07FF);
//			rc.ch4  = (uint16_t) ((data[6]>>4 |data[7] <<4)                     & 0x07FF);
//			rc.ch5  = (uint16_t) ((data[7]>>7 |data[8] <<1 |data[9]<<9)   	 & 0x07FF);
//			rc.ch6  = (uint16_t) ((data[9]>>2 |data[10] <<6)                     & 0x07FF);
//			rc.ch7  = (uint16_t) ((data[10]>>5 |data[11]<<3)                     & 0x07FF);
//			rc.ch8  = (uint16_t) ((data[12]   |data[13]<<8)                     & 0x07FF);
//			rc.ch9  = (uint16_t) ((data[13]>>3|data[14]<<5)                     & 0x07FF);
//			rc.ch10  = (uint16_t) ((data[14]>>6|data[15]<<2 |data[16]<<10) 	 & 0x07FF);
//			rc.ch11  = (uint16_t) ((data[16]>>1|data[17]<<7)                     & 0x07FF);
//			rc.ch12  = (uint16_t) ((data[17]>>4|data[18]<<4)                     & 0x07FF);
//			rc.ch13  = (uint16_t) ((data[18]>>7|data[19]<<1 |data[20]<<9)  	 & 0x07FF);
//			rc.ch14  = (uint16_t) ((data[20]>>2|data[21]<<6)                     & 0x07FF);
//			rc.ch15  = (uint16_t) ((data[21]>>5|data[22]<<3)                     & 0x07FF);

///*if use array rather than struct use the code below*/
            rcdata[0]  = (uint16_t) ((data[1]    |data[2] <<8)                     & 0x07FF);
			rcdata[1]  = (uint16_t) ((data[2]>>3 |data[3] <<5)                     & 0x07FF);
			rcdata[2]  = (uint16_t) ((data[3]>>6 |data[4] <<2 |data[5]<<10)  	 & 0x07FF);
		    rcdata[3]  = (uint16_t) ((data[5]>>1 |data[6] <<7)                     & 0x07FF);
			rcdata[4]  = (uint16_t) ((data[6]>>4 |data[7] <<4)                     & 0x07FF);
			rcdata[5]  = (uint16_t) ((data[7]>>7 |data[8] <<1 |data[9]<<9)   	 & 0x07FF);
			rcdata[6]  = (uint16_t) ((data[9]>>2 |data[10] <<6)                     & 0x07FF);
			rcdata[7]  = (uint16_t) ((data[10]>>5 |data[11]<<3)                     & 0x07FF);
			rcdata[8]  = (uint16_t) ((data[12]   |data[13]<<8)                     & 0x07FF);
			rcdata[9]  = (uint16_t) ((data[13]>>3|data[14]<<5)                     & 0x07FF);
			rcdata[10] = (uint16_t) ((data[14]>>6|data[15]<<2 |data[16]<<10) 	 & 0x07FF);
			rcdata[11] = (uint16_t) ((data[16]>>1|data[17]<<7)                     & 0x07FF);
			rcdata[12] = (uint16_t) ((data[17]>>4|data[18]<<4)                     & 0x07FF);
			rcdata[13] = (uint16_t) ((data[18]>>7|data[19]<<1 |data[20]<<9)  	 & 0x07FF);
			rcdata[14] = (uint16_t) ((data[20]>>2|data[21]<<6)                     & 0x07FF);
			rcdata[15] = (uint16_t) ((data[21]>>5|data[22]<<3)                     & 0x07FF);

#endif

}

void TXEncoder(void)
{
	txdata[0] = _sbusHeader;


    txdata[1] = (uint8_t) ((rcdata[0] & 0x07FF));
  	txdata[2] = (uint8_t) ((rcdata[0] & 0x07FF)>>8 | (rcdata[1] & 0x07FF)<<3);
  	txdata[3] = (uint8_t) ((rcdata[1] & 0x07FF)>>5 | (rcdata[2] & 0x07FF)<<6);
  	txdata[4] = (uint8_t) ((rcdata[2] & 0x07FF)>>2);
  	txdata[5] = (uint8_t) ((rcdata[2] & 0x07FF)>>10 | (rcdata[3] & 0x07FF)<<1);
  	txdata[6] = (uint8_t) ((rcdata[3] & 0x07FF)>>7 | (rcdata[4] & 0x07FF)<<4);
  	txdata[7] = (uint8_t) ((rcdata[4] & 0x07FF)>>4 | (rcdata[5] & 0x07FF)<<7);
  	txdata[8] = (uint8_t) ((rcdata[5] & 0x07FF)>>1);
  	txdata[9] = (uint8_t) ((rcdata[5] & 0x07FF)>>9 | (rcdata[6] & 0x07FF)<<2);
  	txdata[10] = (uint8_t) ((rcdata[6] & 0x07FF)>>6 | (rcdata[7] & 0x07FF)<<5);
  	txdata[11] = (uint8_t) ((rcdata[7] & 0x07FF)>>3);
  	txdata[12] = (uint8_t) ((rcdata[8] & 0x07FF));
  	txdata[13] = (uint8_t) ((rcdata[8] & 0x07FF)>>8 | (rcdata[9] & 0x07FF)<<3);
  	txdata[14] = (uint8_t) ((rcdata[9] & 0x07FF)>>5 | (rcdata[10] & 0x07FF)<<6);
  	txdata[15] = (uint8_t) ((rcdata[10] & 0x07FF)>>2);
  	txdata[16] = (uint8_t) ((rcdata[10] & 0x07FF)>>10 | (rcdata[11] & 0x07FF)<<1);
  	txdata[17] = (uint8_t) ((rcdata[11] & 0x07FF)>>7 | (rcdata[12] & 0x07FF)<<4);
  	txdata[18] = (uint8_t) ((rcdata[12] & 0x07FF)>>4 | (rcdata[13] & 0x07FF)<<7);
  	txdata[19] = (uint8_t) ((rcdata[13] & 0x07FF)>>1);
  	txdata[20] = (uint8_t) ((rcdata[13] & 0x07FF)>>9 | (rcdata[14] & 0x07FF)<<2);
  	txdata[21] = (uint8_t) ((rcdata[14] & 0x07FF)>>6 | (rcdata[15] & 0x07FF)<<5);
  	txdata[22] = (uint8_t) ((rcdata[15] & 0x07FF)>>3);
  	// flags
	txdata[23] = 0x00;

	// footer
	txdata[24] = _sbusFooter;
	
	
	
//HAL_UART_Transmit_IT(&huart2,txdata,SBUS_FRAME_LENGH);
// HAL_UART_Transmit_DMA(&huart2,txdata,SBUS_FRAME_LENGH);
// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);


//if ( HAL_UART_Transmit_DMA(&huart2,txdata,SBUS_FRAME_LENGH)==HAL_BUSY)
//{
// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

//}

//if ( HAL_UART_Transmit_IT(&huart2,txdata,SBUS_FRAME_LENGH)==HAL_BUSY)
//{
// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

//}


/***********
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
**********/



}




