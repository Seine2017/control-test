//receiver.h
//Used for the RFM12B configured as a receiver.
//Based on source code from the official programming guide (https://www.sparkfun.com/datasheets/Wireless/General/RF12B_code.pdf)
//and an unofficial programming guide (http://dlehard.narod.ru/quick_start.pdf)

#ifndef __RECEIVER_H
#define __RECEIVER_H

#include <avr/io.h>

void portInitRx(void);
uint16_t writeCommandRx(uint16_t cmd);
void rxInit(void);
uint8_t receiveByte(void);
void FIFO_reset(void);

#endif