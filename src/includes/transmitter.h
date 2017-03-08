//transmitter.h
//Used for the RFM12B configured as a transmitter.
//Based on source code from the official programming guide (https://www.sparkfun.com/datasheets/Wireless/General/RF12B_code.pdf)
//and an unofficial programming guide (http://dlehard.narod.ru/quick_start.pdf)

#ifndef __TRANSMITTER_H
#define __TRANSMITTER_H

#include <avr/io.h>

void portInitTx(void);
uint16_t writeCommandTx(uint16_t cmd);
void txInit(void);
void sendByte(uint8_t byte);

#endif