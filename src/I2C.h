/*
 * I2C.h
 *
 *  Created on: 1 Mar 2017
 *      Author: Pawel Kostkowski
 */

#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>

static void init_I2C(void){ // will need to put pull up resistor on the SCL and SDA, unless it can be done on the board

    TWSR = 0x00; //setting to zero implies prescaler set to 1
    TWBR = 0x0C; //set SCL to 400kHz we divide 16MHz/(16+2*TWBR*prescaler)=400kHz
    //IN order to set prescaler we manipulate bits TWPS1 and TWPS0 on TWSR Register
    TWCR = (1<<TWEN); //enable TWI
    //also what about configuring pins
}
static void transmit_START(void){
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); //Send START condition
	while (!(TWCR &	(1<<TWINT))); //Wait for TWINT Flag set. This indicates
								// that the START condition has been transmitted
	//if ((TWSR & 0xF8) !=START) ERROR();
}
static void transmit_STOP(void){
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); //Transmit STOP condition
}
static void write_8bit_data(uint8_t data_8bit){
	TWDR = data_8bit;
	TWCR = (1<<TWINT) |	(1<<TWEN);
	while (!(TWCR &	(1<<TWINT))); //Wait for TWINT Flag set. This indicates that
								// address has been transmited and ACK/NACK has been received
}
static uint8_t read_8bit_data_with_ack(void){
	 TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	 while ((TWCR & (1<<TWINT)) == 0);
	 return TWDR;
}
static uint8_t read_8bit_data_with_nack(void){
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}
static uint8_t read_I2C_status(void){
	uint8_t status;
	status = TWSR & 0xF8; //mask status
	return status;
}

#endif
