//spi.h
//Used to use the SPI interface.
//Author: Kiran Patel
//
//Based on code by Steve Gunn, obtained from the Year 1 C7 lab: https://secure.ecs.soton.ac.uk/notes/ellabs/1/c7/

#ifndef __SPI_H
#define __SPI_H

#include <avr/io.h>
#include <avr/interrupt.h>

//Command bytes for the slave to figure out what it needs to do.
#define CMD_RECEIVE_RC_INPUTS 0xC3		//If the slave receives this, it expects to receive the RC input bytes.
#define CMD_SEND_LOGGING_DATA 0x5A		//If the slave receives this, it starts sending the logging data.

void init_spi_master(void);
void init_spi_slave(void);
void init_SPI_interrupts(void);
void enable_SPI_interrupts(void);
void disable_SPI_interrupts(void);
void spi_tx(uint8_t b);
uint8_t spi_rx(void);
uint8_t spi_read_data_register(void);
uint8_t spi_trx(uint8_t b);

#endif