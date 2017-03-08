//spi.c
//Used to use the SPI interface.
//Author: Kiran Patel
//
//Contains code by Steve Gunn, obtained from the Year 1 C7 lab: https://secure.ecs.soton.ac.uk/notes/ellabs/1/c7/

#include "spi.h"

void init_spi_master(void)
{
	DDRB = _BV(PB2) | _BV(PB3) | _BV(PB5);	  //Set Slave Select (/SS), MOSI and SCLK to outputs.
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);  //Enable SPI, configure this device as master, set F_SCK = F_CPU/16 = 750 kHz.
}

void init_spi_slave(void)
{
	DDRB = _BV(PB4);	 		  	//Set MISO to output. 
	SPCR = _BV(SPE);  		 	  	//Enable SPI.
	init_SPI_interrupts();			//Enable SPI interrupts. Only needed by the slave.
}

void init_SPI_interrupts(void)
{
 	SPCR |= _BV(SPIE);				//Enable SPI interrupts. Triggers upon sending or receiving any byte.
 	sei();					  		//Enable global interrupts.
}

void enable_SPI_interrupts(void)
{
	SPCR |= _BV(SPIE);				//Enable SPI interrupts. Don't need to enable global interrupts again.
	//sei();
}

void disable_SPI_interrupts(void)
{
	SPCR &= ~_BV(SPIE);				//Disable SPI interrupts. Don't disable global interrupts since it might break things.
	//cli();
}

void spi_tx(uint8_t b)
{
	SPDR = b;				 		//Write data to the SPI data register. Initiates transmission.
	while(!(SPSR & _BV(SPIF)));		//Wait for SPIF to go high, indicates transfer complete.
}
	
uint8_t spi_rx(void) 
{
	while(!(SPSR & _BV(SPIF)));		//Wait for SPIF to go high, indicates transfer complete.
	return SPDR;				  	//Read received data from SPI data register, return it to be used.
}

uint8_t spi_read_data_register(void)
{
	return SPDR;					//Use only when interrupts are called. The interrupt clears SPIF so we can't just call spi_rx.
}

uint8_t spi_trx(uint8_t b)
{
	SPDR = b;				  		//Write data to the SPI data register. Initiates transmission.
	while(!(SPSR & _BV(SPIF)));		//Wait for SPIF to go high, indicates transfer complete.
	return SPDR;				  	//Read received data from SPI data register, return it to be used.
}