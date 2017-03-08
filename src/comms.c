// This file specifies the interface between the inter-processor communication
// code (written by Kiran) and the control code (written by Kier)

#include "comms_interface.h"
#include "data_structures.h"
#include "includes/spi.h"
#include "debug.h"

// Define a function to initialize SPI communication with the control module
void comms_master_init()
{
	init_spi_master();		//Initialise communications module to be the SPI master.
}

// Define a function to initialize SPI communication with the communication module
void comms_slave_init()
{
	init_spi_slave();		//Initialise control module to be an SPI slave.
}

//The next 2 functions operate on the communications module 

// Define a function to send a data packet with the RC inputs from the 
// communication module to the control module. Doing this at 50 Hz.
void send_master_data_packet(rc_data_packet_t* rc_data_packet)
{
	//Send a command byte to signify that the control module should expect to receive the RC input bytes.
	spi_tx(CMD_RECEIVE_RC_INPUTS);
	
	//Send the 4 bytes of the RC input data packet.
	spi_tx(rc_data_packet->channel_0);
	spi_tx(rc_data_packet->channel_1);
	spi_tx(rc_data_packet->channel_2);
	spi_tx(rc_data_packet->channel_3);
}

// Define a function to receive a data packet with logging data from the 
// control module using SPI. Doing this at 25 Hz.
void receive_master_data_packet(logg_data_packet_t* logg_data_packet)
{
	//Loop variable.
	uint8_t i;
	
	//Send a command byte to signify that the control module is expected to send the logging data.
	spi_tx(CMD_SEND_LOGGING_DATA);
	
	//Receive the logging data bytes.
	for(i = 0 ; i < 4 ; i++)
	{
		logg_data_packet->roll.bytes[i] = spi_trx(0x00);		//Receive the roll bytes.
	}
	
	for(i = 0 ; i < 4 ; i++)
	{
		logg_data_packet->pitch.bytes[i] = spi_trx(0x00);		//Receive the pitch bytes.
	}
	
	for(i = 0 ; i < 4 ; i++)
	{
		logg_data_packet->yaw_vel.bytes[i] = spi_trx(0x00);		//Receive the yaw velocity bytes.
	}
}

// These next 2 functions operate on the control module (slave). Due to its inability to 
// send bytes on its own, the way these need to be used is with an SPI interrupt which
// triggers upon sending or receiving any byte. The received byte would be a command byte 
// and this must be interpreted to determine which of the below functions to call. SPI 
// interrupts should also be disabled/ignored while inside any of the below functions.

// Define a function to send a data packet with the logging data from the
// control module to the communication module. Doing this at 25 Hz.
void send_slave_data_packet(logg_data_packet_t* logg_data_packet)
{
	//Disable SPI interrupts, we don't need them while in this function.
	disable_SPI_interrupts();
	
	//Loop variable.
	uint8_t i;
	
	//Send the logging data bytes.
	for(i = 0 ; i < 4 ; i++)
	{
		spi_tx(logg_data_packet->roll.bytes[i]);		//Send the roll bytes.
	}
	
	for(i = 0 ; i < 4 ; i++)
	{
		spi_tx(logg_data_packet->pitch.bytes[i]);		//Send the pitch bytes.
	}
	
	for(i = 0 ; i < 4 ; i++)
	{
		spi_tx(logg_data_packet->yaw_vel.bytes[i]);		//Send the yaw velocity bytes.
	}
	
	//Re-enable SPI interrupts so one of these 2 functions can be called again when needed.
	enable_SPI_interrupts();
}

// Define a function to receive a data packet with RC inputs from the
// communication module. Doing this at 50 Hz.
void receive_slave_data_packet(rc_data_packet_t* rc_data_packet)
{
	//Disable SPI interrupts, we don't need them while in this function.
	disable_SPI_interrupts();
	
	//Receive the 4 bytes of the RC input data packet.
	rc_data_packet->channel_0 = spi_rx();
	rc_data_packet->channel_1 = spi_rx();
	rc_data_packet->channel_2 = spi_rx();
	rc_data_packet->channel_3 = spi_rx();
	
	//Re-enable SPI interrupts so one of these 2 functions can be called again when needed.
	enable_SPI_interrupts();
}