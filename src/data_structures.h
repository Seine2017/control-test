// This file defines data structures that will flow through the communication module
// Those structures are used by its correspoding modules to propagate the data

#ifndef __DATA_STRUCTURES_H
#define __DATA_STRUCTURES_H

#include <avr/io.h>

// Define a structure for the data packet to be send to the control module
typedef struct{
	uint8_t channel_0;
	uint8_t channel_1;
	uint8_t channel_2;
	uint8_t channel_3;
}rc_data_packet_t;

// Define a structure for the data packet to be received from the control module
// has to be synchorinzed with what is actually send by the control module
typedef struct{
	union
	{
		uint8_t bytes[4];
		float value;
	}roll;
	
	union
	{
		uint8_t bytes[4];
		float value;
	}pitch;
	
	union
	{
		uint8_t bytes[4];
		float value;
	}yaw_vel;
}logg_data_packet_t;

// Define a structure for the data packet to be send to the host microcontroller
typedef struct{
	union
	{
		uint8_t bytes[4];
		float value;
	}roll;
	
	union
	{
		uint8_t bytes[4];
		float value;
	}pitch;
	
	union
	{
		uint8_t bytes[4];
		float value;
	}yaw_vel;
}rf_data_packet_t;

//Define a structure for data encryption
typedef struct{
	
}encrypted_data_packet_t;

//Define a structure for data decryption
typedef struct{
	
}decrypted_data_packet_t;

//Define a structure to be send to the host microcontroller
typedef struct{
	float roll;
	float pitch;
	float yaw_vel;
}host_data_packet_t;

#endif