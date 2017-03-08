#include <avr/interrupt.h>
#include "clock.h"
#include "comms_interface.h"
#include "imu_interface.h"
#include "data_structures.h"
#include "includes/spi.h"

void convert_data(measured_state_t* measured_state, logg_data_packet_t* logg_data_packet);
volatile logg_data_packet_t logg_data_packet;
volatile rc_data_packet_t rc_data_packet;

ISR(SPI_STC_vect)
{
	uint8_t tempByte = spi_read_data_register();
	switch(tempByte)
	{
		case CMD_RECEIVE_RC_INPUTS: 
			receive_slave_data_packet(&rc_data_packet);		//May need to modify argument name.
			break;
		case CMD_SEND_LOGGING_DATA:
			send_slave_data_packet(&logg_data_packet);		//May need to modify argument name.
			break;
		default:
			//This should NOT happen. Maybe print an error?
			break;
	}
}

int main() {
  measured_state_t measured_state;

  clock_init();
  imu_init();
  comms_slave_init();

  // Enable interrupts.
  sei();

  while (1) {
    // Communicate with IMU.
    imu_read(&measured_state);
	convert_data(&measured_state, &logg_data_packet);

  }
}
void convert_data(measured_state_t* measured_state, logg_data_packet_t* logg_data_packet){
	logg_data_packet->roll.value = measured_state->roll;
	logg_data_packet->pitch.value = measured_state->pitch;
	logg_data_packet->yaw_vel.value = measured_state->yaw_vel;
	
	
}
