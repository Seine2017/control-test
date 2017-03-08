#include <avr/interrupt.h>
#include "clock.h"
#include "comms_interface.h"
#include "imu_interface.h"
#include "data_structures.h"
#include "includes/spi.h"
#include "debug.h"

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
			printf("%d %d %d %d\n", rc_data_packet.channel_0, rc_data_packet.channel_1, rc_data_packet.channel_2, rc_data_packet.channel_3);
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
  init_debug_uart0();

  // Enable interrupts.
  sei();
  
  printf("Hello\n");

  while (1) {
    // Communicate with IMU.
    imu_read(&measured_state);
	convert_data(&measured_state, &logg_data_packet);
	//printf("pitch: %d, roll: %d, yaw_vel: %d\n", (int)logg_data_packet.pitch.value, (int)logg_data_packet.roll.value, (int)logg_data_packet.yaw_vel.value);

  }
}
void convert_data(measured_state_t* measured_state, logg_data_packet_t* logg_data_packet){
	logg_data_packet->roll.value = measured_state->roll;
	logg_data_packet->pitch.value = measured_state->pitch;
	logg_data_packet->yaw_vel.value = measured_state->yaw_vel;
	
	
}
