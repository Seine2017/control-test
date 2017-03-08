#include <avr/interrupt.h>
#include "clock.h"
#include "comms_interface.h"
#include "imu_interface.h"
#include "data_structures.h"
#include "includes/spi.h"
//#include "debug.h"

void convert_data(measured_state_t* measured_state, logg_data_packet_t* logg_data_packet);
volatile logg_data_packet_t logg_data_packet;
volatile rc_data_packet_t rc_data_packet;
volatile measured_state_t measured_state;

ISR(SPI_STC_vect)
{
	//printf("Inside interrupt.\n");
	uint8_t tempByte = spi_read_data_register();
	switch(tempByte)
	{
		case CMD_RECEIVE_RC_INPUTS: 
			receive_slave_data_packet(&rc_data_packet);		//May need to modify argument name.
			//printf("%d %d %d %d\n", rc_data_packet.channel_0, rc_data_packet.channel_1, rc_data_packet.channel_2, rc_data_packet.channel_3);
			break;
		case CMD_SEND_LOGGING_DATA:
			//printf("Attempt to send.\n");
			send_slave_data_packet(&logg_data_packet);		//May need to modify argument name.
			//printf("pitch: %d, roll: %d, yaw_vel: %d\n", (int)logg_data_packet.pitch.value, (int)logg_data_packet.roll.value, (int)logg_data_packet.yaw_vel.value);
			break;
		default:
			printf("%d\n", tempByte);
			break;
	}
}

int main() {
  
  
  clock_init();
  imu_init();
  comms_slave_init();
  init_debug_uart0();

  // Enable interrupts.
  sei();
  
  clock_time_t clock_now;

  while (1) {
	 clock_now = clock_get_time();
    // Communicate with IMU.
    //if(!(clock_now%4))
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
