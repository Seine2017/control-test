//receiver.c
//Used for the RFM12B configured as a receiver.
//Based on source code from the official programming guide (https://www.sparkfun.com/datasheets/Wireless/General/RF12B_code.pdf)
//and an unofficial programming guide (http://dlehard.narod.ru/quick_start.pdf)

#include "receiver.h"

//Initialise ports.
void portInitRx(void)
{
	DDRD &= ~_BV(2);			//nIRQ input to indicate previous transmission ended.
	DDRD |= _BV(4) | _BV(5) |  _BV(7);	//Put transmitter's SPI interface on port D.
}

uint16_t writeCommandRx(uint16_t cmd)
{
	//NOTE: The commands are 16 bit, SPI is 8 bit. Hence, need to do SPI manually.
 	  uint8_t i;				//for loop variable.
 	  uint16_t temp = 0;			//Received data if reading the status register.

 	  PORTD &= ~_BV(7); 			//Set SCLK low.
 	  PORTD &= ~_BV(4);			//Set slave select low to select the RFM12B module.
 	  for(i=0 ; i<16 ; i++)
	  {
 		if(cmd & 0x8000)		//If the MSB of the command is high...
		{
 			PORTD |= _BV(5);	//set MOSI high.
 		}
		else				//Else...
		{
 			PORTD &= ~_BV(5);	//set MOSI low.
 		} 
 

 		PORTD |= _BV(7);		//Set SCLK high to clock the RFM12B module.
		temp <<= 1;			//Shift temp left by 1.

		if(PIND & _BV(6))		//If MISO is high...
		{
 			temp |= 0x0001;		//set the LSB of temp to 1. Implied else means it is set to 0.
 		}
 		PORTD &= ~_BV(7); 		//Set SCLK low;
 		cmd <<= 1;			//Shift the command left by 1. The new MSB is the next bit which is used in the next loop.
 	  }
 	  PORTD |= _BV(4);			//Set slave select high to deselect the RFM12B module.
 	  return(temp);				//Return the contents of the status register if it was read.
	
}

//Initialise the RFM12B module.
void rxInit(void)	//The only thing different between transmitter and receiver is the second command. Receiver enabled instead of transmitter.
{
	//Initialise ports for the SPI interface to the RFM12B module.
	portInitRx();
	
	//Write commands to set up the RFM12B module.
 	writeCommandRx(0x80D8);	//Enable transmitter register and receiver FIFO buffer. Use 433 MHz band. Use 12.5 pF load capacitor.
 	writeCommandRx(0x8299);	//Enable receiver, low battery detector, synthesiser and crystal oscillator.
							//Disable transmitter, base band block, wake up timer and clock output of clock pin.
 	writeCommandRx(0xA640);	//Use 434 MHz frequency.
 	writeCommandRx(0xC647);	//Use a data rate of 4.8 kbps.
 	writeCommandRx(0x94A0);	//Set pin 16 to VDI output. Use 134 kHz baseband bandwidth. Set VDI response time to fast.
							//Use LNA gain of 0dBm. Use DRSSI threshold of -103dBm.
 	writeCommandRx(0xC2AC);	//Enable clock recovery auto-lock. Disable clock recovery fast mode. Use digital data filter.
							//Use DQD threshold of 4.
 	writeCommandRx(0xCA81);	//Use FIFO interrupt level of 8. Use 2DD4 as synchronisation pattern. 
							//Disable FIFO fill. Enable high sensitivity reset mode.
	writeCommandRx(0xCED4);	//Use 2DD4 as synchronisation pattern.
 	writeCommandRx(0xC483);	//Keep offset when VDI high. Do not restrict frequency range.
							//Disable AFC high accuracy mode. Enable AFC output register and function.
 	writeCommandRx(0x9850);	//Use 90 kHz frequency deviation. Use maximum output power (0 dBm).
 	writeCommandRx(0xCC17);	//Use 5 or 10 MHz (recommended) for microcontroller clock frequency.
							//Disable PLL loop dithering. Disable phase detector delay. Use higher PLL bandwidth.
 	writeCommandRx(0xE000);	//Unused since wake-up timer is disabled.
 	writeCommandRx(0xC800);	//Unused since low duty cycle mode is disabled.
 	writeCommandRx(0xC040);	//Use 1.66 MHz clock frequency on the CLK pin. Use 2.2V for the low battery detector threshold.
	
	//Reset the FIFO buffer. Need to do this before receiving every packet.
	FIFO_reset();
}

//Wait for and retrieve a received byte from the FIFO buffer.
//Can replace the wait with an interrupt if needed.
uint8_t receiveByte(void)
{
 	uint16_t FIFO_data, status;
	while(1)
	{
		status = writeCommandRx(0x0000);			//Check the status register.
 		if(status & 0x8000)				//If the MSB of the status register is set, we received something.
		{
			FIFO_data = writeCommandRx(0xB000);	//Get received data.
			return (FIFO_data & 0x00FF);		//Extract the byte from the received data.
		}
	}		
}	

//Toggles the FIFO buffer to clear it out.
void FIFO_reset(void)
{
	writeCommandRx(0xCA81);
	writeCommandRx(0xCA83);
}