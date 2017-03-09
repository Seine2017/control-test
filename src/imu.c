// This file difines all the functions that are necessary to
// interface with the IMU. It assumes debug.h library included

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "I2C.h"
#include "imu_reg.h"
#include "MadgwickAHRS.h"
#include "imu_interface.h"
#include "error.h"
#include "settings.h"
#include <math.h>

// Define variables to be used by the imu library
uint8_t state;
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};

// Varibles to store raw values of the accelerometer
uint16_t accel_raw[3]; //possibly can be deined in the imu_read function
uint16_t gyro_raw[3];

// Variables
//float pitch = 0, roll = 0, pitch_accel = 0, roll_accel = 0;

static uint8_t write_IMU_byte(uint8_t IMU_address, uint8_t address, uint8_t data_8bit){
	uint8_t IMU_address_write = (IMU_address << 1);

	transmit_START();
	//if(read_I2C_status() != ???)
	//	return ERROR;
	write_8bit_data(IMU_address_write);
	//if(read_I2C_status() != ???)
	//	return ERROR;
	write_8bit_data(address);
	//if(read_I2C_status() != ???)
	//	return ERROR;
	write_8bit_data(data_8bit);
	//if(read_I2C_status() != ???)
	//	return ERROR;
	transmit_STOP();

	return SUCCESS;
}

static uint8_t read_IMU_byte(uint8_t IMU_address, uint8_t address, uint8_t *data_8bit){
	uint8_t IMU_address_write = (IMU_address << 1);
	uint8_t IMU_address_read = (IMU_address << 1) | 0x01;

	transmit_START();
	//if(read_I2C_status() != ???)
	//	return ERROR;
	write_8bit_data(IMU_address_write);
	//if(read_I2C_status() != ???)
	//	return ERROR;
	write_8bit_data(address);
	//if(read_I2C_status() != ???)
	//	return ERROR;
	transmit_START();
	//if(read_I2C_status() != ???)
	//	return ERROR;
	write_8bit_data(IMU_address_read);
	//if(read_I2C_status() != ???)
	//	return ERROR;
	*data_8bit = read_8bit_data_with_nack();
	//if(read_I2C_status() != ???)
	//	return ERROR;
	transmit_STOP();

	return SUCCESS;
}

static uint8_t read_IMU_bytes(uint8_t IMU_address, uint8_t address, uint8_t count, uint8_t * data){
	uint8_t IMU_address_write = (IMU_address << 1);
	uint8_t IMU_address_read = (IMU_address << 1) | 0x01;

	int i;
	
	transmit_START();
	//if(read_I2C_status() != ???)
	//	return ERROR;
	write_8bit_data(IMU_address_write);
	//if(read_I2C_status() != ???)
	//	return ERROR;
	write_8bit_data(address);
	//if(read_I2C_status() != ???)
	//	return ERROR;
	transmit_START();
	//if(read_I2C_status() != ???)
	//	return ERROR;
	write_8bit_data(IMU_address_read);
	//if(read_I2C_status() != ???)
	//	return ERROR;
    for(i = 0; i < count-1; i++){
     *data++ = read_8bit_data_with_ack();
     //if(read_I2C_status() != ???)
     //	return ERROR;
    }
    *data = read_8bit_data_with_nack();
	//if(read_I2C_status() != ???)
	//	return ERROR;
	transmit_STOP();

	return SUCCESS;
}

static void read_IMU_id(void){
    uint8_t id;
    state = read_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_WHO_AM_I, &id);
    //printf("%d\n",id);

    if (state == ERROR) {
      error(ERROR_I2C, "I2C error");
    }
    if (id != 0x73) {
    	char message[32];
    	snprintf(message, 32, "unexpected ID; received %d", id);
    	error(ERROR_BAD_IMU_ID, message);
    }
}

static void set_IMU_scales(void){
	//set gyroscope full-scale range
	uint8_t gyro_temp_8bit;
	read_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_CONFIG, &gyro_temp_8bit);//add error checking
	//gyro_temp_8bit &= ~0x02; //if gyro fchoice is 01
	gyro_temp_8bit &= ~0x18;
	gyro_temp_8bit |= (MPU9250_GYRO_FS_500<<3) & 0x18; //set to 500dps
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_CONFIG, gyro_temp_8bit);


	//set accelerometer full-scale range
	uint8_t accel_temp_8bit;
	read_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_CONFIG, &accel_temp_8bit);//add error checking
	accel_temp_8bit &= ~0x18;
	accel_temp_8bit |= (MPU9250_ACCEL_FS_8<<3) & 0x18;//set to 8g
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_CONFIG,accel_temp_8bit);
}

static void read_raw_gyro(uint16_t *gyro){
	uint8_t raw_gyro[6];
	read_IMU_bytes(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_XOUT_H,6, &raw_gyro[0]);


	gyro[0] = (((uint16_t)raw_gyro[0]<<8)|raw_gyro[1]);
	gyro[1] = (((uint16_t)raw_gyro[2]<<8)|raw_gyro[3]);
	gyro[2] = (((uint16_t)raw_gyro[4]<<8)|raw_gyro[5]);
}

static void read_raw_accel(uint16_t *accel){
	uint8_t raw_accel[6];
	read_IMU_bytes(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_XOUT_H,6 , &raw_accel[0]);

	accel[0] = (((uint16_t)raw_accel[0]<<8)|raw_accel[1]);
	accel[1]= (((uint16_t)raw_accel[2]<<8)|raw_accel[3]);
	accel[2] = (((uint16_t)raw_accel[4]<<8)|raw_accel[5]);

}

// void calibrate_IMU(float * gyroBias, float * accelBias)
static void calibrate_IMU(void){
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	_delay_ms(100);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_PWR_MGMT_1, MPU9250_CLOCK_PLL_XGYRO);
	//resets power management second register turning on gyro and accel
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_PWR_MGMT_2, 0x00);
	_delay_ms(200);

	// Configure device for bias calculation
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_INT_ENABLE, 0x00);   // Disable all interrupts
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_FIFO_EN, 0x00);      // Disable FIFO
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_I2C_MST_CTRL, 0x00); // Disable I2C master
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	_delay_ms(15);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_USER_CTRL, 0x40);   // Enable FIFO
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	_delay_ms(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	read_IMU_bytes(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		read_IMU_bytes(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) ACCEL_SENSITIVITY;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) ACCEL_SENSITIVITY;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_XG_OFFS_USRH, data[0]);
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_XG_OFFS_USRL, data[1]);
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_YG_OFFS_USRH, data[2]);
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_YG_OFFS_USRL, data[3]);
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ZG_OFFS_USRH, data[4]);
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ZG_OFFS_USRL, data[5]);

	// Output scaled gyro biases for display in the main program
	// gyroBias[0] = (float) gyro_bias[0]/(float) GYRO_SENSITIVITY;
	// gyroBias[1] = (float) gyro_bias[1]/(float) GYRO_SENSITIVITY;
	// gyroBias[2] = (float) gyro_bias[2]/(float) GYRO_SENSITIVITY;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	read_IMU_bytes(MPU9250_DEFAULT_ADDRESS, 0x77, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	read_IMU_bytes(MPU9250_DEFAULT_ADDRESS, 0x7A, 2, &data[0]);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	read_IMU_bytes(MPU9250_DEFAULT_ADDRESS, 0x7D, 2, &data[0]);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Push accelerometer biases to hardware registers
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, 0x77, data[0]);
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, 0x78, data[1]);
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, 0x7A, data[2]);
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, 0x7B, data[3]);
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, 0x7D, data[4]);
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, 0x7E, data[5]);

	// Output scaled accelerometer biases for display in the main program
	//   accelBias[0] = (float)accel_bias[0]/(float)ACCEL_SENSITIVITY;
	//   accelBias[1] = (float)accel_bias[1]/(float)ACCEL_SENSITIVITY;
	//   accelBias[2] = (float)accel_bias[2]/(float)ACCEL_SENSITIVITY;

	//set the initial angle according to the accelerometer readings
	// When using the Madgwick algorithm, this code isn't necessary since
	// roll and pitch will be immediately overwritten.
	/*
	read_raw_accel(&accel_raw[0]);
	float accel_x = (int)accel_raw[0]/(float)ACCEL_SENSITIVITY;
	float accel_y = (int)accel_raw[1]/(float)ACCEL_SENSITIVITY;
	float accel_z = (int)accel_raw[2]/(float)ACCEL_SENSITIVITY;
	roll = -asinf(accel_x/sqrtf(accel_x*accel_x+accel_y*accel_y+accel_z*accel_z))*180/3.1415;
	pitch = asinf(accel_y/sqrtf(accel_x*accel_x+accel_y*accel_y+accel_z*accel_z))*180/3.1415;
	*/
}

static void reset_IMU(){
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_PWR_MGMT_1, 0x80);
	_delay_ms(100);// Delay 100 ms
}

void imu_init(){

	//initialize I2C
	init_I2C();

	//check connection with the IMU by asking for the id
	read_IMU_id();

	//reset IMU
	reset_IMU();

	//calibrate IMU
	calibrate_IMU();

	// Disable sleep mode bit (6), enable all sensors
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_PWR_MGMT_1, 0x00);
	_delay_ms(100);// Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	//set clock source with reference to x-axis gyroscope
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_PWR_MGMT_1, MPU9250_CLOCK_PLL_XGYRO);
	_delay_ms(200);

	//92Hz bandwidth, delay 3.9 ms gives about 250 Hz refresh rate //should i do the mask
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_CONFIG, MPU9250_DLPF_BW_98);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_SMPLRT_DIV, 0x03);  // Use a 250 Hz rate; a rate consistent with the filter update rate
	                                    // determined inset in CONFIG above


	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	uint8_t c;
	read_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_FF_THR, &c); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x00;  // Set accelerometer rate to 1 kHz and bandwidth to 460 Hz, delay 1.94ms gives refresh rate 500Hz
	write_IMU_byte(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_FF_THR, c); // Write new ACCEL_CONFIG2 register value
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 4 to 250 Hz because of the SMPLRT_DIV setting

	//set IMU scales
	set_IMU_scales();

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	//write_IMU_byte(MPU9250_DEFAULT_ADDRESS, 0x37, 0x22);
	//write_IMU_byte(MPU9250_DEFAULT_ADDRESS, 0x38, 0x01);  // Enable data ready (bit 0) interrupt
	//_delay_ms(100);

}
// This function has to be executed at 250Hz frequeny (every 4ms)
void imu_read(measured_state_t *destination){
	read_raw_gyro(&gyro_raw[0]);
	float gyro_x = (int)gyro_raw[0]/(float)GYRO_SENSITIVITY;//-gyroBias[0];
	float gyro_y = (int)gyro_raw[1]/(float)GYRO_SENSITIVITY;//-gyroBias[1];
	float gyro_z = (int)gyro_raw[2]/(float)GYRO_SENSITIVITY;//-gyroBias[2];

	read_raw_accel(&accel_raw[0]);
	float accel_x = (int)accel_raw[0]/(float)ACCEL_SENSITIVITY;//-accelBias[0];
	float accel_y = (int)accel_raw[1]/(float)ACCEL_SENSITIVITY;//-accelBias[1];
	float accel_z = (int)accel_raw[2]/(float)ACCEL_SENSITIVITY;//-accelBias[2];

	/*
	// Integration of the roll and pitch
	// divide by 250Hz, equivalent to multiplying by 4ms
	roll += gyro_y/250;
	pitch += gyro_x/250;
	// Roll and pitch coupling calculation
	roll += pitch * sin(gyro_z*3.1415/180/250);
	pitch -= roll * sin(gyro_z*3.1415/180/250);
	// Roll and pitch calculation using accelerometer readings
	roll_accel = -asin(accel_x/sqrt(accel_x*accel_x+accel_y*accel_y+accel_z*accel_z))*180/3.1415;
	pitch_accel = asin(accel_y/sqrt(accel_x*accel_x+accel_y*accel_y+accel_z*accel_z))*180/3.1415;
	// Complementary filtering of the roll and pitch
	roll =  0.996*roll + 0.004*roll_accel;
	pitch = 0.996*pitch + 0.004*pitch_accel;
	*/
	// Alternativelly, a Magdwick filtering can be performed
	// It seems to bemore accurate, however, coupling between angles is not taken into account
	// or I am doing something wrong

	MadgwickAHRSupdateIMU(gyro_x*3.14/180, gyro_y*3.14/180, gyro_z*3.14/180, accel_x, accel_y, accel_z);
	
	destination->roll = asinf(2*(q0*q2-q1*q3));
	destination->pitch = -atan2f(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
	
	//destination->roll = (acosf(q0/sqrtf(q0*q0+q2*q2))*2.0f)*180/3.14;
	//destination->pitch = atan2f(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))*180/3.14;
	
	destination->yaw_vel = gyro_z;
}

