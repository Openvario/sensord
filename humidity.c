#include "humidity.h"
#include "log.h"

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#define I2C_DEVICE "/dev/i2c-1"

int si7021_crc_check(unsigned int value, uint8_t crc)
{
	uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
	uint32_t msb     = 0x800000;
	uint32_t mask    = 0xFF8000;
	uint32_t result  = (uint32_t)value<<8; // Pad with zeros as specified in spec

	while( msb != 0x80 ) {

		// Check if msb of current value is 1 and apply XOR mask
		if( result & msb )
			result = ((result ^ polynom) & mask) | ( result & ~mask);

		// Shift by one
		msb >>= 1;
		mask >>= 1;
		polynom >>=1;
	}
	if( result == crc ) return 0;
	return 1 ;
}

int sht4x_open (t_ds2482 *sensor, unsigned char i2c_address) {

	uint8_t data[6]={0x94,0x89,0,0,0,0};

	sensor->fd = open(I2C_DEVICE, O_RDWR);
	if (sensor->fd < 0) {
		fprintf(stderr, "Error opening file: %s\n", strerror(errno));
		return 1;
	}
	if (ioctl(sensor->fd, I2C_SLAVE, i2c_address) < 0) {
		fprintf(stderr, "ioctl error: %s\n", strerror(errno));
		return 1;
	}

	if (write(sensor->fd,data,1)!=1) return 2;                                    // Do SHT4x soft reset
	struct timespec nstime = {0,15e6};
	while (nanosleep (&nstime,&nstime)) ;                                           // Wait 15 microseconds after reset
	if (write(sensor->fd,data+2,1)!=1) return 2;                                    // Request Serial Number
	if (read(sensor->fd,data,6)!=6) {
		sensor->sensor_type=SHT4X;
		if (si7021_crc_check((data[0]<<8)|(data[1]),data[2])!=0) return 4;
		if (si7021_crc_check((data[3]<<8)|(data[4]),data[5])!=0) return 4;
		fprintf(stderr, "SHT4x, Serial Number: 0x%x%x%x%x - ",data[0],data[1],data[3],data[4]);
	} else {
		data[0]=0x30;
		data[1]=0xa2;
		data[2]=0x36;
		data[3]=0x82;
		if (write(sensor->fd,data,2)!=2) return 2;                                    // Do SHT4x soft reset
		nstime.tv_nsec=15e6;
		while (nanosleep (&nstime,&nstime)) ;                                           // Wait 15 microseconds after reset
		if (write(sensor->fd,data+2,2)!=2) return 2;                                    // Request Serial Number
		if (read(sensor->fd,data,6)!=6) {
			sensor->sensor_type=SHT85;
			if (si7021_crc_check((data[0]<<8)|(data[1]),data[2])!=0) return 5;
			if (si7021_crc_check((data[3]<<8)|(data[4]),data[5])!=0) return 5;
			fprintf(stderr, "SHT85, Serial Number: 0x%x%x%x%x - ",data[0],data[1],data[3],data[4]);
		} else return 3;
	}

	if (g_debug > 0) {
		if (sensor->sensor_type==SHT4X)
			fprintf(stderr, "Opened SHT4x\n");
		else
			fprintf(stderr, "Opened SHT85\n");
	}
	sensor->address = i2c_address;
	return (0);
}


int si7021_open (t_ds2482 *sensor, unsigned char i2c_address) {

	int sernuma, sernumb;
	uint8_t i;

	sensor->fd = open(I2C_DEVICE, O_RDWR);
	if (sensor->fd < 0) {
		fprintf(stderr, "Error opening file: %s\n", strerror(errno));
		return 1;
	}
	if (ioctl(sensor->fd, I2C_SLAVE, i2c_address) < 0) {
		fprintf(stderr, "ioctl error: %s\n", strerror(errno));
		return 1;
	}

	uint8_t data[8]={0xfe,0xe6,131,0,0xe7,0x1e,0x08,0x0a};
	uint8_t data2[4]={0xfa,0x0f,0xfc,0xc9};

	if (write(sensor->fd,data+5,1)!=1) {                                    // Do HTU31D soft reset
		struct timespec nstime = {0,15e6};
		while (nanosleep (&nstime,&nstime)) ;                                           // Wait 15 microseconds after reset
		if (write(sensor->fd,data+6,1)!=1) {					// Request Status byte
			if (read(sensor->fd,data+6,1)==1) {
				fprintf(stderr, "In4\n");
				if (data[6]!=0) return 3;						// Confirm status is good
				if (write(sensor->fd,data+7,1)!=1) return 3;				// Request Serial Number
				if (read(sensor->fd,data2,4)!=4) return 3;				// Read Serial Number
				if (si7021_crc_check((data2[0]<<16)|(data2[1]<<8)|(data2[2]),data2[2])>0) return 4; // Verify CRC on serial number
				switch (sensor->databits) {                                             // Set proper bit configuration
					case 11 : sensor->databits=0x40; break;
					case 12 : sensor->databits=0x4a; break;
					case 13 : sensor->databits=0x54; break;
					case 14 :
					default : sensor->databits=0x5e; break;
				}
				fprintf(stderr, "HTU31D, Serial Number 0x%x%x%x - ",data[0],data[1],data[2]);
				sensor->sensor_type=HTU31D;
				if (g_debug > 0) fprintf(stderr, "Opened HTU31D on 0x%x\n", i2c_address);         // Debug info
				sensor->address = i2c_address;
				return 0;
			}
		}
	}
	if (write(sensor->fd,data,1)<0) return 2;					// Do soft reset for HTU21D/Si7021
	struct timespec nstime = {0,15e6};
	while (nanosleep (&nstime,&nstime)) ;						// Wait 15 microseconds after reset
	if (write(sensor->fd,data+1,2)<0) return 2;					// Program for 11 bits
	if (write(sensor->fd,data+4,1)<0) return 2;					// Initiate readback configuration
	if (read (sensor->fd,data+3,1)<0) return 3;					// Readback configuration
	if ((data[2]&129)!=(data[3]&129)) return 3;					// Is it valid?

	switch (sensor->databits) {							// Set proper bit configuration
		case 11 : data[2] = 131; break;
		case 12 : data[2] = 3;   break;
		case 13 : data[2] = 130; break;
		case 14 :
		default : data[2] = 2;   break;
	}
	if (write(sensor->fd,data+1,2)<0) return 4;					// Program proper bit configuration
	if (write(sensor->fd,data+4,1)<0) return 4;					// Initiate readback configuration
	if (read(sensor->fd,data+3,1)<0) return 4;					// Read back configuration
	if ((data[2]&129)!=(data[3]&129)) return 4; 					// Is it valid?
	if (write(sensor->fd,data2,2)<0) return 4;					// Initiate Serial Number A readback
	if (read (sensor->fd,data,8)<0) return 4;					// Readback Serial Number A
	for (i=sernuma=0 ; i<8 ; i+=2) {
		if (si7021_crc_check(data[i],data[i+1])==1) return 4;			// Check for valid CRC
		sernuma=(sernuma<<8)|(data[i]);
	}

	if (write(sensor->fd,data2+2,2)<0) return 4;					// Initiate Serial Number B readback
	if (read (sensor->fd,data,6)<0) return 4;					// Readback Serial Number B
	for (i=sernumb=0 ; i<6 ; i+=3) {
		if (si7021_crc_check((data[i]<<8)|(data[i+1]),data[i+2])==1) return 4;	// Check for valid CRC
		sernumb=(sernumb<<16)|(data[i]<<8)|(data[i+1]);				// Concatenate Serial Number B
	}

	switch (data[0]) {								// Display Serial number and set sensor type
		case 0x00 :
		case 0xff : sensor->sensor_type=SI7021;
			    fprintf(stderr, "SI7021, Engineering Sample, Serial Number: 0x%x%x\n",sernuma,sernumb); break;
		case 0x0d :
		case 0x14 :
		case 0x15 : sensor->sensor_type=SI7021;
			    fprintf(stderr, "SI70%d, Serial Number: 0x%x%x\n",data[0],sernuma,sernumb); break;
		case 0x32 : sensor->sensor_type=HTU21D;
			    if ((sernumb&0xffff)!=0x4854)
				fprintf(stderr, "HTU21D, but serial number is invalid - ");
			    else
				fprintf(stderr, "HTU21D, Serial Number: 0x%x%x%x%x%x - ",data[3],data[4],sernuma,data[0],data[1]);
			    break;
		default   : sensor->sensor_type=SI7021;
			    fprintf(stderr, "SI70%d, UNKNOWN SENSOR, Serial Number 0x%x%x\n",data[0],sernuma,sernumb);
	}
	if (sensor->sensor_type==SI7021) {						// If it's a SI7021 get the firmware revision
		data[0]=0x84;
		data[1]=0xb8;
		if (write(sensor->fd,data,2)<0) return 4;
		if (read(sensor->fd,data,1)<0) return 4;
		switch (data[0]) {
			case 0xff : fprintf(stderr, "Firmware revision 1.0 - "); break;
			case 0x20 : fprintf(stderr, "Firmware revision 2.0 - "); break;
			default   : fprintf(stderr, "Firmware revision UNKNOWN 0x%x - ",data[0]);
		}
	}

	if (g_debug > 0) {
		if (sensor->sensor_type==SI7021) fprintf(stderr, "Opened SI7021/HTU21D on 0x%x\n", i2c_address);		// Debug info
		else fprintf(stderr, "Opened HTU21D on 0x%x\n",i2c_address);
	}
	sensor->address = i2c_address;
	return (0);
}

int si7021_start_humidity (t_ds2482 *sensor) {
	char config[2],lngth=1;

	switch (sensor->sensor_type) {
		case SI7021 :
		case HTU21D : config[0]=0xf5; break;
		case HTU31D : config[0]=sensor->databits; break;
		case SHT4X  : config[0]=0xfd; break;
		case SHT85  : config[0] = 0x24; config[1] = 0x00; lngth=2; break;
		default     : return 2;
	}
	if (write(sensor->fd, &config, lngth)!=lngth) return 1;
	return 0;
}

int si7021_start_temp (t_ds2482 *sensor) {

	// This sends a temperature conversion for an HTU21D, a humidity conversion for SI7021 (does both), and a conversion for HTU31D
	char config[2],lngth=1;

	switch (sensor->sensor_type) {
		case SI7021 : config[0]=0xf5; break;
		case HTU21D : config[0]=0xf3; break;
		case HTU31D : config[0]=sensor->databits; break;
		case SHT4X  : config[0]=0xfd; break;
		case SHT85  : config[0]=0x24; config[1]=0x00; lngth=2; break;
		default     : return 2;
	}
	if (write(sensor->fd, &config, lngth)!=lngth) return 1;
	return 0;
}

int si7021_read_temp (t_ds2482 *sensor) {

	uint8_t data[3];
	double temp;

	switch (sensor->sensor_type) {
		case HTU21D :
			sensor->temp_valid=0;
			if (read(sensor->fd, data, 3) != 3) return 4;
			if (si7021_crc_check((data[0]<<8)|data[1],data[2])>0) return 2;
			temp = (double) ((data[0]<<8)|data[1]) * 175.72/65536.0 - 46.85;
			sensor->temperature = temp;
			sensor->temp_valid = sensor->temp_present;
			break;
		default : si7021_read_humidity (sensor);
	}
	return 0;
}

int si7021_read_humidity (t_ds2482 *sensor) {
	uint8_t data[6],x,y;
	double temp;

	sensor->humidity_valid=0;
	if (sensor->sensor_type!=HTU21D) sensor->temp_valid=0;
	switch (sensor->sensor_type) {
		case SI7021 : case HTU21D :
			if (read(sensor->fd, data, 3) != 3) return 4;
			if ((x=si7021_crc_check((data[0]<<8)|data[1],data[2]))==0) {
				temp = (double) ((data[0]<<8)|data[1]) * 125/65536.0 - 6;
				if (sensor->compensate) temp += (25-sensor->temperature) * -0.15;
				sensor->humidity = temp;
				sensor->humidity_valid = sensor->humidity_present;
			}
			if (sensor->sensor_type==HTU21D) return x;
			data[0]=0xe0;
			if (write(sensor->fd,data, 1) != 1) return 4;
			if (read(sensor->fd, data, 3) != 3) return 4;
			if ((y=si7021_crc_check((data[0]<<8)|data[1],data[2]))==0) {
				sensor->temperature = (double) ((data[0]<<8)|data[1]) * 175.72/65536.0 - 46.85;
				sensor->temp_valid = sensor->temp_present;
			}
			return ((y<<1)|x);
			break;
		case HTU31D :
			data[0]=0x0;
			if (write(sensor->fd,data, 1) != 1) return 4;
		case SHT4X : case SHT85 :
			if (read(sensor->fd, data, 6) != 6) return 4;
			x=si7021_crc_check((data[0]<<8)|data[1],data[2]);
			y=si7021_crc_check((data[3]<<8)|data[4],data[5]);
			switch (sensor->sensor_type) {
				case HTU31D :
					sensor->temperature = (double) ((data[0]<<8)|data[1]) * 165/65535.0 - 40;
					sensor->humidity = (double) ((data[3]<<8)|data[4]) * 100/65535.0;
					break;
				case SHT4X :
					sensor->temperature = (double) ((data[0]<<8)|data[1]) * 175/65535.0 - 45;
					sensor->humidity = (double) ((data[3]<<8)|data[4]) * 125/65535.0 - 6;
					break;
				case SHT85 :
					sensor->temperature = (double) ((data[0]<<8)|data[1]) * 175/65535.0 - 45;
 					sensor->humidity = (double) ((data[3]<<8)|data[4]) * 100/65535.0;
					break;
			}
			if (x==0) sensor->temp_valid = sensor->temp_present;
			if (y==0) sensor->humidity_valid = sensor->humidity_present;
			return ((x<<1)|y);
			break;
	}
	return 0;
}

int si7021_configure_heater_value (t_ds2482 *sensor, int value)
{
	uint8_t data[3] = {0x51,0x00,0x11};

	if (sensor->sensor_type==SI7021) {
		data[1]=value&0xf;				// If SI7021 initialize the heat control register
		if (write(sensor->fd,data,2)!=2) return 2;	// Program it
		if (write(sensor->fd,data+2,1)!=1) return 2;    // Start readback
		if (read(sensor->fd,data,1)!=1) return 2;	// Readback
		if (data[1]!=(value&0xf)) return 2;		// Verify readback matches written value
	} else return 1;
	return 0;
}

// return code of 3 or 4 means you didn't successfully perorm the operation.

int si7021_configure_heater_onoff (t_ds2482 *sensor, int value)
{
	uint8_t data[4] = {0xe7,0xe6,0x00,0x08};
	if ((value!=0) && (value!=1)) return 6;
	switch (sensor->sensor_type) {
		case HTU31D:
			if (value==1) data[2]=4; else data[2]=2;
			if (write(sensor->fd,data+2,2)!=2) return 2;
			if (read(sensor->fd,data,1)!=1) return 3;
			if ((data[0]&1)!=(value&1)) return 4;
			break;
		case HTU21D : case SI7021 :
			if (write(sensor->fd,data,1)!=1) return 2;		// Start read user register 1
			if (read(sensor->fd,data+2,1)!=1) return 2;		// Read user register 1
			if (value==1) data[2]|=4; else data[2]&=0xfb;		// Set heater value as desired
			data[3]=0xe7;
			data[0]=data[2];
			if (write(sensor->fd,data+1,2)!=2) return 3;		// Program it
			if (write(sensor->fd,data+3,1)!=1) return 3;		// Start readback
			if (read(sensor->fd,data+2,1)!=1) return 3;		// Readback
			if (data[2]!=data[0]) return 4;				// Verify Readback matches written value
			break;
		case SHT4X :
			data[0]=0x39;
			if (write(sensor->fd,data,1)!=1) return 2;
			break;
		case SHT85 :
			data[0]=0x30;
			data[2]=0xf2;
			data[3]=0x3d;
			if (value==1) data[1]=0x6d; else data[1]=0x66;
			if (write(sensor->fd,data,4)!=4) return 2;
			if (read(sensor->fd,data,3)!=3) return 3;
			if (((data[0]>>5)&1)!=value) return 4;
			if (si7021_crc_check((data[0]<<8)|data[1],data[2])==1) return 5;
	}
	return 0;
}

static uint16_t _calc_crc16(const uint8_t *buf, size_t len) {
	uint16_t crc = 0xFFFF;

	while(len--) {
		crc ^= (uint16_t) *buf++;
		for (unsigned i = 0; i < 8; i++)
			if (crc & 0x0001) crc = (crc>>1) ^ 0xa001; else crc >>= 1;
	}
	return crc;
}

static uint16_t _combine_bytes(uint8_t msb, uint8_t lsb) {
	return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

int am2321_open (t_ds2482 *sensor, unsigned char i2c_address) {

	sensor->fd = open(I2C_DEVICE, O_RDWR);
	if (sensor->fd < 0) {
		fprintf(stderr, "Error opening file: %s\n", strerror(errno));
		return 1;
	}
	if (ioctl(sensor->fd, I2C_SLAVE, i2c_address) < 0) {
		fprintf(stderr, "ioctl error: %s\n", strerror(errno));
		return 1;
	}

	if (am2321_read(sensor)>0) return 1;
	if (g_debug > 0) fprintf(stderr, "Opened AM2321 on 0x%x\n", i2c_address);

	// assign file handle to sensor object
	sensor->address = i2c_address;
	return (0);
}

int am2321_wakeup (t_ds2482 *sensor) {

	//  wake AM2320 up, goes to sleep to not warm up and affect the humidity sensor

	if (write(sensor->fd, NULL, 0)<0) return 3;
	struct timespec nstime = {0,1.0e6};
	while (nanosleep (&nstime,&nstime)) ; /* Wait atleast 1.5ms */
	return 0;
}

int am2321_read (t_ds2482 *sensor) {
	uint8_t data[8];

	data[0] = 0x00;
	if (write(sensor->fd, NULL, 0)<0)
	return 3;

	struct timespec nstime = {0,1.0e6};
	while (nanosleep (&nstime,&nstime)) ; /* Wait atleast 1.5ms */

	/* write at addr 0x03, start reg = 0x00, num regs = 0x04 */
	data[0] = 0x03;
	data[1] = 0x00;
	data[2] = 0x04;
	if (write(sensor->fd, data, 3) < 0) return 3;

	/* wait for AM2320 */
	nstime.tv_nsec = 1.6e6;
	while (nanosleep (&nstime,&nstime)) ; /* Wait atleast 1.5ms */

	/*
	 * Read out 8 bytes of data
	 * Byte 0: Should be Modbus function code 0x03
	 * Byte 1: Should be number of registers to read (0x04)
	 * Byte 2: Humidity msb
	 * Byte 3: Humidity lsb
	 * Byte 4: Temperature msb
	 * Byte 5: Temperature lsb
	 * Byte 6: CRC lsb byte
	 * Byte 7: CRC msb byte
	*/

  	if (read(sensor->fd, data, 8) < 0) return 4;

	/* Check data[0] and data[1] */
	if (data[0] != 0x03 || data[1] != 0x04) return 9;

	/* Check CRC */
	uint16_t crcdata = _calc_crc16(data, 6);
	uint16_t crcread = _combine_bytes(data[7], data[6]);
	if (crcdata != crcread) return 10;

	uint16_t temp16 = _combine_bytes(data[4], data[5]);
	uint16_t humi16 = _combine_bytes(data[2], data[3]);

	/* Temperature resolution is 16Bit,
	 * temperature highest bit (Bit15) is equal to 1 indicates a
	 * negative temperature, the temperature highest bit (Bit15)
	 * is equal to 0 indicates a positive temperature;
	 * temperature in addition to the most significant bit (Bit14 ~ Bit0)
	 *  indicates the temperature sensor string value.
	 * Temperature sensor value is a string of 10 times the
	 * actual temperature value.
	*/

 	if (temp16 & 0x8000) temp16 = -(temp16 & 0x7FFF);

	if (sensor->temp_present) {
		sensor->temperature = (float)temp16 / 10.0;
		sensor->temp_valid = 1;
	}
	if (sensor->humidity_present) {
		sensor->humidity = (float)humi16 / 10.0;
		sensor->humidity_valid = 1;
	}
	return 0;
}


