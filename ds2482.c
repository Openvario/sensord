/*
    sensord - Sensor Interface for XCSoar Glide Computer - http://www.openvario.org/
    Copyright (C) 2014  The openvario project
    A detailed list of copyright holders can be found in the file "AUTHORS"

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 3
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, see <http://www.gnu.org/licenses/>.
*/

#include "ds2482.h"
#include "log.h"

#include <time.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

//http://datasheets.maximintegrated.com/en/ds/DS2482-100.pdf
//http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf
//http://www.wulfden.org/downloads/datasheets/DS2482_AN3684.pdf

// The routines in this code were roughly lifted from https://pastebin.com/0d93ZuRb
// Those assume the use of the Search and Match ROM.  As such a number of
// functions are included that are not needed if a Skip ROM is used which
// is adequate, and faster if only a single 1 Wire device is present.

int ds2482_open(t_ds2482 *sensor, unsigned char i2c_address)
{
	// local variables
	int fd;

	// try to open I2C Bus
	fd = open("/dev/i2c-1", O_RDWR);

	if (fd < 0) {
		fprintf(stderr, "Error opening file: %s\n", strerror(errno));
		return 0;
	}
	if (ioctl(fd, I2C_SLAVE, i2c_address) < 0) {
		fprintf(stderr, "ioctl error: %s\n", strerror(errno));
		return 0;
	}

	if (g_debug > 0) fprintf(stderr, "Opened DS2482 on 0x%x\n", i2c_address);
	//assign file handle to sensor object
	sensor->fd = fd;
	sensor->address = i2c_address;
	return 1;
}

int ds2482_reset(t_ds2482 *sensor) {
	//server.log(format("Function: Resetting DS2482 at %i (%#x)", I2CAddr, I2CAddr));

	if (write(sensor->fd, "\xf0", 1)!=1) return 0; //reset DS2482
	sensor->owDeviceAddress[0]=0;
	sensor->owDeviceAddress[1]=0;
	sensor->owTriplet=4;
	sensor->owLastDevice=0;
	sensor->owLastDiscrepancy=0;
	sensor->temp_present=0;
	sensor->temp_valid=0;
	sensor->temperature=23;
	return 1;
}

int OWReset(t_ds2482 *sensor) {
	int i;
	unsigned char data;

	// server.log("Function: I2C Reset");
	if (write(sensor->fd, "\xb4",1)!=1) { // 1-wire reset
		// Device failed to acknowledge reset
		// server.log("I2C Reset Failed");
		return 0;
	}
	for (i=0;i<100;++i) {
		if (read(sensor->fd,&data,1)==0) { // Read the status register
			// server.log("I2C Read Status Failed");
			return 0;
		} else {
			// server.log(format("Read Status Byte = %d", data[0]));
			if (data & 1) { // 1-Wire Busy bit
				struct timespec nstime = {0,1e6};
				while (nanosleep(&nstime,&nstime)) ; // Wait, try again
			} else {
				// server.log("One-Wire bus is idle");
				if (data & 4) { // Short Detected bit
					// server.log("One-Wire Short Detected");
					return 0;
				}
				if (data & 2) { // Presense-Pulse Detect bit
					// server.log("One-Wire Devices Found");
					return 1;
				} else {
					// server.log("No One-Wire Devices Found");
					return 0;
				}
			}
		}
	}
	// server.log("One-Wire busy too long");
	return 0;
}

// Return values are:
// -1: Failure
//  1: Success

int OWWriteByte(t_ds2482 *sensor, unsigned char writeval) {
	int i;
	unsigned char data[2];

	// server.log("Function: Write Byte to One-Wire");
	if (write(sensor->fd,"\xe1\xf0",2)!=2) { // Device failed to acknowledge
		// server.log("I2C Write Failed");
		return -1;
	}
	for (i=0;i<100;++i) {
		if (read(sensor->fd,data,1)==0) { // Read the status register
			// server.log("I2C Read Status Failed");
			return -1;
		} else {
			// server.log(format("Read Status Byte = %d", data[0]));
			if (data[0] & 0x01) { // 1-Wire Busy bit
				// server.log("One-Wire bus is busy");
				struct timespec nstime = {0,1e6};
				while (nanosleep(&nstime,&nstime)) ; // Wait, try again

			} else {
				// server.log("One-Wire bus is idle");
				break;
			}
		}
	}
	if (i == 100) {
		// server.log("One-Wire busy for too long");
		return -1;
	}
	data[0]=0xa5;
	data[1]=writeval;
	if (write(sensor->fd,data,2)!=2) { // set write byte command (A5) and send data (byte)
		// Device failed to acknowledge
		// server.log(format("I2C Write Byte Failed. Data: %#.2X", byte));
		return -1;
	}
	for (i=0;i<100;++i) {
		if (read(sensor->fd,data,1)==0) { // Read the status register
			// server.log("I2C Read Status Failed");
			return -1;
		} else {
			// server.log(format("Read Status Byte = %d", data[0]));
			if (data[0] & 1) { // 1-Wire Busy bit
				// server.log("One-Wire bus is busy");
				struct timespec nstime = {0,1e6};
				while (nanosleep(&nstime,&nstime)) ; // Wait, try again

			} else {
				// server.log("One-Wire bus is idle");
				break;
			}
		}
	}
	if (i == 100) {
		// server.log("One-Wire busy for too long");
		return -1;
	}

	// server.log("One-Wire Write Byte complete");
	return 1;
}

// Return values are:
// -1:  failure
// >=0: byte that was read out

int OWReadByte(t_ds2482 *sensor) {
	int i;
	unsigned char data;

	// See if the 1wire bus is idle
	// server.log("Function: Read Byte from One-Wire");
	if (write(sensor->fd,"\xe1\xf0",2)!=2) { // Device failed to acknowledge
		// server.log("I2C Write Failed");
		return -1;
	}
	for (i=0;i<100;++i) {
		if (read(sensor->fd,&data,1)==0) { // Read the status register
			// server.log("I2C Read Status Failed");
			return -1;
		} else {
			// server.log(format("Read Status Byte = %d", data));
			if (data & 0x01) { // 1-Wire Busy bit
				// server.log("One-Wire bus is busy");
				struct timespec nstime = {0,1e6};
				while (nanosleep(&nstime,&nstime)) ; // Wait, try again
			} else {
				// server.log("One-Wire bus is idle");
				break;
			}
		}
	}
	if (i == 100) {
		// server.log("One-Wire busy for too long");
		return -1;
	}
	// Send a read command, then wait for the 1wire bus to finish
	if (write(sensor->fd,"\x96",1)!=1) { // Device failed to acknowledge
		// server.log("I2C Write Read-Request Failed");
		return -1;
	}
	for (i=0;i<100;++i) {
		if (read(sensor->fd,&data,1)==0) { // Read the status register
			// server.log("I2C Read Status Failed");
			return -1;
		} else {
			// server.log(format("Read Status Byte = %d", data));
			if (data & 1) { // 1-Wire Busy bit
				// server.log("One-Wire bus is busy");
				struct timespec nstime = {0,1e6};
				while (nanosleep(&nstime,&nstime)) ; // Wait, try again
			} else {
				// server.log("One-Wire bus is idle");
				break;
			}
		}
	}
	if (i == 100) {
		// server.log("One-Wire busy for too long");
		return -1;
	}

	// Go get the data byte
	if (write(sensor->fd,"\xe1\xe1",2)!=2) { // Device failed to acknowledge
		// server.log("I2C Write Failed");
		return -1;
	}
	if (read(sensor->fd,&data,1)==0) { // Read the status register
		// server.log("I2C Read Status Failed");
		return -1;
	}
	return data;
}

// This function is unused and untested

int OWTriplet(t_ds2482 *sensor) {
	unsigned char data[2]={0x78,0};
	int i;

	// server.log("Function: OneWire Triplet");
	data[1]=sensor->owTriplet<<5;
	if (write(sensor->fd,data,2)!=2) { // Device failed to acknowledge
		// server.log("OneWire Triplet  Failed");
		return 0;
	}
	for (i=0;i<100;++i) {
		if (read(sensor->fd,&data,1)==0) { // Read the status register
			// server.log("I2C Read Status Failed");
			return 0;
		} else {
			// server.log(format("Read Status Byte = %d", data[0]));
			if (data[0] & 1) { // 1-Wire Busy bit
				// server.log("One-Wire bus is busy");
				struct timespec nstime = {0,1e6};
				while (nanosleep(&nstime,&nstime)) ; // Wait, try again
			} else {
				// server.log("One-Wire bus is idle");
				sensor->owTriplet=(data[0]>>5)&7;
				return 1;
			}
		}
	}
	if (i == 100) {
		// server.log("One-Wire busy for too long");
		return 0;
	}
	return 1;
}

// The following function is unused and untested

int OWSearch(t_ds2482 *sensor) {
	// server.log("Function: OneWire Search");

	int deviceAddress4ByteMask = 1;

	sensor->temp_present=0;
	if (sensor->owLastDevice) {
		// server.log("OneWire Search Complete");
		sensor->owLastDevice = 0;
		sensor->owLastDiscrepancy = 0;
		sensor->owDeviceAddress[0] = 0xFFFFFFFF;
		sensor->owDeviceAddress[1] = 0xFFFFFFFF;
	}
	if (!sensor->owLastDevice) { // if the last call was not the last one
		if (OWReset(sensor)==0) { // if there are no parts on 1-wire, return false
			sensor->owLastDiscrepancy = 0;
			return 0;
		}
		int bitNumber = 1;
		int lastZero = 0;
		int deviceAddress4ByteIndex = 0; // Fill last 4 bytes first, data from onewire comes LSB first.

		OWWriteByte(sensor, 0xF0); //Issue the Search ROM command
		do { // loop to do the search
			if (bitNumber < sensor->owLastDiscrepancy) {
				if (sensor->owDeviceAddress[deviceAddress4ByteIndex] & deviceAddress4ByteMask)
					sensor->owTriplet |= 4;  else sensor->owTriplet &= 3;
			} else if (bitNumber == sensor->owLastDiscrepancy) // if equal to last pick 1, if not pick 0
				sensor->owTriplet |= 4; else sensor->owTriplet &= 3;
			if (!OWTriplet(sensor)) return 0;

			// if 0 was picked then record its position in lastZero
			if (sensor->owTriplet==0) lastZero = bitNumber;

			// check for no devices on 1-wire
			if ((sensor->owTriplet&3)==3) break;

			// set or clear the bit in the SerialNum byte serial_byte_number with mask
			if (sensor->owTriplet&4)
				sensor->owDeviceAddress[deviceAddress4ByteIndex] |= deviceAddress4ByteMask;
			else
				sensor->owDeviceAddress[deviceAddress4ByteIndex] &= ~deviceAddress4ByteMask;
			bitNumber++; // increment the byte counter bit number
			deviceAddress4ByteMask <<=  1; // shift the bit mask left

			if (!deviceAddress4ByteMask) { // if the mask is 0 then go to other address block and reset mask to first bit
				deviceAddress4ByteIndex++;
				deviceAddress4ByteMask = 1;
			}
		} while (deviceAddress4ByteIndex <2);

		if (bitNumber == 65) { // if the search was successful then
			sensor->owLastDiscrepancy = lastZero;
			if (sensor->owLastDiscrepancy==0) sensor->owLastDevice = 1; else sensor->owLastDevice = 0;
			// server.log(format("OneWire Device Address = %.8X%.8X", owDeviceAddress[1], owDeviceAddress[0]));
			if (OWCheckCRC(sensor)) {
				if ((sensor->owDeviceAddress[0] & 0xff) == 0x28) {
					sensor->temp_present=1; return 1;
				} else {
					// server.log("OneWire device address CRC check failed");
					return 1;
				}
			}
		}
	}
	// server.log("No One-Wire Devices Found, Resetting Search");
	sensor->owLastDiscrepancy = 0;
	sensor->owLastDevice = 0;
	return 0;
}

// The following function is unused and untested, and probably should be an inline function

int OWCheckCRC(t_ds2482 *sensor) {
	int crc, j,i;
	long int da32bit;

	for (i=0;i<2;i++) {
		for (j=0,crc=0,da32bit=sensor->owDeviceAddress[i]; j<4; j++) { // All four bytes
			crc = AddCRC(da32bit & 0xFF, crc);
			// server.log(format("CRC = %.2X", crc));
			da32bit >>= 8; // Shift right 8 bits
		}
	}
	// server.log(format("CRC = %#.2X", crc));
	// server.log(format("DA  = %#.2X", da32bit));
	if ((da32bit & 0xFF) == crc) { // last byte of address should match CRC of other 7 bytes
		// server.log("CRC Passed");
		return 1; // match
	}
	return 0; // bad CRC
}

// The following function is unused and untested, and probably should be an inline function

int  AddCRC(int inbyte, int crc) {
	int i;

	for(i=0; i<8; i++) {
		if ((crc ^ inbyte) & 0x01) crc=(crc>>1)^0x8c; else crc>>=1;
		inbyte >>= 1;
	}
	return crc;
}

// The following function is unused untested

int OWSelect(t_ds2482 *sensor) {
	// server.log("Selecting device");
	int i,j;

	if (OWWriteByte(sensor, 0x4C)==-1) return 0; // Issue the Match ROM command
	for(i=0; i<2; i++) {
		long int da32bit = sensor->owDeviceAddress[i];
		for(j=0; j<4; j++) {
			// server.log(format("Writing byte: %.2X", da32bit & 0xFF));
			OWWriteByte(sensor, da32bit & 0xff); // Send lowest byte
			da32bit >>=  8; // Shift right 8 bits
		}
	}
	return 1;
}

// Return values are:
// 0:  There was a problem
// 1:  Appears to have functioned properly

int OWConfigureBits (t_ds2482 *sensor) {
	unsigned char data[4] = {0x4e,0x00,0x00,0x7f};
	int i,j;

	if (OWReset(sensor)==0) return 0;
	if (OWWriteByte(sensor,0xCC)==-1) return 0;
	switch (sensor->databits) {
		case 9  : data[3]=0x1f; sensor->conversion_time = 6; sensor->delta_conversion_time = 5; break;
		case 10 : data[3]=0x3f; sensor->conversion_time = 13; sensor->delta_conversion_time = 5;  break;
		case 11 : data[3]=0x5f; sensor->conversion_time = 28; sensor->delta_conversion_time = 5; break;
		default : sensor->conversion_time = 58; sensor->delta_conversion_time = 5; break;
	}
	for (i=0,j=1;i<4;++i)
		if (OWWriteByte(sensor,data[i])==-1) j=0;
	return j;
}


// Return values are:
// 0 = Not valid
// 1 = Valid
// 2 = Appears valid, but resolution doesn't match expected value

int OWReadTemperature(t_ds2482 *sensor) {
	int data[5];
	int i,j;

	sensor->temp_valid=0;
	for(i=0,j=0; i<5; i++) { // we only need 5 of the bytes
		// Technically we only need two, but grabbing 5 lets us double check the configuration
		data[i] = OWReadByte(sensor);
		if (data[i]<0) j=1;
		// server.log(format("read byte: %.2X", data[i]));
	}
	if (j) return 0;
	j=1;
	i = (data[1] << 8) | data[0];
	switch (data[4]&0x60) {
		case 0x60 : // server.log("12 bit resolution"); // 750 ms conversion time
			sensor->temperature=i/16.0;
			if (sensor->databits!=12) j=2;
			break;
		case 0x40 : // server.log("11 bit resolution"); // 375 ms
			sensor->temperature=(i>>1)/8.0;
			if (sensor->databits!=11) j=2;
			break;
		case 0x20 : // server.log("10 bit resolution"); // 187.5 ms
			sensor->temperature=(i>>2)/4.0;
			if (sensor->databits!=10) j=2;
			break;
		default : // server.log("9 bit resolution"); // 93.75 ms
			sensor->temperature=(i>>3)/2.0;
			if (sensor->databits!=9) j=2;
	}
	if ((sensor->temperature<125) && (sensor->temperature>-55)) sensor->temp_valid=1; else j=0;
	// server.log(format("Temperature = %.1f °C", celsius));
	debug_print("%s @ 0x18: temperature %f\n",__func__,sensor->temperature);
	return j;
}
