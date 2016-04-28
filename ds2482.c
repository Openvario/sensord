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
	
	This code should be compatible with DS18S20 and DS18B20 sensords
	DS18S20: Fixed 9-bit resolution (1/2°C). Measures temperature in 750 mS.
	DS18B20: Resolution of 9 to 12 bits (12-1/16°C).
*/

#include "ds2482.h"
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
#include "def.h"

extern int g_debug;
extern FILE *fp_console;


#define REG_STATUS 	0xF0
#define REG_READ 	0xE1
#define REG_CONF 	0xC3

#define CMD_RESET 				0xF0
#define CMD_SET_READ_POINTER 	0xE1
#define CMD_WRITE_CONFIG 		0xD2
#define CMD_1_WIRE_RESET 		0xB4
#define CMD_1_WIRE_SINGLE_BIT	0x87
#define CMD_1_WIRE_WRITE_BYTE	0xA5
#define CMD_1_WIRE_READ_BYTE	0x96
#define CMD_1_WIRE_TRIPLET		0x78

#define DS1820_CMD_CONVERT_TEMP		0x44
#define DS1820_CMD_READ_SCRATCHPAD	0xBE
#define DS1820_CMD_SKIP_ROM			0xCC

#define POLL_LIMIT 				30

t_ds2482 *_sensor;

int ds2482_open(t_ds2482 *sensor, unsigned char i2c_address) {
	// local variables
	int fd;
	
	// try to open I2C Bus
	fd = open("/dev/i2c-1", O_RDWR);
	
	if (fd < 0) {
		fprintf(stderr, "Error opening file: %s\n", strerror(errno));
		return 1;
	}
	
	if (ioctl(fd, I2C_SLAVE, i2c_address) < 0) {		
		fprintf(stderr, "ioctl error: %s\n", strerror(errno));
		return 1;
	}
	
	
	ddebug_print("Opened ds2482 on 0x%x\n", i2c_address);
	
	// assign file handle to sensor object
	sensor->fd = fd;
	sensor->address = i2c_address;
	return (0);

}
int ds2482_init(t_ds2482 *sensor) {
	_sensor = sensor;
	return ds2482_reset();
}

int ds2482_measure(t_ds2482 *sensor) {
	_sensor = sensor;
	_sensor->present = 0;
	
	// reset sensor
	if (OWReset()) { //Reset was successful
		OWWriteByte(DS1820_CMD_SKIP_ROM); //Issue the Skip ROM command, there's only one 1-wire slave
		OWWriteByte(DS1820_CMD_CONVERT_TEMP); //start conversion
		
		usleep(5000);
		
		if(OWReset()) {
			OWWriteByte(DS1820_CMD_SKIP_ROM); //Issue the Skip ROM command
			OWWriteByte(DS1820_CMD_READ_SCRATCHPAD); //Read Scratchpad
			OWReadTemperature();
			
			return 0;
		}
	}
	return (1);
}


unsigned char ds2482_reset() {
	unsigned char buf[2];
	unsigned char conf;
	
	ddebug_print("ds2482 - reset\n");
	
	OWWaitForBusyEnd();
	
	// reset
	buf[0] = CMD_RESET;
	ds2482_write(buf, 1);
	
	usleep(5000);
	
	// write config
	//   1-Wire speed (c1WS) = standard (0)
	//   Strong pullup (cSPU) = on (1)
	//   Presence pulse masking (cPPM) = off (0)
	//   Active pullup (cAPU) = on (CONFIG_APU = 0x01)
	conf = 0 | 0 | 0 | 0x01;
	
	buf[0] = CMD_WRITE_CONFIG;
	buf[1] = conf;
	buf[1] = buf[1] | (~buf[1] << 4); // The upper nibble in bitwise inverted when written to the DS2482.
	ds2482_write(buf, 2);
	
	buf[0] = CMD_SET_READ_POINTER;
	buf[1] = REG_CONF;
	ds2482_write(buf, 2);
	
	ds2482_read(buf, 1);
	if(buf[0] != conf) {		
		ddebug_print("ds2482 - config set failed.%x\n", buf[0]);
		return (1);
	}
	return (0);
}
unsigned char ds2482_get_status() {
	unsigned char buf[2];
	buf[0] = CMD_SET_READ_POINTER;
	buf[1] = REG_STATUS;
	ds2482_write(buf, 2);
	ds2482_read(buf, 1);
	return buf[0];
}

unsigned char ds2482_read(unsigned char *buf, unsigned int dataLength) {
	if (read(_sensor->fd, buf, dataLength) != dataLength) {
		ddebug_print("Unable to read from slave ds2482\n");
		return(1);
	} else {
		return (0);
	}
}
unsigned char ds2482_write(unsigned char *buf, unsigned int dataLength) {
	if (write(_sensor->fd, buf, dataLength) != dataLength) {
		ddebug_print("Unable to write to slave ds2482: %x\n", buf[0]);
		return(1);
	} else {
		return (0);
	}
}

int OWReset() {
	unsigned int loopcount = 0;
	unsigned char status;
	unsigned char buf[1];
	
	buf[0] = CMD_1_WIRE_RESET;
	ds2482_write(buf, 1); //1-wire reset
    
	while(1) {
        loopcount++;
        status = ds2482_get_status();
		if (status & 0x01) { // 1-Wire Busy bit
			if (loopcount > POLL_LIMIT) {
				ddebug_print("One-Wire busy too long\n");
				return 0;
			}
			usleep(1000); //Wait, try again
		} else {
			if (status & 0x04) { //Short Detected bit
				ddebug_print("One-Wire Short Detected\n");
				return 0;
			}
			if (status & 0x02) { //Presense-Pulse Detect bit
			   break;
			} else {
				ddebug_print("No One-Wire Devices Found\n");
				return 0;
			}
		}
    }
    return 1;
}

int OWWriteByte(unsigned char byte) {
	if(OWWaitForBusyEnd()) {
		unsigned char buf[2];
		buf[0] = CMD_1_WIRE_WRITE_BYTE;
		buf[1] = byte;
		ds2482_write(buf, 2); //set write byte command (A5) and send data (byte)
		return 1;
	}
	return 0;
}

int OWReadByte() {
	unsigned char buf[2];
	if(OWWaitForBusyEnd()) {
		buf[0] = CMD_1_WIRE_READ_BYTE;
		ds2482_write(buf, 1); //send read byte command (96)
		
		if(OWWaitForBusyEnd()) {
			buf[0] = CMD_SET_READ_POINTER;
			buf[1] = REG_READ;
			ds2482_write(buf, 2); //set read pointer (E1) to the read data register (E1)
	   
			ds2482_read(buf, 1); //Read the data register
			return buf[0];
		}
	}
	return -1;
}

int OWWaitForBusyEnd() {
	unsigned int loopcount = 0;
	unsigned char status;
	do {
		status = ds2482_get_status();
		if(status & 0x01) {
			if (++loopcount > POLL_LIMIT) {
				ddebug_print("One-Wire busy for too long\n");
				return 0;
			} else {
				usleep(100);	
			}
		}
        
    } while((status & 0x01));
	return 1;
}


void OWReadTemperature() {
    unsigned char data[5] = {0, 0, 0, 0, 0};
    unsigned int i;
    for(i = 0; i <= 5; i++) {
        data[i] = OWReadByte();
		if(data[i] == -1) {
			ddebug_print("One-Wire error reading sensor\n");
			return;
		}
    }
	
    unsigned int raw = (data[1] << 8) | data[0];
    unsigned char cfg = data[4] & 0x60;
	
    // this check is for DS18B20	
	if (cfg == 0x60) { // 12-bit, 750 ms conversion time
		
	} else if (cfg == 0x40) { // 11 bit, 375 ms
        raw = raw << 1;
    } else if (cfg == 0x20) { // 10 bit, 187.5 ms
        raw = raw << 2;
    } else { // 9 bit, 93.75 ms
        raw = raw << 3; // DS18S20 is handled here
    }
	
	unsigned char neg = (data[1] >> 7) & 1;
    if (neg) {  // negative
		raw = (raw ^ 0xffff) + 1;
	}
    float celsius = raw / 16.0;
	if(neg) celsius = -celsius;
    if(celsius >= -55.0 && celsius <= 125.0) {
		_sensor->celsius = celsius;
		_sensor->present = 1;
	} else {
		printf("One-Wire temperature out of range %f\n", celsius);
	}
}