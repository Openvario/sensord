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

#include "ams5915.h"
#include <time.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include "def.h"

extern int g_debug;
extern FILE *fp_console;

/**
* @brief Establish connection to AMS5915 pressure sensor
* @param sensor pointer to sensor instance
* @param i2c_address
* @return result
*
* @date 24.03.2016 revised
*
*/ 
int ams5915_open(t_ams5915 *sensor, unsigned char i2c_address)
{
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
	
	if (g_debug > 0) printf("Opened AMS5915 on 0x%x\n", i2c_address);
	
	// assign file handle to sensor object
	sensor->fd = fd;
	sensor->address = i2c_address;
	return (0);
}

/**
* @brief Initialize AMS5915 pressure sensor
* @param sensor pointer to sensor instance
* @return result
*
* @date 24.03.2016 revised
*
*/
int ams5915_init(t_ams5915 *sensor)
{
	// set calibration data
	sensor->digoutpmin = 1638;    // from datasheet
	sensor->digoutpmax = 14745;   // from datasheet
	sensor->pmin = 0;             // from datasheet
	sensor->pmax = 50;            // from datasheet
		
	sensor->sensp = (float)(sensor->digoutpmax - sensor->digoutpmin)/(sensor->pmax - sensor->pmin); 
	ddebug_print("%s @ 0x%x: sensp=%f\n", __func__, sensor->address, sensor->sensp);
	return(0);
}

/**
* @brief Read pressure measurement from AMS5915 pressure sensor
* @param sensor pointer to sensor instance
* @return result
*
* @date 24.03.2016 revised
*
*/ 
int ams5915_measure(t_ams5915 *sensor)
{
	//variables
	uint8_t buf[10]={0x00};

	sensor->prevtime=sensor->curtime;
	clock_gettime(CLOCK_REALTIME,&sensor->curtime);
	if (read(sensor->fd, buf, 4) != 4) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		return(1);
	}
	
	sensor->digoutp = ((buf[0] & (0x3F)) << 8) + buf[1];
	sensor->digoutT = ((buf[2] << 8) + (buf[3] & 0xE0)) >> 5;
	debug_print("%s @ 0x%x: digoutp=0x%x %d\n", __func__, sensor->address, sensor->digoutp, sensor->digoutp);
	debug_print("%s @ 0x%x: digoutT=0x%x %d\n", __func__, sensor->address,sensor->digoutT, sensor->digoutT);
	return(0);
}

/**
* @brief Calculate pressure from AMS5915 pressure sensor
* @param sensor pointer to sensor instance
* @return result
*
* @date 24.03.2016 revised
*
*/ 
int ams5915_calculate(t_ams5915 *sensor)
{
	// calculate differential pressure
	sensor->p = (((sensor->digoutp - sensor->digoutpmin)/sensor->sensp) + sensor->pmin);
	
	// calculate temperature
	sensor->T = ((sensor->digoutT * 200)/ (float)2048)-50;
	
	// correct measured pressure
	sensor->p = sensor->linearity * sensor->p + sensor->offset;
	
	debug_print("%s @ 0x%x: Pressure: %f Temp: %f\n", __func__, sensor->address, sensor->p, sensor->T);

	return (0);
}
