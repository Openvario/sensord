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

#include "ads1110.h"
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

int ads1110_open(t_ads1110 *sensor, unsigned char i2c_address)
{
	// local variables
	int fd;
	unsigned char buf[10]={0x00};

	// try to open I2C Bus
	fd = open("/dev/i2c-1", O_RDWR);

	if (fd < 0) {
		fprintf(stderr, "Error opening file: %s\n", strerror(errno));
		return 1;
	}

	if (ioctl(fd, I2C_SLAVE, i2c_address) < 0) {

		fprintf(stderr, "ioctl error: %s\n", strerror(errno));
		sensor->present = 0;
		return 1;
	}

	// Try to read from sensor to check if it present
	if (read(fd, buf, 3) != 3)
	{
		sensor->present = 0;
		return (1);
	}

	if (g_debug > 0) printf("Opened ADS1110 on 0x%x\n", i2c_address);

	// assign file handle to sensor object
	sensor->fd = fd;
	sensor->address = i2c_address;
	sensor->present = 1;
	return (0);
}

int ads1110_init(t_ads1110 *sensor)
{
	// set calibration data

	ddebug_print("%s @ 0x%x: voltage_scale = %f voltage_offset = %f\n", __func__, sensor->address, 1.0/sensor->scale,sensor->offset);
	return(0);

}

int ads1110_measure(t_ads1110 *sensor)
{
	//variables
	//struct timespec sample_time;
	unsigned char buf[10]={0x00};
  //int digoutp;


	if (read(sensor->fd, buf, 3) != 3) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		return(1);
	}

	sensor->voltage_raw = (buf[0] << 8) + buf[1];

	ddebug_print("%s @ 0x%x: voltage_raw=%d\n", __func__, sensor->address, sensor->voltage_raw);


	return(0);
}


int ads1110_calculate(t_ads1110 *sensor)
{

	sensor->voltage_converted = (sensor->voltage_raw*sensor->scale+sensor->offset);

	debug_print("%s @ 0x%x: Voltage: %fV\n", __func__, sensor->address, sensor->voltage_converted);

	return (0);
}
