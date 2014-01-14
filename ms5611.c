#include "ms5611.h"
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include "def.h"

#define MS5611_CONVERSIONTIME 9000000 

extern int g_debug;
extern FILE *fp_console;

int ms5611_open(t_ms5611 *sensor, unsigned char i2c_address)
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
	
	if (g_debug > 0) printf("Opened MS5611 on 0x%x\n", i2c_address);
	
	// assign file handle to sensor object
	sensor->fd = fd;
	sensor->address = i2c_address;
	return (0);
}

int ms5611_init(t_ms5611 *sensor)
{
	unsigned char buf[10];
	unsigned char a,i;
	
	// get calibration data from sensor
	ddebug_print("Get calibration data ...\n");
	
	for(a = 0xA2, i = 0; a <= 0xAC; a = a +0x02, i++)
	{
		// get calibration values
		buf[0] = a;													// This is the register we want to read from
		if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
			printf("Error writing to i2c slave\n");
			return(1);
		}
		if (read(sensor->fd, buf, 3) != 3) {								// Read back data into buf[]
			printf("Unable to read from slave\n");
			return(1);
		}
		sensor->C[i] = (buf[0] * 256) + buf[1];
	}
	
	// print calibration values if debug is enabled
	ddebug_print("Calibration values:\n");
	for(i=0; i<=5; i++)
	{
		ddebug_print("C%d = %d\n", i+1, sensor->C[i]);
	}
	
	return(0);
}

int ms5611_measure(t_ms5611 *sensor)
{
	//variables
	struct timespec sample_time;
	unsigned char buf[10]={0x00};
	
	sample_time.tv_sec = 0;
	sample_time.tv_nsec = MS5611_CONVERSIONTIME;
	
	//printf("Start sampling ...\n");
	// start conversion for D1
	buf[0] = 0x48;													// This is the register we want to read from
	if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		return(1);
	}
	
	// wait for sensor to complete conversion
	nanosleep(&sample_time, NULL);

	// read result
	buf[0] = 0x00;
	if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		return(1);
	}
	
	if (read(sensor->fd, buf, 3) != 3) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		return(1);
	}
	
	sensor->D1 = buf[0] * 65536 + buf[1] * 256 + buf[2];
	ddebug_print("D1 = %lu\n", sensor->D1);

	// start conversion for D2
	buf[0] = 0x58;													// This is the register we want to read from
	if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		return(1);
	}
	
	// wait for sensor to complete conversion
	sample_time.tv_nsec = MS5611_CONVERSIONTIME;
	nanosleep(&sample_time, NULL);

	// read result
	buf[0] = 0x00;
	if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		return(1);
	}
	
	if (read(sensor->fd, buf, 3) != 3) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		return(1);
	}
	
	sensor->D2 = buf[0] * 65536 + buf[1] * 256 + buf[2];
	ddebug_print("D2 = %lu\n", sensor->D2);
	
	//save time of sample
	clock_gettime( CLOCK_REALTIME, &(sensor->sample));
	
	return(0);
}


int ms5611_calculate(t_ms5611 *sensor)
{
	
	ddebug_print("calc...\n");
			
	//dT = D2 - C5 * 2**8
	//TEMP = 2000 + dT * C6 / 2**32
			
	sensor->dT = sensor->D2 - sensor->C[4] * pow(2,8);
	sensor->temp = 2000 + sensor->dT * sensor->C[5] / pow(2,23);
			
	ddebug_print("dT = %ld\n", sensor->dT);
	ddebug_print("temp = %ld\n", sensor->temp);
			
	// these calculations are copied from the data sheet
			
	//OFF = C2 * 2**16 + (C4 * dT) / 2**7
	//SENS = C1 * 2**15 + (C3 * dT) / 2**8
	//P = (D1 * SENS / 2**21 - OFF) / 2**15
			
	sensor->off = sensor->C[1] * pow(2,16) + (sensor->C[3] * sensor->dT) / pow(2,7);
	sensor->sens = sensor->C[0] * pow(2,15) + (sensor->C[2] * sensor->dT) / pow(2,8);
	sensor->p = (sensor->D1 * sensor->sens / pow(2,21) - sensor->off) / pow(2,15);
			
	ddebug_print("off: %lld\n", sensor->off);
	ddebug_print("sens: %lld\n", sensor->sens);
			
	debug_print("MS5611 @ 0x%x: Pressure: %ld\n", sensor->address, sensor->p);

	return (0);

}