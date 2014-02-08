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
	unsigned short prom[8];
	
	// Print debug info
	ddebug_print("Sensor compensation data: Offset: %f, Linearity: %f\n", sensor->offset, sensor->linearity);
	
	// get calibration data from sensor
	ddebug_print("Get calibration data ...\n");
	
	for(a = 0xA0, i = 0; a <= 0xAC; a = a +0x02, i++)
	{
		// get calibration values
		buf[0] = a;													// This is the register we want to read from
		if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
			printf("Error writing to i2c slave\n");
			return(1);
		}
		if (read(sensor->fd, buf, 2) != 2) {								// Read back data into buf[]
			printf("Unable to read from slave\n");
			return(1);
		}
		prom[i] = (buf[0] * 256) + buf[1];
		ddebug_print("Adr = 0x%x %u\n", a, prom[i]);
	}
	
	
	sensor->C1s = prom[1] << 15;
	sensor->C2s = prom[2] << 16;
	sensor->C3  = prom[3];
	sensor->C4  = prom[4];
	sensor->C5s = prom[5] << 8;
	sensor->C6  = prom[6];
	
	// print calibration values if debug is enabled
	ddebug_print("Calibration values:\n");
	
	ddebug_print("C1s = %lu\n", sensor->C1s);
	ddebug_print("C2s = %lu\n", sensor->C2s);
	ddebug_print("C3  = %lu\n", sensor->C3);
	ddebug_print("C4  = %lu\n", sensor->C4);
	ddebug_print("C5s = %lu\n", sensor->C5s);
	ddebug_print("C6  = %lu\n", sensor->C6);
	
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
		printf("Error writing to i2c slave: start conv: adr %x\n",sensor->address);
		return(1);
	}
	
	// wait for sensor to complete conversion
	nanosleep(&sample_time, NULL);

	// read result
	buf[0] = 0x00;
	if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave: write Read result\n");
		return(1);
	}
	
	if (read(sensor->fd, buf, 3) != 3) {								// Read back data into buf[]
		printf("Unable to read from slave: read result\n");
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
	sensor->last_sample = sensor->sample;
	clock_gettime( CLOCK_REALTIME, &(sensor->sample));
	
	return(0);
}


int ms5611_calculate(t_ms5611 *sensor)
{
	long d2;
	long OFF2=0;
	long SENS2=0;
	
	ddebug_print("calc...\n");
				
	sensor->dT = -1000;
	ddebug_print("dT = %lld\n", sensor->dT);
	//dT = D2 - C5 * 2**8
	//TEMP = 2000 + dT * C6 / 2**23
			
	sensor->dT = sensor->D2 - sensor->C5s;
	sensor->temp = 2000 + ((sensor->dT * sensor->C6) / 8388608);
			
	ddebug_print("dT = %lld\n", sensor->dT);
	ddebug_print("temp = %ld\n", sensor->temp);
	
	// second order pressure comp
	/*if(sensor->temp < 2000)
	{
		d2 = (sensor->temp - 2000);
		d2 = d2 * d2;
		
		OFF2 = (5 * d2) / 2;
		SENS2 = (5 * d2) / 4;
		
		if(sensor->temp < -1500)
		{
			d2 = sensor->temp + 1500;
			d2 = d2 * d2;
			OFF2 += 7 * d2;
			SENS2 += (11 * d2) / 2;
		}
	}*/
	
	// these calculations are copied from the data sheet
	//OFF = C2 * 2**16 + (C4 * dT) / 2**7
	//SENS = C1 * 2**15 + (C3 * dT) / 2**8
	//P = (D1 * SENS / 2**21 - OFF) / 2**15
			
	sensor->off = (sensor->C2s + (sensor->C4 * sensor->dT) / 128) - OFF2;
	sensor->sens = (sensor->C1s + (sensor->C3 * sensor->dT) / 256) - SENS2;
	sensor->p_meas = (sensor->D1 * sensor->sens / 2097152 - sensor->off) / 32768;
			
	// correct measured pressure
	sensor->p = sensor->linearity * (float)sensor->p_meas + sensor->offset;
	
	ddebug_print("off: %lld\n", sensor->off);
	ddebug_print("sens: %lld\n", sensor->sens);
			
	debug_print("MS5611 @ 0x%x: Pressure: %f\n", sensor->address, sensor->p);

	return (0);

}
