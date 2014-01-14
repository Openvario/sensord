#include "ams5915.h"
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

extern int g_debug;
extern FILE *fp_console;

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

int ams5915_init(t_ams5915 *sensor)
{
	// set calibration data
	sensor->digoutpmin = 1638;    // from datasheet
	sensor->digoutpmax = 14745;   // from datasheet
	sensor->pmin = 0;             // from datasheet
	sensor->pmax = 50;            // from datasheet
	
	sensor->sensp = (sensor->digoutpmax - sensor->digoutpmin)/(sensor->pmax - sensor->pmin); 
	ddebug_print("AMS5915: sensp=%f\n", sensor->sensp);
	return(0);
	
}

int ams5915_measure(t_ams5915 *sensor)
{
	//variables
	//struct timespec sample_time;
	unsigned char buf[10]={0x00};
  //int digoutp;
	//unsigned int digoutT;
	
	//sample_time.tv_sec = 0;
	//sample_time.tv_nsec = MS5611_CONVERSIONTIME;
	
	if (read(sensor->fd, buf, 4) != 4) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		return(1);
	}
	
	ddebug_print("read from AMS5915: 0x%x 0x%x 0x%x 0x%x \n", buf[0], buf[1], buf[2], buf[3]);

	sensor->digoutp = ((buf[0] & (0x3F)) << 8) + buf[1];
	sensor->digoutT = (((buf[2] << 8) + (buf[3] & 0xE0)) >> 5);
	ddebug_print("AMS5915: digoutp=0x%x %d\n", sensor->digoutp, sensor->digoutp);
	ddebug_print("AMS5915: digoutT=0x%x %d\n", sensor->digoutT, sensor->digoutT);
	
	//save time of sample
	clock_gettime( CLOCK_REALTIME, &(sensor->sample));
	
	return(0);
}


int ams5915_calculate(t_ams5915 *sensor)
{
	
	ddebug_print("calc...\n");
			
	// calculate differential pressure
	sensor->p = (float) (((sensor->digoutp - sensor->digoutpmin)/sensor->sensp) + sensor->pmin);
	
	// calculate temperature
	sensor->T = ((sensor->digoutT * 200)/2048)-50;
	
	debug_print("AMS5915 @ 0x%x: Pressure: %f Temp: %f\n", sensor->address, sensor->p, sensor->T);

	return (0);
}