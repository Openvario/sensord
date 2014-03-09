#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <syslog.h>
#include "version.h"
#include "nmea.h"
//#include "w1.h"
#include "def.h"
#include "KalmanFilter1d.h"

#include "ms5611.h"
#include "ams5915.h"
 
#define MEASTIMER (SIGRTMAX)

timer_t  measTimer;
int g_debug=0;
int g_log=0;
int adr=0xff;
int mode=0;

FILE *fp_console=NULL;
FILE *fp_replay=NULL;

char replay_filename[50]=" ";

const unsigned long sample_mode[5][4]={{0,9040000, 0x48,4096},
									   {1,4540000, 0x46,2048},
									   {2,2280000, 0x44,1024},
									   {3,1170000, 0x42,512},
									   {4,0540000, 0x40,256}};
	
void sigintHandler(int sig_num){

	signal(SIGINT, sigintHandler);
	
	fclose(fp_replay);
	
	printf("Exiting ...\n");
	exit(0);
}
 
void cmdline_parser(int argc, char **argv){

	// locale variables
	int c;
	
	
	const char* Usage = "\n"\
    "  -v              print version information\n"\
    "  -a [adr]        address of sensor \n"\
	"  -r [filename]   record measurement values to file\n"\
	"\n";
	
	// check commandline arguments
	while ((c = getopt (argc, argv, "va:r:m:")) != -1)
	{
		switch (c) {
			case 'v':
				//sprintf(s, "sensord V0.1 %s %s", __DATE__
				printf("sensord_fastsample V%c.%c RELEASE %c build: %s %s\n", '0', '1', '0',  __DATE__, __TIME__);
				break;

			case 'r':
				// record sensordata for replay
				if (optarg == NULL)
				{
					printf("Missing option for -r\n");
					printf("Exiting ...\n");
					exit(EXIT_FAILURE);
				}
				
				strcpy(replay_filename, optarg);
				
				break;
				
			case 'a':
				// address of sensor to measure
				if (optarg == NULL)
				{
					printf("Missing option for -a\n");
					printf("Exiting ...\n");
					exit(EXIT_FAILURE);
				}
				adr=strtol(optarg, NULL, 16);
				printf("Address used: 0x%x\n", adr);
				break;
			
			case 'm':
				// measurement mode
				if (optarg == NULL)
				{
					printf("Missing option for -m\n");
					printf("Exiting ...\n");
					exit(EXIT_FAILURE);
				}
				mode=strtol(optarg, NULL, 10);
				break;			
			case '?':
				printf("Unknow option %c\n", optopt);
				printf("Usage: sensord [OPTION]\n%s",Usage);
				printf("Exiting ...\n");
				exit(EXIT_FAILURE);
				break;
		}
	}
}
	
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
	struct timespec sample_time;

	
	// get calibration data from sensor
	printf("Get calibration data ...\n");
	
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
	printf("Calibration values:\n");
	for(i=0; i<=5; i++)
	{
		printf("C%d = %d\n", i+1, sensor->C[i]);
	}
	
	// start conversion for D2
	buf[0] = 0x58;													// This is the register we want to read from
	if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		return(1);
	}
	
	// wait for sensor to complete conversion
	sample_time.tv_nsec = sample_mode[mode][1];

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
	printf("D2 = %lu\n", sensor->D2);
	
	//dT = D2 - C5 * 2**8
	//TEMP = 2000 + dT * C6 / 2**32
			
	sensor->dT = sensor->D2 - sensor->C[4] * pow(2,8);
	sensor->temp = 2000 + sensor->dT * sensor->C[5] / pow(2,23);
	
	return(0);
}

int ms5611_measure(t_ms5611 *sensor)
{
	//variables
	struct timespec sample_time;
	unsigned char buf[10]={0x00};
	
	sample_time.tv_sec = 0;
	sample_time.tv_nsec = sample_mode[mode][1];
	
	//printf("Start sampling ...\n");
	// start conversion for D1
	buf[0] = sample_mode[mode][2];													// This is the register we want to read from
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

	//save time of sample
	sensor->last_sample = sensor->sample;
	clock_gettime( CLOCK_REALTIME, &(sensor->sample));
	
	return(0);
}


int ms5611_calculate(t_ms5611 *sensor)
{
	ddebug_print("calc...\n");
			

			
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
	
int main (int argc, char **argv) {
	
	// local variables
	
	long meas_tick=0;

	struct sigaction sigact;

	// Sensor objects
	t_ms5611 sensor;
	
	float dt;
	
	//parse command line arguments
	cmdline_parser(argc, argv);
	
	// install signal handler for CTRL-C
	sigact.sa_handler = sigintHandler;
	sigemptyset (&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	
	// check if filename is valid	
	if (replay_filename[0] == ' ')
	{
		printf("You must enter -r <filename> !!!\n");
		printf("Exiting ...\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		printf("!! RECORD DATA TO %s !!\n", replay_filename);
		// Open the fp to record file
		fp_replay = fopen(replay_filename,"w+");
	}
	
	// check if address is valid
	if (adr == 0xFF)
	{
		printf("You must enter -a <adr> !!!\n");
		printf("Exiting ...\n");
		exit(EXIT_FAILURE);
	}
	
	// print mode we are running
	printf("Measure mode %lu OSR: %lu\n", sample_mode[mode][0], sample_mode[mode][3]);
	// open sensor for static pressure
	/// @todo remove hardcoded i2c address static pressure
	if (ms5611_open(&sensor, adr) != 0)
	{
		fprintf(stderr, "Open sensor failed !!\n");
		return 1;
	}
		
	//initialize velocity pressure sensor
	ms5611_init(&sensor);
	
	// main data acquisition loop
	while (1)
	{
		
		//get data from real sensors
		ms5611_measure(&sensor);
		ms5611_calculate(&sensor);
		
		// calculate dT of measurements
		dt = (sensor.sample.tv_sec + 1.0e-9*sensor.sample.tv_nsec)-\
					(sensor.last_sample.tv_sec + 1.0e-9*sensor.last_sample.tv_nsec);
					
		//save values to record file
		
	
		fprintf(fp_replay, "%f;%ld\n", dt, sensor.p);
		meas_tick++;
		
		// sleep until next measurement
		//usleep(420000);
	}
	return 0;
}
	
	// Buffer
	// maybe we need this code sometimes ...
	
	/*
  sigemptyset(&sigact.sa_mask);
  sigact.sa_flags = SA_SIGINFO;

	//register the Signal Handler
  sigact.sa_sigaction = SignalHandler;  
	
	// Set up sigaction to catch signal first timer
  if (sigaction(MEASTIMER, &sigact, NULL) == -1)
  {
		perror("sigaction failed");
		return -1;
	}
	
	measTimer = SetTimer(MEASTIMER, 200, 1);
	
	// Create and set the timer when to expire
	while(1)
	{
		//measTimer = SetTimer(MEASTIMER, 500, 1);
		//while(1);
		pause();
		if (state == sample_d1)
		{
			printf("calc...\n");
			
			
			//dT = D2 - C5 * 2**8
			//TEMP = 2000 + dT * C6 / 2**32
			
			static_sensor.dT = static_sensor.D2 - static_sensor.C[4] * pow(2,8);
			static_sensor.temp = 2000 + static_sensor.dT * static_sensor.C[5] / pow(2,23);
			
			printf("dT = %ld\n", static_sensor.dT);
			printf("temp = %ld\n", static_sensor.temp);
			
			// these calculations are copied from the data sheet
			
			//OFF = C2 * 2**16 + (C4 * dT) / 2**7
			//SENS = C1 * 2**15 + (C3 * dT) / 2**8
			//P = (D1 * SENS / 2**21 - OFF) / 2**15
			
			static_sensor.off = static_sensor.C[1] * pow(2,16) + (static_sensor.C[3] * static_sensor.dT) / pow(2,7);
			static_sensor.sens = static_sensor.C[0] * pow(2,15) + (static_sensor.C[2] * static_sensor.dT) / pow(2,8);
			static_sensor.p = (static_sensor.D1 * static_sensor.sens / pow(2,21) - static_sensor.off) / pow(2,15);
			
			printf("off: %lld\n", static_sensor.off);
			printf("sens: %lld\n", static_sensor.sens);
			
			printf("Pressure: %ld\n", static_sensor.p);
			
		}
	}*/
	
 
 

 

