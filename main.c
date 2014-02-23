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
//#include "version.h"
#include "nmea.h"
//#include "w1.h"
#include "def.h"
#include "KalmanFilter1d.h"

#include "cmdline_parser.h"
#include "ms5611.h"
#include "ams5915.h"

#define I2C_ADDR 0x76
 
#define MEASTIMER (SIGRTMAX)

timer_t  measTimer;
int g_debug=0;
int g_log=0;

int g_foreground=FALSE;

FILE *fp_console=NULL;
FILE *fp_replay=NULL;
FILE *fp_config=NULL;

enum e_state { sample_d1, get_d1, sample_d2, get_d2} state;
//typedef enum { measure_only, record, replay} t_measurement_mode;
	
void sigintHandler(int sig_num){

	signal(SIGINT, sigintHandler);
	
	// if meas_mode = record -> close fp now
	if (fp_replay != NULL)
		fclose(fp_replay);
	
	//close fp_config if used
	if (fp_config != NULL)
		fclose(fp_config);
		
	printf("Exiting ...\n");
	fclose(fp_console);
	
	exit(0);
}
 

	
int main (int argc, char **argv) {
	
	// local variables
	
	long meas_tick=0;
	int result;
		
	pid_t pid;
	pid_t sid;

	t_measurement_mode meas_mode=measure_only;
	
	struct sigaction sigact;

	// Sensor objects
	t_ms5611 static_sensor;
	t_ms5611 tek_sensor;
	t_ams5915 dynamic_sensor;
	
	// Filter objects
	t_kalmanfilter1d kf;
	
	float tep;
	float dt;
	
	// socket communication
	char s[256];
	int sock;
	struct sockaddr_in server;
	
	
	// initialize variables
	static_sensor.offset = 0.0;
	static_sensor.linearity = 1.0;
	
	dynamic_sensor.offset = 0.0;
	dynamic_sensor.linearity = 1.0;
	
	tek_sensor.offset = 0.0;
	tek_sensor.linearity = 1.0;
	
	//parse command line arguments
	cmdline_parser(argc, argv, &meas_mode);
	
	// get config file options
	if (fp_config != NULL)
		cfgfile_parser(fp_config, &static_sensor, &tek_sensor, &dynamic_sensor);
	
	// check if we are a daemon or stay in foreground
	if (g_foreground == TRUE)
	{
		// stay in foreground
		// install signal handler for CTRL-C
		sigact.sa_handler = sigintHandler;
		sigemptyset (&sigact.sa_mask);
		sigact.sa_flags = 0;
		sigaction(SIGINT, &sigact, NULL);
		
		// open console again, but as file_pointer
		fp_console = stdout;
		stderr = stdout;
		
		// close the standard file descriptors
		close(STDIN_FILENO);
		//close(STDOUT_FILENO);
		close(STDERR_FILENO);
		
		
	}
	else
	{
		// implement handler for kill command
		printf("Daemonizing ...\n");
		pid = fork();
		
		// something went wrong when forking
		if (pid < 0) 
		{
			exit(EXIT_FAILURE);
		}
		
		// we are the parent
		if (pid > 0)
		{
			exit(EXIT_SUCCESS);
		}
		
		// set umask to zero
		umask(0);
				
		/* Try to create our own process group */
		sid = setsid();
		if (sid < 0) {
		syslog(LOG_ERR, "Could not create process group\n");
		exit(EXIT_FAILURE);
		}
		
		// close the standard file descriptors
		close(STDIN_FILENO);
		close(STDOUT_FILENO);
		close(STDERR_FILENO);
		
		//open file for log output
		fp_console = fopen("sensord.log","w+");
		stderr = fp_console;
		
		// install SIGTERM handler
		sigact.sa_handler = sigintHandler;
		sigemptyset (&sigact.sa_mask);
		sigact.sa_flags = 0;
		sigaction(SIGTERM, &sigact, NULL);
	}
	
	// initialite kalmanfilter
	KalmanFilter1d_reset(&kf);
	kf.var_x_accel_ = 0.3;
		
	if ((meas_mode == measure_only) || (meas_mode == record))
	{
	// we need hardware sensors for running !!
		// open sensor for static pressure
		/// @todo remove hardcoded i2c address static pressure
		if (ms5611_open(&static_sensor, 0x76) != 0)
		{
			fprintf(stderr, "Open sensor failed !!\n");
			return 1;
		}
		
		//initialize static pressure sensor
		ms5611_init(&static_sensor);
				
		// open sensor for velocity pressure
		/// @todo remove hardcoded i2c address for velocity pressure
		if (ms5611_open(&tek_sensor, 0x77) != 0)
		{
			fprintf(stderr, "Open sensor failed !!\n");
			return 1;
		}
		
		//initialize velocity pressure sensor
		ms5611_init(&tek_sensor);
				
		// open sensor for differential pressure
		/// @todo remove hardcoded i2c address for differential pressure
		if (ams5915_open(&dynamic_sensor, 0x28) != 0)
		{
			fprintf(stderr, "Open sensor failed !!\n");
			return 1;
		}
		
		//initialize differential pressure sensor
		ams5915_init(&dynamic_sensor);
		
	}
		
	// Open Socket for TCP/IP communication
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == -1)
    fprintf(stderr, "could not create socket\n");
  
	server.sin_addr.s_addr = inet_addr("127.0.0.1");
	server.sin_family = AF_INET;
	server.sin_port = htons(4353);
	
	// try to connect to XCSoar
	while (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
		fprintf(stderr, "failed to connect, trying again\n");
		fflush(stdout);
		sleep(1);
	}
	
	// main data acquisition loop
	while (1)
	{
		// get data from sensors
		if ((meas_mode == measure_only) || (meas_mode == record))
		{
			//get data from real sensors
			ms5611_measure(&static_sensor);
			ms5611_calculate(&static_sensor);
			
			ms5611_measure(&tek_sensor);
			ms5611_calculate(&tek_sensor);
			
			ams5915_measure(&dynamic_sensor);
			ams5915_calculate(&dynamic_sensor);
			
			if (meas_mode == record)
			{
				//save values to record file
				fprintf(fp_replay, "%ld,%f,%f,%f\n", meas_tick, tek_sensor.p, static_sensor.p, dynamic_sensor.p);
				meas_tick++;
			}
		}
		
		// get data from file instead of sensors
		else
		{
			// read file until it ends, then EXIT
			if (fscanf(fp_replay, "%ld,%f,%f,%f", &meas_tick, &tek_sensor.p, &static_sensor.p, &dynamic_sensor.p) == EOF)
			{
				printf("End of File reached\n");
				printf("Exiting ...\n");
				exit(EXIT_SUCCESS);
			}
		}
		
		
		//dt = (static_sensor.sample.tv_sec + 1.0e-9*static_sensor.sample.tv_nsec)-\
					(static_sensor.last_sample.tv_sec + 1.0e-9*static_sensor.last_sample.tv_nsec);
		
		//tep = static_sensor.p/100.0;
		
		static_sensor.last_sample = static_sensor.sample;
		
		//KalmanFiler1d_update(&kf, tep, 0.7, dt);
		
#ifdef NMEA_PAFG 
		// Compose PAFG NMEA sentences
		result = Compose_PAFGB_sentence(&s[0], static_sensor.p, dynamic_sensor.p, tek_sensor.p);
#endif

#ifdef NMEA_POV
		// Compose POV NMEA sentences
		result = Compose_Pressure_POV_sentence(&s[0], static_sensor.p, dynamic_sensor.p, tek_sensor.p);
#endif

		// NMEA sentence valid ?? Otherwise print some error !!
		if (result != 1)
		{
			printf("NMEA Result = %d\n",result);
		}
		
		// Send NMEA string via socket to XCSoar
		if (send(sock, s, strlen(s), 0) < 0)
			fprintf(stderr, "send failed\n");
			
		// Sleep until next measurement
		usleep(440000);
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
	
 
 

 

