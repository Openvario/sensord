/*  sensord - Sensor Interface for XCSoar Glide Computer - http://www.openvario.org/
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
#include "configfile_parser.h"
#include "vario.h"
#include "AirDensity.h"

#define I2C_ADDR 0x76
#define PRESSURE_SAMPLE_RATE 	20	// sample rate of pressure values (Hz)
#define TEMP_SAMPLE_RATE 		5	// sample rate of temp values (Hz)
#define NMEA_SLOW_SEND_RATE		2	// NMEA send rate for SLOW Data (pressures, etc..) (Hz)
 
#define MEASTIMER (SIGRTMAX)
#define DELTA_TIME_US(T1, T2)	(((T1.tv_sec+1.0e-9*T1.tv_nsec)-(T2.tv_sec+1.0e-9*T2.tv_nsec))*1000000)			
					
timer_t  measTimer;
int g_debug=0;
int g_log=0;

int g_foreground=FALSE;

FILE *fp_console=NULL;
FILE *fp_replay=NULL;
FILE *fp_config=NULL;

enum e_state { IDLE, TEMP, PRESSURE} state = IDLE;

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
	int i=0;
	
	// time stamps for statemachine
	struct timespec t;
	struct timespec last_temp;
	struct timespec last_send;
	struct timespec last_pressure;
	struct timespec start_timer;
	
	// for daemonizing
	pid_t pid;
	pid_t sid;

	t_measurement_mode meas_mode=measure_only;
	
	struct sigaction sigact;

	// Sensor objects
	t_ms5611 static_sensor;
	t_ms5611 tep_sensor;
	t_ams5915 dynamic_sensor;
	
	// Filter objects
	t_kalmanfilter1d vkf;
	
	// pressures
	float tep;
	float p_static;
	float p_dynamic;
	
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
	
	tep_sensor.offset = 0.0;
	tep_sensor.linearity = 1.0;
	
	//parse command line arguments
	cmdline_parser(argc, argv, &meas_mode);
	
	// get config file options
	if (fp_config != NULL)
		cfgfile_parser(fp_config, &static_sensor, &tep_sensor, &dynamic_sensor);
	
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
		if (ms5611_open(&tep_sensor, 0x77) != 0)
		{
			fprintf(stderr, "Open sensor failed !!\n");
			return 1;
		}
		
		//initialize tep pressure sensor
		ms5611_init(&tep_sensor);
				
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
	
	// poll sensors for offset compensation
	ms5611_start_temp(&static_sensor);
	usleep(10000);
	ms5611_read_temp(&static_sensor);
	ms5611_start_pressure(&static_sensor);
	usleep(10000);
	ms5611_read_pressure(&static_sensor);
	
	// initialize variables
	p_static = static_sensor.p;
	
	// initialize kalman filter
	KalmanFilter1d_reset(&vkf);
	vkf.var_x_accel_ = 0.3;
	
	for(i=0; i < 1000; i++)
		KalmanFiler1d_update(&vkf, p_static, 0.25, 1);
		
	
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
		// get actual system time
		clock_gettime(CLOCK_REALTIME, &t);
		
		/*
		 * handle pressure sensors
		 */
		switch (state)
		{
		case IDLE:
			// is it time to start temperature measurement??
			if((dt=DELTA_TIME_US(t,last_temp)) > 1000000/TEMP_SAMPLE_RATE)
			{
				// start temperature measurement
				ms5611_start_temp(&static_sensor);
				ms5611_start_temp(&tep_sensor);
				
				state = TEMP;			// set state for temperature
				start_timer = t;		// remember timestamp where measurement was started
				
				//printf("Sample temp: %f\n",dt);
			}
			// is it time to start pressure measurement ??
			else if ((dt=DELTA_TIME_US(t,last_pressure)) > 1000000/PRESSURE_SAMPLE_RATE)
			{
				// start pressure measurement
				ms5611_start_pressure(&static_sensor);
				ms5611_start_pressure(&tep_sensor);
				
				state = PRESSURE;			//set state for pressure
				start_timer = t;		// set timer to measurement delay
				
				//printf("Sample pressure: %f\n",dt);
			}
			break;
		case TEMP:
			// check if measurement is done ??
			if ((dt=DELTA_TIME_US(t,start_timer)) > 10000)
			{
				// measurement done
				// get temperature data
				ms5611_read_temp(&static_sensor);
				ms5611_read_temp(&tep_sensor);
				
				// save timestamp of measurement
				last_temp = t;
				
				// reset state
				state = IDLE;
				
				//printf("dt: %f\n", dt);
			}
			break;
		case PRESSURE:
			// check if measurement is done ??
			if ((dt=DELTA_TIME_US(t,start_timer)) > 10000)
			{
				// measurement done
				// get pressure date
				ms5611_read_pressure(&static_sensor);
				ms5611_read_pressure(&tep_sensor);
								
				// read AMS5915
				ams5915_measure(&dynamic_sensor);
				ams5915_calculate(&dynamic_sensor);
				
				/*
				 * filtering
				 */
				 // of static pressure
				p_static = (3*p_static + static_sensor.p) / 4;
				 // of tep pressure
				KalmanFiler1d_update(&vkf, tep_sensor.p, 0.25, (DELTA_TIME_US(t,last_pressure)/1000000));
				//printf("dt: %f\n", DELTA_TIME_US(t,last_pressure)/1000000);
				 // of dynamic pressure
				p_dynamic = (3*p_dynamic + dynamic_sensor.p) / 4;
				
				// write pressure to file if option is set
				if (meas_mode == record)
				{
					fprintf(fp_replay, "%f,%f,%f\n", vkf.x_abs_, p_static, p_dynamic);
				}
				
				// save timestamp of measurement
				last_pressure = t;
				
				// reset state
				state = IDLE;
			}
			break;
		}
		
		// send task to idle
		if (state != IDLE)
			usleep(1000);
		else
			usleep(1000);
			
		/*f
		 * send NMEA sentences
		 */
		 // is it time to send data to XCSoar ??
		if((dt=DELTA_TIME_US(t,last_send)) > 1000000/NMEA_SLOW_SEND_RATE)
		{
			// some local variables
			float vario;
			float ias;
			float tas;
			int altitude;
			
			// Compute Vario
			vario = ComputeVario(vkf.x_abs_, vkf.x_vel_);
			
			// Compute Altitude
			altitude = (int)(44330.8 - 4946.54 *pow((p_static*100), 0.1902632));
			
			// Compute IAS
			ias = sqrt(21.15918367 * (dynamic_sensor.p*100));	// km/h check formula
			
			// Compute TAS
			tas = ias * AirDensityRatio(altitude);
			
			//printf("x_abs: %f x_vel: %f speed: %f Vario: %f alt: %d ias: %f tas: %f\n",vkf.x_abs_,vkf.x_vel_,p_dynamic, vario, altitude, ias, tas);
			//printf("NMEA dt: %f\n", dt);
			
#ifdef NMEA_PAFG 
			// Compose PAFG NMEA sentences
			result = Compose_PAFGB_sentence(&s[0], p_static, dynamic_sensor.p, tep_sensor.p);
			
			// NMEA sentence valid ?? Otherwise print some error !!
			if (result != 1)
			{
				printf("NMEA Result = %d\n",result);
			}	
		
			// Send NMEA string via socket to XCSoar
			if (send(sock, s, strlen(s), 0) < 0)
				fprintf(stderr, "send failed\n");
#endif

#ifdef NMEA_POV
			// Compose POV slow NMEA sentences
			result = Compose_Pressure_POV_slow(&s[0], p_static/100, p_dynamic*100);

			// NMEA sentence valid ?? Otherwise print some error !!
			if (result != 1)
			{
				printf("POV slow NMEA Result = %d\n",result);
			}	
		
			// Send NMEA string via socket to XCSoar
			if (send(sock, s, strlen(s), 0) < 0)
				fprintf(stderr, "send failed\n");
			
			
			// Compose POV slow NMEA sentences
			result = Compose_Pressure_POV_fast(&s[0], vario);
			
			// NMEA sentence valid ?? Otherwise print some error !!
			if (result != 1)
			{
				printf("POV fast NMEA Result = %d\n",result);
			}	
		
			// Send NMEA string via socket to XCSoar
			if (send(sock, s, strlen(s), 0) < 0)
				fprintf(stderr, "send failed\n");
#endif

			// save timestamp of last send
			clock_gettime(CLOCK_REALTIME, &last_send);
		}
	}
	return 0;
}	
		
		/*
		// get data from sensors
		if ((meas_mode == measure_only) || (meas_mode == record))
		{
						
			if (meas_mode == record)
			{
				//save values to record file
				fprintf(fp_replay, "%ld,%f,%f,%f\n", meas_tick, tep_sensor.p, static_sensor.p, dynamic_sensor.p);
				meas_tick++;
			}
		}
		
		// get data from file instead of sensors
		else
		{
			// read file until it ends, then EXIT
			if (fscanf(fp_replay, "%ld,%f,%f,%f", &meas_tick, &tep_sensor.p, &static_sensor.p, &dynamic_sensor.p) == EOF)
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
		

			
		// Sleep until next measurement
		//usleep(440000);
		
		//clock_gettime(CLOCK_REALTIME, &last_temp);
		
		printf("TIME: %f\n", DELTA_TIME_MS(last_temp,t)); */

	
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
	
 
 

 

