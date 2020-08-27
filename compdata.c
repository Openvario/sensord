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
#include <stdbool.h>
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
#include <sys/time.h>
#include <arpa/inet.h>
#include <syslog.h>
//#include "version.h"
//#include "w1.h"
#include "def.h"
#include "ms5611.h"

#define I2C_ADDR 0x76
#define PRESSURE_SAMPLE_RATE 	20	// sample rate of pressure values (Hz)
#define TEMP_SAMPLE_RATE 		5	// sample rate of temp values (Hz)
#define NMEA_SLOW_SEND_RATE		2	// NMEA send rate for SLOW Data (pressures, etc..) (Hz)
 
#define MEASTIMER (SIGRTMAX)
#define DELTA_TIME_US(T1, T2)	(((T1.tv_sec+1.0e-9*T1.tv_nsec)-(T2.tv_sec+1.0e-9*T2.tv_nsec))*1000000)			
#define DELTA_TIME(T1,T2)       (((T1.tv_sec+1.0e-9*T1.tv_nsec)-(T2.tv_sec+1.0e-9*T2.tv_nsec)))

timer_t  measTimer;
int g_debug=0;
int g_log=0;
int sidx=0;
int tidx=0;
int statval[200000][8];
int tepval[200000][8];

// Sensor objects
t_ms5611 static_sensor;
t_ms5611 tep_sensor;
	
// pressures
float tep;
float p_static;

int g_foreground=FALSE;
int g_secordcomp=FALSE;

FILE *fp_console=NULL;
FILE *fp_sensordata=NULL;
FILE *fp_datalog=NULL;
FILE *fp_config=NULL;
FILE *fp_stat=NULL;
FILE *fp_tep=NULL;

struct timespec sensor_cur;
struct timespec sensor_prev;

//FILE *fp_rawlog=NULL;

enum e_state { IDLE, TEMP, PRESSURE} state = IDLE;

float sensor_wait (float time)
{
	struct timespec curtime;
	float deltaTime;

	clock_gettime(CLOCK_REALTIME,&curtime);
	deltaTime=DELTA_TIME_US(curtime,sensor_prev);
	if (time-deltaTime>2000) usleep(time-deltaTime);
	while (deltaTime<time) 
	{
		usleep(50);
		clock_gettime(CLOCK_REALTIME,&curtime);
		deltaTime=DELTA_TIME_US(curtime,sensor_prev);
	} 
	return (deltaTime-time);
}

//typedef enum { measure_only, record, replay} t_measurement_mode;

/**
* @brief Signal handler if sensord will be interrupted
* @param sig_num
* @return 
* 
* Signal handler for catching STRG-C singal from command line
* Closes all open files handles like log files
* @date 17.04.2014 born
*
*/ 
void sigintHandler(int sig_num){

	signal(SIGINT, sigintHandler);
	
	// if meas_mode = record -> close fp now
	if (fp_datalog != NULL)
		fclose(fp_datalog);
	
	// if sensordata from file
	if (fp_sensordata != NULL)
		fclose(fp_sensordata);
		
	//close fp_config if used
	if (fp_config != NULL)
		fclose(fp_config);
	
	//fclose(fp_rawlog);
	printf("Exiting ...\n");
	fclose(fp_console);
	
	exit(0);
}

/**
* @brief Timming routine for pressure measurement
* @param 
* @return 
* 
* Timing handler to coordinate pressure measurement
* @date 17.04.2014 born
*
*/ 
void pressure_measurement_handler(int record)
{
	static int meas_counter = 1, glitch = 0;
	float deltaTime;

	// if early, wait
	// if more than 2ms late, increase the glitch counter
	if ((deltaTime=sensor_wait(12500))>2000) glitch = 5;
	if (meas_counter&1) {
		// read pressure sensors
		ms5611_read_temp(&tep_sensor,glitch);
		ms5611_read_pressure(&static_sensor);
		ms5611_start_temp(&static_sensor);
		clock_gettime(CLOCK_REALTIME,&sensor_prev);
		ms5611_start_pressure(&tep_sensor);

		// if there was a glitch, compensate for the glitch
		if (glitch>1) 
			if (--glitch==0) glitch=1;
		if (glitch==1) 
			if (tep_sensor.D2>(tep_sensor.D2f-15)) glitch=0;
		if (glitch>0)
		{
			if (record==1) {
				statval[sidx][0]=tep_sensor.D1f;
				statval[sidx][1]=tep_sensor.D1;
				statval[sidx][2]=tep_sensor.D2f;
				statval[sidx][3]=tep_sensor.D2;
				statval[sidx][4]=static_sensor.D1f;
				statval[sidx][5]=static_sensor.D1;
				statval[sidx][6]=static_sensor.D2f;
				statval[sidx++][7]=static_sensor.D2;
				
			}
		} else {
			static_sensor.D1f=(static_sensor.D1f*7+static_sensor.D1)>>3;
		}
	} else {
		// read pressure sensors
		ms5611_read_pressure(&tep_sensor);
		ms5611_read_temp(&static_sensor,glitch);
		ms5611_start_temp(&tep_sensor);
		clock_gettime(CLOCK_REALTIME,&sensor_prev);
		ms5611_start_pressure(&static_sensor);
		// if there was a glitch, compensate for the glitch
		if (glitch>1)
		       if (--glitch==0) glitch=1;
		if (glitch==1)	
			if (static_sensor.D2>(static_sensor.D2f-15)) glitch=0;
		if (glitch>0)	
		{
			if (record==1) {
				tepval[tidx][0]=tep_sensor.D1f;
				tepval[tidx][1]=tep_sensor.D1;
				tepval[tidx][2]=tep_sensor.D2f;
				tepval[tidx][3]=tep_sensor.D2;
				tepval[tidx][4]=static_sensor.D1f;
				tepval[tidx][5]=static_sensor.D1;
				tepval[tidx][6]=static_sensor.D2f;
				tepval[tidx++][7]=static_sensor.D2;
			} 
		} else {
			tep_sensor.D1f=(tep_sensor.D1f*7+tep_sensor.D1)>>3;
		}
	}
	meas_counter++;
}
	
int main (int argc, char **argv) {
	
	// local variables
	int i=0;

	// signals and action handlers
	struct sigaction sigact;
	
	// socket communication
	
	// initialize variables
	static_sensor.offset = 0.0;
	static_sensor.linearity = 1.0;
	
	tep_sensor.offset = 0.0;
	tep_sensor.linearity = 1.0;
	
	//open file for raw output
	//fp_rawlog = fopen("raw.log","w");
		
	// get config file options
	//if (fp_config != NULL)
	//	cfgfile_parser(fp_config, &static_sensor, &tep_sensor, &dynamic_sensor, &voltage_sensor, &config);
	
	// check if we are a daemon or stay in foreground
	// stay in foreground
	// install signal handler for CTRL-C
	sigact.sa_handler = sigintHandler;
	sigemptyset (&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);

	// open console again, but as file_pointer
	fp_console = stdout;
	stderr = stdout;
	setbuf(fp_console, NULL);
	setbuf(stderr, NULL);

	// close the standard file descriptors
	close(STDIN_FILENO);
	//close(STDOUT_FILENO);
	close(STDERR_FILENO);	
	// ignore SIGPIPE
	signal(SIGPIPE, SIG_IGN);
	// we need hardware sensors for running !!
	// open sensor for static pressure
	/// @todo remove hardcoded i2c address static pressure
	if (ms5611_open(&static_sensor, 0x76) != 0)
	{
		fprintf(stderr, "Open static sensor failed !!\n");
		return 1;
	}
		
	//initialize static pressure sensor
	ms5611_reset(&static_sensor);
	usleep(10000);
	ms5611_init(&static_sensor);
	static_sensor.secordcomp = g_secordcomp;
	static_sensor.valid = 1;
				
	// open sensor for velocity pressure
	/// @todo remove hardcoded i2c address for velocity pressure
	if (ms5611_open(&tep_sensor, 0x77) != 0)
	{
		fprintf(stderr, "Open tep sensor failed !!\n");
		return 1;
	}
		
	//initialize tep pressure sensor
	ms5611_reset(&tep_sensor);
	usleep(10000);
	ms5611_init(&tep_sensor);
	tep_sensor.secordcomp = g_secordcomp;
	tep_sensor.valid = 1;
		
	// poll sensors for offset compensation
	tep_sensor.D2f=static_sensor.D2f=0;
	for (i=0;i<120;++i)
	{
		ms5611_start_temp(&static_sensor);
		ms5611_start_temp(&tep_sensor);
		if (i==0) clock_gettime(CLOCK_REALTIME,&sensor_prev);
		sensor_wait(12500);
		clock_gettime(CLOCK_REALTIME,&sensor_prev);
		ms5611_read_temp(&static_sensor,0);
		ms5611_read_temp(&tep_sensor,0);
 	}

	ms5611_start_pressure(&static_sensor);
	ms5611_start_temp(&tep_sensor);
	sensor_wait(12500);
	clock_gettime(CLOCK_REALTIME,&sensor_prev);

	ms5611_read_pressure(&static_sensor);
	ms5611_read_temp(&tep_sensor,0);
	ms5611_start_pressure(&tep_sensor);
	ms5611_start_temp(&static_sensor);
	sensor_wait(12500);
	clock_gettime(CLOCK_REALTIME,&sensor_prev);

	ms5611_read_pressure(&tep_sensor);
	ms5611_read_temp(&static_sensor,0);
	ms5611_calculate_pressure(&tep_sensor);
	ms5611_calculate_pressure(&static_sensor);
	ms5611_start_temp(&tep_sensor);
	ms5611_start_pressure(&static_sensor);

	fp_stat = fopen ("static_data","w+");
	fp_tep  = fopen ("tep_data","w+");

	// main data acquisition loop
	for (i=0;i<1000;++i) 
		pressure_measurement_handler(0);
	for (i=0;i<200000;++i) 
	{
		if (i%200==0)  usleep ((rand()%30)*10e3+50e3);
		if (i%200==101) usleep ((rand()%30)*10e3+50e3);
		pressure_measurement_handler(1);
	}
	for (i=0;i<sidx;++i) 
		fprintf (fp_stat,"%d %d %d %d %d %d %d %d\n",statval[i][0],statval[i][1],statval[i][2],statval[i][3],statval[i][4],statval[i][5],statval[i][6],statval[i][7]);
	for (i=0;i<tidx;++i)
		fprintf (fp_tep, "%d %d %d %d %d %d %d %d\n",tepval[i][0],tepval[i][1],tepval[i][2],tepval[i][3],tepval[i][4],tepval[i][5],tepval[i][6],tepval[i][7]);
	fclose (fp_stat);
	fclose (fp_tep);
	return 0;
}
