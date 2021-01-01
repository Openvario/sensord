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

#include <termios.h>
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
#include "ams5915.h"
#include "ads1110.h"
#include "cmdline_parser.h"
#include "configfile_parser.h"
#include "vario.h"
#include "AirDensity.h"
#include "24c16.h"

#define I2C_ADDR 0x76
#define PRESSURE_SAMPLE_RATE 	20	// sample rate of pressure values (Hz)
#define TEMP_SAMPLE_RATE 		5	// sample rate of temp values (Hz)
#define NMEA_SLOW_SEND_RATE		2	// NMEA send rate for SLOW Data (pressures, etc..) (Hz)
 
#define MEASTIMER (SIGRTMAX)
#define DELTA_TIME_US(T1, T2)	(((T1.tv_sec+1.0e-9*T1.tv_nsec)-(T2.tv_sec+1.0e-9*T2.tv_nsec))*1000000)			
#define DELTA_TIME(T1,T2)       (((T1.tv_sec+1.0e-9*T1.tv_nsec)-(T2.tv_sec+1.0e-9*T2.tv_nsec)))

// Sensor objects
t_ms5611 static_sensor;
t_ms5611 tep_sensor;
t_ams5915 dynamic_sensor;
t_ads1110 voltage_sensor;

t_config config;
t_io_mode io_mode;
timer_t  measTimer;
int g_debug=0;
int g_log=0;
int sidx=0;
int tidx=0;
double statval[1048576][3];
double tepval[1048576][3];
int glitch=0;
int noglitch=0;
int idx=0;
int urun=0;
int orun=0;

// pressures
float tep;
float p_static;

int g_foreground=FALSE;
int g_secordcomp=FALSE;
int tj=FALSE;

char config_filename[50];
FILE *fp_console=NULL;
FILE *fp_sensordata=NULL;
FILE *fp_config=NULL;
FILE *fp_datalog=NULL;
//FILE *fp1=NULL;
//FILE *fp2=NULL;

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
	static unsigned int meas_counter=1, glitchstart=0,shutdown=0,deltaxmax=0;
	int x=0, deltax;
	double deltaTime, comp;

	// if early, wait
	// if more than 2ms late, increase the glitch counter
	if ((deltaTime=sensor_wait(12500))>2000) { glitchstart=8; glitch+=8; }
	if (meas_counter&1) {
		// read pressure sensors
		x|=ms5611_read_temp(&tep_sensor,glitch);
		x|=ms5611_read_pressure(&static_sensor);
		x|=ms5611_start_temp(&static_sensor);
		clock_gettime(CLOCK_REALTIME,&sensor_prev);
		x|=ms5611_start_pressure(&tep_sensor);

		if (abs((int)static_sensor.D1l-(int) static_sensor.D1)<100e3) {
			if (glitch==0) {
				shutdown=0;
				if (tep_sensor.D2l>tep_sensor.D2+300) { glitchstart=8; glitch=8; }
			}
			if (glitchstart) {
				if (glitch>glitchstart)
					deltax=abs((int)tep_sensor.D2l-(int)tep_sensor.D2);
				else
					deltax=abs((int)tep_sensor.D2f-(int)tep_sensor.D2);
				if (deltax>deltaxmax) deltaxmax=deltax;
				if ((--glitchstart)==0) {
					if (deltaxmax>15)
						glitch += ((int) round(log((double)(deltaxmax)*.0666666666666)*45))+12;
					deltaxmax=0;
				}
			}
			if (glitch>0) {
				if (--glitch>399) glitch=399;
				if (!glitch) ++urun;
				if (++shutdown==800) {
					glitch=shutdown=0;
					++orun;
				}
				if (tep_sensor.D2>(int)round(tep_sensor.D2f-15)) glitch=0;
				if (glitch && record && ((int)round(tep_sensor.D2f)>tep_sensor.D2+500)) {
					comp=static_sensor.p*1e-5;
					comp=comp*comp*static_sensor.Pcomp2+comp*static_sensor.Pcomp1+static_sensor.Pcomp0;
					statval[sidx][0]=((double)static_sensor.D1f-(double)static_sensor.D1)/comp;
					statval[sidx][1]=((double)tep_sensor.D2f-(double)tep_sensor.D2);
					statval[sidx++][2]=x;
				}
			} else {
				static_sensor.D1f=(static_sensor.D1f*7+static_sensor.D1)>>3;
				ms5611_calculate_pressure(&static_sensor);
			}
		}	 
	} else {
		// read pressure sensors
		x|=ms5611_read_pressure(&tep_sensor);
		x|=ms5611_read_temp(&static_sensor,glitch);
		x|=ms5611_start_temp(&tep_sensor);
		clock_gettime(CLOCK_REALTIME,&sensor_prev);
		x|=ms5611_start_pressure(&static_sensor);

		if (abs((int)tep_sensor.D1l-(int) tep_sensor.D1)<100e3) {
			if (glitch==0) {
				shutdown=0;
				if (static_sensor.D2l>static_sensor.D2+300) { glitchstart=8; glitch=8; }
			}
			if (glitchstart) {
				if (glitch>glitchstart)
					deltax=abs((int)static_sensor.D2l-(int)static_sensor.D2);
				else
					deltax=abs((int)static_sensor.D2f-(int)static_sensor.D2);
				if (deltax>deltaxmax) deltaxmax=deltax;
				if ((--glitchstart)==0) {
					if (deltaxmax>15)
						glitch += ((int) round(log((double)(deltaxmax)*.0666666666666)*45))+12;
					deltaxmax=0;
				}
			}
			if (glitch>0) {
				if (--glitch>399) glitch=399;
				if (!glitch) ++urun;
				if (++shutdown==800) {
					glitch=shutdown=0;
					++orun;
				}
				if (static_sensor.D2>(int)round(static_sensor.D2f-15)) glitch=0;
				if (glitch && record && ((int)round(static_sensor.D2f)>static_sensor.D2+500)) {
					comp=tep_sensor.p*1e-5;
					comp=comp*comp*tep_sensor.Pcomp2+comp*tep_sensor.Pcomp1+tep_sensor.Pcomp0;
					tepval[tidx][0]=((double)tep_sensor.D1f-(double)tep_sensor.D1)/comp;
					tepval[tidx][1]=((double)static_sensor.D2f-(double)static_sensor.D2);
					tepval[tidx++][2]=x;

				}
			} else {
				tep_sensor.D1f=(tep_sensor.D1f*7+tep_sensor.D1)>>3;
				ms5611_calculate_pressure(&tep_sensor);
			}
		}
	}
	if (glitch) noglitch=0; else noglitch++;
	meas_counter++;
	if (io_mode.sensordata_to_file == TRUE) {
		if ((meas_counter%2==1) && (record))
			fprintf(fp_datalog, "%.4f %.4f %f %d %d %d %d %d %d %d %d %d\n",tep_sensor.p,static_sensor.p,dynamic_sensor.p,static_sensor.D1,static_sensor.D1f,tep_sensor.D1,tep_sensor.D1f,static_sensor.D2,static_sensor.D2f,tep_sensor.D2,tep_sensor.D2f,glitch);
	}
}

void polyfit (double val[][3], int idx, double pf[], double rmserr[])
{
	int i;
	double  x[4], y[3], tmp, inv[3][3];
	double det,cor,meanval,det2;

	for (i=0;i<3;++i) x[i]=y[i]=0;
	x[3]=0;
	for (i=0;i<idx;++i) {
		x[0]+=val[i][1];
		y[0]+=val[i][0];
		tmp=val[i][1]*val[i][1];
		x[1]+=tmp;
		x[2]+=tmp*val[i][1];
		x[3]+=tmp*tmp;
		y[1]+=val[i][0]*val[i][1];
		y[2]+=val[i][0]*tmp;
        }
	det=x[3]*(x[1]*idx-x[0]*x[0])-x[2]*(x[2]*idx-x[0]*x[1])+x[1]*(x[2]*x[0]-x[1]*x[1]);
	inv[0][0]=x[1]*idx-x[0]*x[0];
	inv[0][1]=-(x[2]*idx-x[1]*x[0]);
	inv[0][2]=x[2]*x[0]-x[1]*x[1];
	inv[1][0]=-(x[2]*idx-x[1]*x[0]);
	inv[1][1]=x[3]*idx-x[1]*x[1];
	inv[1][2]=-(x[3]*x[0]-x[1]*x[2]);
	inv[2][0]=x[2]*x[0]-x[1]*x[1];
	inv[2][1]=-(x[3]*x[0]-x[2]*x[1]);
	inv[2][2]=x[3]*x[1]-x[2]*x[2];
	det=1/det;
	for (i=0;i<3;++i)
		pf[i]=(inv[i][0]*y[2]+inv[i][1]*y[1]+inv[i][2]*y[0])*det;
	for (i=0,meanval=det=0;i<idx;++i) {
		cor=(val[i][1]*val[i][1]*pf[0]+val[i][1]*pf[1]+pf[2])-val[i][0];
		meanval+=cor;
		det+=cor*cor;
	}
	meanval=meanval/(double)idx;
	for (i=0,det2=0;i<idx;++i) {
		cor=(val[i][1]*val[i][1]*pf[0]+val[i][1]*pf[1]+pf[2])-val[i][0];
		det2+=(cor-meanval)*(cor-meanval);
	}
	rmserr[0]=meanval;
	rmserr[2]=sqrt(det/idx);
	rmserr[1]=sqrt(det2/idx);
}
	
int main (int argc, char **argv) {
	
	// local variables
	int iter=300,ch,pos=0,oldpos=0,i=0;
	double pf[2][3], rmserr[2][3];
	char valstr[5]={'0','3','0','0',0};
	char goodbadstr[2][21] = {{0x1b,'[','3','2','m','G','O','O','D',0x1b,'[','0','m',0},{0x1b,'[','3','1','m','B','A','D',0x1b,'[','0','m',0,0}};
	char colors[2][6] ={{0x1b,'[','0','m',0},{0x1b,'[','3','1','m',0}};
	// signals and action handlers
	struct sigaction sigact;
		
	// socket communication
	
	// initialize variables
	static_sensor.offset = 0.0;
	static_sensor.linearity = 1.0;

	tep_sensor.offset = 0.0;
	tep_sensor.linearity = 1.0;
	tep_sensor.Pcomp2 = static_sensor.Pcomp2 = -0.0000004638;
	tep_sensor.Pcomp1 = static_sensor.Pcomp1 =  0.9514328801;
	tep_sensor.Pcomp0 = static_sensor.Pcomp0 =  0.1658634996;

	//open file for raw output
	//fp_rawlog = fopen("raw.log","w");
		
	//parse command line arguments
	cmdline_parser(argc, argv, &io_mode);

	// get config file options
	if (fp_config != NULL)
		cfgfile_parser(fp_config, &static_sensor, &tep_sensor, &dynamic_sensor, &voltage_sensor, &config);
	
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
	//close(STDOUT_FILENO);
	close(STDERR_FILENO);	
	// ignore SIGPIPE
	signal(SIGPIPE, SIG_IGN);
	// we need hardware sensors for running !!
	// open sensor for static pressure
	/// @todo remove hardcoded i2c address static pressure

	struct termios tio, tio2;

	tcgetattr( 0, &tio );
	tcgetattr( 0, &tio2 );
	tio.c_lflag &= (~ICANON & ~ECHO);
	tcsetattr( 0, TCSANOW, &tio );

	printf ("This program calculates compenation data to compensate for timing glitches on the static and tek sensors.\nThe pressure sensors are sensitive to air movement.\nFor best results, ensure the air is as still as possible when taking measurements.\nHow many thousand data points? %s\033[4D",valstr);
	do {
		ch=getc(stdin);
		if (ch==0x1b) 
			if (getc(stdin)==0x5b) 
				ch=getc(stdin)+256;
		if ((ch==0x143) && (pos<3)) pos++;
		if ((ch==0x144) && (pos>0)) pos--;
		if (ch==0x141) 
			switch (pos) {
				case 0: iter+=1000;
					break;
				case 1: iter+=100;
					break;
				case 2: iter+=10;
					break;
				case 3: iter+=1;
					break;
			}
		if (ch==0x142) 
			switch (pos) {
				case 0: iter-=1000;
					break;
				case 1: iter-=100;
					break;
				case 2: iter-=10;
					break;
				case 3: iter-=1;
					break;
			}
		if ((ch>='0') && (ch<='9')) { 
			valstr[pos]=ch;
			sscanf (valstr,"%d",&iter);
		}
		if (iter<1) iter=1;
		if (iter>1000) iter=1000;
		sprintf (valstr,"%04d",iter);
		for (i=0;i<oldpos;++i) printf ("\033[1D");
		printf ("%s",valstr);
		for (i=0;i<(4-pos);++i) printf("\033[1D");
		oldpos=pos;
	} while (ch!=0xa);
	printf ("\033[1C\n");
	if (iter>1000) iter=1000;
	iter*=1000;
	printf ("Collecting %d data points.\n",iter);

	tcgetattr( 0, &tio );
	tio.c_lflag |= ICANON | ECHO;
	tcsetattr( 0, TCSANOW, &tio ); 

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
	for (i=0;i<1000;++i) 
		pressure_measurement_handler(0);
	// main data acquisition loop

	for (i=0;sidx<iter;) 
	{
		if (noglitch>29) usleep (250e3);
		pressure_measurement_handler(1);
		if ((sidx%1000)==0) { 
			if (i==0) { 
				printf ("%d points collected.\n",sidx); 
				i=1;
			}
		} else i=0;
	}
	if (fp_datalog!=NULL) fclose(fp_datalog);
	if (fp_config!=NULL) fclose(fp_config);
	fp_config  = fopen (config_filename,"a+");

	polyfit (statval, sidx, &pf[0][0],&rmserr[0][0]);
	polyfit (tepval, tidx, &pf[1][0],&rmserr[1][0]);
	pos=0;
	printf ("Total data points (the more the better): static: %d, tek: %d\n",sidx, tidx);
	if (rmserr[0][2]>=80) i=3; else i=0;
	if (rmserr[1][2]>=80) i|=5; 
	printf ("%s\tTotal RMS Error (lower is better, ideally below 80): static: %s%f%s, tek: %s%f%s\n",goodbadstr[i&1],colors[(i>>1)&1],rmserr[0][2],colors[0],colors[(i>>2)&1],rmserr[1][2],colors[0]);
	pos|=i;
	if (rmserr[0][0]>=1e-3) i=3; else i=0;
	if (rmserr[1][0]>=1e-3) i|=5; 
	printf ("%s\tMean Error (This should be very close to zero): static %s%f%s, tek: %s%f%s\n",goodbadstr[i&1],colors[(i>>1)&1],rmserr[0][0],colors[0],colors[(i>>2)&1],rmserr[1][0],colors[0]);
	pos|=i;
	if (rmserr[0][1]>=80) i=3; else i=0;
	if (rmserr[1][1]>=80) i|=5; 
	printf ("%s\tStd Deviation of Error (should be RMS error or slightly less): static: %s%f%s, tek: %s%f%s\n",goodbadstr[i&1],colors[(i>>1)&1],rmserr[0][1],colors[0],colors[(i>>2)&1],rmserr[1][1],colors[0]);
	pos|=i;
	if (urun>0) i=3; else i=0;
	if (orun>0) i|=5;
	if (i) printf ("%s\tGlitches (zero is ideal), Underrun: %s%d%s, Overrun: %s%d%s\n",goodbadstr[1],colors[(i>>1)&1],urun,colors[0],colors[(i>>2)&1],orun,colors[0]); 
	else printf ("%s\tNo overruns/underruns errors found during glitches.\n",goodbadstr[0]);
	pos|=i;
	printf ("static_comp %.10lf %.10lf %.10lf\ntek_comp %.10lf %.10lf %.10lf\n",pf[0][0],pf[0][1],pf[0][2],pf[1][0],pf[1][1],pf[1][2]);
	printf ("Empirical testing seems to show: \n");
	if ((2.5e-6<pf[0][0]) || (pf[0][0]<-2.5e-6)) i=3; else i=0;
	if ((2.5e-6<pf[1][0]) || (pf[1][0]<-2.5e-6)) i|=5;
	printf ("%s\t2.5e-6 > quadrature term (%s%4.3e%s and %s%4.3e%s) > -2.5e-6\n",goodbadstr[i&1],colors[(i>>1)&1],pf[0][0],colors[0],colors[(i>>2)&1],pf[1][0],colors[0]);
	pos|=i;
	if ((-.28<pf[0][1]) || (pf[0][1]<-0.35)) i=3; else i=0;
	if ((-.28<pf[1][1]) || (pf[1][1]<-0.35)) i|=5;
	printf ("%s\t-0.28 > linear term (%s%6.5f%s and %s%6.5f%s) > -0.35\n",goodbadstr[i&1],colors[(i>>1)&1],pf[0][1],colors[0],colors[(i>>2)&1],pf[1][1],colors[0]);
	pos|=i;
	if ((0<pf[0][2]) || (pf[0][2]<-10)) i=3; else i=0;
	if ((0<pf[1][2]) || (pf[1][2]<-10)) i|=5;
	printf ("%s\t0 > constant term (%s%6.5f%s and %s%6.5f%s) > -10\n",goodbadstr[i&1],colors[(i>>1)&1],pf[0][2],colors[0],colors[(i>>2)&1],pf[1][2],colors[0]);
	pos=(pos|i)&1;
	if (pos) 
		printf ("These results could be better.  Suggest trying again.\n");
	else
		printf ("These results are good.\n");
	if (fp_config!=NULL) {
		tcgetattr( 0, &tio );
		tio.c_lflag &= (~ICANON & ~ECHO);
		tcsetattr( 0, TCSANOW, &tio );
		if (pos) strcpy(valstr,"no "); else strcpy(valstr,"yes");
		printf ("Do you wish to save results? %s\033[3D",valstr);
		do {
			ch=getc(stdin);
			if (ch==0x1b)
				if (getc(stdin)==0x5b)
					ch=getc(stdin)+256;
			switch (ch) {
				case 0x141 : case 0x142 : case 0x143 : case 0x144 : pos ^=1; break;
				case 'n' : case 'N' : pos = 1; ch = 0xa; break;
				case 'y' : case 'Y' : pos = 0; ch = 0xa; break;
				default : break;
			}
			if (pos) strcpy(valstr,"no "); else strcpy(valstr,"yes");
			if (ch==0xa) printf ("%s\n",valstr); else  printf ("%s\033[3D",valstr);
		} while (ch!=0xa);
		tcgetattr( 0, &tio );
		tio.c_lflag |= ICANON | ECHO;
		tcsetattr( 0, TCSANOW, &tio );

		if (pos==0) {
			printf ("Saving...\n");	
			fprintf(fp_config,"\nstatic_comp %.10lf %.10lf %.10lf\ntek_comp %.10lf %.10lf %.10lf\n",pf[0][0],pf[0][1],pf[0][2],pf[1][0],pf[1][1],pf[1][2]);
		}
		fclose (fp_config);
	}

	return 0;
}
