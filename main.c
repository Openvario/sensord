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

#define I2C_ADDR 0x76
 
#define MEASTIMER (SIGRTMAX)

timer_t  measTimer;
int g_debug=0;
int g_log=0;
FILE *fp_log;
FILE *fp_console;

enum e_state { sample_d1, get_d1, sample_d2, get_d2} state;
	
 void sigintHandler(int sig_num)
 {
	signal(SIGINT, sigintHandler);
	if(g_log == 1)
	{
		printf("Closing log ...\n");
		fclose(fp_log);
	}
	fclose(fp_console);
	
	printf("Exiting ...\n");
	exit(0);
 }
 
 
int main (int argc, char **argv) {
	
	char s[256];
	int c;
	long meas_tick=0;
	
	pid_t pid;
	pid_t sid;
	
	const char* Usage = "\n"\
    "  -v              print version information\n"\
    "  -d[n]           set debug level. n can be [1..2]. default=1\n"\
		"  -l              raw measured values to file\n"\
		"\n";
	
	char filename[50]="sensordaemon_measurement.csv";
	
	struct sigaction sigact;

	int foreground=FALSE;
	
	t_ms5611 static_sensor;
	t_ms5611 pitot_sensor;
	t_ams5915 diff_sensor;
	
	t_kalmanfilter1d kf;
	
	float tep;
	float dt;
	
	int sock;
	struct sockaddr_in server;
	
	state = sample_d1;

	opterr=0;
	
	// check commandline arguments
	while ((c = getopt (argc, argv, "vdf::lh")) != -1)
	{
		switch (c) {
			case 'v':
				//sprintf(s, "sensordaemon V0.1 %s %s", __DATE__
				printf("sensordaemon V%c.%c RELEASE %c build: %s %s\n", VERSION_MAJOR, VERSION_MINOR, VERSION_RELEASE,  __DATE__, __TIME__);
				break;
			
			case 'l':
			  //logging option on ??
				printf("!! LOGGER Enabled File=%s !!\n",filename);
				g_log = 1;
				break;
				
			case 'd':
				if (optarg == NULL)
					g_debug = 1;
				else
					g_debug = atoi(optarg);
				
				printf("!! DEBUG LEVEL %d !!\n",g_debug);
				break;
			case 'f':
				// don't daemonize
				printf("!! STAY in FOREGROUND !!\n");
				foreground = TRUE;
				break;
				
			case '?':
				printf("Unknow option %c\n", optopt);
				printf("Usage: sensordaemon [OPTION]\n%s",Usage);
				printf("Exiting ...\n");
				exit(0);
				break;
		}
	}
	
	// stay in foreground
	if (foreground == TRUE)
	{
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
		fp_console = fopen("sensordaemon.log","w+");
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
	
	
	// open logfile if logfile enabled
	if(g_log == 1)
	{
		fp_log = fopen(filename, "w");
		log(fp_log,"sample,time,0x77,0x76,0x28\n");
	}
		
	
	// open sensor for static pressure
	/// @todo remove hardcoded i2c address static pressure
	if (ms5611_open(&static_sensor, 0x77) != 0)
	{
		fprintf(stderr, "Open sensor failed !!\n");
		return 1;
	}
	
	//initialize static pressure sensor
	ms5611_init(&static_sensor);
	
	// open sensor for velocity pressure
	/// @todo remove hardcoded i2c address for velocity pressure
	if (ms5611_open(&pitot_sensor, 0x76) != 0)
	{
		fprintf(stderr, "Open sensor failed !!\n");
		return 1;
	}
	
	//initialize velocity pressure sensor
	ms5611_init(&pitot_sensor);
	
	
	// open sensor for differential pressure
	/// @todo remove hardcoded i2c address for differential pressure
	if (ams5915_open(&diff_sensor, 0x28) != 0)
	{
		fprintf(stderr, "Open sensor failed !!\n");
		return 1;
	}
	
	//initialize differential pressure sensor
	ams5915_init(&diff_sensor);
	
	sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock == -1)
    fprintf(stderr, "could not create socket\n");
  
  server.sin_addr.s_addr = inet_addr("127.0.0.1");
  server.sin_family = AF_INET;
  server.sin_port = htons(4353);

  while (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
    fprintf(stderr, "failed to connect, trying again\n");
    fflush(stdout);
    sleep(1);
  }
	
	while (1)
	{
		ms5611_measure(&static_sensor);
		ms5611_calculate(&static_sensor);
		
		ms5611_measure(&pitot_sensor);
		ms5611_calculate(&pitot_sensor);
		
		ams5915_measure(&diff_sensor);
		ams5915_calculate(&diff_sensor);
		
		//printf("%f,%f,%f,%f\n",0.0, static_sensor.p/100.0, pitot_sensor.p/100.0, diff_sensor.p);
		log(fp_log,"%ld,%f,%f,%f,%f\n",meas_tick, 0.0, static_sensor.p/100.0, pitot_sensor.p/100.0, diff_sensor.p);
		meas_tick++;
		
		dt = (static_sensor.sample.tv_sec + 1.0e-9*static_sensor.sample.tv_nsec)-\
					(static_sensor.last_sample.tv_sec + 1.0e-9*static_sensor.last_sample.tv_nsec);
		
		tep = static_sensor.p/100.0;
		
		static_sensor.last_sample = static_sensor.sample;
		
		//debug_print("Pressure: %f  %f\n",tep, dt);
		
		//KalmanFiler1d_update(&kf, tep, 0.7, dt);
		
		//debug_print("Kalman X_ABS: %f, X_VEL: %f\n", kf.x_abs_, kf.x_vel_);
		
		Compose_PAFGB_sentence(&s[0], static_sensor.p/100.0, pitot_sensor.p/100.0, 0.0);
		
		//get_temp_ds18b20();
		
		if (send(sock, s, strlen(s), 0) < 0)
        fprintf(stderr, "send failed\n");
		usleep(440000);
	}
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
		return 0;

 }
 

 

