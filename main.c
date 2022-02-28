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

#include "main.h"
#include "cmdline_parser.h"
#include "nmea.h"
#include "KalmanFilter1d.h"
#include "ds2482.h"
#include "humidity.h"
#include "ms5611.h"
#include "ams5915.h"
#include "ads1110.h"
#include "configfile_parser.h"
#include "vario.h"
#include "AirDensity.h"
#include "24c16.h"
#include "wait.h"
#include "clock.h"
#include "log.h"

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

// Sensor objects
static t_ms5611 static_sensor;
static t_ms5611 tep_sensor;
static t_ams5915 dynamic_sensor;
static t_ads1110 voltage_sensor;
static t_ds2482 temp_sensor;


// configuration object
static t_config config;

// Filter objects
static t_kalmanfilter1d vkf;

// pressures
static float p_static;
static float p_dynamic;

static t_io_mode io_mode;

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
static void sigintHandler(int sig_num)
{
	(void)sig_num;

	signal(SIGINT, sigintHandler);

	// if meas_mode = record -> close fp now
	if (fp_datalog != NULL)
		fclose(fp_datalog);

	// if sensordata from file
	if (fp_sensordata != NULL)
		fclose(fp_sensordata);

	fprintf(stderr, "Exiting ...\n");

	exit(0);
}


/**
* @brief Command handler for NMEA messages
* @param sock Network socket handler
* @return
*
* Message handler called by main-loop to generate timing of NMEA messages
* @date 17.04.2014 born
*
*/
static int NMEA_message_handler(int sock)
{
	// some local variables
	float vario;
	int sock_err = 0;
	static int nmea_counter = 1;
	int result;
	char s[256];

	if (temp_sensor.temp_valid) {
		// Compose POV NMEA sentences
		result=Compose_Temperature_POV(&s[0], temp_sensor.temperature);
		// fprintf(stderr, "temp: %f\n",temp_sensor.temperature);
		// // NMEA sentence valid ?? Otherwise print some error !!
		if (result != 1) fprintf(stderr, "POV Temperature NMEA Result = %d\n",result);
		// Send NMEA string via socket to XCSoar send complete sentence including terminating '\0'
		if ((sock_err = send(sock, s, strlen(s), 0)) < 0) fprintf(stderr, "send failed %s\n",s);
	}
	if (temp_sensor.humidity_valid) {
		// Compose POV NMEA sentences
		result=Compose_Humidity_POV(&s[0], temp_sensor.humidity);
		// fprintf(stderr, "humid: %f\n",temp_sensor.humidity);
		// NMEA sentence valid ?? Otherwise print some error !!
		if (result != 1) fprintf(stderr, "POV Humidity NMEA Result = %d\n",result);
		// Send NMEA string via socket to XCSoar send complete sentence including terminating '\0'
		if ((sock_err = send(sock, s, strlen(s), 0)) < 0) fprintf(stderr, "send failed %s\n",s);
	}

	if ((nmea_counter++)%4==0)
	{
		// Compute Vario
		vario = ComputeVario(vkf.x_abs_, vkf.x_vel_);

		if (config.output_POV_P_Q == 1)
		{
			// Compose POV slow NMEA sentences
			result = Compose_Pressure_POV_slow(&s[0], p_static/100, p_dynamic*100);
			// NMEA sentence valid ?? Otherwise print some error !!
			if (result != 1)
			{
				fprintf(stderr, "POV slow NMEA Result = %d\n",result);
			}
			// Send NMEA string via socket to XCSoar
			if ((sock_err = send(sock, s, strlen(s), 0)) < 0)
			{
			  fprintf(stderr, "send failed %s\n",s);
				return sock_err;
			}
		}

		if (config.output_POV_E == 1)
		{
			if (tep_sensor.valid != 1)
			{
				vario = 99;
			}
			// Compose POV slow NMEA sentences
			result = Compose_Pressure_POV_fast(&s[0], vario);
			// NMEA sentence valid ?? Otherwise print some error !!
			if (result != 1)
			{
				fprintf(stderr, "POV fast NMEA Result = %d\n",result);
			}
			// Send NMEA string via socket to XCSoar
			if ((sock_err = send(sock, s, strlen(s), 0)) < 0)
			{
				fprintf(stderr, "send failed\n");
				return sock_err;
			}
		}

		if (config.output_POV_V == 1 && voltage_sensor.present)
		{

			// Compose POV slow NMEA sentences
			result = Compose_Voltage_POV(&s[0], voltage_sensor.voltage_converted);

			// NMEA sentence valid ?? Otherwise print some error !!
			if (result != 1)
			{
				fprintf(stderr, "POV voltage NMEA Result = %d\n",result);
			}
			// Send NMEA string via socket to XCSoar
			if ((sock_err = send(sock, s, strlen(s), 0)) < 0)
			{
			  fprintf(stderr, "send failed %s\n",s);
				return sock_err;
			}
		}
	}

	return(sock_err);
}

/**
* @brief Timming routine for pressure measurement
* @param
* @return
*
* Timing handler to coordinate pressure measurement
* @date 17.04.2014 born
*
* The MS5611 has been shown to have multiple different error modes.  The most common is that it is sensitive to timing jitter.  For reasons that are unknown, whenever the
* MS5611 is sampled it appears to bias an internal node in the sampler.  As a result, the output for any given steady state conversion frequency there will be a fixed offset.
* Generally speaking, as long as the offset is fixed this isn't a big deal.  However, if there is a significant variation in timing, the the offset is no longer fixed and it
* will induce what appears to be a delta function that decays off as if going through a 1st order LPF.  This offset can be seen in both temperature and pressure readings
* and will be on both sensors.  The offset error is ratiometric between temperature and pressure, and as such if timing jitter is detected the pressure measurement handler
* can determine how much change there has been between the filtered temperature value and the present temperature value and then put that into a quadratic function to add
* a compensating value for pressure.  Since pressure on 1 and temperature on 2 are simultaneously measured (and vice versa) the temperature reading on one sensor is used to
* compensate the pressure reading on the other.
*
* Rarely, other glitches have been seen where the temperature and/or pressure reading on one sensor just drops a substantial amount.  As such, any reading that is more than 100,000
* away from the previous reading is rejected.
*/
static void pressure_measurement_handler(void)
{
	static int meas_counter = 1, glitch = 0;
	int reject = 0;
	static struct timespec kalman_prev;


	// Initialize timers if first time through.
	if (meas_counter==1) clock_gettime(CLOCK_MONOTONIC, &kalman_prev);

	// read ADS1110
	if (voltage_sensor.present && (meas_counter%4==0))
	{
		ads1110_measure(&voltage_sensor);
		ads1110_calculate(&voltage_sensor);
	}

	if (!io_mode.sensordata_from_file)
	{
		static int glitchstart = 0, deltaxmax = 0, shutoff = 0;

		// read AMS5915
		ams5915_measure(&dynamic_sensor);

		// if early, wait

		// if more than 2ms late, increase the glitch counter

		if ((sensor_wait(12500))>2000) { glitchstart=8; glitch+=8; }
		if (meas_counter&1) {
			// read pressure sensors
			int deltax;

			ms5611_read_temp(&tep_sensor,glitch);
			ms5611_read_pressure(&static_sensor);
			ms5611_start_pressure(&tep_sensor);
			sensor_wait_mark();
			ms5611_start_temp(&static_sensor);
			if (abs((int)static_sensor.D1l-(int) static_sensor.D1)>100e3)  reject=1;
			if (!glitch)
				if (tep_sensor.D2l>tep_sensor.D2+300) { glitchstart=8; glitch=8; }
			if (glitchstart) {
				if (glitch>glitchstart)
					deltax=abs((int) tep_sensor.D2l-(int)tep_sensor.D2);
				else
					deltax=abs((int)tep_sensor.D2f-(int)tep_sensor.D2);
				if (deltax>deltaxmax) deltaxmax=deltax;
				if ((--glitchstart)==0) {
					if (deltaxmax>15)
						glitch += ((int) round(log((double)(deltaxmax)*config.timing_log)*config.timing_mult))+config.timing_off;
					deltaxmax=0;
				}
			}

			// if there was a glitch, compensate for the glitch
			if (glitch) {
				double correction,cor2;

				if (--glitch>350) glitch=350;
				if ((++shutoff)>399) {
					shutoff=glitch=0;
					tep_sensor.D2f=tep_sensor.D2;
					static_sensor.D2f=static_sensor.D2;
				}
				// compensate for the glitch
				deltax = (int) tep_sensor.D2f-(int) tep_sensor.D2;
				if (!glitchstart)
					if (abs(deltax)<15) glitch=0;
				correction = deltax*deltax*static_sensor.comp2+deltax*static_sensor.comp1+static_sensor.comp0;
				cor2=static_sensor.p*1e-5;
				correction *= (cor2*cor2*static_sensor.Pcomp2+cor2*static_sensor.Pcomp1+static_sensor.Pcomp0);
				static_sensor.D1+=(int) round(correction);
			} else {
				static_sensor.D1f=(static_sensor.D1f*7+static_sensor.D1)/8;
				shutoff=0;
			}
			ms5611_calculate_pressure(&static_sensor);
		} else {
			// read pressure sensors
			int deltax;

			ms5611_read_pressure(&tep_sensor);
			ms5611_read_temp(&static_sensor,glitch);
			ms5611_start_temp(&tep_sensor);
			sensor_wait_mark();
			ms5611_start_pressure(&static_sensor);
			if (abs((int) tep_sensor.D1l-(int) tep_sensor.D1)>100e3) reject=1;
			if (!glitch)
				if (static_sensor.D2l>static_sensor.D2+300) { glitchstart=8; glitch=8; }
			if (glitchstart) {
				if (glitch>glitchstart)
					deltax=abs((int) static_sensor.D2l-(int)static_sensor.D2);
				else
					deltax=abs((int)static_sensor.D2f-(int) static_sensor.D2);
				if (deltax>deltaxmax) deltaxmax=deltax;
				if ((--glitchstart)==0) {
					if (deltaxmax>15)
						glitch += ((int) round(log((double)(deltaxmax)*config.timing_log)*config.timing_mult))+config.timing_off;
					deltaxmax=0;
				}
			}

			// if there was a glitch, compensate for the glitch
			if (glitch)
			{
				double correction,cor2;

				if (--glitch>350) glitch=350;
				// compensate for the glitch
				deltax = (int) static_sensor.D2f-(int) static_sensor.D2;
				if (!glitchstart)
					if (abs(deltax)<15) glitch=0;
				correction = deltax*deltax*tep_sensor.comp2+deltax*tep_sensor.comp1+tep_sensor.comp0;
				cor2=tep_sensor.p*1e-5;
				correction *= (cor2*cor2*tep_sensor.Pcomp2+cor2*tep_sensor.Pcomp1+tep_sensor.Pcomp0);
				tep_sensor.D1+=(int) round(correction);
			} else {
				tep_sensor.D1f = (tep_sensor.D1f*7+tep_sensor.D1)/8;
				shutoff=0;
			}
			ms5611_calculate_pressure(&tep_sensor);
		}
		ams5915_calculate(&dynamic_sensor);
	} else {

		// read from sensor data from file if desired
		if (fscanf(fp_sensordata, "%f,%f,%f", &tep_sensor.p, &static_sensor.p, &dynamic_sensor.p) == EOF)
		{
			fprintf(stderr, "End of File reached\n");
			fprintf(stderr, "Exiting ...\n");
			exit(EXIT_SUCCESS);
		}
	}
	// filtering
	//
	// of dynamic pressure
	p_dynamic = (15*p_dynamic + dynamic_sensor.p) / 16;
	//fprintf(stderr, "Pdyn: %f\n",p_dynamic*100);
	// mask speeds < 10km/h
	if (p_dynamic < 0.04)
	{
		p_dynamic = 0.0;
	}

	if (reject==0) {
		if (meas_counter&1) {
			// of static pressure
			p_static = (7*p_static + static_sensor.p) / 8;
		} else {
			// check tep_pressure input value for validity
			if ((tep_sensor.p/100 < 100) || (tep_sensor.p/100 > 1200))
			{
				// tep pressure out of range
				tep_sensor.valid = 0;
			} else {
				// of tep pressure
				struct timespec kalman_cur;

				tep_sensor.valid=1;
				clock_gettime(CLOCK_MONOTONIC, &kalman_cur);
				KalmanFiler1d_update(&vkf, tep_sensor.p/100, 0.25, timespec_delta_s(&kalman_cur, &kalman_prev));
				kalman_prev=kalman_cur;
			}
			if (io_mode.sensordata_to_file) {
				if (tj)
					fprintf(fp_datalog, "%.4f %.4f %f %u %u %u %u %u %u %u %u %d\n",tep_sensor.p,static_sensor.p,dynamic_sensor.p,static_sensor.D1,static_sensor.D1f,tep_sensor.D1,tep_sensor.D1f,static_sensor.D2,static_sensor.D2f,tep_sensor.D2,tep_sensor.D2f,glitch);
				else fprintf(fp_datalog, "%.4f,%.4f,%.4f\n",tep_sensor.p,static_sensor.p,dynamic_sensor.p);
			}
		}
	}
	meas_counter++;
}

static void temperature_measurement_handler(void)
{
	static int done = 0, temp_counter = 0, rollover=0;

	temp_sensor.temp_valid=temp_sensor.humidity_valid=0;
	if (temp_sensor.temp_present|temp_sensor.humidity_present) {
		switch (temp_sensor.sensor_type) {
			case DS18B20 :
				if (temp_sensor.temp_present) {
					if (temp_counter==0) {
						if (OWReset(&temp_sensor)==1) { // Reset
							if (OWWriteByte(&temp_sensor,0xCC)==1) { // Skip ROM
								OWWriteByte(&temp_sensor,0x44); // Initiate Conversion
							}
						}
						temp_counter=1;
						done=0;
					} else {
						if (!done) {
							if (OWReadByte(&temp_sensor)>0) {
								if (OWReset(&temp_sensor)==1) {  // Reset
									if (OWWriteByte(&temp_sensor,0xCC)==1) { // Skip ROM
										if (OWWriteByte(&temp_sensor,0xBE)==1) { //Read Scratchpad
											OWReadTemperature(&temp_sensor); // Convert output to temperature
											done = temp_sensor.temp_valid+1;
										}
									}
								}
							}
						}
						if (++temp_counter>=temp_sensor.rollover) {
							if (done) temp_counter=0;
								else if (temp_counter>temp_sensor.maxrollover) temp_counter=0;
						} else if (done==1) temp_counter=0;
					}
				}
				break;
			case AM2321 :
				if (++temp_counter>=temp_sensor.rollover) {
					if (!am2321_read(&temp_sensor)) done=temp_sensor.temp_valid+1;
					temp_counter=0;
				}
				break;
			case HTU21D :
				if (temp_counter==0) {
					if ((temp_sensor.humidity_present) && (rollover)) {
						si7021_read_humidity(&temp_sensor);
						fprintf(stderr, "Humidity: %f\n",temp_sensor.humidity);
					}
					if (temp_sensor.temp_present) si7021_start_temp(&temp_sensor);
					if (temp_sensor.temp_valid) temp_sensor.compensate=1; else temp_sensor.compensate=0;
				}
				if (temp_counter==temp_sensor.rollover/2) {
					if (temp_sensor.temp_present) {
						si7021_read_temp(&temp_sensor);
						fprintf(stderr, "Temperature: %f\n",temp_sensor.temperature);
					}
					if (temp_sensor.humidity_present) si7021_start_humidity(&temp_sensor);
				}
				if (++temp_counter>=temp_sensor.rollover) {
					temp_counter=0;
					rollover=1;
				}
				break;
			case HTU31D : case SI7021 : case SHT4X : case SHT85 :
				if (temp_counter==0) {
					if (rollover) si7021_read_humidity(&temp_sensor);
					si7021_start_humidity(&temp_sensor);
				}
				if (++temp_counter>=temp_sensor.rollover) {
					temp_counter=0;
					rollover=1;
				}
				break;

		}
	}
}

int main (int argc, char **argv) {

	// local variables
	int i = 0, j = 0;
	int result;

	t_24c16 eeprom;
	t_eeprom_data data;

	// for daemonizing
	pid_t pid;
	pid_t sid;

	io_mode.sensordata_from_file = false;
	io_mode.sensordata_to_file = false;

	// signals and action handlers
	struct sigaction sigact;

	// initialize variables
	static_sensor.offset = 0.0;
	static_sensor.linearity = 1.0;
	static_sensor.comp2 = -0.0000004875;
	static_sensor.comp1 = -0.2670286916;
	static_sensor.comp0 = -18.7077108239;

	dynamic_sensor.offset = 0.0;
	dynamic_sensor.linearity = 1.0;

	tep_sensor.offset = 0.0;
	tep_sensor.linearity = 1.0;
	tep_sensor.comp2 = -0.0000015538;
	tep_sensor.comp1 = -0.2489404356;
	tep_sensor.comp0 = -27.3219042156;
	tep_sensor.Pcomp2 = static_sensor.Pcomp2 = -0.0000004638;
	tep_sensor.Pcomp1 = static_sensor.Pcomp1 =  0.9514328801;
	tep_sensor.Pcomp0 = static_sensor.Pcomp0 =  0.1658634996;

	config.timing_log        = 0.066666666666666666666;
	config.timing_mult       = 50;
	config.timing_off        = 12;
	config.output_POV_E      = config.output_POV_P_Q = config.output_POV_T = config.output_POV_H = 0;

	temp_sensor.rollover = temp_sensor.maxrollover = temp_sensor.databits = temp_sensor.sensor_type = temp_sensor.compensate = 0;

	//parse command line arguments
	cmdline_parser(argc, argv, &io_mode);

	// get config file options
	if (fp_config != NULL) {
		cfgfile_parser(fp_config, &static_sensor, &tep_sensor, &dynamic_sensor, &voltage_sensor, &temp_sensor, &config);
		fclose(fp_config);
	}

	// check if we are a daemon or stay in foreground
	if (g_foreground)
	{
		// stay in foreground
		// install signal handler for CTRL-C
		sigact.sa_handler = sigintHandler;
		sigemptyset (&sigact.sa_mask);
		sigact.sa_flags = 0;
		sigaction(SIGINT, &sigact, NULL);
	}
	else
	{
		// implement handler for kill command
		fprintf(stderr, "Daemonizing ...\n");
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
		if (sid < 0)
		{
			syslog(LOG_ERR, "Could not create process group\n");
			exit(EXIT_FAILURE);
		}

		// close the standard file descriptors
		int null_fd = open("/dev/null", O_RDONLY);
		if (null_fd >= 0) {
			dup2(null_fd, STDIN_FILENO);
			close(null_fd);
		}

		//open file for log output
		int log_fd = open("sensord.log", O_CREAT|O_WRONLY|O_TRUNC,
				  0666);
		if (log_fd >= 0) {
			dup2(log_fd, STDOUT_FILENO);
			dup2(log_fd, STDERR_FILENO);
			close(log_fd);
		}
	}

	setbuf(stdout, NULL);
	setbuf(stderr, NULL);

	// ignore SIGPIPE
	signal(SIGPIPE, SIG_IGN);

	// get config from EEPROM
	// open eeprom object
	result = eeprom_open(&eeprom, 0x50);
	if (result != 0)
	{
		fprintf(stderr, "No EEPROM found !!\n");
	}
	else
	{
		if (eeprom_read_data(&eeprom, &data) == 0)
		{
			fprintf(stderr, "Using EEPROM calibration values ...\n");
			dynamic_sensor.offset = data.zero_offset;
		}
		else
		{
			fprintf(stderr, "EEPROM Checksum wrong !!\n");
		}
	}

	close(eeprom.fd);

	// print runtime config
	print_runtime_config();

	if (!io_mode.sensordata_from_file)
	{
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

		// open sensor for differential pressure
		/// @todo remove hardcoded i2c address for differential pressure
		if (ams5915_open(&dynamic_sensor, 0x28) != 0)
		{
			fprintf(stderr, "Open dynamic sensor failed !!\n");
			return 1;
		}

		// open sensor for battery voltage
		/// @todo remove hardcoded i2c address for voltage sensor
		if (ads1110_open(&voltage_sensor, 0x48) != 0)
		{
			fprintf(stderr, "Open voltage sensor failed !!\n");
		}

		// open temperature sensor and initialize
		if (config.output_POV_T|config.output_POV_H) {
			int autodetect=0,x=1;
			switch (temp_sensor.sensor_type) {
				case AUTO : fprintf(stderr, "Autodetecting temperature/humidity sensor\n");
					    autodetect=1;
					    // fallthrough
				case DS18B20 :
					if (!ds2482_open(&temp_sensor,0x18))
						fprintf (stderr,"DS2482 interface failure !!\n");
					else {
						ds2482_reset(&temp_sensor);
						if (OWConfigureBits(&temp_sensor)) {
							temp_sensor.temp_present=config.output_POV_T;
							temp_sensor.humidity_present=0;
							fprintf(stderr, "DS18B20 temperature sensor present\n");
							temp_sensor.sensor_type=DS18B20;
							autodetect=0;
							if (temp_sensor.rollover==0) {
								temp_sensor.rollover=80;
								temp_sensor.maxrollover=100;
							}
						} else {
							if (!autodetect) fprintf (stderr,"Open DS18B20 sensor failed!\n");
							else fprintf(stderr, "DS18B20 temperature sensor not detected\n");
							close (temp_sensor.fd);
						}
					}
					if (!autodetect) break;
					// fallthrough
				case AM2321 :
					if (am2321_open(&temp_sensor,0x5c)) {
						if (!autodetect) fprintf (stderr,"Open AM2321 temperature/humidity Sensor failed !!\n"); else
						fprintf(stderr, "AM2321 temperature/humidity sensor not detected\n");
						close(temp_sensor.fd);
					} else {
						fprintf(stderr, "AM2321 temperature/humidity sensor present\n");
						temp_sensor.humidity_present=config.output_POV_H;
						temp_sensor.temp_present=config.output_POV_T;
						temp_sensor.sensor_type=AM2321;
						if (temp_sensor.rollover==0) temp_sensor.rollover=160;
						autodetect=0;
					}
					if (!autodetect) break;
					// fallthrough
				case SHT4X : case SHT85 :
					switch (sht4x_open(&temp_sensor,0x45)) {
						case 0 : fprintf(stderr, "sensor present\n");
							 temp_sensor.humidity_present=config.output_POV_H;
							 temp_sensor.temp_present=config.output_POV_T;
							 if (temp_sensor.rollover==0) temp_sensor.rollover=80;
							 autodetect=0;
							 x=0;
							 break;
						case 3 : fprintf(stderr, "SHT4X sensor may be detected on 0x45, but not working\n");
							 fprintf (stderr,"SHT4X sensor may be detected on 0x45, but not working\n");
							 close(temp_sensor.fd);
							 break;
						case 4 : if ((temp_sensor.sensor_type)==SHT4X) {
								 fprintf(stderr, "SHT4X sensor detected on 0x45, but failed to read serial number\n");
								 fprintf (stderr,"SHT4X sensor detected on 0x45, but failed to read serial nunber\n");
							 }
							 temp_sensor.humidity_present=config.output_POV_H;
							 temp_sensor.temp_present=config.output_POV_T;
							 if (temp_sensor.rollover==0) temp_sensor.rollover=80;
							 autodetect=0;
							 break;
						default :
							close(temp_sensor.fd);
							break;
					}
					if (x)
						switch (sht4x_open(&temp_sensor,0x44)) {
							case 0 : fprintf(stderr, "sensor present\n");
								temp_sensor.humidity_present=config.output_POV_H;
								 temp_sensor.temp_present=config.output_POV_T;
								 if (temp_sensor.rollover==0) temp_sensor.rollover=80;
								 autodetect=0;
								 x=0;
							break;
							case 3 : fprintf(stderr, "SHT4X/SHT85 sensor may be detected on 0x44, but not working\n");
								 fprintf (stderr,"SHT4X/SHT85 sensor may be detected on 0x44, but not working\n");
								 close(temp_sensor.fd);
								 break;
							case 4 : if ((temp_sensor.sensor_type)==SHT4X) {
									fprintf(stderr, "SHT4X sensor detected on 0x44, but failed to read serial number\n");
									fprintf (stderr,"SHT4X sensor detected on 0x44, but failed to read serial nunber\n");
								 } else {
									fprintf(stderr, "SHT85 sensor detected, but failed to read serial number\n");
									fprintf (stderr,"SHT85 sensor detected, but failed to read serial number\n");
								 }
								 temp_sensor.humidity_present=config.output_POV_H;
								 temp_sensor.temp_present=config.output_POV_T;
								 if (temp_sensor.rollover==0) temp_sensor.rollover=80;
								 autodetect=0;
								 break;
							default : if (!autodetect) fprintf (stderr,"Open SHT4X/SHT85 temperature/humidity sensor failed !!\n"); else
									  fprintf(stderr, "SHT4X/SHT85 tenperature/humidity sensor not detected\n");
								close(temp_sensor.fd);
						}
					if (!autodetect) break;
					// fallthrough
				case SI7021 : case HTU21D : case HTU31D :
					if ((temp_sensor.sensor_type!=HTU21D) && (temp_sensor.sensor_type!=SI7021)) {
						switch (si7021_open(&temp_sensor,0x41)) {
							case 0 : fprintf(stderr, "sensor present on 0x41\n");
								temp_sensor.humidity_present=config.output_POV_H;
								temp_sensor.temp_present=config.output_POV_T;
								if (temp_sensor.rollover==0) temp_sensor.rollover=80;
								autodetect=0;
								x=0;
								break;
							case 3 : fprintf (stderr,"HTU31D may be detected on 0x41, but not working\n");
								fprintf(stderr, "HTU31D sensor may be detected on 0x41, but not working\n");
								close(temp_sensor.fd);
								break;
							case 4 : fprintf(stderr, "HTU31D sensor detected on 0x41, but failed to read serial number\n");
								 fprintf (stderr,"HTU31D sensor detected on 0x41, but failed to read serial number\n");
								temp_sensor.humidity_present=config.output_POV_H;
								temp_sensor.temp_present=config.output_POV_T;
								if (temp_sensor.rollover==0) temp_sensor.rollover=80;
								autodetect=0;
								break;
							default :
								close(temp_sensor.fd);
								break;
						}
					}
					if (x)
						switch (si7021_open(&temp_sensor,0x40)) {
							case 0 : fprintf(stderr, "sensor present on 0x40\n");
								 temp_sensor.humidity_present=config.output_POV_H;
								 temp_sensor.temp_present=config.output_POV_T;
								 if (temp_sensor.rollover==0) {
									 if (temp_sensor.sensor_type==HTU21D) temp_sensor.rollover=40; else temp_sensor.rollover=80;
								 }
								 autodetect=0;
								 break;
							case 3 : fprintf(stderr, "HTU31D sensor may be detected on 0x40, but not working\n");
								 fprintf (stderr,"HTU31D sensor may be detected on 0x40, but not working\n");
								 close(temp_sensor.fd);
								 break;
							case 4 : fprintf(stderr, "HTU31D sensor detected on 0x40, but failed to read serial number\n");
								 fprintf (stderr,"HTU31D sensor detected on 0x40, but failed to read serial number\n");
								temp_sensor.humidity_present=config.output_POV_H;
								temp_sensor.temp_present=config.output_POV_T;
								if (temp_sensor.rollover==0) temp_sensor.rollover=80;
								autodetect=0;
								break;
							case 6 : fprintf(stderr, "SI7021/HTU21D detected but failed to read serial number\n");
								fprintf (stderr, "SI7021/HTU21D detected but failed to read serial number\n");
								temp_sensor.humidity_present=config.output_POV_H;
								temp_sensor.temp_present=config.output_POV_T;
								if (temp_sensor.rollover==0) temp_sensor.rollover=80;
								autodetect=0;
								break;
							case 7 : fprintf(stderr, "failed to read firmware revision\n");
								fprintf (stderr, "SI7021 detected but failed to read firmware revision\n");
								temp_sensor.humidity_present=config.output_POV_H;
								temp_sensor.temp_present=config.output_POV_T;
								if (temp_sensor.rollover==0) temp_sensor.rollover=80;
								autodetect=0;
								break;
							case 2 : case 5 : default :
								if (!autodetect) fprintf (stderr,"Open SI7021/HTU21D/HTU31D temperature/humidity sensor failed !!\n"); else
								fprintf(stderr, "SI7021/HTU21D/HTU31D tenperature/humidity sensor not detected\n");
								close(temp_sensor.fd);
						}
					if (!autodetect) break;
					// fallthrough
				default :
					config.output_POV_T=config.output_POV_H=0;
					break;
			}
			if (autodetect) fprintf(stderr, "No temperature/humidity sensor detected !!\n");
		}
		//initialize differential pressure sensor
		ams5915_init(&dynamic_sensor);
		dynamic_sensor.valid = 1;

		//initialize voltage sensor
		// if(voltage_sensor.present)
		ads1110_init(&voltage_sensor);

		// poll sensors for offset compensation
		tep_sensor.D2f=static_sensor.D2f=0;
		for (i=0;i<120;++i)
		{
			ms5611_start_temp(&static_sensor);
			ms5611_start_temp(&tep_sensor);
			if (i==0) sensor_wait_mark();
			sensor_wait(12500);
			sensor_wait_mark();
			ms5611_read_temp(&static_sensor,0);
			ms5611_read_temp(&tep_sensor,0);
		}

		ms5611_start_pressure(&static_sensor);
		ms5611_start_temp(&tep_sensor);
		sensor_wait(12500);
		sensor_wait_mark();

		ms5611_read_pressure(&static_sensor);
		ms5611_read_temp(&tep_sensor,0);
		ms5611_start_pressure(&tep_sensor);
		ms5611_start_temp(&static_sensor);
		sensor_wait(12500);
		sensor_wait_mark();

		ms5611_read_pressure(&tep_sensor);
		ms5611_read_temp(&static_sensor,0);
		ms5611_calculate_pressure(&tep_sensor);
		ms5611_calculate_pressure(&static_sensor);
		ms5611_start_temp(&tep_sensor);
		ms5611_start_pressure(&static_sensor);

		ams5915_measure(&dynamic_sensor);
		ams5915_calculate(&dynamic_sensor);

		// initialize variables
		p_static = static_sensor.p;
		p_dynamic = dynamic_sensor.p;
	}
	else
	{
		p_static = 101325.0;
		p_dynamic = 0;
	}

	// initialize kalman filter
	KalmanFilter1d_reset(&vkf);
	vkf.var_x_accel_ = config.vario_x_accel;
	for(i=0; i < 1000; i++)
		KalmanFiler1d_update(&vkf, tep_sensor.p/100, 0.25, 25e-3);
	while(1)
	{
		// socket communication
		int sock;
		struct sockaddr_in server;
		int sock_err = 0;

		// Open Socket for TCP/IP communication
		sock = socket(AF_INET, SOCK_STREAM, 0);
		if (sock == -1) fprintf(stderr, "could not create socket\n");
		server.sin_addr.s_addr = inet_addr("127.0.0.1");
		server.sin_family = AF_INET;
		server.sin_port = htons(4353);

		// try to connect to XCSoar
		while (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0)
		{
			fprintf(stderr, "failed to connect, trying again\n");
			fflush(stdout);
			sleep(1);
		}
		tep_sensor.D2f=tep_sensor.D2;
		static_sensor.D2f=static_sensor.D2;
		// socket connected
		// main data acquisition loop
		while(sock_err >= 0)
		{
			if (tj)
				if ((++j)%1023==100) sensor_wait(250e3);
			pressure_measurement_handler();
			temperature_measurement_handler();
			sock_err = NMEA_message_handler(sock);
			//debug_print("Sock_err: %d\n",sock_err);

		}

		// connection dropped
		close(sock);
	}
	return 0;
}

void print_runtime_config(void)
{
	// print actual used config
	fprintf(stderr, "=========================================================================\n");
	fprintf(stderr, "Runtime Configuration:\n");
	fprintf(stderr, "----------------------\n");
	fprintf(stderr, "Vario:\n");
	fprintf(stderr, "  Kalman Accel:\t%f\n",config.vario_x_accel);
	fprintf(stderr, "Sensor TEK:\n");
	fprintf(stderr, "  Offset: \t%f\n",tep_sensor.offset);
	fprintf(stderr, "  Linearity: \t%f\n", tep_sensor.linearity);
	fprintf(stderr, "Sensor STATIC:\n");
	fprintf(stderr, "  Offset: \t%f\n",static_sensor.offset);
	fprintf(stderr, "  Linearity: \t%f\n", static_sensor.linearity);
	fprintf(stderr, "Sensor TOTAL:\n");
	fprintf(stderr, "  Offset: \t%f\n",dynamic_sensor.offset);
	fprintf(stderr, "  Linearity: \t%f\n", dynamic_sensor.linearity);
	fprintf(stderr, "=========================================================================\n");

}


