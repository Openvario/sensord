/*  
	sensord - Sensor Interface for XCSoar Glide Computer - http://www.openvario.org/
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

#include "ms5611.h"
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include "def.h"

#define DELTA_TIME_US(T1, T2)   (((T1.tv_sec+1.0e-9*T1.tv_nsec)-(T2.tv_sec+1.0e-9*T2.tv_nsec))*1000000)     

extern int g_debug;
extern FILE *fp_console;

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


/**
* @brief Establish connection to MS5611 pressure sensor
* @param sensor pointer to sensor instance
* @param i2c_address
* @return result
*
* @date 24.03.2016 revised
*
*/ 
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

/**
* @brief calculate CRC for MS5611
* @param PROM value array
* @return CRC
*
* @date 24.03.2016 born
*
*/
uint8_t crc4(uint16_t n_prom[])
{
	int cnt; // simple counter
	uint16_t n_rem; // crc reminder
	uint16_t crc_read; // original value of the crc
	uint8_t n_bit;
	n_rem = 0x00;
	crc_read=n_prom[7]; //save read CRC
	n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
	for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
	{// choose LSB or MSB
		if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))
			{
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else
			{
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem= (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
	n_prom[7]=crc_read; // restore the crc_read to its original place
	return (n_rem ^ 0x0);
}

/**
* @brief Initialize MS5611 pressure sensor
* @param sensor pointer to sensor instance
* @return result
*
* @date 24.03.2016 revised
*
*/ 
int ms5611_init(t_ms5611 *sensor)
{
	uint8_t buf[10];
	uint8_t a,i,n_crc, crc;
	//uint16_t prom[8]={0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x4500};
	uint16_t prom[8];
	
	// Print debug info
	ddebug_print("Sensor compensation data: Offset: %f, Linearity: %f\n", sensor->offset, sensor->linearity);
	
	// get calibration data from sensor
	ddebug_print("Get calibration data ...\n");
	
	for(a = 0xA0, i = 0; a <= 0xAE; a = a +0x02, i++)
	{
		// get calibration values
		buf[0] = a;													// This is the register we want to read from
		if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
			printf("Error writing to i2c slave (write cal reg)\n");
			return(1);
		}
		usleep(10000);
		if (read(sensor->fd, buf, 2) != 2) {								// Read back data into buf[]
			printf("Unable to read from slave (get cal reg)\n");
			return(1);
		}
		ddebug_print("Read adr: 0x%x data: 0x%x 0x%x\n", a, buf[0], buf[1]);
		prom[i] = (buf[0] * 256) + buf[1];
		ddebug_print("Adr = 0x%x %u\n", a, prom[i]);
	}
	

	n_crc=crc4(prom);
	crc = prom[7] & 0xF;
	
	ddebug_print("CRC Check: calc: 0x%x read: 0x%x => ", n_crc, crc);
	// check if CRC is correct
	if (n_crc == crc)
	{
		ddebug_print("OK\n");
	}
	else
	{
		ddebug_print("WRONG !!!!\n");
	}
	
	sensor->C1s = prom[1] << 15;
	sensor->C2s = prom[2] << 16;
	sensor->C3  = prom[3];
	sensor->C4  = prom[4];
	sensor->C5s = prom[5] << 8;
	sensor->C6  = prom[6];
	
	// print calibration values if debug is enabled
	ddebug_print("Calibration values:\n");
	
	ddebug_print("C1s = %u\n", sensor->C1s);
	ddebug_print("C2s = %u\n", sensor->C2s);
	ddebug_print("C3  = %u\n", sensor->C3);
	ddebug_print("C4  = %u\n", sensor->C4);
	ddebug_print("C5s = %u\n", sensor->C5s);
	ddebug_print("C6  = %u\n", sensor->C6);
	
	return(0);
}

/**
* @brief Reset command to MS5611 pressure sensor
* @param sensor pointer to sensor instance
* @return result
*
* @date 24.03.2016 revised
*
*/ 
int ms5611_reset(t_ms5611 *sensor)
{
	//variables
	unsigned char buf[10]={0x00};

	// clock out sensor
	//buf[0] = 0x00;										// This is the register we want to read from
	//if ((write(sensor->fd, buf, 1)) != 1) {				// Send register we want to read from	
//		printf("Error writing to i2c slave (%s)\n", __func__);
	//	return(1);
	//}
	
	// reset sensor
	buf[0] = 0x1E;										// This is the register we want to read from
	if ((write(sensor->fd, buf, 1)) != 1) {				// Send register we want to read from	
		printf("Error writing to i2c slave (%s)\n", __func__);
		return(1);
	}
	
	return(0);
}

/**
* @brief Trigger temperature measurement at MS5611 pressure sensor
* @param sensor pointer to sensor instance
* @return result
*
* @date 24.03.2016 revised
*
*/ 
int ms5611_start_temp(t_ms5611 *sensor)
{
	//variables
	unsigned char buf[10]={0x00};

	// start conversion for D2
	buf[0] = 0x58;										// This is the register we want to read from
	if ((write(sensor->fd, buf, 1)) != 1) {				// Send register we want to read from	
		printf("Error writing to i2c slave (%s)\n", __func__);
		return(1);
	}
	
	return(0);
}

/**
* @brief Trigger pressure measurement at MS5611 pressure sensor
* @param sensor pointer to sensor instance
* @return result
*
* @date 24.03.2016 revised
*
*/ 
int ms5611_start_pressure(t_ms5611 *sensor)
{
	//variables
	//struct timespec sample_time;
	uint8_t buf[10]={0x00};
	
	// start conversion for D1
	buf[0] = 0x48;													// This is the register we want to read from
	if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave: start conv: adr %x\n",sensor->address);
		return(1);
	}
	
	return(0);
}

/**
* @brief Read temperature from MS5611 sensor
* @param sensor pointer to sensor instance
* @return result
*
* @date 24.03.2016 revised
*
*/ 
int ms5611_read_temp(t_ms5611 *sensor, int glitch)
{
	//variables
	uint8_t buf[10]={0x00};
	//variables
	//long d2;
	int64_t OFF2=0;
	int64_t SENS2=0;
	int64_t T2=0;
	
	// read result
	buf[0] = 0x00;
	if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}
	
	if (read(sensor->fd, buf, 3) != 3) {								// Read back data into buf[]
		printf("Unable to read from slave(%s)\n", __func__);
		return(1);
	}
	
	// Put temperature readings together
	sensor->D2l = sensor->D2;
	sensor->D2 = (buf[0] << 16) + (buf[1] << 8) + buf[2];
	if (glitch==0)
		if ((sensor->D2<=sensor->D2l+100e3) && (sensor->D2l<=sensor->D2+300)) 
		{
			sensor->D2f = (sensor->D2f*7+sensor->D2)/8;

			// calculate dT and absolute temperature
			sensor->dT = sensor->D2f - sensor->C5s;
			sensor->temp = 2000 + (((int64_t)sensor->dT * sensor->C6) / 8388608);
			
			// these calculations are copied from the data sheet
			//OFF = C2 * 2**16 + (C4 * dT) / 2**7
			//SENS = C1 * 2**15 + (C3 * dT) / 2**8
			//P = (D1 * SENS / 2**21 - OFF) / 2**15

			sensor->off = (sensor->C2s + (((int64_t)sensor->C4 * sensor->dT) >> 7));
			sensor->sens = (sensor->C1s + (((int64_t)sensor->C3 * sensor->dT) >> 8));

			if (sensor->secordcomp)
			{
				// second order correction
				if (sensor->temp < 2000)
				{
					T2 = (sensor->dT * sensor->dT) >> 31;
					OFF2 = 5 * ((int64_t)(sensor->temp - 2000) * (sensor->temp - 2000)) >> 1;
					SENS2 = 5 * ((int64_t)(sensor->temp - 2000) * (sensor->temp - 2000)) >> 2;

					if (sensor->temp < -1500)
					{
						OFF2 = OFF2 + 7 * ((int64_t)(sensor->temp + 1500) * (sensor->temp + 1500));
						SENS2 = SENS2 + ((11 * ((int64_t)(sensor->temp + 1500) * (sensor->temp + 1500))) >> 1);
					}

					sensor->temp = sensor->temp - T2;
					sensor->off = sensor->off - OFF2;
					sensor->sens = sensor->sens - SENS2;
				}
			}
		}


	// debug print
	ddebug_print("%s @ 0x%x: D2 = %u\n", __func__, sensor->address, sensor->D2);
	ddebug_print("%s @ 0x%x: dT = %d\n", __func__, sensor->address, sensor->dT);
	debug_print("%s @ 0x%x: temp = %d\n", __func__, sensor->address, sensor->temp);
	
	return(0);
}

/**
* @brief Read pressure from MS5611 sensor
* @param sensor pointer to sensor instance
* @return result
*
* @date 24.03.2016 revised
*
*/ 
int ms5611_read_pressure(t_ms5611 *sensor)
{
	//variables
	//long d2;
	uint8_t buf[10]={0x00};
	
	// read result
	buf[0] = 0x00;
	if ((write(sensor->fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave: write Read result(%s)\n", __func__);
		return(1);
	}
	
	if (read(sensor->fd, buf, 3) != 3) {								// Read back data into buf[]
		printf("Unable to read from slave: read result(%s)\n", __func__);
		return(1);
	}
	
	// put pressure reading together
	sensor->D1l = sensor->D1;
	sensor->D1 = (buf[0] << 16) + (buf[1] << 8) + buf[2];

	return 0;
}

int ms5611_calculate_pressure(t_ms5611 *sensor) 
{
	sensor->p_meas = (((sensor->D1 * sensor->sens) >> 21) - sensor->off) >> 11;
	
	// check for valid range
	if ((sensor->temp > -4000) && (sensor->temp < 8500) && (sensor->p_meas > 16000) && (sensor->p_meas < 1920000))
	{
		// correct measured pressure
		sensor->p = (sensor->linearity * (float)(sensor->p_meas + sensor->offset))*.0625;
		
		// some debugging output
		ddebug_print("%s @ 0x%x: D1: %u\n", __func__, sensor->address, sensor->D1);
		ddebug_print("%s @ 0x%x: OFF: %lld\n", __func__, sensor->address, sensor->off);
		ddebug_print("%s @ 0x%x: SENS: %lld\n", __func__, sensor->address, sensor->sens);	
		debug_print("%s @ 0x%x: Pressure measured: %ld, %f\n", __func__, sensor->address, (unsigned long)sensor->p_meas, sensor->p);
		return(0);
	}
	else
	{
		// TODO add error handling here !!
		return(1);
	}
}
