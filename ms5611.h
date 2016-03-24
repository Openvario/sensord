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

#include <time.h>
#include <stdint.h>

// variable definitions

// define struct for MS5611 sensor
typedef struct {
	int fd;
	unsigned char address;
	uint32_t C1s;
	uint32_t C2s;
	uint16_t C3;
	uint16_t C4;
	uint32_t C5s;
	uint16_t C6;
	uint32_t D1;
	uint32_t D2;
	int32_t dT;
	int32_t temp;
	int64_t off;
	int64_t sens;
	int32_t p_meas;
	float p;
	float linearity;
	float offset;
	int valid;
	struct timespec sample;
	struct timespec last_sample;
} t_ms5611;

// prototypes
int ms5611_init(t_ms5611 *);
int ms5611_reset(t_ms5611 *);
int ms5611_measure(t_ms5611 *);
int ms5611_calculate(t_ms5611 *);
int ms5611_open(t_ms5611 *, unsigned char);

int ms5611_read_pressure(t_ms5611 *);
int ms5611_read_temp(t_ms5611 *);
int ms5611_start_temp(t_ms5611 *);
int ms5611_start_pressure(t_ms5611 *);