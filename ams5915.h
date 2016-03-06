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

// variable definitions

// define struct for AMS5915 sensor
typedef struct {
	int fd;
	unsigned char address;
	int digoutpmin;
	int digoutpmax;
	int digoutp;
	unsigned int digoutT;
	int pmin;
	int pmax;
	float sensp;
	float T;
	float p;
	float linearity;
	float offset;
	int valid;
	struct timespec sample;
	struct timespec last_sample;
} t_ams5915;

// prototypes
int ams5915_init(t_ams5915 *);
int ams5915_measure(t_ams5915 *);
int ams5915_calculate(t_ams5915 *);
int ams5915_open(t_ams5915 *, unsigned char);