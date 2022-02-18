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

#include "wait.h"

#include <time.h>

#define DELTA_TIME_US(T1, T2)   (((T1.tv_sec+1.0e-9*T1.tv_nsec)-(T2.tv_sec+1.0e-9*T2.tv_nsec))*1000000)

struct timespec sensor_prev;

float sensor_wait (float time)
{
	struct timespec curtime;

	clock_gettime(CLOCK_REALTIME,&curtime);
	curtime.tv_nsec=((long int) time)*1000 - (curtime.tv_nsec+(curtime.tv_sec-sensor_prev.tv_sec)*1e9 - sensor_prev.tv_nsec);
	if (curtime.tv_nsec>1e9) {
		curtime.tv_sec=curtime.tv_nsec/1e9;
		curtime.tv_nsec-=curtime.tv_sec*1e9;
	} else {
		curtime.tv_sec=0;
		if (curtime.tv_nsec<0) curtime.tv_nsec=0;
	}
	while (nanosleep(&curtime,&curtime)) ;
	clock_gettime(CLOCK_REALTIME,&curtime);
	return (DELTA_TIME_US(curtime,sensor_prev)-time);
}
