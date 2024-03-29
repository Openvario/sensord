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
#include "clock.h"

#include <time.h>
#include <stdint.h>

static struct timespec sensor_prev;

void sensor_wait_mark(void)
{
	clock_gettime(CLOCK_MONOTONIC, &sensor_prev);
}

float sensor_wait (float time)
{
	struct timespec until = sensor_prev;
	timespec_add_us(&until, time);
	while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &until, NULL)) {}

	struct timespec curtime;
	clock_gettime(CLOCK_MONOTONIC, &curtime);
	return timespec_delta_us(&curtime, &sensor_prev) - time;
}
