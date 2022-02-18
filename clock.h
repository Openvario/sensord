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
#pragma once

#include <time.h>

static inline float timespec_delta_s(const struct timespec *a,
				     const struct timespec *b)
{
	return ((a->tv_sec+1.0e-9*a->tv_nsec)-(b->tv_sec+1.0e-9*b->tv_nsec));
}

static inline float timespec_delta_us(const struct timespec *a,
				      const struct timespec *b)
{
	return timespec_delta_s(a, b) * 1000000;
}
