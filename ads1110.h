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

// define struct for AMS5915 sensor
typedef struct {
	float scale;
        float offset;
	int voltage_raw;
	float voltage_converted;
	int fd;
	unsigned char address;
	unsigned char present;
} t_ads1110;

// prototypes
int ads1110_init(t_ads1110 *);
int ads1110_measure(t_ads1110 *);
int ads1110_calculate(t_ads1110 *);
int ads1110_open(t_ads1110 *, unsigned char);
