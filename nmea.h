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

#ifndef NMEA_H
#define NMEA_H 

unsigned char NMEA_checksum(char *);
int Compose_Pressure_POV_slow(char *, float, float);
int Compose_Pressure_POV_fast(char *, float);
int Compose_Voltage_POV(char *sentence, float voltage);
int Compose_Temperature_POV(char *sentence, float temperature);

#endif
