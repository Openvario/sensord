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

// define struct for DS2482+DS18B20 sensor
typedef struct {
	float celsius;
	unsigned char present;
	
	int fd;
	unsigned char address;
} t_ds2482;

// prototypes
int ds2482_open(t_ds2482 *, unsigned char);
int ds2482_init(t_ds2482 *);
int ds2482_start(t_ds2482 *);
int ds2482_measure(t_ds2482 *);

unsigned char ds2482_reset(t_ds2482 *);
unsigned char ds2482_get_status(t_ds2482 *);
unsigned char ds2482_read(t_ds2482 *, unsigned char *buf, unsigned int dataLength);
unsigned char ds2482_write(t_ds2482 *, unsigned char *buf, unsigned int dataLength);

int OWReset(t_ds2482 *);
int OWWriteByte(t_ds2482 *, unsigned char byte);
int OWReadByte(t_ds2482 *) ;
int OWWaitForBusyEnd(t_ds2482 *);
void OWReadTemperature(t_ds2482 *);
