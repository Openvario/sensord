
/*  
	sensord - Sensor Interface for XCSoar Glide Computer - http://www.openva
rio.org/
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

// define struct for DS2482 sensor

#define AUTO 0
#define HTU21D 1
#define HTU31D 2
#define SI7021 3
#define DS18B20 4
#define AM2321 5
#define SHT4X 6
#define SHT85 7

typedef struct {
	int fd;
	unsigned char address;
	unsigned long int owDeviceAddress[2]; // should init to 0
	int owTriplet; // Should be init to 4
	int owLastDevice; // should init to 0
	int owLastDiscrepancy; // should init to 0
	float temperature;
	float humidity;
	char temp_present;
	char humidity_present;
	char temp_valid;
	char humidity_valid;
	char sensor_type;
	int conversion_time;
	int delta_conversion_time;
	int databits;
	int humidity_databits;
	int configbits;
	int rollover;
	int maxrollover;
	int compensate;
} t_ds2482;

int ds2482_open(t_ds2482 *, unsigned char);
int ds2482_reset(t_ds2482 *);
int OWReset(t_ds2482 *);
int OWWriteByte(t_ds2482 *, unsigned char);
int OWReadByte(t_ds2482 *);
int OWTriplet(t_ds2482 *);
int OWSearch(t_ds2482 *);
int OWCheckCRC(t_ds2482 *);
int AddCRC(int, int);
int OWSelect(t_ds2482 *);
int OWConfigureBits (t_ds2482 *);
int OWReadTemperature(t_ds2482 *);
