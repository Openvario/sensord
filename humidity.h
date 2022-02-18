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

// This header file is for si7021.c which supports the following sensors:
// Sensirion: SHT40, SHT41, SHT45, SHT85 (SHT71 and SHT75 may work)
// Silicon Labs: SI7006, SI7013, SI7020, SI7021
// Tyco Electronics/Measurement Specialties: HTU21D, HTU31D
//
// Note: The only sensor that has been tested is the HTU21D, the routines
// support the use of the heater function, but it is NOT used by sensord

int si7021_crc_check (unsigned int, uint8_t);
int sht4x_open (t_ds2482 *, unsigned char);
int si7021_open (t_ds2482 *, unsigned char);
int si7021_start_humidity (t_ds2482 *);
int si7021_start_temp (t_ds2482 *);
int si7021_read_temp (t_ds2482 *);
int si7021_read_humidity (t_ds2482 *);
int si7021_configure_heater_value (t_ds2482 *, int);
int si7021_configure_heater_onoff (t_ds2482 *, int);
int am2321_open (t_ds2482 *, unsigned char);
int am2321_read (t_ds2482 *);
int am2321_wakeup (t_ds2482 *);

