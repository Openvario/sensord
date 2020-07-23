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

typedef struct {
	char output_POV_E;
	char output_POV_P_Q;
	char output_POV_V;
	char output_POV_T;
	float vario_x_accel;
} t_config;

int cfgfile_parser(FILE *, t_ms5611 *, t_ms5611 *, t_ams5915 *, t_ads1110 *, t_ds2482 *, t_config *);
