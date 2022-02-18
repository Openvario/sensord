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

#include <stdio.h>

double get_temp_ds18b20(void)
{
	FILE *w1_slave_dat;
	char *line = NULL;
	int len=0;

	if ( (w1_slave_dat = fopen("/sys/bus/w1/devices/w1_bus_master1/28-000004f8e2bf/w1_slave","r")) == NULL)
	{
		fprintf(stderr, "\nKonnte Datei nicht öffnen!\n");
    //*temperatur = 0.0;                        // Wert zu 0 setzen
  }

	getline(&line, &len, w1_slave_dat);            // 1. Zeile auslesen, ignor
  getline(&line, &len, w1_slave_dat);            // 2. Zeile auslesen

	printf("W1: %s\n",line);

	fclose( w1_slave_dat );                    // Datei wieder schließen

	return(0.0);

}