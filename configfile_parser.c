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
#include <stdlib.h>
#include <string.h>

#include "def.h"

#include "ms5611.h"
#include "ams5915.h"
#include "ads1110.h"
#include "ds2482.h"
#include "configfile_parser.h"

extern int g_debug;
extern FILE *fp_console;

int cfgfile_parser(FILE *fp, t_ms5611 *static_sensor, t_ms5611 *tek_sensor, t_ams5915 *dynamic_sensor, t_ads1110 *voltage_sensor, t_ds2482 *temp_sensor, t_config *config)
{
	char line[70];
	char tmp[20];
		
	// is config file used ??
	if (fp)
	{
		// read whole config file
		while (! feof(fp))
		{
			// get line from config file
			fgets(line, 70, fp);
			//printf("getting line: '%69s'\n", line);
			
			// check if line is comment
			if((!(line[0] == '#')) && (!(line[0] == '\n')))
			{
			
				// get first config tag
				sscanf(line, "%19s", tmp);
				
				// check for output of POV_E sentence
				if (strcmp(tmp,"output_POV_E") == 0)
				{	
					config->output_POV_E = 1;
					//printf("OUTput POV_E enabled !! \n");
				}
				
				// check for output of POV_P_Q sentence
				if (strcmp(tmp,"output_POV_P_Q") == 0)
				{	
					config->output_POV_P_Q = 1;
					//printf("OUTput POV_P_Q enabled !! \n");
				}
				
				// check for output of POV_V sentence
				if (strcmp(tmp,"output_POV_V") == 0)
				{	
					config->output_POV_V= 1;
					//printf("OUTput POV_V enabled !! \n");
				}

				// check for output of POV_T sentence
				if (strcmp(tmp,"output_POV_T") == 0)
				{	
					config->output_POV_T= 1;
					//printf("OUTput POV_T enabled !! \n");
				}
				
				// check for static_sensor
				if (strcmp(tmp,"static_sensor") == 0)
				{
					// get config data for static sensor
					sscanf(line, "%19s %f %f", tmp, &static_sensor->offset, &static_sensor->linearity);
				}
				
				// check for tek_sensor
				if (strcmp(tmp,"tek_sensor") == 0)
				{
					// get config data for tek sensor
					sscanf(line, "%19s %f %f", tmp, &tek_sensor->offset, &tek_sensor->linearity);	
				}
				
				// check for dynamic_sensor
				if (strcmp(tmp,"dynamic_sensor") == 0)
				{
					// get config data for dynamic sensor
					sscanf(line, "%19s %f %f", tmp, &dynamic_sensor->offset, &dynamic_sensor->linearity);
				}
				
				// check for vario config
				if (strcmp(tmp,"vario_config") == 0)
				{
					// get config data for dynamic sensor
					sscanf(line, "%19s %f", tmp, &config->vario_x_accel);
				}
				
				// check for voltage sensor config
				if (strcmp(tmp,"voltage_config") == 0)
				{
					// get config data for dynamic sensor
				        sscanf(line, "%19s %f %f", tmp, &voltage_sensor->scale,&voltage_sensor->offset);
				        voltage_sensor->scale=1.0/voltage_sensor->scale;
				}

				// check for temp config
				if (strcmp(tmp,"temp_config") == 0) {
				  // get config data for temperature sensor
				  sscanf(line, "%19s %d", tmp, &temp_sensor->databits);
				  if ((temp_sensor->databits<9) || (temp_sensor->databits>12)) {
				    printf ("Temp sensor databits (%d) is not valid, must be in the range of 9-12.  Using default value of 10 bits.\n",temp_sensor->databits);
				    temp_sensor->databits=10;
				  }
				}
			}
	
			
		}
		return(1);
	}
	else
	{
		// no config file used
		return (0);
	}
}
