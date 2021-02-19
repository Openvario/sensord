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
#include <math.h>
#include <ctype.h>

#include "def.h"

#include "ds2482.h"
#include "ms5611.h"
#include "ams5915.h"
#include "ads1110.h"
#include "configfile_parser.h"

extern int g_debug;
extern FILE *fp_console;

int cfgfile_parser(FILE *fp, t_ms5611 *static_sensor, t_ms5611 *tek_sensor, t_ams5915 *dynamic_sensor, t_ads1110 *voltage_sensor, t_ds2482 *temp_sensor, t_config *config)
{
	char line[70];
	char tmp[20],tmp2[20];
	float data;
		
	// is config file used ??
	if (fp)
	{
		// read whole config file
		while (! feof(fp))
		{
			// get line from config file
			if (fgets(line, 70, fp)!=NULL) {
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
                                	        config->output_POV_T = 1;
                                        	//printf("OUTput POV_T enabled !! \n");
	                                }
	
                                       // check for output of POV_H sentence
                                        if (strcmp(tmp,"output_POV_H") == 0)
                                        {
                                                config->output_POV_H = 1;
                                                //printf("OUTput POV_H enabled !! \n");
                                        }

					// check for static_sensor
					if (strcmp(tmp,"static_sensor") == 0)
					{
						// get config data for static sensor
						sscanf(line, "%19s %f %f", tmp, &static_sensor->offset, &static_sensor->linearity);
						static_sensor->offset*=16;
					}

					// check for temp_sensor_type
					if (strcmp(tmp,"temp_sensor_type") == 0)
					{
						// get config data for temp sensor type
						sscanf(line, "%19s %19s", tmp, tmp2);
						char *s = tmp2;
						while (*s) {
							*s = toupper((unsigned char) *s);
							s++;
						}
						if (!strcmp(tmp2,"AUTO")) temp_sensor->sensor_type=AUTO;
						else if (!strcmp(tmp2,"HTU21D")) temp_sensor->sensor_type=HTU21D;
							else if (!strcmp(tmp2,"HTU31D")) temp_sensor->sensor_type=HTU31D;
								else if (!strcmp(tmp2,"SHT85")) temp_sensor->sensor_type=SHT85;
									else if (!strcmp(tmp2,"SHT4X")) temp_sensor->sensor_type=SHT4X;
										else if (!strcmp(tmp2,"SI7021")) temp_sensor->sensor_type=SI7021;
											else if (!strcmp(tmp2,"DS18B20")) temp_sensor->sensor_type=DS18B20;
												else if (!strcmp(tmp2,"AM2321")) temp_sensor->sensor_type=AM2321;
													else printf ("Invalid sensor type.  Supported types are: auto, htu21d, htu31d, sht4x, sht85, si7021, ds18b20, and am2321 !!\n");
					}

					if (strcmp(tmp,"static_comp") == 0)
					{
						// get compensation data for static sensor
						sscanf(line, "%19s %lf %lf %lf",tmp, &static_sensor->comp2, &static_sensor->comp1, &static_sensor->comp0);
					}
	
					if (strcmp(tmp,"static_Pcomp") == 0)
					{
						// get pressure compensation data for static sensor
						sscanf(line, "%19s %lf %lf %lf",tmp, &static_sensor->Pcomp2, &static_sensor->Pcomp1, &static_sensor->Pcomp0);
					}
	
					// check for tek_sensor
					if (strcmp(tmp,"tek_sensor") == 0)
					{
						// get config data for tek sensor
						sscanf(line, "%19s %f %f", tmp, &tek_sensor->offset, &tek_sensor->linearity);
						tek_sensor->offset*=16;
					}
	
					if (strcmp(tmp,"tek_comp") == 0)
					{
						// get compensation data for tek sensor
						sscanf (line, "%19s %lf %lf %lf",tmp, &tek_sensor->comp2, &tek_sensor->comp1, &tek_sensor->comp0);
					}
	
					if (strcmp(tmp,"tek_Pcomp") == 0)
					{
						// get pressure compensation data for tek sensor
						sscanf(line, "%19s %lf %lf %lf",tmp, &tek_sensor->Pcomp2, &tek_sensor->Pcomp1, &tek_sensor->Pcomp0);
					}
					
					// check for glitch watchdog_timing
					if (strcmp(tmp,"glitch_timing") == 0)
					{
						// get config data for glitch watchdog timer
						sscanf(line, "%19s %lf %lf %lf", tmp, &config->timing_log, &config->timing_mult, &config->timing_off);
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
	
					// check for temperature sensor config
					if (strcmp(tmp,"temp_databits") == 0) {
						// get config data for temperature sensor
						sscanf(line,"%19s %d",tmp, &temp_sensor->databits);
					}
	
					// check for temperature sensor sample rate
					if (strcmp(tmp,"temp_rate") == 0) {
						// get config data for temperature sensor sample rate
						sscanf (line,"%19s %f",tmp, &data);
						temp_sensor->rollover=((int)(round(80.0/data)));
						temp_sensor->maxrollover=temp_sensor->rollover+40;
						if (temp_sensor->maxrollover<100) temp_sensor->maxrollover=100;
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
