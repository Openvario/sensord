#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "def.h"

#include "ms5611.h"
#include "ams5915.h"

extern int g_debug;
extern FILE *fp_console;

int cfgfile_parser(FILE *fp, t_ms5611 *static_sensor, t_ms5611 *tek_sensor, t_ams5915 *dynamic_sensor)
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
			
			// check if line is comment
			if(!(line[0] == '#') || (line[0] == '\n'))
			{
				// get first config tag
				sscanf(line, "%s", tmp);
				
				// check for static_sensor
				if (strcmp(tmp,"static_sensor") == 0)
				{
					// get config data for static sensor
					sscanf(line, "%s %f %f", tmp, &static_sensor->offset, &static_sensor->linearity);
				}
				
				// check for tek_sensor
				if (strcmp(tmp,"tek_sensor") == 0)
				{
					// get config data for tek sensor
					sscanf(line, "%s %f %f", tmp, &tek_sensor->offset, &tek_sensor->linearity);	
				}
				
				// check for dynamic_sensor
				if (strcmp(tmp,"dynamic_sensor") == 0)
				{
					// get config data for dynamic sensor
					sscanf(line, "%s %f %f", tmp, &dynamic_sensor->offset, &dynamic_sensor->linearity);
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