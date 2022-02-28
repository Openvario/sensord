/*  sensorcal - Sensor Calibration for Openvario Sensorboard - http://www.openvario.org/
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

#include "sensorcal.h"
#include "24c16.h"
#include "ams5915.h"
#include "wait.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

int g_debug=0;
FILE *fp_console=NULL;

static int calibrate_ams5915(t_eeprom_data* data)
{
	t_ams5915 dynamic_sensor;
	float offset=0.0;
	int i;

	// open sensor for differential pressure
	/// @todo remove hardcoded i2c address for differential pressure
	printf("Open sensor ...");
	if (ams5915_open(&dynamic_sensor, 0x28) != 0)
	{
		printf(" failed !!\n");
		return 1;
	}
	else
	{
		printf(" success!\n");
	}

	dynamic_sensor.offset = 0.0;
	dynamic_sensor.linearity = 1.0;

	//initialize differential pressure sensor
	ams5915_init(&dynamic_sensor);
	usleep(112500);
	sensor_wait_mark();

	for (i=0;i<800;i++)
	{
		// read AMS5915
		sensor_wait(12500);
		ams5915_measure(&dynamic_sensor);
		sensor_wait_mark();
		ams5915_calculate(&dynamic_sensor);

		// wait some time ...
		printf("Measured: %f\n",dynamic_sensor.p);

		// calc offset
		offset += dynamic_sensor.p;
	}

	data->zero_offset = -1*(offset/800);

	return(0);
}

int main (int argc, char **argv) {

	// local variables
	t_24c16 eeprom;
	t_eeprom_data data;

	int exit_code=0;
	int result;
	int c;
	int i;
	char sn[7];
	char zero[1]={0x00};


	// usage message
	const char* Usage = "\n"\
	"  -c              calibrate sensors\n"\
	"  -s [serial]     write serial number (6 characters)\n"\
    "  -i              initialize EEPROM. All values will be cleared !!!\n"\
	"\n";

	// disable buffering
	setbuf(stdout, NULL);

	// print banner
	printf("sensorcal V%c.%c RELEASE %c build: %s %s\n", VERSION_MAJOR, VERSION_MINOR, VERSION_RELEASE,  __DATE__, __TIME__);
	printf("sensorcal Copyright (C) 2014  see AUTHORS on www.openvario.org\n");
	printf("This program comes with ABSOLUTELY NO WARRANTY;\n");
	printf("This is free software, and you are welcome to redistribute it under certain conditions;\n");

	// open eeprom object
	result = eeprom_open(&eeprom, 0x50);
	if (result != 0)
	{
		printf("No EEPROM found !!\n");
		exit(1);
	}

	sn[6]=0;
	// check commandline arguments
	while ((c = getopt (argc, argv, "his:cde")) != -1)
	{
		switch (c) {
			case 'h':
				printf("Usage: sensorcal [OPTION]\n%s",Usage);
				break;

			case 'i':
				printf("Initialize EEPROM ...\n");
				for (i=0; i<128; i++)
				{
					result = eeprom_write(&eeprom, &zero[0], i, 1);
				}
				strcpy(data.header, "OV");
				data.data_version = EEPROM_DATA_VERSION;
				memset(data.serial,'0',6);
				data.zero_offset=0.0;
				update_checksum(&data);
				printf("Writing data to EEPROM ...\n");
				result = eeprom_write(&eeprom, (char*)&data, 0x00, sizeof(data));
				break;

			case 'c':
				// read actual EEPROM values
				printf("Reading EEPROM values ...\n\n");
				if( eeprom_read_data(&eeprom, &data) == 0)
				{
					calibrate_ams5915(&data);
					printf("New Offset: %f\n",(data.zero_offset));
					update_checksum(&data);
					printf("Writing data to EEPROM ...\n");
					result = eeprom_write(&eeprom, (char*)&data, 0x00, sizeof(data));
				}
				else
				{
					printf("EEPROM content not valid !!\n");
					printf("Please use -i to initialize EEPROM !!\n");
					exit_code=2;
					break;
				}
				break;
			case 'e':
				// delete complete EEPROM
				printf("Delete whole EEPROM ...\n\n");
				for (i=0; i< sizeof(data); i++)
				{
						result = eeprom_write(&eeprom, &zero[0], 0x00, 1);
				}
				printf("EEPROM cleared !!\n");
				exit_code=3;
				printf("End ...\n");
				exit(exit_code);
				break;
			case 'd':
				// read actual EEPROM values
				printf("Reading EEPROM values ...\n\n");
				if( eeprom_read_data(&eeprom, &data) == 0)
				{
				  memcpy(sn,data.serial,6);
					printf("Actual EEPROM values:\n");
					printf("---------------------\n");
					printf("Serial: \t\t\t%s\n", sn);
					printf("Differential pressure offset:\t%f\n",data.zero_offset);
				}
				else
				{
					printf("EEPROM content not valid !!\n");
					printf("Please use -i to initialize EEPROM !!\n");
					exit_code=2;
					break;
				}
				printf("End ...\n");
				exit(exit_code);
				break;

			case 's':
				if( strlen(optarg) == 6)
				{
					// read actual EEPROM values
					if( eeprom_read_data(&eeprom, &data) == 0)
					{
						for(i=0; i<6;i++)
						{
							sn[i]=data.serial[i]=*optarg;
							optarg++;
						}
						printf("New Serial number: %s\n",sn);
						update_checksum(&data);
						printf("Writing data to EEPROM ...\n");
						result = eeprom_write(&eeprom, (char*)&data, 0x00, sizeof(data));
					}
					else
					{
						printf("EEPROM content not valid !!\n");
						printf("Please use -i to initialize EEPROM !!\n");
						exit_code=2;
						break;
					}
				}
				else
				{
					printf("ERROR: Serialnumber has to have exactly 6 characters !!\n");
					exit_code=1;
					break;
				}
				break;

			case '?':
				printf("Unknow option %c\n", optopt);
				printf("Usage: sensorcal [OPTION]\n%s",Usage);
				printf("Exiting ...\n");
		}
	}

	printf("End ...\n");
	return(exit_code);
}

