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

#include "cmdline_parser.h"
#include "version.h"
#include "log.h"

#include <stdlib.h>
#include <unistd.h>
#include <string.h>

char config_filename[50];

bool g_foreground = false;
bool g_inetd = false;
bool g_secordcomp = false;
bool tj = false;

FILE *fp_sensordata=NULL;
FILE *fp_datalog=NULL;
FILE *fp_config=NULL;

void cmdline_parser(int argc, char **argv, t_io_mode *io_mode){

	// locale variables
	int c;
	char datalog_filename[50];
	char sensordata_filename[50];

	const char* Usage = "\n"\
	"  -v              print version information\n"\
	"  -f              don't daemonize, stay in foreground\n"\
	"  -i              inetd mode\n"\
	"  -c [filename]   use config file [filename]\n"\
	"  -d[n]           set debug level. n can be [1..2]. default=1\n"\
   	"  -j              turn on timing jitter... for testing only!\n"\
	"  -r [filename]   record measurement values to file\n"\
	"  -s              second order temperature compensation for MS5611 enable\n"\
	"  -p [filename]   use values from file instead of measuring\n"\
	"\n";

	// check commandline arguments
	while ((c = getopt (argc, argv, "vd::fijlhr:p:c:s")) != -1)
	{
		switch (c) {
			case 'v':
				//sprintf(s, "sensord V0.1 %s %s", __DATE__
				printf("sensord V%c.%c RELEASE %c build: %s %s %s\n", VERSION_MAJOR, VERSION_MINOR, VERSION_RELEASE, VERSION_GIT, __DATE__, __TIME__);
				printf("sensord  Copyright (C) 2014  see AUTHORS on www.openvario.org\n");
				printf("This program comes with ABSOLUTELY NO WARRANTY;\n");
				printf("This is free software, and you are welcome to redistribute it under certain conditions;\n");
				break;

			case 'c':
				// use config file
				// record sensordata for replay
				if (optarg == NULL)
				{
					fprintf(stderr, "Missing option for -c\n");
					fprintf(stderr, "Exiting ...\n");
					exit(EXIT_FAILURE);
				}
				else
				{
					strcpy(config_filename, optarg);
					fprintf(stderr, "!! Using config file %s !!\n", config_filename);

					// Open the fp to config file
					fp_config = fopen(config_filename,"r");

					//check if config file opened ok ...
					if( fp_config == NULL)
					{
						fprintf(stderr, "Error opening config file: %s\n", config_filename);
						fprintf(stderr, "Exiting ...\n");
						exit(EXIT_FAILURE);
					}
				}
				break;

			case 'd':
				if (optarg == NULL)
				{
					g_debug = 1;
				}
				else
					g_debug = atoi(optarg);

				fprintf(stderr, "!! DEBUG LEVEL %d !!\n",g_debug);
				break;

			case 'f':
				// don't daemonize
				fprintf(stderr, "!! STAY in g_foreground !!\n");
				g_foreground = true;
				break;

			case 'i':
				// inetd mode
				g_foreground = true;
				g_inetd = true;
				break;

			case 'j':
				// Cause deliberate timing jitter
				fprintf(stderr, "!! Timing jitter on !!\n");
				tj = true;
				break;

			case 's':
				// enable second order compensation
				fprintf(stderr, "Second order compensation ENABLE\n");
				g_secordcomp = true;
				break;

			case 'r':
				// record sensordata for replay
				if (optarg == NULL)
				{
					fprintf(stderr, "Missing option for -r\n");
					fprintf(stderr, "Exiting ...\n");
					exit(EXIT_FAILURE);
				}

				io_mode->sensordata_to_file = true;
				strcpy(datalog_filename, optarg);
				fprintf(stderr, "!! RECORD DATA TO %s !!\n", datalog_filename);

				// Open the fp to record file
				fp_datalog = fopen(datalog_filename,"w+");
				break;

			case 'p':
				// replay sensordata instead of measuring
				if (optarg == NULL)
				{
					fprintf(stderr, "Missing option for -p\n");
					fprintf(stderr, "Exiting ...\n");
					exit(EXIT_FAILURE);
				}

				io_mode->sensordata_from_file = true;
				strcpy(sensordata_filename, optarg);
				fprintf(stderr, "!! REPLAY DATA FROM %s !!\n", sensordata_filename);
				// Open the fp to replay file
				fp_sensordata = fopen(sensordata_filename,"r");
				if (fp_sensordata == NULL)
				{
					fprintf(stderr, "Error opening file %s for replay !!\n", sensordata_filename);
					fprintf(stderr, "Exiting ...\n");
					exit(EXIT_FAILURE);
				}
				break;

			case '?':
				fprintf(stderr, "Unknown option %c\n", optopt);
				fprintf(stderr, "Usage: sensord [OPTION]\n%s",Usage);
				fprintf(stderr, "Exiting ...\n");
				exit(EXIT_FAILURE);
				break;
		}
	}
}

