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