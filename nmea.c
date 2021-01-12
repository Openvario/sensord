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
#include <string.h>
#include "nmea.h"
#include "def.h"

extern int g_debug;
extern FILE *fp_console;
 	
/**
* @brief Implements the $POV NMEA Sentence for pressure data
* @param sentence char pointer for created string
* @param static_pressure
* @param dynamic_pressure
* @param tek_pressure // not implemented yet !!
* @return result
* 
* Implementation of the properitary NMEA sentence for AKF Glidecomputer
* \n
*
*     $POV,P,static_pressure,Q,dynamic_pressure*CRC
*       |  |        |        |         |         
*       1  2        3        4         5   
*
*     1: $P            		Properitary NMEA Sentence
*        OV         		Manufacturer Code: OpenVario
*
*     2: P               	Code for static pressure in hPa
*     
*     3: static_pressure    Format: 1013.34
*                     		Range: 0..1500 
*     
*     4: Q                  Code for dynamic pressure in Pa
*
*     5: dynamic_pressure   Format: 12.34
* 							Range: -999 .. 999
*   
*
* @date 23.02.2014 born
*
*/ 
		
int Compose_Pressure_POV_slow(char *sentence, float static_pressure, float dynamic_pressure)
{
	int length;
	int success = 1;

	// check static_pressure input value for validity
	if ((static_pressure < 0) || (static_pressure > 2000))
	{
		static_pressure = 9999;
		success = 10;
	}
	
	// check dynamic_pressure input value for validity
	/// @todo add useful range for dynamic_pressure vales !!
	if ((dynamic_pressure < -999.0) || (dynamic_pressure > 9998.0))
	{
		dynamic_pressure = 9999;
		success = 20;
	}

	// compose NMEA String
	length = sprintf(sentence, "$POV,P,%+07.4f,Q,%+05.2f", static_pressure, (dynamic_pressure)); 
	
	// Calculate NMEA checksum and add to string
	sprintf(sentence + length, "*%02X\r\n", NMEA_checksum(sentence));
	
	//print sentence for debug
	debug_print("POV slow NMEA sentence: %s\n", sentence);
	return (success);
}

/**
* @brief Implements the $POV NMEA Sentence for fast data
* @param sentence char pointer for created string
* @param TE vario
* @return result
* 
* Implementation of the properitary NMEA sentence for AKF Glidecomputer
* \n
*
*     $POV,E,TE_vario*CRC
*       |  |    |      |
*       1  2    3      4
*
*     1: $P            		Properitary NMEA Sentence
*        OV         		Manufacturer Code: OpenVario
*
*     2: E               	Code for TE vario in m/s
*     
*     3: TE vario           Format: 3.4
*
* @date 09.03.2014 born
*
*/ 
		
int Compose_Pressure_POV_fast(char *sentence, float te_vario)
{
	int length;
	int success = 1;

	// check te_vario input value for validity
	if ((te_vario < -50) || (te_vario > 50))
	{
		te_vario = 99;
		success = 10;
	}
	
	// compose NMEA String
	length = sprintf(sentence, "$POV,E,%+05.4f", te_vario); 
	
	// Calculate NMEA checksum and add to string
	sprintf(sentence + length, "*%02X\r\n", NMEA_checksum(sentence));
	
	//print sentence for debug
	debug_print("NMEA sentence: %s\n", sentence);
	return (success);
}

/**
* @brief Implements the $POV NMEA Sentence for voltage data
* @param sentence char pointer for created string
* @param Battery Voltage
* @return result
* 
* Implementation of the properitary NMEA sentence for AKF Glidecomputer
* \n
*
*     $POV,V,Bat_voltage*CRC
*       |  |      |       |
*       1  2      3       4
*
*     1: $P            		Properitary NMEA Sentence
*        OV         		Manufacturer Code: OpenVario
*
*     2: V               	Code for Battery Voltage in V
*     
*     3: Battery Voltage           Format: 12.02
*
* @date 13.03.2016 born
*
*/ 

int Compose_Voltage_POV(char *sentence, float voltage)
{
	int length;
	int success = 1;

	// check voltage input value for validity
	if ((voltage < 2.) || (voltage > 20.))
	{
		voltage = 0.;
		success = 10;
	}
	
	// compose NMEA String
	length = sprintf(sentence, "$POV,V,%+05.2f", voltage); 
	
	// Calculate NMEA checksum and add to string
	sprintf(sentence + length, "*%02X\r\n", NMEA_checksum(sentence));
	
	//print sentence for debug
	debug_print("NMEA sentence: %s\n", sentence);
	return (success);
}

/**
* @brief Implements the $POV NMEA Sentence for voltage data
* @param sentence char pointer for created string
* @param Battery Voltage
* @return result
* 
* Implementation of the properitary NMEA sentence for AKF Glidecomputer
* \n
*
*     $POV,T,Temperature*CRC
*       |  |      |       |
*       1  2      3       4
*
*     1: $P            		Properitary NMEA Sentence
*        OV         		Manufacturer Code: OpenVario
*
*     2: T               	Code for Temperature in C
*     
*     3: Temperature        Format: 27.125
*
* @date 13.03.2016 born
*
*/ 

int Compose_Temperature_POV(char *sentence, float temperature)
{
	int length;
	int success = 1;

 	// check voltage input value for validity
 	if ((temperature < -55) || (temperature > 125)) {
 		temperature = 23;
 		success = 10;
	}

	// compose NMEA String
	length = sprintf(sentence, "$POV,T,%+05.4f", temperature); 

	// Calculate NMEA checksum and add to string
	sprintf(sentence + length, "*%02X\r\n", NMEA_checksum(sentence));

	//print sentence for debug
	debug_print("NMEA sentence: %s\n", sentence);
	return (success);
}

/**
* @brief Implements the NMEA Checksum
* @param char* NMEA string
* @return int Calculated Checksum for string
* 
* The NMEA Checksum is calulated over the NMEA string between $ and *\n
* The checksum is a hexadecimal number representing the XOR of all bytes
*/

unsigned char NMEA_checksum(char *string)
{
  unsigned char value=0;
	int i=1;
	int l;
	
	l = strlen(string);
	
	for (; i < l; i++)
	{
    value ^= string[i];
  }
  return value;
}
