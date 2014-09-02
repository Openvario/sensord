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
* @brief Implements the $PAFGA NMEA Sentence for fast data
* @param sentence char pointer for created string
* @param roll
* @param pitch
* @param mag_head
* @param g_force
* @return result
* 
* Implementation of the properitary NMEA sentence for AKF Glidecomputer
* $PAFGA represents the fast data, meaning the data with high update rate (5Hz)\n
* \n
*
*     $PAFGA,Roll,Pitch,MagneticHeading,G_Force*CRC
*       |     |    |            |         |     |
*       1     2    3            4         5     6
*    
*     1: $P                 Properitary NMEA Sentence\n
*        AFG                Manufacturer Code: AkaFliegGraz\n
*        A                  Sentence A (fast data)\n
*     
*     2: Roll               Roll Angle in Degree (1 degree resolution ??)\n
*                           Format: +087.5\n
*                           Range: -180 .. 180\n
*     
*     3: Pitch              Pitch Angle in Degree (1 degree resolution ??)\n
*                           Format: +66.5\n
*                           Range: -90 .. +90\n
*     
*     4: MagneticHeading    Magnetic Heading in Degree (0.1 degree resolution ??)\n
*                           Format: 134.4\n
*                           Range: 0 .. 360\n
*     
*     5: G_force            G Force on aircraft in Gs\n
*                           Magnitude of vektor without direction\n
*                           Format: 0.003\n
*                           Range: 0 .. 8\n
*     
*     6: CRC                NMEA Checksum\n
*   
*     Example:
*     $PAFGA,010.5,05.6,080.0,2.300,*CRC
*
* @date 24.11.2013 born
*
*/ 
		
int Compose_PAFGA_sentence(char *sentence, float roll, float pitch, float mag_head, float g_force)
{
	int length;
	int success = 1;

	// check roll input value for validity
	if ((roll < -180.0) || (roll > 180.0))
	{
		roll = 0.0;
		success = 0;
	}
	
	// check pitch input value for validity
	if ((pitch < -90.0) || (pitch > 90.0))
	{
		pitch = 0.0;
		success = 0;
	}
	
	// check mag Heading input value for validity
	if ((mag_head < 0.0) || (mag_head > 360.0))
	{
		mag_head = 0.0;
		success = 0;
	}
	
	// check g_force input value for validity
	if ((g_force < 0.0) || (g_force > 80.0))
	{
		g_force = 0.0;
		success = 0;
	}
	// compose NMEA String
	length = sprintf(sentence, "$PAFGA,%+06.1f,%+06.1f,%+05.1f,%+05.3f", roll, pitch, mag_head,g_force); 
	
	// Calculate NMEA checksum and add to string
	sprintf(sentence + length, "*%02X\n", NMEA_checksum(sentence));
	
	return (success);
}

/**
* @brief Implements the $PAFGA NMEA Sentence for fast data
* @param sentence char pointer for created string
* @param static_pressure
* @param dynamic_pressure
* @param tek_pressure
* @return result
* 
* Implementation of the properitary NMEA sentence for AKF Glidecomputer
* $PAFGB represents the fast data, meaning the data with medium update rate (2Hz)\n
* \n
*
*     $PAFGB,static_pressure,dynamic_pressure,tek_pressure*CRC
*                |           |           |
*                1           2           3
*
*     1: $P            		Properitary NMEA Sentence
*        AFG         		Manufacturer Code: AkaFliegGraz
*        B          		Sentence B (Slow Data)
*
*     2: static_pressure   	Static pressure (mbar 0.01 resolution ??)
*                     		Format: 1013.11
*                     		Range: 450..1100
*     
*     3: dynamic_pressure   dynamic pressure (mbar 0.01 resolution ??)
*     						Format: 23.34
*                     		Range: 0..50 
*     
*     4: tek_pressure 		TEK-pressure (Total-Energy-Kompensated)
*                     		????
*   
*     Example:
*     $PAFGB,,*CRC
*
* @date 24.11.2013 born
*
*/ 
		
int Compose_PAFGB_sentence(char *sentence, float static_pressure, float dynamic_pressure, float tek_pressure)
{
	int length;
	int success = 1;

	// check static_pressure input value for validity
	if ((static_pressure < - 0) || (static_pressure > 2000))
	{
		static_pressure = 999;
		success = 10;
	}
	
	// check dynamic_pressure input value for validity
	/// @todo add useful range for dynamic_pressure vales !!
	if ((dynamic_pressure < -999.0) || (dynamic_pressure > 9999.0))
	{
		dynamic_pressure = 0.0;
		success = 20;
	}
	
	// check tek_pressure Heading input value for validity
	/// @todo add useful range for tek_pressure vales !!
	if ((tek_pressure < 0.0) || (tek_pressure > 9999))
	{
		tek_pressure = 9999;
		success = 30;
	}
	
	// compose NMEA String
	length = sprintf(sentence, "$PAFGB,%+07.2f,%+05.2f,%+05.2f", static_pressure, dynamic_pressure, tek_pressure); 
	
	// Calculate NMEA checksum and add to string
	sprintf(sentence + length, "*%02X\n", NMEA_checksum(sentence));
	
	//print sentence for debug
	debug_print("NMEA sentence: %s\n", sentence);
	return (success);
}
 	
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
		static_pressure = 999;
		success = 10;
	}
	
	// check dynamic_pressure input value for validity
	/// @todo add useful range for dynamic_pressure vales !!
	if ((dynamic_pressure < -999.0) || (dynamic_pressure > 9999.0))
	{
		dynamic_pressure = 0.0;
		success = 20;
	}
		
	// compose NMEA String
	length = sprintf(sentence, "$POV,P,%+07.2f,Q,%+05.2f", static_pressure, (dynamic_pressure)); 
	
	// Calculate NMEA checksum and add to string
	sprintf(sentence + length, "*%02X\n", NMEA_checksum(sentence));
	
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
	/*if ((te_vario < -10) || (te_vario > 10))
	{
		te_vario = 99;
		success = 10;
	}*/
	
	// compose NMEA String
	length = sprintf(sentence, "$POV,E,%+05.2f", te_vario); 
	
	// Calculate NMEA checksum and add to string
	sprintf(sentence + length, "*%02X\n", NMEA_checksum(sentence));
	
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