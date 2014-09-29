

#include "24c16.h"
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

int update_checksum(t_eeprom_data* data)
{
	char* p_data;
	int i;
	char checksum=0x00;
	
	p_data = (char*)data;
	
	for (i=0; i<sizeof(data); i++)
	{
		checksum += *p_data;
		p_data++;
	}
	
	//printf("Checksum: %x\n", checksum);
	data->checksum = checksum;
	return (0);
}

char verify_checksum(t_eeprom_data* data)
{
	char* p_data;
	int i;
	char checksum=0x00;
	
	p_data = (char*)data;
	
	for (i=0; i<sizeof(data); i++)
	{
		checksum += *p_data;
		p_data++;
	}
	//printf("Checksum read: %x\n", data->checksum);
	//printf("Checksum calc: %x\n", checksum);
	if (checksum == data->checksum)
	{
		return (1);
	}
	else
	{
		return(0);
	}
}

int eeprom_open(t_24c16 *eeprom, unsigned char i2c_address)
{
	// local variables
	int fd;
	
	// try to open I2C Bus
	fd = open("/dev/i2c-1", O_RDWR);
	
	if (fd < 0) {
		fprintf(stderr, "Error opening file: %s\n", strerror(errno));
		return 1;
	}

	if (ioctl(fd, I2C_SLAVE, i2c_address) < 0) {
		fprintf(stderr, "ioctl error: %s\n", strerror(errno));
		return 1;
	}
	
	printf("Opened 24C16 on 0x%x\n", i2c_address);
	
	// assign file handle to sensor object
	eeprom->fd = fd;
	eeprom->address = i2c_address;
	return (0);
}

char eeprom_write(t_24c16 *eeprom, char *s, unsigned char offset, unsigned char count)
{
	unsigned char buf[2];
	int i;
	
	//memcpy(&buf[1], s, count);
	
	//write data at offset to eeprom
	buf[0] = offset;														// This is the register we want to read from
	
	// write byte by byte to EEPROM
	for (i=0; i<count;i++)
	{
		// copy data to write buffer
		buf[1]=*(s);		
		//printf("buf[1]: '%c'\n",buf[1]);
		// Write data to EEPROM
		if ((write(eeprom->fd, &buf[0], 2)) != 2) {				// Send register we want to read from	
			printf("Error writing to i2c slave (%s)\n", __func__);
			return(1);
		}
		
		// give EEPROM time for write ...
		usleep(10000);
		buf[0]++;
		s++;
	}
	return(0);
}

char eeprom_read(t_24c16 *eeprom, char *s, char offset, char count)
{	
	//write address offset to eeprom
	if ((write(eeprom->fd, &offset, 1)) != 1) {				// Send register we want to read from	
		printf("Error writing to i2c slave (%s)\n", __func__);
		return(1);
	}
	
	if (read(eeprom->fd, s, count) != count) {		// Read back data into buf[]
		printf("Unable to read from slave\n");
		return(1);
	}
	
	return(0);
}