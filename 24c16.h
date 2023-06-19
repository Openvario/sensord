#define EEPROM_ADR 0x50

#define EEPROM_DATA_VERSION 1

#define SN_LENGTH 8

// define struct for MS5611 sensor
typedef struct {
	int fd;
	unsigned char address;
} t_24c16;

typedef struct {
	char header[3];
	char data_version;
	float zero_offset;
	char serial[SN_LENGTH];
	char padding[3];
	char checksum;
} t_eeprom_data;



// prototypes
int eeprom_open(t_24c16 *, unsigned char);
char eeprom_write(t_24c16 *, char *, unsigned char, unsigned char);
char eeprom_read(t_24c16 *, char *, char, char);
int update_checksum(t_eeprom_data*);
char verify_checksum(t_eeprom_data*);
int eeprom_read_data(t_24c16 *, t_eeprom_data *);



