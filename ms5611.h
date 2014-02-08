#include <time.h>

// variable definitions

// define struct for MS5611 sensor
typedef struct {
	int fd;
	unsigned char address;
	unsigned long C1s;
	unsigned long C2s;
	unsigned long C3;
	unsigned long C4;
	unsigned long C5s;
	unsigned long C6;
	unsigned long int D1;
	unsigned long int D2;
	long long dT;
	long int temp;
	long long int off;
	long long int sens;
	long int p_meas;
	float p;
	float linearity;
	float offset;
	struct timespec sample;
	struct timespec last_sample;
} t_ms5611;

// prototypes
int ms5611_init(t_ms5611 *);
int ms5611_measure(t_ms5611 *);
int ms5611_calculate(t_ms5611 *);
int ms5611_open(t_ms5611 *, unsigned char);