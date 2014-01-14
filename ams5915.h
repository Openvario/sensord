#include <time.h>

// variable definitions

// define struct for AMS5915 sensor
typedef struct {
	int fd;
	unsigned char address;
	int digoutpmin;
	int digoutpmax;
	int digoutp;
	unsigned int digoutT;
	int pmin;
	int pmax;
	float sensp;
	float T;
	float p;
	struct timespec sample;
	struct timespec last_sample;
} t_ams5915;

// prototypes
int ams5915_init(t_ams5915 *);
int ams5915_measure(t_ams5915 *);
int ams5915_calculate(t_ams5915 *);
int ams5915_open(t_ams5915 *, unsigned char);