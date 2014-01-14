#ifndef NMEA.h
#define NMEA.h 

unsigned char NMEA_checksum(char *);
int Compose_PAFGA_sentence(char *, float, float, float, float);
int Compose_PAFGB_sentence(char *, float, float, float);

#endif