#ifndef TIME_SPEC_H
#define TIME_SPEC_H
#include <stdlib.h>
#include <stdint.h>
struct timeSpec{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
};
void toString24(struct timeSpec*, char* buffer);
void addSecond(struct timeSpec*);
void resetClock(struct timeSpec*);


#endif
