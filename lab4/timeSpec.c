#include "timeSpec.h"
#include <stdio.h>

void resetClock(struct timeSpec* ts)
{
	ts->seconds = 0;
	ts->minutes = 0;
	ts->hours = 0;
}


void toString24(struct timeSpec* ts, char* buffer)
{
	//sprintf(buffer, "%02d:%02d:%02d", ts->hours, ts->minutes, ts->seconds);
	sprintf(buffer, "%02d:%02d:%02d", ts->hours, ts->minutes, ts->seconds);
}

void addSecond(struct timeSpec* ts)
{
	ts->seconds++;
	if(ts->seconds >= 60)
	{
		ts->seconds = 0;
		ts->minutes ++;
	}
	if(ts->minutes >= 60)
	{
		ts->minutes = 0;
		ts->hours ++;
	}
	ts->hours %= 24;

}
