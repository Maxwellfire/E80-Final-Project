#ifndef _SENSORANALOG_h
#define _SENSORANALOG_h

#include "DataSource.h"

class SensorAnalog : public DataSource
{
public:
	SensorAnalog(const char* names, int pinNumber_in);

	void init(void);

	// Managing state

	int sample;
	void updateSample(void);
	String printSample(void);

	// Write out
	size_t writeDataBytes(unsigned char * buffer, size_t idx);

	int lastExecutionTime = -1;

private:
	const char* name;
	int pinNumber;
};

#endif