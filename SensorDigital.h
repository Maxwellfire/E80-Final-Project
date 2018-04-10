#ifndef _SENSORDigital_h
#define _SENSORANALOG_h

#include "DataSource.h"

class SensorDigital : public DataSource
{
public:
	SensorDigital(const char* name_in, int pinNumber_in);

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