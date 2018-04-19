#ifndef _SENSORENCODER_h
#define _SENSORENCODER_h

#include "DataSource.h"

class SensorEncoder : public DataSource 
{
public:
	SensorEncoder(const char* name_in, int pinNumberA_in, int pinNumberB_in);

	void init(void);

	// Managing state

	volatile int encoderCount;
	String printCount(void);

	// Write out
	size_t writeDataBytes(unsigned char * buffer, size_t idx);

	int lastExecutionTime = -1;

	int pinNumberA;
	int pinNumberB;

	uint8_t state;

	//ISR functions
	void changeA(void);
	void changeB(void);

private:
	const char* name;

	volatile bool AState = 0;

};


#endif

