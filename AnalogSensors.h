#pragma once
#include "DataSource.h"

// include relevant libraries, always good to have these two
#include <Arduino.h>
#include "Pinouts.h"

// controls how often and when in the loop this class's functions run
#define ANALOGSENSOR_LOOP_INTERVAL 100 // ms
#define ANALOGSENSOR_LOOP_OFFSET 0// ms

class AnalogSensors :
	public DataSource
{
public:
	AnalogSensors();
	~AnalogSensors();

	void init();
	bool loopTime(int loopStartTime);
	void read();

	size_t writeDataBytes(unsigned char * buffer, size_t idx);

private:
	int lastLoopTime = -1;
	int sensorReading1;
	int sensorReading2;
	int sensorReading3;
};
