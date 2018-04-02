#include "AnalogSensors.h"

AnalogSensors::AnalogSensors()
	:DataSource("ADC1_Count,ADC2_Count,ADC3_Count ", "int,int,int")
{

}


AnalogSensors::~AnalogSensors()
{
}

void AnalogSensors::init()
{
	pinMode(A2, INPUT); //A2
	pinMode(A3, INPUT);  //A3
	pinMode(0, INPUT); //

}

// tells the main loop when to activate
bool AnalogSensors::loopTime(int loopStartTime) {
	int currentTime = millis();
	if (lastLoopTime == -1) {
		lastLoopTime = loopStartTime - ANALOGSENSOR_LOOP_INTERVAL + ANALOGSENSOR_LOOP_OFFSET;
	}
	if (currentTime - lastLoopTime >= ANALOGSENSOR_LOOP_INTERVAL) {
		lastLoopTime = currentTime;
		return true;
	}
	return false;
}

size_t AnalogSensors::writeDataBytes(unsigned char * buffer, size_t idx)
{
	uint16_t * int_slot = (uint16_t *)(buffer + idx);
	int_slot[0] = sensorReading1;
	int_slot[0] = sensorReading2;
	int_slot[0] = sensorReading3;
	idx += 1 * sizeof(int);
	return idx;
}

void AnalogSensors::read()
{
	sensorReading1 = analogRead(10);
	sensorReading2 = analogRead(11);
	sensorReading3 = analogRead(12);

}