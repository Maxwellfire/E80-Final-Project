#include "SensorAnalog.h"
#include <Arduino.h>
#include "Printer.h"

extern Printer printer;

SensorAnalog::SensorAnalog(const char* name_in, int pinNumber_in)
	:DataSource(name_in,"int") // from DataSource
{
	name = name_in;
	pinNumber = pinNumber_in;
}

void SensorAnalog::init(void)
{

	//Configure our pin to be an input
	pinMode(pinNumber, INPUT);
}

void SensorAnalog::updateSample(void)
{
	sample = analogRead(pinNumber);
}

String SensorAnalog::printSample(void)
{
	String printString = String(name) + ": " + String(sample);

	return printString;
}

size_t SensorAnalog::writeDataBytes(unsigned char * buffer, size_t idx)
{
	int* data_slot = (int*) &buffer[idx];
	data_slot[0] = sample;
	return idx +  1 * sizeof(int);
}
