#include <Arduino.h>
#include "SensorDigital.h"
#include "Printer.h"

extern Printer printer;

SensorDigital::SensorDigital(const char* name_in, int pinNumber_in)
	:DataSource(name, "int") // from DataSource
{
	name = name_in;
	pinNumber = pinNumber_in;
}

void SensorDigital::init(void)
{
	//Configure our pin to be an input
	pinMode(pinNumber, INPUT);
}

void SensorDigital::updateSample(void)
{
	sample = digitalRead(pinNumber);
}

String SensorDigital::printSample(void)
{
	String printString = String(name) + ": " + String(sample);

	return printString;
}

size_t SensorDigital::writeDataBytes(unsigned char * buffer, size_t idx)
{
	bool * data_slot = (bool *) &buffer[idx];
	data_slot[0] = sample;
	return idx + 1 * sizeof(bool);
}
