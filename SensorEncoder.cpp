#include "SensorEncoder.h"
#include <Arduino.h>
#include "Printer.h"

extern Printer printer;

SensorEncoder::SensorEncoder(const char* name_in, int pinNumberA_in, int pinNumberB_in)
	:DataSource(name, "int") // from DataSource
{
	name = name_in;

	pinNumberA = pinNumberA_in;
	pinNumberB = pinNumberB_in;
}

void SensorEncoder::init(void)
{
	//Configure our pin to be an input
	pinMode(pinNumberA, INPUT);
	pinMode(pinNumberB, INPUT);
}

void SensorEncoder::riseA(void)
{
	AState = 1;
}

void SensorEncoder::fallA(void)
{
	AState = 0;
}

void SensorEncoder::riseB(void)
{
	if (AState == 1)
	{
		encoderCount++;
	}
	else
	{
		encoderCount--;
	}
}

String SensorEncoder::printCount(void)
{
	String printString = String(name) + ": " + String(encoderCount);

	return printString;
}

size_t SensorEncoder::writeDataBytes(unsigned char * buffer, size_t idx)
{
	bool * data_slot = (bool *)&buffer[idx];
	data_slot[0] = encoderCount;
	return idx + 1 * sizeof(bool);
}