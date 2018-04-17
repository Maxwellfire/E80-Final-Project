#include "SensorEncoder.h"
#include <Arduino.h>
#include "Printer.h"

extern Printer printer;

SensorEncoder::SensorEncoder(const char* name_in, int pinNumberA_in, int pinNumberB_in)
	:DataSource(name_in, "int") // from DataSource
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
	switch (state)
	{
	case 0:
		state = 1;
		encoderCount++;
		break;
	case 3:
		state = 2;
		break;
	default:
		break;
	}
}

void SensorEncoder::fallA(void)
{
	switch (state)
	{
	case 1:
		state = 0;
		encoderCount--;
		break;
	case 2:
		state = 3;
		break;
	default:
		break;
	}
}

void SensorEncoder::riseB(void)
{
	switch (state)
	{
	case 0:
		state = 3;
		break;
	case 1:
		state = 2;
		break;
	default:
		break;
	}
}

void SensorEncoder::fallB(void)
{
	switch (state)
	{
	case 2:
		state = 1;
		break;
	case 3:
		state = 0;
		break;
	default:
		break;
	}
}

String SensorEncoder::printCount(void)
{
	String printString = String(name) + ": " + String(encoderCount);

	return printString;
}

size_t SensorEncoder::writeDataBytes(unsigned char * buffer, size_t idx)
{
	int * data_slot = (int *)&buffer[idx];
	data_slot[0] = encoderCount;
	return idx + 1 * sizeof(int);
}