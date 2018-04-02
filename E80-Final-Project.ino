/********
Default E80 Lab 01 
Current Author: Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)
Previous Contributors:  Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
                        Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)
*/

/* Libraries */

// general
#include <Arduino.h>
#include <Wire.h>
#include "Pinouts.h"

// 80-specific
#include "SensorGPS.h"
#include "SensorIMU.h"
#include "StateEstimator.h"
#include "Adafruit_GPS.h"
#include "ADCSampler.h"
#include "MotorDriver.h"
#include "Logger.h"
#include "Printer.h"
#include "PControl.h"
#include "SoftwareSerial.h"
#include "AnalogSensors.h"
#define mySerial Serial1

// Origin for school testing
//#define ORIGIN_LAT  (34.106465*PI/180.0)
//#define ORIGIN_LON  (-117.712488*PI/180.0)
// Origin for BFS testing
#define ORIGIN_LAT  (34.109463*PI/180.0)
#define ORIGIN_LON  (-117.712776*PI/180.0)
#define RADIUS_OF_EARTH  6371300 // [m]

// template library
#include "LED.h"

/* Global Variables */

// Motors
MotorDriver motorDriver;

// State Estimator
StateEstimator state_estimator;

// Control
PControl pcontrol;

// GPS
SensorGPS gps;
Adafruit_GPS GPS(&mySerial);

// IMU
SensorIMU imu;

// Logger
Logger logger;
bool keepLogging = true;

// Printer
Printer printer;

// Temperature Sensors
AnalogSensors analog_sensors;

//Led
LED led;

// loop start recorder
int loopStartTime;
int current_way_point = 0;

double waypoints[6] = { -3.5, 20.0,
-3.5, 40.0,
-3.5, 20.0 };

void setup() {
	printer.init();

	/* Initialize the Logger */
	logger.include(&imu);
	logger.include(&gps);
	logger.include(&state_estimator);
	logger.include(&motorDriver);
	logger.include(&pcontrol);
	logger.init();

	pcontrol.init(3, 2, waypoints);

	/* Initialise the sensors */
	imu.init();

	mySerial.begin(9600);
	gps.init(&GPS);

	/* Initialize motor pins */
	motorDriver.init();

	/* Done initializing, turn on LED */
	led.init();

	/* Keep track of time */
	printer.printMessage("Starting main loop", 10);
	loopStartTime = millis();
}

void loop() {

	/* set the motors to run based on time since the loop started */
	/* loopStartTime is in units of ms */
	/* The motorDriver.drive function takes in 4 inputs arguments m1_power, m2_power, m3_power, m4_power: */
	/*       void motorDriver.drive(int m1_power, int m2_power, int m3_power, int m4_power) */
	/* the value of m!_power can range from -255 to 255 */
	/* Note: we typically avoid m3, it hasn't worked well in the past */


	if (printer.loopTime(loopStartTime)) {
		printer.printToSerial();  // To stop printing, just comment this line out
	}

	if (pcontrol.loopTime(loopStartTime)) {
		pcontrol.control(&state_estimator.state);
		//pcontrol.printState();
		//PControlCustom()
	}

	if (imu.loopTime(loopStartTime)) {
		imu.read(); // this is a sequence of blocking I2C read calls
		imu.printState(); // a lot of random information
	}

	if (true) {//(gps.loopTime(loopStartTime)) {
		gps.read(&GPS); // this is a sequence of blocking I2C read calls
		gps.printState(); // a lot of random information
	}

	if (state_estimator.loopTime(loopStartTime)) {
		LongLatToXY();
		state_estimator.printState(); // a lot of random information
	}

	// uses the LED library to flash LED -- use this as a template for new libraries!
	if (led.loopTime(loopStartTime)) {
		led.flashLED();
	}

	if (analog_sensors.loopTime(loopStartTime))
	{
		analog_sensors.read();
	}

	if (logger.loopTime(loopStartTime) && keepLogging) {
		keepLogging = logger.log();
	}

}

void LongLatToXY() 
{
	// This function sets the values of state_estimator.state.x, state_estimator.state.y, and state_estimator.state.heading
	// The function accesses the current GPS latitude and longitude readings with gps.state.lat and gps.state.lon
	// And accesses the current imu heading with imu.state.heading

	float current_lat = gps.state.lat*PI / 180.0; // computes current latitude from gps in radians
	float current_lon = -1*gps.state.lon*PI / 180.0; // computes current longitude from gps in radians
	// calculates deviation of latitude and longitude from the  predefined origin [rad]
	float delta_lat = current_lat - ORIGIN_LAT;
	float delta_lon = current_lon - ORIGIN_LON;
	// simple arc length calculation
	state_estimator.state.y = delta_lat * RADIUS_OF_EARTH;
	// arc length calculation where effetive radius is a function of current_lat
	state_estimator.state.x = delta_lon * RADIUS_OF_EARTH*cos(ORIGIN_LAT);
	// update heading of robot in radians based on imu magnetometer data
	state_estimator.state.heading = angleDiff(imu.state.heading*PI / 180.0);

  printer.printMessage("X Position: " + String(state_estimator.state.x),1);
  printer.printMessage("Y Position: " + String(state_estimator.state.y),1);
  
}

float angleDiff(float a) {
	while (a>PI)
		a = a - 2 * PI;

	while (a<(-PI))
		a = a + 2 * PI;

	return a;

}
