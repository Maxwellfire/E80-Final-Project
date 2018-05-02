/********
Default E80 Lab 01 
Current Author: Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)
Previous Contributors:  Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
                        Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)
*/

//ERRORS
//Problems with pointer to waypoints not being copied
//Logger::log declared as returning bool, but doesn't

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "TimingOffsets.h"
#include "Pinouts.h"
#include "SensorGPS.h"
#include "SensorIMU.h"
#include "StateEstimator.h"
#include "ADCSampler.h"
#include "MotorDriver.h"
#include "Logger.h"
#include "Printer.h"
#include "PControl.h"
#include "SensorAnalog.h"
#include "SensorDigital.h"
#include "SensorEncoder.h"

#define mySerial Serial1
#include "LED.h"  // A template of a data soruce library

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
StateEstimator state_estimator;
PControl pcontrol;
SensorGPS gps;
Adafruit_GPS GPS(&mySerial);  // FIX THIS
ADCSampler adc;
SensorIMU imu;

SensorAnalog pressure("Pressure", 15);
SensorAnalog temperature1("Temperature_1", 17);
//SensorAnalog temperature2("Temperature_2", 26);

SensorAnalog batteryVoltage("Battery_Voltage", 29);
SensorAnalog batteryCurrent("Battery_Current", 30);

SensorDigital button1("Button_1", 7);

SensorEncoder encoder("Encoder", 26, 27);

Logger logger;
Printer printer;
LED led;

//define waypoints in the global scope as the PControl class doesn't make a deep copy.
double waypoints[] = {25, 80, 20.5, 65, 30.5, 62, 35, 77, 45, 74, 40.5, 59, 50.5, 56, 55, 71, 25, 80};   // listed as x0,y0,x1,y1, ... etc.

const int number_of_waypoints = 9;
const int waypoint_dimensions = 2;       // waypoints have two pieces of information, x then y.


// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;


bool waiting = 1; 

////////////////////////* Setup *////////////////////////////////

void setup() {
  
  logger.include(&imu);
  logger.include(&gps);
  logger.include(&state_estimator);
  logger.include(&motor_driver);
  logger.include(&pcontrol);
  logger.include(&pressure);
  logger.include(&temperature1);
  //logger.include(&temperature2);
  logger.include(&button1);
  logger.include(&encoder);
  logger.include(&batteryVoltage);
  logger.include(&batteryCurrent);
  //logger.include(&adc);p

  logger.init();

  printer.init();
  imu.init();
  mySerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  led.init();
  //adc.init();

  pressure.init();
  temperature1.init();
  //temperature2.init();
  button1.init();
  encoder.init();

  batteryVoltage.init();
  batteryCurrent.init();


  attachInterrupt(digitalPinToInterrupt(encoder.pinNumberA), &EncoderChangeA, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoder.pinNumberA), &EncoderFallA, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder.pinNumberB), &EncoderChangeB, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoder.pinNumberB), &EncoderFallB, FALLING);

  button1.init();

  pcontrol.init(number_of_waypoints, waypoint_dimensions, waypoints);
  
  //const float origin_lat = 34.106465;
  //const float origin_lon = -117.712488;
  
  const float origin_lat = 33.461887;
  const float origin_lon = -117.706134;

  state_estimator.init(origin_lat, origin_lon);

  printer.printMessage("Starting main loop", 10);
  loopStartTime = millis();
  printer.lastExecutionTime         = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
  imu.lastExecutionTime             = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  gps.lastExecutionTime             = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
  adc.lastExecutionTime             = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
  state_estimator.lastExecutionTime = loopStartTime - LOOP_PERIOD + STATE_ESTIMATOR_LOOP_OFFSET;
  pcontrol.lastExecutionTime        = loopStartTime - LOOP_PERIOD + P_CONTROL_LOOP_OFFSET;
  logger.lastExecutionTime          = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
}
//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime=millis();

  if ( currentTime - printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
	printer.printValue(0, pressure.printSample() + " "
		+ temperature1.printSample() + " "
		//+ temperature2.printSample() + " "
		+ button1.printSample() + " "
		+ encoder.printCount() + " "
		+ batteryCurrent.printSample() + " "
		+ batteryVoltage.printSample()
	);

    printer.printValue(1,logger.printState());
    printer.printValue(2,gps.printState());   
    printer.printValue(3,state_estimator.printState());     
    printer.printValue(4,pcontrol.printWaypointUpdate());
    printer.printValue(5,pcontrol.printString());
    printer.printValue(6,motor_driver.printState());
    printer.printValue(7,imu.printRollPitchHeading());        
    printer.printValue(8,imu.printAccels());
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  if ( currentTime - pcontrol.lastExecutionTime > 20 && !waiting) {
    pcontrol.lastExecutionTime = currentTime;
    pcontrol.calculateControl(&state_estimator.state);
    
	//motor_driver.drive(0, 255, 255, 0);
	motor_driver.drive(pcontrol.uL, pcontrol.uR, 0, 0);
	//motor_driver.driveForward(255, 255);
	//motor_driver.drive(imu.state.pitch * 4000.0, imu.state.pitch * 4000.0, imu.state.pitch * 4000.0, imu.state.pitch * 4000.0);

  }

  if ( currentTime - adc.lastExecutionTime > LOOP_PERIOD ) {
    adc.lastExecutionTime = currentTime;
	pressure.updateSample();
	temperature1.updateSample();
	//temperature2.updateSample();
	button1.updateSample();
	waiting = !button1.sample && waiting;

	batteryVoltage.updateSample();
	batteryCurrent.updateSample();

  }

  if ( currentTime - imu.lastExecutionTime > 20 ) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
  }
  
  if (true){
    gps.lastExecutionTime = currentTime;
    gps.read(&GPS); // blocking UART calls
  }

  if ( currentTime - state_estimator.lastExecutionTime > LOOP_PERIOD ) {
    state_estimator.lastExecutionTime = currentTime;
    state_estimator.updateState(&imu.state, &gps.state);
  }
  
  // uses the LED library to flash LED -- use this as a template for new libraries!
  if (currentTime-led.lastExecutionTime > LOOP_PERIOD) {
    led.lastExecutionTime = currentTime;
    led.flashLED();
  }

  if (currentTime- logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}


void EncoderChangeA(void)
{
	//printer.printMessage("foo", 2);
	encoder.changeA();
}

void EncoderChangeB(void)
{
	encoder.changeB();
}
