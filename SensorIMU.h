#ifndef __SENSOR_IMU_H__
#define __SENSOR_IMU_H__

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Mahony.h>
#include <Madgwick.h>
#include "DataSource.h"
#include "Pinouts.h"
#include "TimingOffsets.h"

class SensorIMU : public DataSource {
public:
  SensorIMU(void);

  // Starts the connection to the sensor
  void init(void);

  // Reads data from the sensor
  void read(void);

  void getOrientation(float ax, float ay, float az, float mx, float my, float mz); // float gx, float gy, float gz, 

  // Latest reported orientation data is stored here
  sensors_vec_t state;
  sensors_vec_t simple;  // simple state calculation
  sensors_vec_t acceleration;

  // prints state to serial
  String printRollPitchHeading(void);
  String printAccels(void);
  String printSimple(void);

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  // Create sensor instances.
  //Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
  Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

  // Offsets applied to raw x/y/z mag values
  float mag_offsets[3]            = { -36.88F, 0.4F, -69.4F };

  // Soft iron error compensation matrix
  float mag_ironcomp[3][3] =  { {  0.943,  0.011,  0.020 },
                                {  0.022,  0.918, -0.008 },
                                {  0.020, -0.008,  1.156 } };

  float mag_field_strength = 50.23F;

  // Offsets applied to compensate for gyro zero-drift error for x/y/z
  //float gyro_zero_offsets[3] = { 0.0F, 0.0F, 0.0F };

  // Mahony is lighter weight filter for slower systems
  // (Madgwick can be used for fast rotation)
  //Mahony filter;


};

#endif
