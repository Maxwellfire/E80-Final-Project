#ifndef __PCONTROL_H__
#define __PCONTROL_H__

#include "StateEstimator.h"
#include "DataSource.h"

#define SUCCESS_RADIUS 2.0 // success radius in meters

class PControl : public DataSource 
{
public:
  PControl(void);

  // defines the waypoints used for pControl
  void init(const int totalWayPoints_in, const int stateDims_in, double * wayPoints_in);

  // sets the motor speeds using P-Control
  void calculateControl(state_t * state);

  String printString(void);
  String printWaypointUpdate(void);

  int lastExecutionTime = -1;

  // control fields
  float yaw_des;
  float yaw;
  float dist;            // distance to waypoint
  float u;               // control effort
  float Kp=150;         // proportional control gain
  float Kr=1.0;          // right motor gain correction
  float Kl=1.0;          // left motor gain correction
  float avgPower = 230;  // average forward thrust
  double uR;             // right motor effort
  double uL;             // left motor effort

// from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

private:

  // updates the current waypoint if necessary
  void updatePoint(float x, float y);

  int getWayPoint(int dim);

  int totalWayPoints, stateDims;
  double * wayPoints;
  int currentWayPoint = 0;

};

#endif
