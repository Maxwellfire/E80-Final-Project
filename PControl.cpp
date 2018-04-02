#include "PControl.h"
#include "Printer.h"
extern Printer printer;

PControl::PControl(void)
:DataSource("control_effort,yaw_error", "float,float") {

}

void PControl::init(const int totalWayPoints_in, const int stateDims_in, double * wayPoints_in)

{
  totalWayPoints = totalWayPoints_in;
  stateDims = stateDims_in;


  //In order for this to work, waypoints must be define globally, so the pointer we get here doesn't get deleted
  wayPoints = wayPoints_in;

}

double PControl::getWayPoint(int dim) {
  return wayPoints[currentWayPoint*stateDims+dim];
}

void PControl::control(state_t * state) {
  updatePoint(state->x, state->y);
  if (currentWayPoint == totalWayPoints) return; // stops motors at final point

  // set up variables
  float K_P = 50.0;
  double x_des = getWayPoint(0);
  double y_des = getWayPoint(1);

  // determine diff between current state and des state
  float yaw_des = atan2(y_des - state->y, x_des - state->x);
  float yaw = -PI/2.0+state->heading;

  while (yaw>PI)
	  yaw = yaw - 2 * PI;

  while (yaw<(-PI))
	  yaw = yaw + 2 * PI;

  yaw_diff = yaw_des - yaw;
  
  if (yaw_diff> PI) yaw_diff -= 2*PI;
  if (yaw_diff<-PI) yaw_diff += 2*PI;

  // actual PControl line
  controlEffort = K_P*yaw_diff;
  driveMotors(controlEffort); // this function actually sets the motor speeds

  String printString = "PControl: Yaw_Des: " + String(yaw_des)
	  + " Yaw: " + String(yaw)
	  + " u: " + String(controlEffort)
	  //+ " x_des: " + String(x_des)
	  //+ " y_des: " + String(y_des);
	  + " n: " + String(currentWayPoint);
  printer.printValue(3,printString);
}


void PControl::driveMotors(float u) {
  double uL, uR;
  float avgPower = 50;
  float KL = 1.0; // allows for easy L/R motor calibration
  float KR = 1.0;

  uL = max(0.0,min(255.0,(avgPower - u)*KL));
  uR = max(0.0,min(255.0,(avgPower + u)*KR));
  motorDriver.driveForward(uL,uR);
}

void PControl::updatePoint(float x, float y) {
  if (currentWayPoint == totalWayPoints) return; // don't check if finished

  int x_des = getWayPoint(0);
  int y_des = getWayPoint(1);
  float dist = sqrt(pow(x-x_des,2) + pow(y-y_des,2));

  String wayPointUpdate = "PControl: Current Waypoint: " + String(currentWayPoint)
    + " Distance from Waypoint: " + String(dist);
  printer.printValue(2, wayPointUpdate);

  if (dist < SUCCESS_RADIUS && currentWayPoint < totalWayPoints) {
    String changingWPMessage = "Got to waypoint " + String(currentWayPoint)
      + ", now directing to next point";
    int cwpmTime = 20;
    currentWayPoint++;
    if (currentWayPoint == totalWayPoints) {
      changingWPMessage = "Congratulations! You completed the path! Stopping motors.";
      motorDriver.stopDriving();
      cwpmTime = 0;
    }
    printer.printMessage(changingWPMessage,cwpmTime);
  }
}

bool PControl::loopTime(int loopStartTime) {
  int currentTime = millis();
  if (lastLoopTime == -1) {
    lastLoopTime = loopStartTime-PC_LOOP_INTERVAL+PC_LOOP_OFFSET;
  }
  if (currentTime - lastLoopTime >= PC_LOOP_INTERVAL) {
    lastLoopTime = currentTime;
    return true;
  }
  return false;
}

void PControl::printState(void)
{
	String printString = "Control Effort: ";
	printString += String(controlEffort, 4);
	printString += "Yaw Error: ";
	printString += String(yaw_diff, 6);

	printer.printValue(2, printString);
}

size_t PControl::writeDataBytes(unsigned char * buffer, size_t idx)
{
	float * float_slot = (float *)(buffer + idx);
	float_slot[0] = controlEffort;
	float_slot[1] = yaw_diff;
	idx += 2 * sizeof(float);
	return idx;
}