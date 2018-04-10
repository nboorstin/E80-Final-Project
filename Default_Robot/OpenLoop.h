// only define this class if it hasn't been before
#ifndef __OPEN_LOOP_H__
#define __OPEN_LOOP_H__

// include relevant libraries, always good to have these two
#include <Arduino.h>
#include "Pinouts.h"
#include "StateEstimator.h"
#include "BigMotor.h"
#include "Force.h"
#include <pControl.h>

//for timings
//#define OPEN_LOOP_LOOP_OFFSET 65

// controls how often and when in the loop this class's functions run


class OpenLoop{
public: // for functions outside code might call
  enum Mode {AIR, OPEN_LOOP, ANCHOR_LOWER, ANCHOR_WAIT, ANCHOR_RAISE, GPS};
  
  OpenLoop(BigMotor& motor, PControl& control, Force& f);

  void init(const int totalWayPoints_in, const int stateDims_in, int * wayPoints_in);
  
  void calculateControl(state_t * state);

  String printState(void);

  int lastExecutionTime = -1;
  
  double uL = 0, uR = 0;
  
private:
  Mode mode = AIR;
  int currentWayPoint = 0;
  
  long modeStartTime;
  
  int getWayPoint(int dim);
  int totalWayPoints, stateDims;
  int * wayPoints;
  
  BigMotor& bigMotor;
  PControl& pcontrol;
  Force& force;

  bool anchorLanded;
  
  
  
};

#endif
