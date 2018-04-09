// this includes everything from the header file
#include "OpenLoop.h"

//Pin definitions
#define START_BUTTON_PIN 0

const int START_BUTTON_TRESHOLD = 5;
const long ANCHOR_WAIT_TIME = 30*1000; //30 seconds

// this allows you to use print calls
// (see IMU or ADC libraries for good examples)
#include "Printer.h"
extern Printer printer;

// constructor for class objects
OpenLoop::OpenLoop(BigMotor& motor, PControl& control) : bigMotor(motor), pcontrol(control){
}

void OpenLoop::init(const int totalWayPoints_in, const int stateDims_in, int * wayPoints_in) {
  totalWayPoints = totalWayPoints_in;
  stateDims = stateDims_in;
  wayPoints = wayPoints_in;
  //setup pins as input
  pinMode(START_BUTTON_PIN, INPUT);
  // this lets you print messages
  // the string gets displayed in the messages area
  // the number counts the number of times it is displayed at 10 Hz
  printer.printMessage("Initialized open loop control at " + String(millis()), 10);
}

int OpenLoop::getWayPoint(int dim) {
  return wayPoints[currentWayPoint*stateDims+dim];
}


void OpenLoop::calculateControl(state_t * state) {
	switch(mode) {
	case AIR:
		uL = uR = 0;
		if(analogRead(START_BUTTON_PIN) < START_BUTTON_TRESHOLD) {
			mode = OPEN_LOOP;
			modeStartTime = millis();
		}
		break;
	case OPEN_LOOP:
		break;
		
	case ANCHOR_LOWER:
		
		break;
		
	case ANCHOR_WAIT:
		uL = uR = 0;
		if(millis() - modeStartTime > ANCHOR_WAIT_TIME) {
			mode = ANCHOR_RAISE;
			modeStartTime = millis();
			bigMotor.setDirection(BigMotor::BACK); //raise the motor
		}
		break;
		
	case ANCHOR_RAISE:
	
		break;
		
	case GPS:
		pcontrol.calculateControl(state);
		uL = pcontrol.uL;
		uR = pcontrol.uR;
		break;		
	}
	
}

String OpenLoop::printState(void) {
  String printString = "OpenLoop: ";
  printString += mode;
  printString += ", ";
  printString += currentWayPoint;
  return printString;
}

