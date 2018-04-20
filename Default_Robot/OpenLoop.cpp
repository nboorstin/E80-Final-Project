// this includes everything from the header file
#include "OpenLoop.h"

//Pin definitions
#define START_BUTTON_PIN 0

const int START_BUTTON_TRESHOLD = 10;
const long OPEN_LOOP_DURATION = 10*1000; //10 seconds
const long ANCHOR_WAIT_TIME = 30*1000; //30 seconds
const int FORCE_THRESHOLD = 45; //Here's hoping...
const int ANCHOR_LOWER_DURATION_A = 50000; // 5 seconds
const int ANCHOR_LOWER_DURATION_B = 1000; //1 second
const int ENCODER_THRESHOLD = 100; //4(?) revolutions

// this allows you to use print calls
// (see IMU or ADC libraries for good examples)
#include "Printer.h"
extern Printer printer;

// constructor for class objects
OpenLoop::OpenLoop(BigMotor& motor, PControl& control, Force& f, RotaryEncoder& e) : bigMotor(motor), pcontrol(control), force(f), encoder(e){
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
	case AIR: {
    //test big motor control circuit for shorts
    int secondsInForty = (millis() / 1000);
    /*if(secondsInForty < 10)
      bigMotor.setDirection(BigMotor::FLOATINGG);
    else if(secondsInForty < 15)
      bigMotor.setDirection(BigMotor::FWD);
    else if(secondsInForty < 20)
      bigMotor.setDirection(BigMotor::BACK);
    else
      bigMotor.setDirection(BigMotor::STOP);*/
    bigMotor.setDirection(BigMotor::FLOATINGG);
    
		uL = uR = 0;
		if(analogRead(START_BUTTON_PIN) < START_BUTTON_TRESHOLD) {
      //init P control to here
      const int number_of_waypoints = 1;
      const int waypoint_dimensions = 2;       // waypoints have two pieces of information, x then y.
      //I hope this works?
      double waypoints [] = { state->x, state->y};   // listed as x0,y0,x1,y1, ... etc.
      pcontrol.init(number_of_waypoints, waypoint_dimensions, waypoints);
  
			mode = OPEN_LOOP;
      currentWayPoint = 0;
      forceStart = force.filteredForce;
			modeStartTime = millis();
		}}
		break;
    
	case OPEN_LOOP:{
    uL = getWayPoint(0);
    uR = getWayPoint(1);
    if(millis() - modeStartTime > OPEN_LOOP_DURATION) { //move onto next time
      anchorLanded = false;
      currentWayPoint++;
      uL=uR=0;
      mode = ANCHOR_LOWER;
      startLength = encoder.encodedLength;
      bigMotor.setDirection(BigMotor::FWD);
      modeStartTime = millis();
    }}
		break;
		
	case ANCHOR_LOWER: {
		uL = uR = 0;
    if(abs(force.filteredForce - forceStart) >= FORCE_THRESHOLD || (millis() - modeStartTime > ANCHOR_LOWER_DURATION_A)/*) {
      anchorLanded = true;
      modeStartTime = millis();
    }
    if(((anchorLanded == true) && (millis() - modeStartTime > ANCHOR_LOWER_DURATION_B)) */||
          (analogRead(START_BUTTON_PIN) < START_BUTTON_TRESHOLD)){
      mode = ANCHOR_WAIT;
      bigMotor.setDirection(BigMotor::STOP);
      modeStartTime = millis();
    }
    }
		break;
		
	case ANCHOR_WAIT: {
		uL = uR = 0;
    if(analogRead(START_BUTTON_PIN) < START_BUTTON_TRESHOLD) {
      while(true);
    }
		if(millis() - modeStartTime > ANCHOR_WAIT_TIME) {
			mode = ANCHOR_RAISE;
			modeStartTime = millis();
			bigMotor.setDirection(BigMotor::BACK); //raise the motor
		}}
		break;
		
	case ANCHOR_RAISE: {
	  uL = uR = 0;
    if((startLength - encoder.encodedLength <= ENCODER_THRESHOLD) || (analogRead(START_BUTTON_PIN) < START_BUTTON_TRESHOLD)) {
      if(currentWayPoint >= totalWayPoints) {
        mode = GPS;
      }
      else {
        mode = OPEN_LOOP;
      }
      bigMotor.setDirection(BigMotor::STOP);
      modeStartTime = millis();
    }}
		break;
		
	case GPS: {
		pcontrol.calculateControl(state);
		uL = pcontrol.uL;
		uR = pcontrol.uR;
	  }
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

