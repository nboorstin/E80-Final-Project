// this includes everything from the header file
#include "BigMotor.h"

//Pin definitions
#define BIG_MOTOR_GND_A A16
#define BIG_MOTOR_PWR_A A19
#define BIG_MOTOR_PWR_B A18
#define BIG_MOTOR_GND_B A15

// this al0s you to use print calls
// (see IMU or ADC libraries for good examples)
#include "Printer.h"
extern Printer printer;

// constructor for class objects
BigMotor::BigMotor(void) : DataSource("currDir,desiredDir","int32,int32"){
}

void BigMotor::init(void) {
  //setup pins as output
  pinMode(BIG_MOTOR_GND_A, OUTPUT);
  pinMode(BIG_MOTOR_PWR_A, OUTPUT);
  pinMode(BIG_MOTOR_PWR_B, OUTPUT);
  pinMode(BIG_MOTOR_GND_B, OUTPUT);
  
  //assume they all start FLOATING
  currDir = FLOATINGG;
  desiredDir = STOP;
  
  //and set them all to FLOATING, because why not?
  analogWrite(BIG_MOTOR_GND_A, 0);
  analogWrite(BIG_MOTOR_PWR_A, 0);
  analogWrite(BIG_MOTOR_PWR_B, 0);
  analogWrite(BIG_MOTOR_GND_B, 0);
  
  //and set the current time
  stateSwitchTime = millis();

  // this lets you print messages
  // the string gets displayed in the messages area
  // the number counts the number of times it is displayed at 10 Hz
  printer.printMessage("Initialized Big Motor at " + String(millis()), 10);
}

void BigMotor::updateDirection(void) {
	if(currDir != desiredDir && millis() - stateSwitchTime >= motorSwitchDelay) {
		switch(desiredDir) {
			case FLOATINGG:
				 analogWrite(BIG_MOTOR_GND_A, 0);
				 analogWrite(BIG_MOTOR_PWR_A, 0);
				 analogWrite(BIG_MOTOR_PWR_B, 0);
				 analogWrite(BIG_MOTOR_GND_B, 0);
				 break;
			case STOP:
				 analogWrite(BIG_MOTOR_GND_A, 255);
				 analogWrite(BIG_MOTOR_PWR_A, 0);
				 analogWrite(BIG_MOTOR_PWR_B, 0);
				 analogWrite(BIG_MOTOR_GND_B, 255);
				 break;
			case FWD:
				 analogWrite(BIG_MOTOR_GND_A, 255);
				 analogWrite(BIG_MOTOR_PWR_A, 0);
				 analogWrite(BIG_MOTOR_PWR_B, 255);
				 analogWrite(BIG_MOTOR_GND_B, 0);
				 break;
			case BACK:
				 analogWrite(BIG_MOTOR_GND_A, 0);
				 analogWrite(BIG_MOTOR_PWR_A, 255);
				 analogWrite(BIG_MOTOR_PWR_B, 0);
				 analogWrite(BIG_MOTOR_GND_B, 255);
				 break;				
			//default:
				//log some panic thing here
		}
		//and update vars
		currDir = desiredDir;
	}
}

void BigMotor::setDirection(BigMotor::Direction dir) {
  if(desiredDir != dir) {
    desiredDir = dir;
    //and wait at FLOATINGG for a bit
    analogWrite(BIG_MOTOR_GND_A, 0);
    analogWrite(BIG_MOTOR_PWR_A, 0);
    analogWrite(BIG_MOTOR_PWR_B, 0);
    analogWrite(BIG_MOTOR_GND_B, 0);
    stateSwitchTime = millis();
  }
}

String BigMotor::printState(void) {
  String printString = "BigMotor: ";
  printString += currDir;
  return printString;
}

size_t BigMotor::writeDataBytes(unsigned char * buffer, size_t idx) {
  int * data_slot = (int *) &buffer[idx];
  data_slot[0] = currDir;
  data_slot[1] = desiredDir;
  return idx + 2*sizeof(int);
}
