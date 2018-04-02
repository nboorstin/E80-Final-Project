// this includes everything from the header file
#include "BigMotor.h"

//Pin definitions
#define BIG_MOTOR_GND_A 10
#define BIG_MOTOR_PWR_A 12
#define BIG_MOTOR_PWR_B 15
#define BIG_MOTOR_GND_B 15

// this allows you to use print calls
// (see IMU or ADC libraries for good examples)
#include "Printer.h"
extern Printer printer;

// constructor for class objects
BigMotor::BigMotor(void) : DataSource("currDir,desiredDir","int,int"){
}

void BigMotor::init(void) {
  //setup pins as output
  pinMode(BIG_MOTOR_GND_A, OUTPUT);
  pinMode(BIG_MOTOR_PWR_A, OUTPUT);
  pinMode(BIG_MOTOR_PWR_B, OUTPUT);
  pinMode(BIG_MOTOR_GND_B, OUTPUT);
  
  //assume they all start floating
  currDir = FLOATING;
  desiredDir = STOP;
  
  //and set them all to floating, because why not?
  digitalWrite(BIG_MOTOR_GND_A, LOW);
  digitalWrite(BIG_MOTOR_PWR_A, LOW);
  digitalWrite(BIG_MOTOR_PWR_B, LOW);
  digitalWrite(BIG_MOTOR_GND_B, LOW);
  
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
			case FLOATING:
				 digitalWrite(BIG_MOTOR_GND_A, LOW);
				 digitalWrite(BIG_MOTOR_PWR_A, LOW);
				 digitalWrite(BIG_MOTOR_PWR_B, LOW);
				 digitalWrite(BIG_MOTOR_GND_B, LOW);
				 break;
			case STOP:
				 digitalWrite(BIG_MOTOR_GND_A, HIGH);
				 digitalWrite(BIG_MOTOR_PWR_A, LOW);
				 digitalWrite(BIG_MOTOR_PWR_B, LOW);
				 digitalWrite(BIG_MOTOR_GND_B, HIGH);
				 break;
			case FWD:
				 digitalWrite(BIG_MOTOR_GND_A, HIGH);
				 digitalWrite(BIG_MOTOR_PWR_A, LOW);
				 digitalWrite(BIG_MOTOR_PWR_B, HIGH);
				 digitalWrite(BIG_MOTOR_GND_B, LOW);
				 break;
			case BACK:
				 digitalWrite(BIG_MOTOR_GND_A, LOW);
				 digitalWrite(BIG_MOTOR_PWR_A, HIGH);
				 digitalWrite(BIG_MOTOR_PWR_B, LOW);
				 digitalWrite(BIG_MOTOR_GND_B, HIGH);
				 break;				
			//default:
				//log some panic thing here
		}
		//and update vars
		currDir = desiredDir;
	}
}

void BigMotor::setDirection(BigMotor::Direction dir) {
  desiredDir = dir;
  //and wait at floating for a bit
  digitalWrite(BIG_MOTOR_GND_A, LOW);
  digitalWrite(BIG_MOTOR_PWR_A, LOW);
  digitalWrite(BIG_MOTOR_PWR_B, LOW);
  digitalWrite(BIG_MOTOR_GND_B, LOW);
  stateSwitchTime = millis();
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
