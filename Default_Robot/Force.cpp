// this includes everything from the header file
#include "Force.h"

//Pin definitions
#define FORCE_PIN 2

// this allows you to use print calls
// (see IMU or ADC libraries for good examples)
#include "Printer.h"
extern Printer printer;

// constructor for class objects
Force::Force(void) : DataSource("force,forceFiltered","float,float"){
}

void Force::init(void) {
  //setup pins as input
  pinMode(FORCE_PIN, INPUT);

  currentForce = 0;
  firstRead = true;

  // this lets you print messages
  // the string gets displayed in the messages area
  // the number counts the number of times it is displayed at 10 Hz
  printer.printMessage("Initialized pressure at " + String(millis()), 10);
}

String Force::printState(void) {
  String printString = "Force: ";
  printString += filteredForce;
  return printString;
}

void Force::readForce(void) {
  currentForce = analogRead(FORCE_PIN);
  if(firstRead) {
    firstRead = false;
    filteredForce = currentForce;
  }
  else {
    filteredForce = (lambda * filteredForce) + ((1.0 - lambda) * currentForce);
  }
}

size_t Force::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = currentForce;
  data_slot[1] = filteredForce;
  return idx + sizeof(float);
}
