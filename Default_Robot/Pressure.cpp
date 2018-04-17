// this includes everything from the header file
#include "Pressure.h"

//Pin definitions
#define PRESSURE_PIN 1

// this allows you to use print calls
// (see IMU or ADC libraries for good examples)
#include "Printer.h"
extern Printer printer;

// constructor for class objects
Pressure::Pressure(void) : DataSource("pressure,pressureFiltered","float,float"){
}

void Pressure::init(void) {
  //setup pins as input
  pinMode(PRESSURE_PIN, INPUT);

  currentPressure = 0;
  firstRead = true;

  // this lets you print messages
  // the string gets displayed in the messages area
  // the number counts the number of times it is displayed at 10 Hz
  printer.printMessage("Initialized pressure at " + String(millis()), 10);
}

String Pressure::printState(void) {
  String printString = "Pressure: ";
  printString += pressureFiltered;
  return printString;
}

void Pressure::readPressure(void) {
	currentPressure = analogRead(PRESSURE_PIN);
  if(firstRead) {
    firstRead = false;
    pressureFiltered = currentPressure;
  }
  else {
    pressureFiltered = (lambda * pressureFiltered) + ((1.0 - lambda) * currentPressure);
  }
}

size_t Pressure::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = currentPressure;
  data_slot[1] = pressureFiltered;
  return idx + 2*sizeof(float);
}
