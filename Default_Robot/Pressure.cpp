// this includes everything from the header file
#include "Pressure.h"

//Pin definitions
#define PRESSURE_PIN 1

// this allows you to use print calls
// (see IMU or ADC libraries for good examples)
#include "Printer.h"
extern Printer printer;

// constructor for class objects
Pressure::Pressure(void) : DataSource("pressure","int32"){
}

void Pressure::init(void) {
  //setup pins as input
  pinMode(PRESSURE_PIN, INPUT);

  currentPressure = 0;

  // this lets you print messages
  // the string gets displayed in the messages area
  // the number counts the number of times it is displayed at 10 Hz
  printer.printMessage("Initialized pressure at " + String(millis()), 10);
}

String Pressure::printState(void) {
  String printString = "Pressure: ";
  printString += currentPressure;
  return printString;
}

void Pressure::readPressure(void) {
	currentPressure = analogRead(PRESSURE_PIN);
}

size_t Pressure::writeDataBytes(unsigned char * buffer, size_t idx) {
  int * data_slot = (int *) &buffer[idx];
  data_slot[0] = currentPressure;
  return idx + sizeof(int);
}
