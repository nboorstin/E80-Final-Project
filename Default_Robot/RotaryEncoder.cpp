// this includes everything from the header file
#include "RotaryEncoder.h"

//Pin definitions
#define ENCODER_PIN_A 8
#define ENCODER_PIN_B 9

// this allows you to use print calls
// (see IMU or ADC libraries for good examples)
#include "Printer.h"
extern Printer printer;

// constructor for class objects
RotaryEncoder::RotaryEncoder(void) : DataSource("encoder","int32"){
}

RotaryEncoder* RotaryEncoder::encoder = nullptr;

void RotaryEncoder::init(void) {
  //setup pins as input
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);

  encodedLength = 0;
  
  encoder = this;

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), readEncoder, RISING);

  // this lets you print messages
  // the string gets displayed in the messages area
  // the number counts the number of times it is displayed at 10 Hz
  printer.printMessage("Initialized encoder at " + String(millis()), 10);
}

String RotaryEncoder::printState(void) {
  String printString = "Encoder: ";
  printString += encodedLength;
  return printString;
}

void RotaryEncoder::readEncoder(void) {
  if(encoder != nullptr) {
      if(digitalRead(ENCODER_PIN_B)) {
        encoder -> encodedLength++;
      }
      else {
        encoder -> encodedLength--;
      }
  }
}

size_t RotaryEncoder::writeDataBytes(unsigned char * buffer, size_t idx) {
  int * data_slot = (int *) &buffer[idx];
  data_slot[0] = encodedLength;
  return idx + sizeof(int);
}
