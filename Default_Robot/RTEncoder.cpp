/*
  RTEncoder.cpp - Library for receiving data from an incremental rotary encoder.
  Converted to library form by Michael Fernandez from a sketch created by author rt (practicalusage.com), April 3, 2018.
  Released into the public domain.
*/

#include "Arduino.h"
#include "RTEncoder.h"
#include "Printer.h"
extern Printer printer;

//Pin Definitions

#define RTENCODER_PINA 9
#define RTENCODER_PINB 8

RTEncoder::RTEncoder(void): DataSource("counter","int"){

}

void RTEncoder::init(void){
  //setup pins as input
  pinMode(RTENCODER_PINA, INPUT);//Pin 9
  pinMode(RTENCODER_PINB, INPUT);//Pin 8
  counter = 0;
  anchorDrop = false;
  printer.printMessage("Initialized Rotary Encoder at " +  String(millis()),10);
  printer.printMessage("Encoder state set to anchorDrop = " + anchorDrop,10);
  
}

String RTEncoder::printState(void) {
  String printString = "Counter: ";
  printString += counter;
  return printString;

}

void RTEncoder::readRTEncoder(void) {
  if (anchorDrop == false) {
    counter += 0;
    }
  else 
  { if (digitalRead(RTENCODER_PINA) == digitalRead(RTENCODER_PINB)) {
    counter ++;
  }
    else if (digitalRead(RTENCODER_PINA) != digitalRead(RTENCODER_PINB)) {
      counter --;
    }
    else
    {
      counter += 0;
    }
  }
}
//RTEncoder::RTEncoder(int pin1, int pin2)
//{
  

  //_pin1 = pin1;
  //_pin2 = pin2;
  //attachInterrupt(0, RTEncoder::decoder, FALLING);
  //goingUp = false;
 // goingDown = false;
  //anchorDrop;
//}

//void RTEncoder::decoder()
//very short interrupt routine 
//Remember that the routine is only called when _pin1
//changes state, so it's the value of _pin2 that we're
//interested in here
//{
//if (digitalRead(_pin1) == digitalRead(_pin2)){
//	goingUp = true; //if encoder channels are the same, direction is CW
//	}
//else 
//{
//	goingDown = true; //if they are not the same, direction is CCW
//	}
//}
size_t RTEncoder::writeDataBytes(unsigned char * buffer, size_t idx) {
  int * data_slot = (int *) &buffer[idx];
  data_slot[0] = counter;
  return idx + sizeof(int);
}
