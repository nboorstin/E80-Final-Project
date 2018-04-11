// only define this class if it hasn't been before
#ifndef __ROTARY_ENCODER_H__
#define __ROTARY_ENCODER_H__

// include relevant libraries, always good to have these two
#include <Arduino.h>
#include "Pinouts.h"
#include "DataSource.h"

//for timings
#define ROTARY_ENCODER_LOOP_OFFSET 80

// controls how often and when in the loop this class's functions run


class RotaryEncoder : public DataSource {
public: // for functions outside code might call
  RotaryEncoder(void);

  void init(void);

  static void readEncoder(void);

  String printState(void);

  int encodedLength;
  
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;
  
  static RotaryEncoder* encoder;// = nullptr; //ugh this is prolly not the best
  
};

#endif
