// only define this class if it hasn't been before
#ifndef __PRESSURE_H__
#define __PRESSURE_H__

// include relevant libraries, always good to have these two
#include <Arduino.h>
#include "Pinouts.h"
#include "DataSource.h"

//for timings
#define PRESSURE_LOOP_OFFSET 65

// controls how often and when in the loop this class's functions run


class Pressure : public DataSource {
public: // for functions outside code might call
  Pressure(void);

  void init(void);

  void readPressure(void);

  String printState(void);

  int currentPressure;
  
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;
  
};

#endif
