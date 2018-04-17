// only define this class if it hasn't been before
#ifndef __FORCE_H__
#define __FORCE_H__

// include relevant libraries, always good to have these two
#include <Arduino.h>
#include "Pinouts.h"
#include <DataSource.h>

//for timings
#define FORCE_LOOP_OFFSET 75

// controls how often and when in the loop this class's functions run

class Force : public DataSource {
public: // for functions outside code might call
  Force(void);

  void init(void);

  void readForce(void);

  bool firstRead = true;
  double currentForce, filteredForce;

  String printState(void);
  
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  const float lambda = 0.5;
};

#endif
