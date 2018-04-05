// only define this class if it hasn't been before
#ifndef __BIG_MOTOR_H__
#define __BIG_MOTOR_H__

// include relevant libraries, always good to have these two
#include <Arduino.h>
#include "Pinouts.h"
#include "DataSource.h"

//for timings
#define BIG_MOTOR_LOOP_OFFSET 60

// controls how often and when in the loop this class's functions run


class BigMotor : public DataSource {
public: // for functions outside code might call
  enum Direction {FLOATINGG, STOP, FWD, BACK};
  
  Direction currDir, desiredDir;

  BigMotor(void);

  void init(void);

  void updateDirection(void);
  
  void setDirection(Direction dir);

  String printState(void);
  
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;
  
  unsigned long stateSwitchTime = 0;
  
private:
  const unsigned long motorSwitchDelay = 400; //0.4 seconds
  
};

#endif
