/* DUMMY CLASS! USE AS A TEMPLATE FOR NEW LIBRARIES! */

// only define this class if it hasn't been before
#ifndef __BIG_MOTOR_H__
#define __BIG_MOTOR_H__

// include relevant libraries, always good to have these two
#include <Arduino.h>
#include "Pinouts.h"

// controls how often and when in the loop this class's functions run


class BigMotor {
public: // for functions outside code might call
  enum Direction {FLOATING, STOP, FWD, BACK};
  
  Direction currDir, desiredDir;

  BigMotor(void);

  void init(void);

  void updateDirection(void);
  
  void setDirection(Direction dir);

  int lastExecutionTime = -1;
  
  unsigned long stateSwitchTime = 0;
  
private:
  const unsigned long motorSwitchDelay = 100; //0.1 seconds
  
};

#endif
