/*
  RTEncoder.h - Library for receiving data from an incremental rotary encoder.
  Converted to library form by Michael Fernandez from a sketch created by author rt (practicalusage.com), April 3, 2018.
  Released into the public domain.
*/
#ifndef RTEncoder_h
#define RTEncoder_h

#include "Arduino.h"
#include "Pinouts.h"
#include "DataSource.h"

#define RTENCODER_LOOP_OFFSET 80
// controls how often and when in the loop this class's functions run

class RTEncoder : public DataSource
{
  public://for functions outside code might call
    RTEncoder(void);
    void init(void);
    void readRTEncoder(void);
    String printState(void);
    int counter;
    //void decoder();
    //int _pin1;
    //int _pin2;
    bool anchorDrop;
    size_t writeDataBytes(unsigned char * buffer, size_t idx);
    int lastExecutionTime = -1;


};

#endif