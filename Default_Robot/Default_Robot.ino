/********
Default E80 Lab 01 
Current Author: Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)
Previous Contributors:  Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
                        Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)
*/

#include <Arduino.h>
#include <Wire.h>
#include <Pinouts.h>
#include <TimingOffsets.h>
#include "RTEncoder.h"
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <StateEstimator.h>
#include <Adafruit_GPS.h>
#include <ADCSampler.h>
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
#include <PControl.h>
#define mySerial Serial1
#include <LED.h>  // A template of a data soruce library
#include "BigMotor.h"
#include "Pressure.h"
#include "Force.h"

/////////////////////////* Global Variables *////////////////////////
RTEncoder RTEncoder;
MotorDriver motor_driver;
StateEstimator state_estimator;
PControl pcontrol;
SensorGPS gps;
Adafruit_GPS GPS(&mySerial);  // FIX THIS
ADCSampler adc;
SensorIMU imu;
Logger logger;
Printer printer;
LED led;
BigMotor bigMotor;
Pressure pressure;
Force force;

//Defining variables for rotary encoder
//int counter;
//int pin1 = 2;
//int pin2 = 3;
//bool goingUp = false;
//bool goingDown = false;

// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;

////////////////////////* Setup *////////////////////////////////

void setup() {
  
  logger.include(&imu);
  logger.include(&gps);
  logger.include(&state_estimator);
  logger.include(&motor_driver);
  logger.include(&adc);
  logger.include(&bigMotor);
  logger.include(&pressure);
  logger.include(&force);
  logger.include(&RTEncoder);
  logger.init();

  printer.init();
  imu.init();
  mySerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  led.init();
  bigMotor.init();
  pressure.init();
  force.init();
  RTEncoder.init();
 

  //Setup code for Rotary Encoder
  //counter = 0;

  //Setup Encoder pins as inputs
  //pinMode(pin1, INPUT);
  //pinMode(pin2, INPUT);

  //Encoder pin on interrupt 0 (pin 2 right now but going to change)
  //attachInterrupt(0,decoder,FALLING);
  
  //End of Rotary Encoder setup code

  const int number_of_waypoints = 2;
  const int waypoint_dimensions = 2;       // waypoints have two pieces of information, x then y.
  double waypoints [] = { 0, 10, 0, 0 };   // listed as x0,y0,x1,y1, ... etc.
  pcontrol.init(number_of_waypoints, waypoint_dimensions, waypoints);
  
  const float origin_lat = 34.106465;
  const float origin_lon = -117.712488;
  state_estimator.init(origin_lat, origin_lon); 

  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
  printer.lastExecutionTime         = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
  imu.lastExecutionTime             = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  gps.lastExecutionTime             = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
  adc.lastExecutionTime             = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
  state_estimator.lastExecutionTime = loopStartTime - LOOP_PERIOD + STATE_ESTIMATOR_LOOP_OFFSET;
  pcontrol.lastExecutionTime        = loopStartTime - LOOP_PERIOD + P_CONTROL_LOOP_OFFSET;
  bigMotor.lastExecutionTime        = loopStartTime - LOOP_PERIOD + BIG_MOTOR_LOOP_OFFSET;
  pressure.lastExecutionTime        = loopStartTime - LOOP_PERIOD + PRESSURE_LOOP_OFFSET;
  force.lastExecutionTime           = loopStartTime - LOOP_PERIOD + FORCE_LOOP_OFFSET;
  RTEncoder.lastExecutionTime       = loopStartTime - LOOP_PERIOD + RTENCODER_LOOP_OFFSET;
  logger.lastExecutionTime          = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
}

//////////////////////////*Defining Decoder Function for Rotary Encoder*/////////////////////////////////

//void decoder()
//very short interrupt routine 
//Remember that the routine is only called when pin1
//changes state, so it's the value of pin2 that we're
//interrested in here
//{
//if (digitalRead(pin1) == digitalRead(pin2))
//{
//goingUp = 1; //if encoder channels are the same, direction is CW
//}
//else
//{
//goingDown = 1; //if they are not the same, direction is CCW/
//}
//}
//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime=millis();

  //test big motor
  int secondsInForty = (currentTime / 1000) % 40;
  if(secondsInForty < 10)
    bigMotor.setDirection(BigMotor::FLOATINGG);
  else if(secondsInForty < 20)
    bigMotor.setDirection(BigMotor::FWD);
  else if(secondsInForty < 30)
    bigMotor.setDirection(BigMotor::BACK);
  else
    bigMotor.setDirection(BigMotor::STOP);
  //NOTE: Right after the AUV arrives at desired Location we set 
  //anchorDrop to True to start actual polling of encoder
  RTEncoder.anchorDrop = true;

  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0,adc.printSample());
    printer.printValue(1,logger.printState());
    printer.printValue(2,gps.printState());   
    printer.printValue(3,state_estimator.printState());     
    printer.printValue(4,pcontrol.printWaypointUpdate());
    printer.printValue(5,pcontrol.printString());
    printer.printValue(6,motor_driver.printState());
    printer.printValue(7,imu.printRollPitchHeading());        
    printer.printValue(8,imu.printAccels());
    printer.printValue(9,bigMotor.printState());
    printer.printValue(10,pressure.printState());
    printer.printValue(11, force.printState());
    printer.printValue(12, RTEncoder.printState()); 
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  if ( currentTime-pcontrol.lastExecutionTime > LOOP_PERIOD ) {
    pcontrol.lastExecutionTime = currentTime;
    pcontrol.calculateControl(&state_estimator.state);
    motor_driver.driveForward(pcontrol.uL,pcontrol.uR);
  }

  if ( currentTime-adc.lastExecutionTime > LOOP_PERIOD ) {
    adc.lastExecutionTime = currentTime;
    adc.updateSample(); 
  }

  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
  }
  
  if (true){//(gps.loopTime(loopStartTime)) {
    gps.lastExecutionTime = currentTime;
    gps.read(&GPS); // blocking UART calls
  }

  if ( currentTime-state_estimator.lastExecutionTime > LOOP_PERIOD ) {
    state_estimator.lastExecutionTime = currentTime;
    state_estimator.updateState(&imu.state, &gps.state);
  }
  
  // uses the LED library to flash LED -- use this as a template for new libraries!
  if (currentTime-led.lastExecutionTime > LOOP_PERIOD) {
    led.lastExecutionTime = currentTime;
    led.flashLED();
  }
  if (currentTime-bigMotor.lastExecutionTime > LOOP_PERIOD) {
    bigMotor.lastExecutionTime = currentTime;
    bigMotor.updateDirection();
  }

  if (currentTime-pressure.lastExecutionTime > LOOP_PERIOD) {
    pressure.lastExecutionTime = currentTime;
    pressure.readPressure();
  }

  if (currentTime-force.lastExecutionTime > LOOP_PERIOD) {
    force.lastExecutionTime = currentTime;
    force.readForce();
  }
  
  if (currentTime-RTEncoder.lastExecutionTime > LOOP_PERIOD) {
    RTEncoder.lastExecutionTime = currentTime;
    RTEncoder.readRTEncoder();
  }
  if (currentTime- logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
  //Rotary Encoder Portion of loop()
  //using while statement to stay in the loop for continuous interrupts
  /*while(RTEncoder.goingUp == 1) // CW motion in the rotary encoder
  {
    RTEncoder.goingUp = 0; // Reset the flag  
    RTEncoder.counter ++;
    Serial.println(RTEncoder.counter);
  }

  while(RTEncoder.goingDown == 1) // CCW motion in rotary encoder
  {
    RTEncoder.goingDown = 0; // clear the flag
    RTEncoder.counter --;
    Serial.println(RTEncoder.counter);
  }
*/
//Loop Code for Encoder Interrupts
//using while statement to stay in the loop for continuous interrupts
//while(goingUp==1) // CW motion in the rotary encoder
//{
//goingUp=0; // Reset the flag
//counter ++;
//}

//while(goingDown==1) // CCW motion in rotary encoder
//{
//goingDown=0; // clear the flag
//counter --;
//}
  
}

