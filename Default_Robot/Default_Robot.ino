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

#include <SensorGPS.h>
#include <SensorIMU.h>
#include <StateEstimator.h>
#include <Adafruit_GPS.h>
//#include <ADCSampler.h>
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
#include <PControl.h>
#define mySerial Serial1
//#include <LED.h>  // A template of a data soruce library
#include "BigMotor.h"
#include "Pressure.h"
#include "Force.h"
#include "RotaryEncoder.h"
#include "OpenLoop.h"

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
StateEstimator state_estimator;
PControl pcontrol;
SensorGPS gps;
Adafruit_GPS GPS(&mySerial);  // FIX THIS
//ADCSampler adc;
SensorIMU imu;
Logger logger;
Printer printer;
//LED led;
BigMotor bigMotor;
Pressure pressure;
Force force;
RotaryEncoder encoder;
OpenLoop openLoop(bigMotor, pcontrol, force, encoder);

// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;

////////////////////////* Setup *////////////////////////////////
const int number_of_waypoints = 2;
const int waypoint_dimensions = 2;       // waypoints have two pieces of information, x then y.
int waypoints [] = { 100, 100, 100, 50};   // listed as x0,y0,x1,y1, ... etc.

void setup() {
  
  logger.include(&imu);
  logger.include(&gps);
  logger.include(&state_estimator);
  logger.include(&motor_driver);
  //logger.include(&adc);
  logger.include(&bigMotor);
  logger.include(&pressure);
  logger.include(&force);
  logger.include(&encoder);
  logger.init();

  printer.init();
  imu.init();
  mySerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  //led.init();
  

  
  openLoop.init(number_of_waypoints, waypoint_dimensions, waypoints);

  /*const int number_of_waypoints = 1;
  const int waypoint_dimensions = 2;       // waypoints have two pieces of information, x then y.
  double waypoints [] = { 0, 10};   // listed as x0,y0,x1,y1, ... etc.
  pcontrol.init(number_of_waypoints, waypoint_dimensions, waypoints);*/
  
  const float origin_lat = 34.106465;
  const float origin_lon = -117.712488;
  state_estimator.init(origin_lat, origin_lon);

  bigMotor.init();
  pressure.init();
  force.init(); 
  encoder.init();

  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
  printer.lastExecutionTime         = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
  imu.lastExecutionTime             = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  gps.lastExecutionTime             = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
  //adc.lastExecutionTime             = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
  state_estimator.lastExecutionTime = loopStartTime - LOOP_PERIOD + STATE_ESTIMATOR_LOOP_OFFSET;
  openLoop.lastExecutionTime        = loopStartTime - LOOP_PERIOD + P_CONTROL_LOOP_OFFSET;
  bigMotor.lastExecutionTime        = loopStartTime - LOOP_PERIOD + BIG_MOTOR_LOOP_OFFSET;
  pressure.lastExecutionTime        = loopStartTime - LOOP_PERIOD + PRESSURE_LOOP_OFFSET;
  force.lastExecutionTime           = loopStartTime - LOOP_PERIOD + FORCE_LOOP_OFFSET;
  encoder.lastExecutionTime         = loopStartTime - LOOP_PERIOD + ROTARY_ENCODER_LOOP_OFFSET;
  logger.lastExecutionTime          = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
}



//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime=millis();

  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    //printer.printValue(0,adc.printSample());
    printer.printValue(1,logger.printState());
    printer.printValue(2,gps.printState());   
    printer.printValue(3,state_estimator.printState());     
    //printer.printValue(4,pcontrol.printWaypointUpdate());
    printer.printValue(5,openLoop.printState());
    printer.printValue(6,motor_driver.printState());
    //printer.printValue(7,imu.printRollPitchHeading());        
    //printer.printValue(8,imu.printAccels());
    printer.printValue(9,bigMotor.printState());
    printer.printValue(8,pressure.printState());
    printer.printValue(4, force.printState());
    printer.printValue(7, encoder.printState());
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  if ( currentTime-openLoop.lastExecutionTime > LOOP_PERIOD ) {
    openLoop.lastExecutionTime = currentTime;
    openLoop.calculateControl(&state_estimator.state);
    motor_driver.driveForward(openLoop.uL,openLoop.uR);
  }

  /*if ( currentTime-adc.lastExecutionTime > LOOP_PERIOD ) {
    adc.lastExecutionTime = currentTime;
    adc.updateSample(); 
  }*/

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
  /*if (currentTime-led.lastExecutionTime > LOOP_PERIOD) {
    led.lastExecutionTime = currentTime;
    led.flashLED();
  }*/

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

  if (currentTime- logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}

