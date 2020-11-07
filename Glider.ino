#include "Arduino.h"
#include "config.hpp"
#include "AUTOPILOT.hpp"
#include "CONTROL_OUTPUT.hpp"
#include "IMU.hpp"
#include "GPS.hpp"


//Class instances here
AUTOPILOT autopilot;
CONTROL_OUTPUT control_output;
IMU imu;
GPS gps;


void setup() {

  autopilot.init();
  control_output.init();
  imu.init();
  gps.init();

  Serial.begin(serial_baud_rate);
  Serial.print("Init methods complete!");

}


void loop() {

  //Refresh the member variables from each sensor class
  imu.update();
  gps.update();

  //Update the member variables for the autopilot object
  autopilot.update(imu, gps);

  //Set the servo setpoint/output computed by the autopilot to the servos
  control_output.update(autopilot);


}
