#pragma once
#include "IMU.hpp"
#include "GPS.hpp"
#include "config.hpp"

class AUTOPILOT {
  public:

    //Servo parameters
    int aileron_target = 0;
    int elevator_target = 0;
    int rudder_target = 0;


    AUTOPILOT(){

    }

    ~AUTOPILOT(){


    }

    bool init(){

    }


    bool update(IMU imu, GPS gps){
      //What is the aircraft's O&P now

      //Determine new outputs to make it to goal

      //Constrain the values and make sure they are within the servo parameters
      //Set the latest targets to the member variables
      aileron_target = constrain(aileron_target, servo_lower_limit, servo_upper_limit);
      elevator_target = constrain(elevator_target, servo_lower_limit, servo_upper_limit);
      rudder_target = constrain(rudder_target, servo_lower_limit, servo_upper_limit);



    }

  private:


};
