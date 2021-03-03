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

    int rudder_calc = 0;



    AUTOPILOT(){

    }

    ~AUTOPILOT(){


    }

    bool init(){

    }


    bool update(IMU imu, GPS gps){
      //What is the aircraft's O&P now

      //Get distance from target to glider

      //Get bearing to target


      //gps.longitude
      //gps.lattitude
      //imu.heading
      //imu.acceleration


      //Determine new outputs to point aircraft at target

      //Servo test code (Servo sweep)
      if (millis() - last_time > 10){
        rudder_calc = rudder_calc + 1;
        last_time = millis();
      }


      if (rudder_calc >= 180){
        rudder_calc = 0;
      }

      //Constrain the values and make sure they are within the servo parameters
      //Set the latest targets to the member variables
      aileron_target = constrain(aileron_target, servo_lower_limit, servo_upper_limit);
      elevator_target = constrain(elevator_target, servo_lower_limit, servo_upper_limit);
      rudder_target = constrain(rudder_calc, servo_lower_limit, servo_upper_limit);



    }

  private:
      unsigned long last_time = millis();

};
