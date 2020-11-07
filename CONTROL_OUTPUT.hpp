#pragma once
#include "config.hpp"
#include "AUTOPILOT.hpp"
#include <Servo.h>

Servo aileron;
Servo elevator;
Servo rudder;


class CONTROL_OUTPUT {
  public:


    CONTROL_OUTPUT(){

    }

    ~CONTROL_OUTPUT(){


    }

    bool init(){
      aileron.attach(aileron_pin);
      elevator.attach(elevator_pin);
      rudder.attach(rudder_pin);
    }


    bool update(AUTOPILOT autopilot){

      //Write the data to the servos
      aileron.write(autopilot.aileron_target);
      elevator.write(autopilot.elevator_target);
      rudder.write(autopilot.rudder_target);

    }

  private:


};
















//Spacer
