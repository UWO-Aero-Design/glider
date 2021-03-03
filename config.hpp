//File for constants and other parameters
#pragma once

//Constants--------------

//Target GPS coordinates
const float target_lat = 43.005482;
const float target_long = -81.271671;


//Serial Constants
const int serial_baud_rate = 9600;


//Output pin allocations
const int aileron_pin = 9;
const int elevator_pin = 10;
const int rudder_pin = 7;



//Maximum servo outputs
const int servo_upper_limit = 140;
const int servo_lower_limit = 40;

//IMU constants
const int DEFAULT_BAUD = 112500;
const int IMU_IRQ_PIN = 36;
