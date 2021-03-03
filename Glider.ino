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

//Global timer initialisatoin
unsigned long start_time = millis();

//IMU functions
bool IMU_data_ready;
void new_data();
void new_data() {
	IMU_data_ready = true;
}

void setup() {

  autopilot.init();
  control_output.init();
  imu.init();
  gps.init();

  Serial.begin(serial_baud_rate);
  Serial.print("Init methods complete!");

  //Copied IMU Here
  Serial.begin(DEFAULT_BAUD);

  Wire.begin();
  Wire.setClock(1000000);

  pinMode(IMU_IRQ_PIN, INPUT);

  if (imu.init(Wire)) {
    Serial.println("IMU online.");
  }
  else {
    Serial.println("Error connecting to IMU.");
  }

  attachInterrupt(digitalPinToInterrupt(IMU_IRQ_PIN), new_data, RISING);

  delay(500);
  interrupts();

}

//IMU varaibles
float gyro[3];
float accel[3];
float mag[3];
float quat[4];



void loop() {

  //Refresh the member variables from each sensor class
  if (IMU_data_ready) {
  IMU_data_ready = false;
  imu.update(accel, gyro, quat);
  Serial.printf("accel: %7.4f, %7.4f, %7.4f gyro: %7.4f, %7.4f, %7.4f \r\n\n", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
  }

  gps.update();

  //Update the member variables for the autopilot object
  autopilot.update(imu, gps);

  //Set the servo setpoint/output computed by the autopilot to the servos
  control_output.update(autopilot);


}
