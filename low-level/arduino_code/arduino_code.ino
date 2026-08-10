#include <Arduino.h>
#include "L298N.hpp"

L298N motor_FL(3, 2, 4);   // Front Left
L298N motor_FR(5, 7, 8);   // Front Right
L298N motor_RL(6, 12, 13); // Rear Left
L298N motor_RR(11, 9, 10); // Rear Right

void setup() {
  Serial.begin(9600);

  motor_FL.Setup();
  motor_FR.Setup();
  motor_RL.Setup();
  motor_RR.Setup();

  motor_FL.Stop();
  motor_FR.Stop();
  motor_RL.Stop();
  motor_RR.Stop();
}

void loop() {

    if (Serial.available() > 0) {

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {

      int speed_FL = Serial.parseInt();
      int speed_FR = Serial.parseInt();
      int speed_RL = Serial.parseInt();
      int speed_RR = Serial.parseInt();

      motor_FL.Move(speed_FL);
      motor_FR.Move(speed_FR);
      motor_RL.Move(speed_RL);
      motor_RR.Move(speed_RR);
    }
  }
}