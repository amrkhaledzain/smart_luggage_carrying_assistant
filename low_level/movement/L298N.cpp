#include <Arduino.h>
#include "L298N.hpp"

 L298N::L298N(int ena, int pin1, int pin2){
      Pin1 = pin1;
      Pin2 = pin2;
      ENA = ena;
 };
    void L298N::Setup(){

      pinMode(Pin1, OUTPUT);
      pinMode(Pin2, OUTPUT);
      pinMode(ENA, OUTPUT);
    };
    void L298N::Move(int Speed){
      if (Speed > 0){
        digitalWrite(Pin1, HIGH);
        digitalWrite(Pin2, LOW);
        analogWrite(ENA, Speed); 
      }
      else {
        digitalWrite(Pin1, LOW);
        digitalWrite(Pin2, HIGH);
        analogWrite(ENA, -Speed); 
      }

    };
    void L298N::Stop() {
      digitalWrite(Pin1, LOW);
      digitalWrite(Pin2, LOW);
      analogWrite(ENA,0);
    };