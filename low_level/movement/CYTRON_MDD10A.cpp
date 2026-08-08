#include <Arduino.h>
#include "CYTRON_MDD10A.hpp"

CYTRON_MDD10A:: CYTRON_MDD10A(int dir, int pwm){
      DIR = dir;
      PWM = pwm;
    };
    void CYTRON_MDD10A::Setup() {
      pinMode(DIR, OUTPUT);
      pinMode(PWM, OUTPUT);
    };
    void CYTRON_MDD10A::Move(int Speed) {
      if (Speed > 0){
        digitalWrite(DIR, HIGH);
        analogWrite(PWM, Speed);
      }
      else if(Speed < 0){ 
        digitalWrite(DIR, LOW);
        analogWrite(PWM, -Speed);  
      }else{
        Stop();
      }
    };
    void CYTRON_MDD10A::Stop() {
      analogWrite(PWM, 0);
    };