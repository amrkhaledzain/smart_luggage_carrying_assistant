#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <Timer_class.h>

//for overflowing
#define MAX_COUNT 32767 //maximum possible difference before detecting an overflow
#define UNSIGNED_MAX_COUNT 65536
#define TIME_THRESHOLD 1e-3

//class blueprint
class Encoder
{
  private:
    timer& hwTimer ;         //reference to hardware timer
    uint32_t last_count;     //last encoder count
    float rps ;    //rps
    uint32_t resolution;  //encoder resolution
    float wheel_circum;       //for linear speed and distance
    uint32_t last_time ;
    uint32_t dt_ms;


  public:
    Encoder(timer& t ,uint32_t resolution , float circum = 1.0);

    uint32_t get_count();
    float get_speed_rpm() ;
    void update_speed();

    //calculation methods
    int32_t get_rps() ;
    int get_direction();
    void reset();
    float get_linear_speed() ;
};

#endif