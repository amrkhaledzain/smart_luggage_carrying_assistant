#include "encoder.h"

//constructor 
Encoder::Encoder(timer& t, uint32_t resolution, float circum)
    : hwTimer(t), last_count(0), rps(0), 
    resolution(resolution), wheel_circum(circum),last_time(0) {
    }

//returns number of ticks using the timer class
uint32_t Encoder::get_count() {
    return hwTimer.get_count();
}

//resets calculation values
void Encoder::reset() {
    last_count = 0;
    rps = 0;
}

//returns direction depending on speed_ticks , different between counts
int Encoder::get_direction() {
    if (rps > 0) return 1;
    if (rps < 0) return -1;
    return 0;
}

//updating speed
void Encoder::update_speed() {
    uint32_t current_time = millis();
    uint32_t dt_ms = current_time - last_time; //calculate time_diff

    //to avoid overflow , redundant readings 
    if (dt_ms < TIME_THRESHOLD) return ; 

    //
    uint32_t current_count = get_count() ;
    int32_t count_difference = (int32_t)current_count - (int32_t)last_count;

    //handle over/under flow
    //if diff is very large add or subtract 2^^16 to get the correct values
    if (count_difference > MAX_COUNT)   count_difference -= UNSIGNED_MAX_COUNT;
    if (count_difference < -MAX_COUNT)  count_difference += UNSIGNED_MAX_COUNT;

    //counts per second
    //rps = ((count_difference / (int32_t)dt_ms) * (1000 / resolution));
    rps = count_difference * 1000 / (int32_t)dt_ms  ;

    //updating values for next calculation 
    last_count = current_count;
    last_time = current_time ;
}

//last speed in ticks/sec
int32_t Encoder::get_rps() {
    return rps ;
}

//Converting to RPM
float Encoder::get_speed_rpm() {
    if (resolution == 0) return 0;
    return (rps * 60) / resolution;
}

// rps = ((counts / 00000000000000000000000000000000000000000000000 ) * 1 / cpr ) 
// rpm = 60 * rps 

//converting to linear speed , wheel_circum in m
float Encoder::get_linear_speed() {
    if (resolution == 0) return 0;
    return (rps / (float)resolution) * wheel_circum;
}

