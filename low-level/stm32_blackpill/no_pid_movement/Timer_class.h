#ifndef TIMER_CLASS_H
#define TIMER_CLASS_H
#include <stm32f4xx.h>

//
struct GPIO_Config {
    GPIO_TypeDef* port_ch1;
    uint8_t pin_ch1;
    GPIO_TypeDef* port_ch2;
    uint8_t pin_ch2;
    uint8_t AFn;
};


//Functions declarations 
GPIO_Config getGPIOConfigForTimer(TIM_TypeDef* tim) ;
inline void enableTimerClock(TIM_TypeDef* tim) ;

//Timer class
class timer{
 
  private :
    uint32_t timer_count = 0;
    TIM_TypeDef* TIM;
    GPIO_Config pin_configs;

  public:
    //Constructor
    timer(TIM_TypeDef* timer) ;
    uint32_t get_count();


};
#endif