#include "Timer_class.h"


//mapping timers instance to GPIO pins
//returns a GPIO_config structure
GPIO_Config getGPIOConfigForTimer(TIM_TypeDef* tim)
{
    //Timers for encoder mode
    if (tim == TIM2) return {GPIOA, 15, GPIOB, 3, 1};
    if (tim == TIM3) return {GPIOA, 6, GPIOA, 7, 2};
    if (tim == TIM4) return {GPIOB,6, GPIOB,7,2};
    if (tim == TIM5) return {GPIOA, 0, GPIOA, 1, 2};

    //return invalid if unsupported
    return {nullptr, 0xFF, nullptr, 0xFF, 0xFF};
}


//enabling clocks for timers -pg137 in RM-
inline void enableTimerClock(TIM_TypeDef* tim) {
    //encoder
    if (tim == TIM2)      RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (tim == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if (tim == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    else if (tim == TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    //motor driving
    else if (tim == TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    else if (tim == TIM9) RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
}


//Constructor implementation 
timer :: timer(TIM_TypeDef* timer)  : TIM(timer)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    //if motor driving , skip encoder steps
    if (TIM == TIM1 || TIM == TIM9)
    {
        return ;
    }

    //encoder setup
    pin_configs = getGPIOConfigForTimer(TIM);

    //Enabling timer and GPIO clocks , -pg14 in datasheet-
    enableTimerClock(TIM);

    //----------------------
    //configure encoder mode 
    //----------------------
    
    //writing in SMS -pg279 in RM-
    TIM->SMCR &= ~TIM_SMCR_SMS; //clearing sms
    TIM->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;  //encoder mode 3 , calculating on both edges

    //configuring channels 1,2 as input capture ,
    TIM->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

    TIM->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    //configuring pins  to alternate functions
    //resetting then writing on the 2 bits representing mode
    //pg 158 in RM
    pin_configs.port_ch1->MODER &= ~(0b11 << (2 * pin_configs.pin_ch1)); 
    pin_configs.port_ch1->MODER |=  (0b10 << (2 * pin_configs.pin_ch1));

    pin_configs.port_ch2->MODER &= ~(0b11 << (2 * pin_configs.pin_ch2));
    pin_configs.port_ch2->MODER |=  (0b10 << (2 * pin_configs.pin_ch2));

    //configuring to the correct AF mode , AFH for pins >= 8
    //AFL for pins < 8 
    //pg 151 in RM
    pin_configs.port_ch1->AFR[(pin_configs.pin_ch1 <8) ? 0 : 1] &= 
        ~(0b1111 << ((pin_configs.pin_ch1 % 8) * 4));
    pin_configs.port_ch1->AFR[(pin_configs.pin_ch1 < 8) ? 0 : 1] |=
        (pin_configs.AFn << (4 * (pin_configs.pin_ch1 % 8)));
    pin_configs.port_ch2->AFR[(pin_configs.pin_ch2 < 8) ? 0 : 1] &=
        ~(0b1111 << (4 * (pin_configs.pin_ch2 % 8)));
    pin_configs.port_ch2->AFR[(pin_configs.pin_ch2 < 8) ? 0 : 1] |=
        (pin_configs.AFn << (4 * (pin_configs.pin_ch2 % 8)));

    //initializing timer
    TIM->CR1 |= TIM_CR1_CEN;      //enabling timer
    TIM->CNT = 0;                //initializing count on the register as 0
    TIM->ARR = 0xFFFFFFFF;       //maximum value before resetting
}

uint32_t timer::get_count()
{
    return TIM->CNT ;
}


