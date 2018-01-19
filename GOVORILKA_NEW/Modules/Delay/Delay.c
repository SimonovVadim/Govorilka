#include "Delay.h"

void delay_us(uint16_t Delay)
{
  TIM7->CR1 = 0;
  TIM7->PSC = 23;
  TIM7->ARR = Delay;
  TIM7->EGR |= TIM_EGR_UG;
  TIM7->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
  
  // Эта строчка появилась
  while(TIM7->CR1&TIM_CR1_CEN != 0);
  
}

void delay_ms(uint16_t delay)
{
  for(volatile uint16_t ze = 0 ; ze < delay ; ze++)
  { delay_us(1000); }

}
