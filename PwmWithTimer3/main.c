#include <stm32f4xx.h>
#include "functions.h"

int main(void) {
    configure_gpio_pa6_alternate_push_pull();
    configure_pwm_ch1_20khz(TIM3);
    set_pulse_percentage(TIM3, 0x100);
    configure_timer2_with_IT();
    start_timer(TIM3);
    start_timer(TIM2);
    
    for(;;){}
			
    return 0;
}

void TIM2_IRQHandler(void) {
    static int pulse = 0;
    TIM2->SR &= ~TIM_SR_UIF;
    pulse = (pulse + 5) % 101;
    set_pulse_percentage(TIM3, pulse);
}
