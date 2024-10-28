#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>

uint8_t flag = 0;
unsigned char d;
char text[22];
uint16_t digital;
float DC, servo;
uint32_t a, b, c;

void SysTick_Wait(uint32_t n){
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0;
    while (((SysTick->CTRL & 0x00010000) >> 16) == 0);
}

void SysTick_ms(uint32_t x){
    for (uint32_t i = 0; i < x; i++){
        SysTick_Wait(16000);
    }
}

extern "C"{
    
    void USART3_IRQHandler(void){ 
        if(((USART3->ISR & 0x20) >> 5) == 1){
            d = USART3->RDR;
            if(d == 'a'){
                flag = 1;
            }
        }
    }
}


int main(){

    //GPIOs fuera del while
    RCC->AHB1ENR |= ((1<<1)|(1<<2));

    GPIOB->MODER &= ~((0b11<<0)|(0b11<<14));
    GPIOB->MODER |= ((1<<0)|(1<<14));
    GPIOC->MODER &= ~(0b11<<26);

    GPIOB->OTYPER &= ~((1<<0)|(1<<7));
    GPIOB->OSPEEDR |= (((1<<1)|(1<<0)|(1<<15)|(1<<14)));
    GPIOC->OSPEEDR |= ((1<<27)|(1<<26));
    GPIOB->PUPDR &= ~((0b11<<0)|(0b11<<14));
    GPIOC->PUPDR &= ~(0b11<<26);
    GPIOC->PUPDR |= (1<<27);

    //Systick
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);

    // Interrupts
//    RCC->APB2ENR |= (1<<14);
//    SYSCFG->EXTICR[3] &= ~(0b1111<<4);
//    SYSCFG->EXTICR[3] |= (1<<5);
//    EXTI->IMR |= (1<<13);
//    EXTI->RTSR |= (1<<13);
//    NVIC_EnableIRQ(EXTI15_10_IRQn);
//    
    //UART Configuration (omitida para simplificar)

    //TIMER
//    RCC->APB1ENR |= (1<<1); 
//    TIM3->PSC = 24; 
//    TIM3->ARR = 63999;
//    TIM3->DIER |= (1<<0);
//    TIM3->CR1 |= (1<<0); 
//    NVIC_EnableIRQ(TIM3_IRQn);

    //PWM Configuration (TIM5)
    RCC->AHB1ENR |= (1<<0); //Enable the GPIOA clock (TIM5_CH1, TIM5_CH2, TIM5_CH3 and TIM5_CH4 are connected on PA0, PA1, PA2 and PA3, respectively)
    GPIOA->MODER |= (1<<7)|(1<<5)|(1<<3)|(1<<1); //Set alternate function for PA0, PA1, PA2, PA3
    GPIOA->AFR[0] |= (1<<13)|(1<<9)|(1<<5)|(1<<1); // Set AF2 for TIM5 channels
    RCC->APB1ENR |= (1<<3); // TIM5 clock enable
    TIM5->PSC = 4; //Prescaler for 20ms time
    TIM5->ARR = 63999; // Max count for 20ms
    TIM5->CR1 |= (1<<0); // Enable counting
    TIM5->CCMR1 |= (0b110<<12)|(0b110<<4); // Set PWM mode for CH1 and CH2
    TIM5->CCMR2 |= (0b110<<12)|(0b110<<4); // Set PWM mode for CH3 and CH4
    TIM5->CCER |= (1<<12)|(1<<8)|(1<<4)|(1<<0); // Enable all PWM channels
    TIM5->EGR |= (1<<0); // Reinitialize registers

    //UART
    //USART3->CR1 |= (1<<0);

    // Iniciar la secuencia del servo
    while(1){
        // Posición 0°
       TIM5->CCR1 = 1600;  // Aproximadamente 1 ms para 0°
        SysTick_ms(1000);   // Esperar 1 segundo
        
			// Posición 0°
       TIM5->CCR2 = 1600;  // Aproximadamente 1 ms para 0°
        SysTick_ms(1000);   // Esperar 1 segundo
        
			// Posición 0°
       TIM5->CCR3 = 1600;  // Aproximadamente 1 ms para 0°
        SysTick_ms(1000);   // Esperar 1 segundo
        
			// Posición 0°
       TIM5->CCR4 = 1600;  // Aproximadamente 1 ms para 0°
        SysTick_ms(1000);   // Esperar 1 segundo
        
			
        // Posición 90°
     //  TIM5->CCR1 = 4800;  // Aproximadamente 1.5 ms para 90°
       //SysTick_ms(1000);   // Esperar 1 segundo
       
        // Posición 180°
     // TIM5->CCR1 = 8000;  // Aproximadamente 2 ms para 180°
    //SysTick_ms(1000);   // Esperar 1 segundo
    }
}

