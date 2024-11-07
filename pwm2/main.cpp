#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>
#include <stdlib.h>  // Para la función atoi
#define BUFFER_SIZE 3
uint8_t flag = 0;


void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0;
    while (((SysTick->CTRL & 0x00010000) >> 16) == 0);
}

void SysTick_ms(uint32_t x) {
    for (uint32_t i = 0; i < x; i++) {
        SysTick_Wait(16000);
    }
}

void configuracionPWMservo2() {
    // Habilitar el reloj para GPIOB y configurar PB6, PB7, PB8, PB9 para función alternativa
    RCC->AHB1ENR |= (1 << 1);  // Habilitar reloj GPIOB
    GPIOB->MODER |= (0b1 << 13) | (0b1 << 15) | (0b1 << 17) | (0b1 << 19);  // Modo AF para PB6, PB7, PB8, PB9
    GPIOB->AFR[0] |= (1 << 25) | (1 << 29);  // PB6 y PB7 en AF2 (TIM4)
    GPIOB->AFR[1] |= (1 << 1) | (1 << 5);  // PB8 y PB9 en AF2 (TIM4)

    // Configurar TIM4 para PWM
    RCC->APB1ENR |= (1 << 2);  // Habilitar reloj para TIM4
    TIM4->PSC = 4;             // Prescaler para obtener un tiempo de ciclo adecuado
    TIM4->ARR = 63999;         // Ciclo de PWM a 20ms (ajustar según necesidades de motores)
    TIM4->CR1 |= (1 << 0);     // Habilitar contador de TIM4

    // Configurar los canales de TIM4 en modo PWM
    TIM4->CCMR1 |= (0b110 << 12) | (0b110 << 4);  // Configurar PWM en CH1 (PB6) y CH2 (PB7)
    TIM4->CCMR2 |= (0b110 << 12) | (0b110 << 4);  // Configurar PWM en CH3 (PB8) y CH4 (PB9)
    
    TIM4->CCER |= (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);  // Habilitar canales CH1, CH2, CH3, CH4

    TIM4->EGR |= (1 << 0);  // Actualizar los registros
}

int main() {

    // Systick
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);
	
		
    configuracionPWMservo2();

    // Bucle principal
    while (1) {
//        GPIOB->ODR |= (1 << 0);  // Enciende el LED en PB0
//        SysTick_ms(100);   // Esperar 1 segundo
//        GPIOB->ODR &= ~(1 << 0);  // Apaga el LED en PB0
//        SysTick_ms(100);   // Esperar 1 segundo
				 // Ahora puedes usar el valor entero recibido
			GPIOB->ODR &= ~(1 << 7);  // PB9 en LOW para otra dirección
       TIM4->CCR1 = 4800;  // Ángulo 180° en PB6
			SysTick_ms(1000);   // Esperar 1 segundo
			
			GPIOB->ODR &= ~(1 << 9);  // PB9 en LOW para otra dirección
			TIM4->CCR3 = 8000;  // Ángulo 180° en PB6
			SysTick_ms(1000);   // Esperar 1 segundo
    }
}
