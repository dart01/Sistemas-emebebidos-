#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>
#include <stdlib.h>  // Para la función atoi

void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0;
    while ((SysTick->CTRL & 0x00010000) == 0);
}

void SysTick_ms(uint32_t x) {
    for (uint32_t i = 0; i < x; i++) {
        SysTick_Wait(16000);
    }
}

void configuracionPWMmotoresDC() {
    // Habilitar reloj para GPIOB y configurar PB6, PB7, PB8, PB9
    RCC->AHB1ENR |= (1 << 1);  // Habilitar reloj GPIOB

    // Configurar PB6 y PB8 en modo alterno para PWM (TIM4)
    GPIOB->MODER |= (0b10 << 12) | (0b10 << 16);  // PB6 y PB8 como AF (modo alternativo)
    GPIOB->AFR[0] |= (2 << 24);  // PB6 en AF2 (TIM4_CH1)
    GPIOB->AFR[1] |= (2 << 0);   // PB8 en AF2 (TIM4_CH3)

    // Configurar PB7 y PB9 como salidas para controlar la dirección
    GPIOB->MODER |= (0b01 << 14) | (0b01 << 18);  // PB7 y PB9 como salida
    GPIOB->OTYPER &= ~((1 << 7) | (1 << 9));      // Push-pull para PB7 y PB9

    // Configurar TIM4 para generar PWM
    RCC->APB1ENR |= (1 << 2);  // Habilitar TIM4
    TIM4->PSC = 160 - 1;       // Prescaler para ajustar la frecuencia
    TIM4->ARR = 1000 - 1;      // Periodo del PWM (1 kHz aproximadamente)

    // Configurar PWM en modo PWM1
    TIM4->CCMR1 |= (0b110 << 4);  // Modo PWM1 en canal 1 (PB6)
    TIM4->CCMR2 |= (0b110 << 4);  // Modo PWM1 en canal 3 (PB8)
    
    TIM4->CCER |= (1 << 0) | (1 << 8);  // Habilitar canales CH1 y CH3
    TIM4->CR1 |= (1 << 0);              // Activar TIM4
}

void setMotor(uint8_t motor, uint8_t sentido, uint8_t velocidad_porcentaje) {
    // Convertir el porcentaje de velocidad a valor de duty cycle
    uint16_t duty_cycle = (velocidad_porcentaje * 999) / 100;

    if (motor == 1) {  // Motor 1 en PB6 y dirección en PB7
        if (sentido) {
            GPIOB->ODR |= (1 << 7);   // PB7 en HIGH para una dirección
        } else {
            GPIOB->ODR &= ~(1 << 7);  // PB7 en LOW para otra dirección
        }
        TIM4->CCR1 = duty_cycle;  // PWM en PB6 para Motor 1
    } else if (motor == 2) {  // Motor 2 en PB8 y dirección en PB9
        if (sentido) {
            GPIOB->ODR |= (1 << 9);   // PB9 en HIGH para una dirección
        } else {
            GPIOB->ODR &= ~(1 << 9);  // PB9 en LOW para otra dirección
        }
        TIM4->CCR3 = duty_cycle;  // PWM en PB8 para Motor 2
    }
}

int main() {
    // Configurar PWM y GPIOs para los motores
    configuracionPWMmotoresDC();

    while (1) {
			setMotor(2, 1, 100);   // Motor 1, dirección 0, velocidad media
			
			setMotor(1, 1, 100);   // Motor 1, dirección 0, velocidad media
//			setMotor(1, 1, 100);  // Motor 1 en reversa, velocidad al 100%
//        setMotor(2, 1, 100);  // Motor 2 en reversa, velocidad al 100%
//				

    }
}
