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
    // Habilitar reloj para GPIOE y configurar PE9, PE11, PE13, PE14
    RCC->AHB1ENR |= (1 << 4);  // Habilitar reloj GPIOE

    // Configurar PE9, PE11, PE13 y PE14 en modo alterno para PWM (TIM1)
    GPIOE->MODER |= (0b10 << 18) | (0b10 << 22) | (0b10 << 26) | (0b10 << 28);  // PE9, PE11, PE13, PE14 como AF
    GPIOE->AFR[1] |= (1 << 4) | (1 << 12) | (1 << 20) | (1 << 24);  // AF1 para TIM1_CH1, CH2, CH3 y CH4

    // Configurar TIM1 para generar PWM
    RCC->APB2ENR |= (1 << 0);  // Habilitar TIM1
    TIM1->PSC = 160 - 1;       // Prescaler para ajustar la frecuencia
    TIM1->ARR = 1000 - 1;      // Periodo del PWM (1 kHz aproximadamente)

    // Configurar PWM en modo PWM1 para los canales correspondientes
    TIM1->CCMR1 |= (0b110 << 4) | (0b110 << 12);  // Modo PWM1 en canales 1 y 2 (PE9 y PE11)
    TIM1->CCMR2 |= (0b110 << 4) | (0b110 << 12);  // Modo PWM1 en canales 3 y 4 (PE13 y PE14)
    
    TIM1->CCER |= (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);  // Habilitar canales CH1, CH2, CH3 y CH4
    TIM1->CR1 |= (1 << 0);              // Activar TIM1
    TIM1->BDTR |= (1 << 15);            // Activar el bit MOE (Main Output Enable) para TIM1
}

void setMotor(uint8_t motor, uint8_t sentido, uint8_t velocidad_porcentaje) {
    // Convertir el porcentaje de velocidad a valor de duty cycle
    uint16_t duty_cycle = (velocidad_porcentaje * 999) / 100;

    if (motor == 1) {  // Motor derecho en PE9 y PE11
        if (sentido) {
            TIM1->CCR1 = duty_cycle;  // PWM en PE9 para mover hacia adelante
            TIM1->CCR2 = 0;           // PE11 apagado
        } else {
            TIM1->CCR1 = 0;           // PE9 apagado
            TIM1->CCR2 = duty_cycle;  // PWM en PE11 para mover hacia atrás
        }
    } else if (motor == 2) {  // Motor izquierdo en PE13 y PE14
        if (sentido) {
            TIM1->CCR3 = duty_cycle;  // PWM en PE13 para mover hacia adelante
            TIM1->CCR4 = 0;           // PE14 apagado
        } else {
            TIM1->CCR3 = 0;           // PE13 apagado
            TIM1->CCR4 = duty_cycle;  // PWM en PE14 para mover hacia atrás
        }
    }
}

int main() {
    // Configurar PWM y GPIOs para los motores
    configuracionPWMmotoresDC();

    while (1) {
//        setMotor(1, 1, 100);   // Motor derecho, dirección hacia adelante, velocidad al 100%
//        setMotor(2, 1, 100);   // Motor izquierdo, dirección hacia adelante, velocidad al 100%
//        SysTick_ms(1000);

        setMotor(1, 0, 50);    // Motor derecho, dirección hacia atrás, velocidad al 50%
        setMotor(2, 0, 50);    // Motor izquierdo, dirección hacia atrás, velocidad al 50%
        SysTick_ms(1000);
    }
}
