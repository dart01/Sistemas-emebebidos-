#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

// Variables globales para ADC y voltaje
uint16_t digital;
float voltaje;

void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0;
    while ((SysTick->CTRL & 0x00010000) == 0);
}

void SysTick_ms(uint32_t x) {
    for (uint32_t i = 0; i < x; i++) {
        SysTick_Wait(16000);  // Espera 1 ms
    }
}

void configuracionADCfotoCeldaLdr() {
    // Habilitar el reloj para GPIOC y ADC2
    RCC->AHB1ENR |= (1 << 2);       // Habilitar el reloj para GPIOC
    RCC->APB2ENR |= (1 << 9);       // Activar el reloj del ADC2
    
    // Configuración de PC3 como modo analógico
    GPIOC->MODER |= (0b11 << 6);    // PC3 (ADC123_IN13) en modo analógico

    // Configuración del ADC2
    ADC2->CR2 |= (1 << 0);          // Habilitar ADC2
    ADC2->CR2 |= (1 << 10);         // EOC (End of Conversion)
    ADC2->CR1 &= ~(0b11 << 24);     // Limpiar bits de resolución
    ADC2->CR1 |= (1 << 24);         // Configurar resolución de 10 bits
    ADC2->SMPR1 |= (1 << 3);        // Tiempo de muestreo de 15 ciclos ADCCLK para canal 13 (PC3)
    ADC2->SQR3 &= ~(0b11111);       // Limpiar secuencia de conversión regular
    ADC2->SQR3 |= (13 << 0);        // Seleccionar canal 13 (PC3) para la primera conversión
}

void conversionADC_Ldr() {
    // Configuración de PD1 como salida para el LED
    RCC->AHB1ENR |= (1 << 3);       // Habilitar reloj para GPIOD
    GPIOD->MODER &= ~(0b11 << (3 * 2));  // Limpiar bits 6 y 7 para PD3
    GPIOD->MODER |= (0b01 << (3 * 2));   // Configurar PD3 como salida (01)
    GPIOD->OSPEEDR &= ~(0b11 << (3 * 2)); // Limpiar bits de velocidad para PD3
    GPIOD->OSPEEDR |= (0b10 << (3 * 2));  // Configurar alta velocidad para PD3 (10)

    // Iniciar conversión ADC y esperar a que termine
    ADC2->CR2 |= (1 << 30);         // Iniciar conversión A/D  
    while ((ADC2->SR & (1 << 1)) == 0) {}  // Esperar final de conversión  
    ADC2->SR &= ~(1 << 1);          // Limpiar bit EOC  

    // Leer el valor del ADC y calcular voltaje
    digital = ADC2->DR;             // Leer valor digital del ADC
    voltaje = (float)digital * (3.3 / 1023.0); // Calcular voltaje

    // Comparar el voltaje y encender/apagar el LED en PD1
    if (voltaje >= 2.1) {
        GPIOD->ODR |= (1 << 3);      // Encender LED en PD1
    } else {
        GPIOD->ODR &= ~(1 << 3);     // Apagar LED en PD1
    }
}

int main() {
    // Configuración de SysTick
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);

    // Configuración inicial del ADC
    configuracionADCfotoCeldaLdr();

    // Bucle principal
    while (1) {		
        conversionADC_Ldr();
        SysTick_ms(500); // Espera 500 ms entre cada lectura para darle tiempo a la conversión
    }
}
