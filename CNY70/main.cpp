#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>

uint8_t flag = 0, i;
unsigned char d;
char text[16];  // Cambiado de 11 a 16 para permitir mayor longitud de cadena
uint16_t digital;
float voltaje;

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

// Funciones para manejar el LCD

#define CD 0x01
#define RH 0x02
#define EMS 0x06
#define DC 0x0F
#define DSr 0x1C
#define DSl 0x18
#define FS 0x28 // 0x38 (8 bits)
#define RAW1 0x80
#define RAW2 0xC0
#define time 10


// (las funciones LCD ya están correctas y no necesitan modificaciones)

void LCD(unsigned char val) {
    GPIOC->ODR |= 1UL << 8; // PA8 (E)
    SysTick_ms(10);
    GPIOA->ODR = (GPIOA->ODR & ~0xF0) | ((val & 0x0F) << 4); // PA4-PA7 (D4-D7)
    SysTick_ms(1);
    GPIOC->ODR &= ~(1UL << 8); // PA8 (E)
    SysTick_ms(1);
}

void settingsLCD(unsigned char val) {
    GPIOC->ODR &= ~(1UL << 9); // PA9 (RS)
    LCD(val >> 4);
    LCD(val & 0x0f);
}

void writeLCD(unsigned char val) {
    GPIOC->ODR |= 1UL << 9; // PA9 (RS)
    LCD(val >> 4);
    LCD(val & 0x0f);
}

void actualizarMensajeLCD(char* mensaje) {
    settingsLCD(0x80);  // Posicionar el cursor en la primera línea
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}
void setCursorLCD(uint8_t row, uint8_t col) {
    uint8_t pos = 0;
    if (row == 1) {
        pos = 0x80 + (col - 1);  // Fila 1
    } else if (row == 2) {
        pos = 0xC0 + (col - 1);  // Fila 2
    }
    settingsLCD(pos);  // Enviar el comando de posición
}

// Ahora para escribir el valor en la columna 8, fila 1:
void actualizarMensajeLCDEnColumna(char* mensaje) {
    setCursorLCD(1, 9);  // Fila 1, Columna 8
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en el LCD
        SysTick_ms(2);  // Agregar un retraso pequeño para asegurar que el LCD procese
    }
}


// Configuración general
void settings() {
    // Enable the GPIOB clock 
    RCC->AHB1ENR |= ((1<<0)|(1<<1)|(1<<2));  // GPIO_A-B-C

    //Systick
    SysTick->LOAD = 0x00FFFFFF; // Inicializar con el valor máximo de 24 bits
    SysTick->CTRL |= (0b101); // Fuente de reloj del procesador (AHB) y habilitar contador

    // Pines LCD (PA4, PA5, PA6, PA7 para D4, D5, D6, D7)
    GPIOA->MODER |= (1 << (4 * 2)) | (1 << (5 * 2)) | (1 << (6 * 2)) | (1 << (7 * 2)); // Salida PA4-PA7
    GPIOA->OTYPER |= 0;
    GPIOA->OSPEEDR |= 0x0000FF00; // PA4-PA7 como Very High Speed
    GPIOA->PUPDR |= 0x0000AA00; // PA4-PA7 con pull-up

    // Pines PC8 y PC9 ya configurados
    GPIOC->MODER |= (1 << (8 * 2)) | (1 << (9 * 2)); // PC8 y PC9 como salida
    GPIOC->OTYPER |= 0;
    GPIOC->OSPEEDR |= 0x000FFF00; // PC8 y PC9 como Very High Speed
    GPIOC->PUPDR |= 0x000AAA00; // PC8 y PC9 como Pull-up

    // Configuración LCD
    settingsLCD(0x02);
    settingsLCD(EMS);		
    settingsLCD(DC);
    settingsLCD(FS);
    settingsLCD(CD);
}



int main() {
		settings(); 
    //GPIOs
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


    //ADC
    GPIOC->MODER |= (0b11<<0); // PC0 (ADC123_IN10) modo analógico		
    RCC->APB2ENR |= (1<<9); // Activar reloj del ADC2
    ADC2->CR2 |= ((1<<10)|(1<<0)); // EOC y habilitar ADC
    ADC2->CR1 &= ~(0b11<<24); // Limpiar bits de resolución
    ADC2->CR1 |= (1<<24); // Resolución de 10 bits
    ADC2->SMPR1 |= (1<<0); // 15 ciclos ADCCLK en canal 10 (PC0)
    ADC2->SQR3 &= ~(0b11111<<0); // Limpiar secuencia regular
    ADC2->SQR3 |= (0b1010<<0); // Canal 10 en primera conversión

    // Loop principal
    while(1){
    GPIOB->ODR |= 1<<0;  
    SysTick_ms(500);  
    GPIOB->ODR &= ~(1<<0);  
    SysTick_ms(500);

    // Iniciar conversión ADC y esperar a que termine
    ADC2->CR2 |= (1<<30); // Iniciar conversión A/D  
    while(((ADC2->SR & (1<<1)) >> 1) == 0){} // Esperar final conversión  
    ADC2->SR &= ~(1<<1); // Limpiar bit EOC  
    digital = ADC2->DR;  
    voltaje = (float)digital * (3.3 / 1023.0); // Calcular voltaje  

    // Mostrar el valor en la columna 8 de la fila 1 del LCD
    sprintf(text, "V:%.2f", voltaje);  // Formatear cadena para mostrar  
    actualizarMensajeLCDEnColumna(text);  // Mostrar el valor en la pantalla LCD en columna 8
}

}
