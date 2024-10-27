#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>

uint8_t flag = 0, i, cont = 0;
unsigned char d;
char text[16];  // Cadena para mostrar los minutos y segundos en la LCD
uint16_t digital;
float voltaje;
uint32_t a, b, c;
int segundos = 0;
int minutos = 0;  // Variable para contar los minutos

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

void apagarLCD(void) {
    settingsLCD(0x01); // Limpiar pantalla
    settingsLCD(0x08); // Apagar la pantalla (apaga display y cursor)
}

void limpiarLineaLCD(int linea) {
    if (linea == 1) {
        settingsLCD(RAW1);  // Posiciona el cursor en la primera línea
    } else if (linea == 2) {
        settingsLCD(RAW2);  // Posiciona el cursor en la segunda línea
    }
}

void actualizarMensajeLCD(char* mensaje) {
    limpiarLineaLCD(1);        // Limpiar la primera línea de la pantalla LCD
    settingsLCD(RAW1);         // Posicionar el cursor en la primera línea
    
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}

void settings() {
    // Habilitar el reloj de GPIOA, GPIOB y GPIOC
    RCC->AHB1ENR |= ((1<<0)|(1<<1)|(1<<2));  // GPIO_A-B-C

    // Systick
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

extern "C"{
    void TIM3_IRQHandler(void){ // Interrupción de Timer 
        TIM3->SR &= ~(1<<0); // Limpiar la bandera de interrupción de TIM3 
        cont += 1;
        if (cont == 10) {  // 10 interrupciones = 1 segundo (si 100 ms por interrupción)
            cont = 0;
            segundos++;  // Incrementar segundos
            if (segundos == 60) {
                segundos = 0;
                minutos++;  // Incrementar minutos cuando segundos lleguen a 60
            }
            flag = 1;  // Activar la bandera para actualizar la LCD
        }
    }
}

int main(){
    // GPIOs
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

    // Systick
    SysTick->LOAD = 0x00FFFFFF; 
    SysTick->CTRL |= (0b101);

    // Configuración del Timer3 (para contar 1 segundo)
    RCC->APB1ENR |= (1<<1); // Habilitar el reloj de TIM3 
    TIM3->PSC = 16000 - 1;  // Prescaler para contar en ms (16 MHz / 16,000 = 1 kHz)
    TIM3->ARR = 100 - 1;    // 100 ms por interrupción
    TIM3->DIER |= (1<<0);   // Habilitar interrupciones de actualización
    TIM3->CR1 |= (1<<0);    // Iniciar el temporizador
    NVIC_EnableIRQ(TIM3_IRQn);  // Habilitar IRQ para TIM3

    // Inicialización de la LCD
    settings();

    // Bucle principal
    while(1){
        GPIOB->ODR |= 1<<0; 
        SysTick_ms(500);
        GPIOB->ODR &= ~(1<<0);
        SysTick_ms(500);

        if(flag == 1){
            flag = 0;
            // Formatear el mensaje con minutos y segundos
            sprintf(text, "%02d:%02d", minutos, segundos);
            actualizarMensajeLCD(text);  // Actualizar la pantalla LCD
        }  
    }
}
