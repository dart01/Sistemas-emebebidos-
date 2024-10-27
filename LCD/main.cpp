#include <stdio.h>
#include <stm32f7xx.h>

void SysTick_Wait(uint32_t n) {
		SysTick->LOAD = n - 1; // Carga el valor en el registro SysTick->VAL cuando el contador está habilitado
		SysTick->VAL = 0; // Limpia la bandera de cuenta
		while (((SysTick->CTRL & 0x00010000) >> 16) == 0); // Espera a que la bandera de cuenta sea 1
}

void SysTick_ms(uint32_t x) {
		for (uint32_t i = 0; i < x; i++) { // Espera por x milisegundos
				SysTick_Wait(16000); // 1ms
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

// Cadenas de caracteres terminadas con \0 automáticamente
char princ1[] = "CRONOMETRO:    "; // Espacio añadido para añadir texto al lado

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
		
		
		GPIOB->MODER &= ~(0b11<<0); //clear (00) pin PB0(bits 1:0) and set as Input (00) for default 
		GPIOB->MODER |= (1<<0); //pin PB0(bits 1:0) as Output (01)
		
		GPIOB->OTYPER &= ~(1<<0);  // clear (0) pin PB0 (bit 0) --> Output push pull (HIGH or LOW)
		GPIOB->OSPEEDR |= ((1<<1)|(1<<0));//(0b11<<0)  // Pin PB0 (bits 1:0) as Very High Speed (11)
		
		GPIOB->PUPDR &= ~(0b11<<0); //~((1<<1)|(1<<0)) // Pin PB0 (bits 1:0) are 0:0 --> no pull up or pull down
			
}




int main(){
	
	  settings();
		settingsLCD(CD); // limpiar la lcd
		settingsLCD(RAW1); // posicion
		for(int i=0;i < 12;i++){
					writeLCD(princ1[i]);
				}
	
		
	
	

	
	
while(1){
		
	

	}
}