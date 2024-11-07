#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>
#include <stdlib.h>  // Para la función atoi

volatile uint8_t received_angle = 0;  // Variable para almacenar el ángulo recibido
char buffer[4];  // Buffer para almacenar el número recibido
int buffer_index = 0;  // Índice del buffer

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
		GPIOC->ODR |= 1UL << 8; // PC8 (E)
		SysTick_ms(10);
		GPIOA->ODR = (GPIOA->ODR & ~0xF0) | ((val & 0x0F) << 4); // PA4-PA7 (D4-D7)
		SysTick_ms(1);
		GPIOC->ODR &= ~(1UL << 8); // PC8 (E)
		SysTick_ms(1);
}

void settingsLCD(unsigned char val) {
		GPIOC->ODR &= ~(1UL << 9); // PC9 (RS)
		LCD(val >> 4);
		LCD(val & 0x0f);
}

void writeLCD(unsigned char val) {
		GPIOC->ODR |= 1UL << 9; // PC9 (RS)
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
void setCursorLCD(uint8_t row, uint8_t col) {
    uint8_t pos = 0;
    if (row == 1) {
        pos = 0x80 + (col - 1);  // Fila 1
    } else if (row == 2) {
        pos = 0xC0 + (col - 1);  // Fila 2
    }
    settingsLCD(pos);  // Enviar el comando de posición
}
void actualizarMensajeLCD(char* mensaje) {
    settingsLCD(0x80);  // Posicionar el cursor en la primera línea
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
		limpiarLineaLCD(1);
}

void settings() {
		// Enable the GPIOB clock 
		RCC->AHB1ENR |= ((1<<0)|(1<<1)|(1<<2));  // GPIO_A-B-C

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

void configuracionUART3comunicacionPc() {
    // UART
    RCC->AHB1ENR |= (1 << 3); // Enable the GPIOD clock (UART3 is connected on PD9 (RX) and PD8 (TX))
    GPIOD->MODER &= ~((0b11 << 18) | (0b11 << 16)); //Clear (00) pins PD9 (bits 19:18) and PD8 (bits 17:16)
    GPIOD->MODER |= (1 << 19) | (1 << 17); // Set (10) pins PD9=RX (bits 19:18) and PD8=TX (bits 17:16) as alternant function
    GPIOD->AFR[1] &= ~((0b1111 << 4) | (0b1111 << 0)); //Clear alternant functions for pins PD9 (bits 7:4) and PD8 (bits 3:0)
    GPIOD->AFR[1] |= (0b111 << 4) | (0b111 << 0); // Set the USART3 (AF7) alternant function for pins PD9=RX and PD8=TX
    RCC->APB1ENR |= (1 << 18); // Enable the USART3 clock
    USART3->BRR = 0x683; // Set the baud rate on 9600 baud to 16 MHz (HSI)
    USART3->CR1 |= ((1 << 5) | (0b11 << 2)); // RXNE interrupt enable, transmitter enable and receiver enable
    USART3->CR1 |= (1 << 0); // Enable USART
    NVIC_EnableIRQ(USART3_IRQn); // Enable the interrupt function on the NVIC module
}

void configuracionPWM() {
    //PWM Configuration (TIM5)
    RCC->AHB1ENR |= (1 << 0); // Enable the GPIOA clock (TIM5_CH1, TIM5_CH2, TIM5_CH3 and TIM5_CH4 are connected on PA0, PA1, PA2 and PA3)
    GPIOA->MODER |= (1 << 7) | (1 << 5) | (1 << 3) | (1 << 1); // Set alternate function for PA0, PA1, PA2, PA3
    GPIOA->AFR[0] |= (1 << 13) | (1 << 9) | (1 << 5) | (1 << 1); // Set AF2 for TIM5 channels
    RCC->APB1ENR |= (1 << 3); // TIM5 clock enable
    TIM5->PSC = 4; // Prescaler for 20ms time
    TIM5->ARR = 63999; // Max count for 20ms
    TIM5->CR1 |= (1 << 0); // Enable counting
    TIM5->CCMR1 |= (0b110 << 12) | (0b110 << 4); // Set PWM mode for CH1 and CH2
    TIM5->CCMR2 |= (0b110 << 12) | (0b110 << 4); // Set PWM mode for CH3 and CH4
    TIM5->CCER |= (1 << 12) | (1 << 8) | (1 << 4) | (1 << 0); // Enable all PWM channels
    TIM5->EGR |= (1 << 0); // Reinitialize registers
}

extern "C" {
			void USART3_IRQHandler(void) {
				 if ((USART3->ISR & USART_ISR_RXNE) != 0) {  // RXNE flag: Dato recibido listo para leer
            received_angle = USART3->RDR;           // Leer el valor binario recibido
        }
			}
}
void ajustarServo() {
    if (received_angle <= 180) {  // Asegurarse de que el valor esté en el rango esperado
        TIM5->CCR1 = (1600 + ((received_angle * 6400) / 180));  // Calcular y asignar el valor PWM
    }
}
int main() {
	
    // Systick
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);
		
		settings();
		configuracionPWM();
		configuracionUART3comunicacionPc();
		
    
		//Mostar incicialmente cronometro en 00:00
		actualizarMensajeLCD("hola mundo");

    // Bucle principal
    while (1) {
			ajustarServo();
			TIM5->CCR1 = 1600;  // Ángulo 0°
			TIM5->CCR2 = 1600;  // Ángulo 0°
			TIM5->CCR3 = 6000;  // Ángulo 45°
			//TIM5->CCR1 = 4800;  // Ángulo 90° 
			//TIM5->CCR1 = 6400;  // Ángulo 135° 
			//TIM5->CCR1 = 8000;  // Ángulo 180° 
			}
		}					