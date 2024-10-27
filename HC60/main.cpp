#include <stdio.h>
#include <stm32f7xx.h>
#include <string.h>




unsigned char d;       // Variable para datos recibidos por UART

// pc10 uart4 TX --- pc11 uart4 RX

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
    settingsLCD(0x80);  // Posicionar el cursor en la primera línea
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}

void actualizarMensaje2LCD(char* mensaje) {
    settingsLCD(0xC0);  // Posicionar el cursor en la segunda línea
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


void configurarMotor() {
    // Habilitar el reloj para el puerto D
    RCC->AHB1ENR |= (1 << 3);  // Habilita el GPIO D (bit 3)

   // Configurar PD0, PD1, PD2 y PD3 como salidas
	GPIOD->MODER |= (1 << (0 * 2)) | (1 << (1 * 2)) | (1 << (2 * 2)) | (1 << (3 * 2)); // PD0, PD1, PD2, y PD3 en modo salida

	// Configurar las salidas como push-pull
	GPIOD->OTYPER &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));  // PD0, PD1, PD2, PD3 en modo push-pull

	// Configurar la velocidad de los pines como very high speed
	GPIOD->OSPEEDR |= (3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2));  // Very high speed para PD0, PD1, PD2 y PD3

	// Configurar sin pull-up ni pull-down
	GPIOD->PUPDR &= ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2))); // Sin pull-up ni pull-down en PD0, PD1, PD2 y PD3

}
// Mover ambos motores hacia adelante
void moverMotoresAdelante() {
    // Motor 1 (PD0, PD1)
    GPIOD->ODR |= (1 << 0);  // PD0 = 1
    GPIOD->ODR &= ~(1 << 1); // PD1 = 0
    
    // Motor 2 (PD2, PD3)
    GPIOD->ODR |= (1 << 2);  // PD2 = 1
    GPIOD->ODR &= ~(1 << 3); // PD3 = 0
}

// Mover ambos motores hacia atrás
void moverMotoresAtras() {
    // Motor 1 (PD0, PD1)
    GPIOD->ODR &= ~(1 << 0); // PD0 = 0
    GPIOD->ODR |= (1 << 1);  // PD1 = 1
    
    // Motor 2 (PD2, PD3)
    GPIOD->ODR &= ~(1 << 2); // PD2 = 0
    GPIOD->ODR |= (1 << 3);  // PD3 = 1
}

// Girar a la izquierda (Motor izquierdo atrás, motor derecho adelante)
void girarIzquierda() {
    // Motor 1 (PD0, PD1) - Motor izquierdo hacia atrás
    GPIOD->ODR &= ~(1 << 0); // PD0 = 0
    GPIOD->ODR |= (1 << 1);  // PD1 = 1
    
    // Motor 2 (PD2, PD3) - Motor derecho hacia adelante
    GPIOD->ODR |= (1 << 2);  // PD2 = 1
    GPIOD->ODR &= ~(1 << 3); // PD3 = 0
}

// Girar a la derecha (Motor izquierdo adelante, motor derecho atrás)
void girarDerecha() {
    // Motor 1 (PD0, PD1) - Motor izquierdo hacia adelante
    GPIOD->ODR |= (1 << 0);  // PD0 = 1
    GPIOD->ODR &= ~(1 << 1); // PD1 = 0
    
    // Motor 2 (PD2, PD3) - Motor derecho hacia atrás
    GPIOD->ODR &= ~(1 << 2); // PD2 = 0
    GPIOD->ODR |= (1 << 3);  // PD3 = 1
}

// Detener ambos motores
void detenerMotor() {
    // Detener Motor 1 (PD0, PD1)
    GPIOD->ODR &= ~((1 << 0) | (1 << 1)); // PD0 = 0, PD1 = 0
    // Detener Motor 2 (PD2, PD3)
    GPIOD->ODR &= ~((1 << 2) | (1 << 3)); // PD2 = 0, PD3 = 0
}



void configuracionUART3comunicacionPc(){
	//UART
    RCC->AHB1ENR |= (1<<3); //Enable the GPIOD clock (UART3 is connected on PD9 (RX) and PD8 (TX))
    GPIOD->MODER &= ~((0b11<<18)|(0b11<<16)); //Clear (00) pins PD9 (bits 19:18) and PD8 (bits 17:16)
    GPIOD->MODER |= (1<<19)|(1<<17); //Set (10) pins PD9=RX (bits 19:18) and PD8=TX (bits 17:16) as alternant function
    GPIOD->AFR[1] &= ~((0b1111<<4)|(0b1111<<0)); //Clear (0000) alternant functions for pins PD9 (bits 7:4) and PD8 (bits 3:0)
    GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0); //Set the USART3 (AF7) alternant function for pins PD9=RX (bits 7:4) and PD8=TX (bits 3:0)
    RCC->APB1ENR |= (1<<18); //Enable the USART3 clock
    USART3->BRR = 0x683; //Set the baud rate on 9600 baud to 16 MHz (HSI)
    USART3->CR1 |= ((1<<5)|(0b11<<2)); //RXNE interrupt enable, transmitter enable and receiver enable
    USART3->CR1 |= (1<<0); //USART enable
    NVIC_EnableIRQ(USART3_IRQn); //Enable the interrupt function on the NVIC module
		
}





void configurarUART4() {
    // Habilitar el reloj para GPIOC y UART4
    RCC->AHB1ENR |= (1 << 2);   // Habilitar el reloj para GPIOC (bit 2)
    RCC->APB1ENR |= (1 << 19);  // Habilitar el reloj para UART4 (bit 19)

    // Configurar PC10 (TX) y PC11 (RX) como función alternativa (AF8)
    GPIOC->MODER &= ~((0b11 << (10 * 2)) | (0b11 << (11 * 2)));  // Limpiar los bits de moder
    GPIOC->MODER |= (0b10 << (10 * 2)) | (0b10 << (11 * 2));     // Función alternativa para PC10 y PC11

    // Configurar la función alternativa AF8 (UART4) para los pines PC10 y PC11
    GPIOC->AFR[1] &= ~((0b1111 << ((10 - 8) * 4)) | (0b1111 << ((11 - 8) * 4))); // Limpiar AF para PC10 y PC11
    GPIOC->AFR[1] |= (0b1000 << ((10 - 8) * 4)) | (0b1000 << ((11 - 8) * 4));    // Configurar AF8 (UART4)

    // Configuración de UART4
    UART4->BRR = 0x683;  // Baud rate a 9600 (para un reloj de 16 MHz HSI)
    UART4->CR1 |= (1 << 2) | (1 << 3);  // Habilitar RX (bit 2) y TX (bit 3)
    UART4->CR1 |= (1 << 0);  // Habilitar UART4 (bit 0)

    // Opcional: Habilitar interrupción por recepción si es necesario
    UART4->CR1 |= (1 << 5);  // Habilitar interrupción por RXNE (data register not empty)
    NVIC_EnableIRQ(UART4_IRQn);  // Habilitar UART4 en el NVIC
}

extern "C" {
	
	

	void USART3_IRQHandler(void){ //Receive interrupt
    if (USART3->ISR & USART_ISR_RXNE) { // Si se ha recibido un dato
            d = USART3->RDR;  // Leer el dato recibido   
//						 // Evaluar el dato recibido 
            switch(d) {
                case 'W':
                    moverMotoresAdelante();
                    break;
                case 'S':
                    moverMotoresAtras();
                    break;
                case 'A':
										girarIzquierda();
                    // Implementar movimiento izquierda (ej. moverMotorIzquierda())
                    break;
                case 'D':
										girarDerecha();
                    // Implementar movimiento derecha (ej. moverMotorDerecha())
                    break;
								case 'C':  // Comando para iniciar cronómetro
										 
										break;
                default:
                    detenerMotor();  // Detener el motor si no es un comando válido
                      
										break;
            }
				
				}
    }
	


		void UART4_IRQHandler(void) {
			if (UART4->ISR & USART_ISR_RXNE) {  // Verificar si hay datos recibidos
					d = UART4->RDR;      // Leer dato recibido
					// Procesar el dato recibido (mover motor)
				// Evaluar el dato recibido y mover el motor en consecuencia
            switch(d) {
                case 'W':
                    moverMotoresAdelante();
                    break;
                case 'S':
                    moverMotoresAtras();
                    break;
                case 'A':
										girarIzquierda();
                    // Implementar movimiento izquierda (ej. moverMotorIzquierda())
                    break;
                case 'D':
										girarDerecha();
                    // Implementar movimiento derecha (ej. moverMotorDerecha())
                    break;
								case 'C':  // Comando para iniciar cronómetro
										 
										break;
                default:
                    detenerMotor();  // Detener el motor si no es un comando válido
                      
										break;
			}
		}

}
		
}
int main(){
		settings(); 
	  configurarMotor();
		configurarUART4();

		// Mostrar inicialmente "OBJ dtc: no" en la pantalla
		actualizarMensajeLCD("s1: no");
	
	while(1){
	
				
		
   }
 }