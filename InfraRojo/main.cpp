#include <stdio.h>
#include <stm32f7xx.h>


unsigned char d;       // Variable para datos recibidos por UART
uint8_t deteccion = 0; // Estado de detección del sensor
   


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
    limpiarLineaLCD(1);        // Limpiar la primera línea de la pantalla LCD
    settingsLCD(RAW1);         // Posicionar el cursor en la primera línea
    
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
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




// Manejo de la interrupción externa EXTI2 (PD2)
void configurarSensorPA0() {
    // Habilitar el reloj para el puerto A
    RCC->AHB1ENR |= (1 << 0);  // Habilita el GPIO A (bit 0)

    // Configurar PA0 como entrada
    GPIOA->MODER &= ~(0b11 << (0 * 2));  // PA0 como entrada (00)
    GPIOA->PUPDR &= ~(0b11 << (0 * 2));  // Sin pull-up ni pull-down en PA0
    GPIOA->OSPEEDR &= ~(3 << (0 * 2));   // Limpiar los bits de OSPEEDR para PA0
    GPIOA->OTYPER &= ~(1 << 0);          // Asegurarse de que OTYPER esté en push-pull
		
		//configuracion de PB0 para usar led de usuario 1
		GPIOB->MODER &= ~(0b11<<0); //clear (00) pin PB0(bits 1:0) and set as Input (00) for default 
		GPIOB->MODER |= (1<<0); //pin PB0(bits 1:0) as Output (01)
		
		GPIOB->OTYPER &= ~(1<<0);  // clear (0) pin PB0 (bit 0) --> Output push pull (HIGH or LOW)
		GPIOB->OSPEEDR |= ((1<<1)|(1<<0));//(0b11<<0)  // Pin PB0 (bits 1:0) as Very High Speed (11)
		
		GPIOB->PUPDR &= ~(0b11<<0); //~((1<<1)|(1<<0)) // Pin PB0 (bits 1:0) are 0:0 --> no pull up or pull down
			
	
    // Configuración de la interrupción externa en PA0 (EXTI0)
    RCC->APB2ENR |= (1 << 14);  // Habilitar el reloj para SYSCFG (EXTI)
    SYSCFG->EXTICR[0] &= ~(0b1111 << 0); // Limpiar bits para EXTI0
    SYSCFG->EXTICR[0] |= (0b0000 << 0);  // Configurar EXTI0 para que use PA0 (EXTI0 = 0000 para port A)

    // Habilitar interrupción en EXTI0
    EXTI->IMR |= (1 << 0);  // Habilitar la interrupción en el pin 0 (PA0)
    EXTI->FTSR |= (1 << 0); // Borde de caída (falling edge) cuando PA0 se pone en bajo
    EXTI->RTSR |= (1 << 0); // Borde de subida (rising edge) cuando PA0 vuelve a estar en alto

    // Habilitar la interrupción en el NVIC para EXTI0
    NVIC_EnableIRQ(EXTI0_IRQn);
}




// Manejador de la interrupción para EXTI0 (PA0)
extern "C" {
    void EXTI0_IRQHandler(void) {
        if (EXTI->PR & (1 << 0)) { // Verifica si la interrupción fue generada por PA0
            EXTI->PR |= (1 << 0);  // Limpia la interrupción para PA0

            if ((GPIOA->IDR & (1 << 0)) == 0) { // Si se detecta un objeto (PA0 en bajo)
                deteccion = 1;  // Actualiza el estado de detección
                GPIOB->ODR |= (1 << 0);  // Enciende el LED en PB0
								// Actualiza la pantalla LCD con "OBJ dtc: si"
                actualizarMensajeLCD("OBJ dtc: si");


            } else {  // Si no hay detección (PA0 en alto)
                deteccion = 0;  // Actualiza el estado de detección
                GPIOB->ODR &= ~(1 << 0);  // Apaga el LED en PB0
							  // Actualiza la pantalla LCD con "OBJ dtc: no"
                actualizarMensajeLCD("OBJ dtc: no");
               
            }
        }
    }
}

int main(){
    settings();               // Configuración inicial de hardware y LCD
    configurarSensorPA0();     // Configura el sensor infrarrojo en PA0
    
    // Mostrar inicialmente "OBJ dtc: no" en la pantalla
    actualizarMensajeLCD("OBJ dtc: no");
    
    while(1) {
        // No necesitas actualizar continuamente la LCD aquí, ya que
        // la detección y la actualización del mensaje se manejan en la interrupción.
    }
}
