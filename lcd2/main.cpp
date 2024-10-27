#include <stdio.h>
#include <stm32f7xx.h>


unsigned char d;       // Variable para datos recibidos por UART
uint32_t milliseconds = 0; // Contador de milisegundos
uint32_t seconds = 0; // Contador de segundos (para el cronómetro)
char buffer[16];       // Buffer para almacenar el texto a mostrar en la LCD
uint8_t flag_update_lcd = 0; // Flag para actualizar la LCD


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

// Función para convertir número a cadena (itoa)
void intToStr(char *str, uint32_t num) {
    sprintf(str, "%02lu:%02lu", num / 60, num % 60); // Convierte a formato mm:ss
}

//// Función para mostrar el cronómetro en la LCD
void mostrarCronometro(uint32_t seconds) {
    char buffer[6]; // Cadena para mm:ss
    intToStr(buffer, seconds); // Convierte los segundos a mm:ss
    settingsLCD(RAW1); // Posiciona el cursor en la primera línea
    for (int i = 0; i < 5; i++) {
        writeLCD(buffer[i]); // Escribe cada carácter del cronómetro
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



// Manejo de la interrupción externa EXTI2 (PD2)
void configurarSensorPA0() {
    // Habilitar el reloj para el puerto A
    //RCC->AHB1ENR |= (1 << 0);  // Habilita el GPIO A (bit 0)

    // Configurar PA0 como entrada
    GPIOA->MODER &= ~(0b11 << (0 * 2));  // PA0 como entrada (00)
    GPIOA->PUPDR &= ~(0b11 << (0 * 2));  // Sin pull-up ni pull-down en PA0
    GPIOA->OSPEEDR &= ~(3 << (0 * 2));   // Limpiar los bits de OSPEEDR para PA0
    GPIOA->OTYPER &= ~(1 << 0);          // Asegurarse de que OTYPER esté en push-pull

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

// Configuración del Timer3
void configurarTIM3() {
    RCC->APB1ENR |= (1 << 1);  // Habilita el reloj para TIM3
    TIM3->PSC = 16000 - 1;     // Prescaler para que el contador se incremente cada 1 ms (16 MHz / 16000 = 1 KHz)
    TIM3->ARR = 1000 - 1;      // Auto-reload cada 1000 ms (1 segundo)
    TIM3->DIER |= (1 << 0);    // Habilitar interrupción por actualización
    TIM3->CR1 |= (1 << 0);     // Habilitar el temporizador
    NVIC_EnableIRQ(TIM3_IRQn); // Habilitar la interrupción en NVIC
}
// Función para mostrar el tiempo en formato mm:ss en la LCD
//void mostrarCronometro(uint32_t seconds) {
//    uint32_t minutos = seconds / 60;
//    uint32_t segundos = seconds % 60;
//    sprintf(buffer, "%02lu:%02lu", minutos, segundos);  // Formato mm:ss
//    settingsLCD(0x80);  // Posiciona el cursor en la primera línea
//    for (int i = 0; buffer[i] != '\0'; i++) {
//        writeLCD(buffer[i]);  // Escribir el tiempo en la LCD
//    }
//}

// Manejador de la interrupción para EXTI0 (PA0)
extern "C" {
//	void EXTI0_IRQHandler(void) {
//    if (EXTI->PR & (1 << 0)) { // Verifica si la interrupción fue generada por PA0
//        EXTI->PR |= (1 << 0); // Limpia la interrupción para PA0

//        // Cambia el estado de detección según el pin PA0
//        deteccion = ((GPIOA->IDR & (1 << 0)) == 0) ? 1 : 0;
//    }
//    }

//	void USART3_IRQHandler(void){ //Interrupción de UART3 (Recibir)
//        if(((USART3->ISR & 0x20) >> 5) == 1){ //Flag RXNE (Receive)
//            d = USART3->RDR; //Lee el dato recibido
//        }
//    }
		 void TIM3_IRQHandler(void){ // Interruption Timer 
        if (TIM3->SR & TIM_SR_UIF) {  // Verifica si la interrupción fue por actualización
        TIM3->SR &= ~TIM_SR_UIF;  // Limpia la bandera de interrupción

        milliseconds++;  // Incrementa el contador de milisegundos

        if (milliseconds == 1000) {  // Si han pasado 1000 ms (1 segundo)
            milliseconds = 0;  // Reinicia el contador de milisegundos
            seconds++;         // Incrementa el contador de segundos
            flag_update_lcd = 1;  // Señala que se debe actualizar la LCD
        }
    }
    }	
}

// Función para mostrar el estado del sensor en la LCD
void mostrarEstadoSensor() {
    
}




int main(){
	
	  settings();
		//configurarMotor();  // Configura los pines del motor
		//configurarSensorPA0();  // Configura el sensor infrarrojo en PA0
    configurarTIM3();      // Configurar el temporizador TIM3
	
	while(1){

			mostrarCronometro(seconds); // Muestra el valor del cronómetro
			//mostrarEstadoSensor(); // Muestra el estado del sensor
			
      seconds++; // Incrementa los segundos
			

//			moverMotorAdelante();  // Mueve el motor hacia adelante
			 
		}
}