#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

uint8_t last_angle = 255;  // Variable para guardar el último ángulo mostrado en LCD, inicializado en un valor imposible para forzar la primera actualización
uint8_t last_angle2 = 255;  // Último ángulo mostrado en LCD para slider 2
uint8_t last_angle3 = 255;  // Último ángulo mostrado en LCD para slider 3
uint8_t last_botton = 255; 
//uint8_t flag = 0;


volatile uint8_t received_angle = 0;  // Variable para almacenar el ángulo recibido
//char buffer[4];  // Buffer para almacenar el número recibido
int buffer_index = 0;  // Índice del buffer

volatile uint8_t slider1_angle = 0;  // Valor del ángulo para el slider 1
volatile uint8_t slider2_angle = 0;  // Valor del ángulo para el slider 2
volatile uint8_t slider3_angle = 0;  // Valor del ángulo para el slider 3
volatile uint8_t byte_count = 0;     // Contador para bytes recibidos
volatile uint8_t received_data[2];   // Array para almacenar el identificador y el valor del ángulo

volatile uint8_t botton_angle = 0;


char text[16];
//variables para PWM de motores DC
int estadoPlataforma=0;
int seleccionMotorVelocidad= 0;

//variables timer para cronometro de la plataforma 
uint8_t flag_cronometro = 0, cont = 0, cronometro_iniciado = 0;
int segundos = 0;
int minutos = 0;  // Variable para contar los minutos
int valorMaximoCronometro=0;


//variables para sensor infrarojo
int alertaInfrarojoGui = 0;  // bandera para modificar estado de detección en la GUI
char estado_deteccion[12] = "INFR: no";
uint8_t flag_deteccion = 0;

//varibles para adc del cny
uint16_t digital;
float voltaje;
int alertCNYGui=0;//bandera para modificar esatdo de deteccion en la GUI

//variables para adc de LDR
volatile int estadoDeteccionLdr = 0; // Inicialmente en 0, plataforma inactiva
int apagadoBrazoRobot=0;
// pc10 uart4 TX --- pc11 uart4 RX

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
		
}
void actualizarMensaje2LCD(const char* mensaje) {
		setCursorLCD(1, 5);  // Fila 1, Columna 5
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
		
}

void actualizarMensaje3LCD(const char* mensaje) {
		setCursorLCD(1, 9);  // Fila 1, Columna 9
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}

void actualizarMensaje4LCD(const char* mensaje) {
		setCursorLCD(1, 13);  // Fila 1, Columna 13
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}

void actualizarMensajeCronometroLCD(char* mensaje) {
		setCursorLCD(2, 1);  // Fila 2, Columna 1
    //settingsLCD(0xC0);  // Posicionar el cursor en la segunda línea
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}

void actualizarMensajeEstadoPlataformaLCD(char* mensaje) {
		setCursorLCD(2, 6);  // Fila 2, Columna 1
    //settingsLCD(0xC0);  // Posicionar el cursor en la segunda línea
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}

void actualizarMensajeVelocidadMotorDC1LCD(char* mensaje) {
		setCursorLCD(2, 10);  // Fila 2, Columna 1
    //settingsLCD(0xC0);  // Posicionar el cursor en la segunda línea
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}
void actualizarMensajeVelocidadMotorDC2LCD(char* mensaje) {
		setCursorLCD(2, 14);  // Fila 2, Columna 1
    //settingsLCD(0xC0);  // Posicionar el cursor en la segunda línea
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}
//configuracion pantalla lcd
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

void configuracionTimerCronometro(){
		// Configuración del Timer3 (para contar 1 segundo)
    RCC->APB1ENR |= (1<<1); // Habilitar el reloj de TIM3 
    TIM3->PSC = 16000 - 1;  // Prescaler para contar en ms (16 MHz / 16,000 = 1 kHz)
    TIM3->ARR = 100 - 1;    // 100 ms por interrupción
    TIM3->DIER |= (1<<0);   // Habilitar interrupciones de actualización
    TIM3->CR1 |= (1<<0);    // Iniciar el temporizador
    NVIC_EnableIRQ(TIM3_IRQn);  // Habilitar IRQ para TIM3
}

void configurarUART4BluetoothHc50() {
		// pc10 uart4 TX --- pc11 uart4 RX
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

void configurarSensorPD0() {
    // Habilitar el reloj para los puertos D y B
    RCC->AHB1ENR |= (1 << 3) | (1 << 1);  // Habilitar GPIOD y GPIOB

    // Configurar PD0 como entrada
    GPIOD->MODER &= ~(0b11 << (0 * 2));  // PD0 como entrada (00)
    GPIOD->PUPDR &= ~(0b11 << (0 * 2));  // Sin pull-up ni pull-down en PD0
    GPIOD->OSPEEDR &= ~(3 << (0 * 2));   // Velocidad baja en PD0
    GPIOD->OTYPER &= ~(1 << 0);          // PD0 en push-pull

    // Configuración de PB0 para el LED
    GPIOB->MODER &= ~(0b11 << 0);  // Limpiar bits (00) para PB0
    GPIOB->MODER |= (1 << 0);      // Configurar PB0 como salida (01)
    GPIOB->OTYPER &= ~(1 << 0);    // Salida en push-pull
    GPIOB->OSPEEDR |= (3 << 0);    // Alta velocidad en PB0
    GPIOB->PUPDR &= ~(0b11 << 0);  // Sin pull-up ni pull-down en PB0

    // Configuración de la interrupción externa en PD0 (EXTI0)
    RCC->APB2ENR |= (1 << 14);      // Habilitar reloj para SYSCFG (EXTI)
    SYSCFG->EXTICR[0] &= ~(0b1111); // Limpiar bits para EXTI0
    SYSCFG->EXTICR[0] |= (0b0011);  // Configurar EXTI0 para PD0 (valor 0011 para puerto D)

    // Habilitar interrupción en EXTI0 en borde de bajada
    EXTI->IMR |= (1 << 0);         // Habilitar interrupción en PD0
    EXTI->FTSR |= (1 << 0);        // Borde de bajada para detectar "activo bajo"
    EXTI->RTSR |= (1 << 0);       // Deshabilitar borde de subida

    // Habilitar interrupción en el NVIC para EXTI0
    NVIC_EnableIRQ(EXTI0_IRQn);
}

void actualizarCronometro(){
	// actualizacion cronometro display
				if (flag_cronometro) {  // Si hubo un cambio en el cronómetro
            flag_cronometro = 0;
            sprintf(text, "%02d:%02d", minutos, segundos);  // Formatear el mensaje con minutos y segundos
            // setCursorLCD(2, 1);  // Posicionar el cursor en la columna 1 de la fila 2
						actualizarMensajeCronometroLCD(text);  // Actualizar la segunda línea con el cronómetro
						
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

void avanzar(){
				setMotor(2, 1, 100);   // Motor 1, dirección 0, velocidad alta
				setMotor(1, 1, 100);   // Motor 1, dirección 0, velocidad alta
        
}
void girarIzquierda(){
				setMotor(1, 0, 100);   // Motor 2, dirección 1, velocidad alta
				setMotor(2, 1, 100);   // Motor 1, dirección 0, velocidad alta
        
}
void girarDerecha(){
				setMotor(2, 0, 100);   // Motor 2, dirección 0, velocidad alta
				setMotor(1, 1, 100); // Motor 1, dirección 1, velocidad alta
        
}
void reversa(){
				setMotor(2, 0, 50);   // Motor 2, dirección 0, velocidad media
				setMotor(1, 0, 50);   // Motor 1, dirección 0, velocidad media
        
}
void detener(){
				setMotor(2, 1, 0);   // Motor 2, dirección 0, velocidad media
				setMotor(1, 1, 0);   // Motor 1, dirección 0, velocidad media
        
}
void velocidad35(int motor){
				setMotor(motor, 1, 35); 

}
void velocidad55(int motor){
				setMotor(motor, 1, 55); 

}
void velocidad70(int motor){
				setMotor(motor, 1, 75); 

}
void velocidad90(int motor){
				setMotor(motor, 1, 90); 

}
// Configuración del ADC para el sensor CNY70 en PC0
void configuracionADCcny70(){
    // ADC
    GPIOC->MODER |= (0b11 << 0); // PC0 (ADC123_IN10) en modo analógico
    RCC->APB2ENR |= (1 << 9);    // Activar reloj del ADC2
    ADC2->CR2 |= ((1 << 10) | (1 << 0)); // EOC y habilitar ADC
    ADC2->CR1 &= ~(0b11 << 24);  // Limpiar bits de resolución
    ADC2->CR1 |= (1 << 24);      // Resolución de 10 bits
    ADC2->SMPR1 |= (1 << 0);     // 15 ciclos ADCCLK en canal 10 (PC0)
    ADC2->SQR3 &= ~(0b11111 << 0); // Limpiar secuencia regular
    ADC2->SQR3 |= (0b1010 << 0); // Canal 10 en primera conversión
}

// Configuración del ADC para la fotorresistencia LDR en PC3
void configuracionADCfotoCeldaLdr() {
    // Configuración de ADC y GPIO
    RCC->AHB1ENR |= (1 << 2);          // Habilitar reloj para GPIOC
    RCC->APB2ENR |= (1 << 9);          // Activar reloj del ADC2
    GPIOC->MODER |= (0b11 << 6);       // PC3 (ADC123_IN13) en modo analógico
    ADC2->CR2 |= (1 << 0);             // Habilitar ADC2
    ADC2->CR2 |= (1 << 10);            // EOC (End of Conversion)
    ADC2->CR1 &= ~(0b11 << 24);        // Limpiar bits de resolución
    ADC2->CR1 |= (1 << 24);            // Configurar resolución de 10 bits
    ADC2->SMPR1 |= (1 << 3);           // Tiempo de muestreo de 15 ciclos ADCCLK para canal 13 (PC3)
    ADC2->SQR3 &= ~(0b11111);          // Limpiar secuencia de conversión regular
    ADC2->SQR3 |= (13 << 0);           // Canal 13 en primera conversión
}

// Conversión del ADC para el sensor CNY70
void conversionADC() {
    // Configuración de PB7 como salida para el LED
    RCC->AHB1ENR |= (1 << 1);           // Habilitar reloj para GPIOB
    GPIOB->MODER &= ~(0b11 << (7 * 2)); // Limpiar bits 14 y 15 para PB7
    GPIOB->MODER |= (0b01 << (7 * 2));  // Configurar PB7 como salida (01)
    GPIOB->OSPEEDR &= ~(0b11 << (7 * 2)); // Limpiar bits de velocidad para PB7
    GPIOB->OSPEEDR |= (0b10 << (7 * 2));  // Configurar alta velocidad para PB7

    // Configurar canal 10 (PC0) en el ADC2 para el CNY70
    ADC2->SQR3 &= ~(0b11111 << 0); // Limpiar secuencia de conversión regular
    ADC2->SQR3 |= (10 << 0);       // Canal 10 en primera conversión

    // Iniciar conversión ADC y esperar a que termine
    ADC2->CR2 |= (1 << 30);            // Iniciar conversión A/D
    while ((ADC2->SR & (1 << 1)) == 0) {} // Esperar final de conversión
    ADC2->SR &= ~(1 << 1);             // Limpiar bit EOC
    int digital = ADC2->DR;            // Leer valor digital del ADC
    float voltaje = (float)digital * (3.3 / 1023.0); // Calcular voltaje

    if (voltaje >= 2.0) {
        GPIOB->ODR |= (1 << 7);        // Encender LED en PB7
        alertCNYGui = 1;               // Actualizar GUI
				estadoPlataforma = 5; 
				
		} else {
        GPIOB->ODR &= ~(1 << 7);       // Apagar LED en PB7
        alertCNYGui = 0;
    }
}

// Conversión del ADC para la fotorresistencia LDR
void conversionADC_Ldr() {
		RCC->AHB1ENR |= (1 << 3);            // Habilitar reloj para GPIOD
		// Configuración de PD1 como salida para el LED
		GPIOD->MODER &= ~(0b11 << (1 * 2));    // Limpiar bits 2 y 3 para PD1
		GPIOD->MODER |= (0b01 << (1 * 2));     // Configurar PD1 como salida (01)
		GPIOD->OSPEEDR &= ~(0b11 << (1 * 2));  // Limpiar bits de velocidad para PD1
		GPIOD->OSPEEDR |= (0b10 << (1 * 2));   // Configurar alta velocidad para PD1
    
		// Configuración de PD3 como salida para el LED
    GPIOD->MODER &= ~(0b11 << (3 * 2));  // Limpiar bits 6 y 7 para PD3
    GPIOD->MODER |= (0b01 << (3 * 2));   // Configurar PD3 como salida (01)
    GPIOD->OSPEEDR &= ~(0b11 << (3 * 2)); // Limpiar bits de velocidad para PD3
    GPIOD->OSPEEDR |= (0b10 << (3 * 2));  // Configurar alta velocidad para PD3

    // Configurar canal 13 (PC3) en el ADC2 para el LDR
    ADC2->SQR3 &= ~(0b11111 << 0); // Limpiar secuencia de conversión regular
    ADC2->SQR3 |= (13 << 0);       // Canal 13 en primera conversión

    // Iniciar conversión ADC y esperar a que termine
    ADC2->CR2 |= (1 << 30);             // Iniciar conversión A/D
    while ((ADC2->SR & (1 << 1)) == 0) {} // Esperar final de conversión
    ADC2->SR &= ~(1 << 1);              // Limpiar bit EOC
    int digital = ADC2->DR;             // Leer valor digital del ADC
    float voltaje = (float)digital * (3.3 / 1023.0); // Calcular voltaje

    if (voltaje >= 2.5) {//2.1V EN LA CASA----2.5 EN SALON
					
			  switch (estadoDeteccionLdr) {
						case 0:
								GPIOD->ODR |= (1 << 3);         // Encender LED en PD3
								GPIOD->ODR &= ~(1 << 1); 
								// Primera vez que se detecta la condición: permitir el movimiento
								estadoDeteccionLdr = 1;
								apagadoBrazoRobot=++apagadoBrazoRobot;
										
									
								
								break;

						case 1:
								estadoPlataforma = 5; 
								estadoDeteccionLdr=0;
								
								
								break;
				}
		} else {
        GPIOD->ODR &= ~(1 << 3);        // Apagar LED en PD3
				GPIOD->ODR |= (1 << 1);      
		}
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
	
	//interrupcion de comunicacion usart3 para interfaz en matlab
void USART3_IRQHandler(void) {
    if ((USART3->ISR & USART_ISR_RXNE) != 0) {  // Dato recibido
        uint8_t received_byte = USART3->RDR;    // Leer el byte recibido
        
        // Almacenar el byte recibido
        if (byte_count < sizeof(received_data)) { // Evitar desbordamiento
            received_data[byte_count++] = received_byte;
        }

        // Si se han recibido dos bytes (identificador y valor)
        if (byte_count >= 2) {
            uint8_t identifier = received_data[0];  // Primer byte es el identificador
            uint8_t angle = received_data[1];       // Segundo byte es el ángulo

            // Procesar el identificador
            switch (identifier) {
                case 1:
                    slider1_angle = angle;  // Guardar ángulo para slider 1
                    break;
                case 2:
                    slider2_angle = angle;  // Guardar ángulo para slider 2
                    break;
                case 3:
                    slider3_angle = angle;  // Guardar ángulo para slider 3
                    break;
                case 4:
                    botton_angle = angle;   // Guardar ángulo para botón
                    break;
                case 5: // Botón adelante accionado
                    estadoPlataforma = 1;
                    break;
                case 6: // Botón derecha accionado 
                    estadoPlataforma = 2;
                    break;
                case 7: // Botón izquierda accionado 
                    estadoPlataforma = 3;
                    break;
                case 8: // Botón reversa accionado 
                    estadoPlataforma = 4;
                    break;
                case 9: // Señal para activación cronómetro
                    cronometro_iniciado = 1;
                    break;
                case 10: // Botón detener
                    estadoPlataforma = 5;
                    break;
                case 11: // 35% de velocidad
                    estadoPlataforma = 6;
                    seleccionMotorVelocidad = angle;
                    break;
								case 12: // Señal para activación cronómetro
                    estadoPlataforma = 7;
                    seleccionMotorVelocidad = angle;
                    break;
								case 13: // Señal para activación cronómetro
                    estadoPlataforma = 8;
                    seleccionMotorVelocidad = angle;
                    break;
								case 14: // Señal para activación cronómetro
                    estadoPlataforma = 9;
                    seleccionMotorVelocidad = angle;
                    break;
								case 15: // Señal para activación cronómetro
                    estadoPlataforma = 10;
                    seleccionMotorVelocidad = angle;
                    break;
								case 16: // Señal para activación cronómetro
                    estadoPlataforma = 11;
                    seleccionMotorVelocidad = angle;
                    break;
								case 17: // Señal para activación cronómetro
                    estadoPlataforma = 12;
                    seleccionMotorVelocidad = angle;
                    break;
								case 18: // Señal para activación cronómetro
                    estadoPlataforma = 13;
                    seleccionMotorVelocidad = angle;
                    break;
                default:
									
                    // Manejo de identificador no reconocido
                    break;
            }

            // Reiniciar contador para la próxima recepción
            byte_count = 0;  
        }
    }
}
		
		//interrupcion de comunicacion usart4 para modulo hc05
void UART4_IRQHandler(void) {

					static uint8_t buffer[8]; // Buffer para almacenar "id:valor"
					static uint8_t index = 0;

					if ((UART4->ISR & USART_ISR_RXNE) != 0) {
							uint8_t received_byte = UART4->RDR; // Leer byte recibido
							
							if (received_byte != '\n') { // Fin del mensaje
									buffer[index++] = received_byte;
							} else {
									buffer[index] = '\0'; // Añadir terminador
									index = 0; // Reiniciar índice para el próximo mensaje

									// Procesar el mensaje: "id:angle"
									uint8_t id = buffer[0] - '0'; // Convertir '1' a 1 - Convertir '2' a 2 - Convertir '3' a 3 etc ...
									uint8_t angle = atoi((const char*)&buffer[1]); // buffer[2] es la dirección del segundo carácter en el arreglo buffer
												
								
											if(apagadoBrazoRobot==0){
													
													TIM5->CR1 &= ~(1 << 0);
												}
											else if(apagadoBrazoRobot==1){
													configuracionPWM();
											}
									
											// Procesar el identificador
											switch (id) {
													case 1:
															slider1_angle = angle;  // Guardar ángulo para slider 1
															break;
													case 2:
															slider2_angle = angle;  // Guardar ángulo para slider 2
															break;
													case 3:
															slider3_angle = angle;  // Guardar ángulo para slider 3
															break;
													case 4:
															botton_angle = angle;   // Guardar ángulo para botón
															break;
													case 5: // Botón adelante accionado
															estadoPlataforma = 1;
															break;
													case 6: // Botón derecha accionado 
															estadoPlataforma = 2;
															break;
													case 7: // Botón izquierda accionado 
															estadoPlataforma = 3;
															break;
													case 8: // Botón reversa accionado 
															estadoPlataforma = 4;
															break;
													case 9: // Señal para activación cronómetro
															cronometro_iniciado = 1;
															break;
													case 10: // Botón detener
															estadoPlataforma = 5;
															break;
													case 11:
															estadoPlataforma = 6;
															seleccionMotorVelocidad = 2;
															break;
													case 12: // Señal para activación cronómetro
															estadoPlataforma = 7;
															seleccionMotorVelocidad = 2;
															break;
													case 13: // Señal para activación cronómetro
															estadoPlataforma = 8;
															seleccionMotorVelocidad = 2;
															break;
													case 14: // Señal para activación cronómetro
															estadoPlataforma = 9;
															seleccionMotorVelocidad = angle;
															break;
													case 15: // Señal para activación cronómetro
															estadoPlataforma = 10;
															seleccionMotorVelocidad = 1;
															break;
													case 16: // Señal para activación cronómetro
															estadoPlataforma = 11;
															seleccionMotorVelocidad = 1;
															break;
													case 17: // Señal para activación cronómetro
															estadoPlataforma = 12;
															seleccionMotorVelocidad = angle;
															break;
													case 18: // Señal para activación cronómetro
															estadoPlataforma = 13;
															seleccionMotorVelocidad = angle;
															break;
													default:
														
															// Manejo de identificador no reconocido
															break;
											}
										
							}
					}
			}		


				//timer cronometro 
void TIM3_IRQHandler(void){ 
    if (TIM3->SR & TIM_SR_UIF) {  // Verificar si hay una actualización (bandera UIF)
        TIM3->SR &= ~TIM_SR_UIF;  // Limpiar la bandera de interrupción

        if (cronometro_iniciado) {  // Solo contar si el cronómetro ha sido iniciado
            cont += 1;
            if (cont == 10) {  // 10 interrupciones = 1 segundo (100 ms por interrupción)
                cont = 0;
                segundos++;  // Incrementar segundos
                if (segundos == 60) {
                    segundos = 0;
                    minutos++;  // Incrementar minutos cuando segundos lleguen a 60
										// Verificar si ha alcanzado 10 minutos
										if (minutos == 10){
												// Detener la plataforma										
												valorMaximoCronometro=1; // cuando valor maximo para el cronometro el alcanzado se envia 1
												estadoPlataforma=5;
												TIM5->CR1 &= ~(1 << 0); // Desactivar el contador (apagar PWM completamente) para bloqear servo motores
												 apagadoBrazoRobot=2;
										}
								}
                flag_cronometro = 1;   // Activar la bandera para actualizar la LCD

							}
						}
				}
			}
		
			


// Interrupción externa para sensor infrarrojo PD0
    void EXTI0_IRQHandler(void) {
        if (EXTI->PR & (1 << 0)) {  // Verifica si la interrupción fue generada por PD0
            EXTI->PR |= (1 << 0);   // Limpia la interrupción para PD0

            // Cambia el estado de detección según el pin PD0
            if ((GPIOD->IDR & (1 << 0)) == 0) {  // Si PD0 está en bajo (objeto detectado)
                GPIOB->ODR |= (1 << 0);          // Enciende el LED en PB0
								estadoPlataforma = 5; 
								strcpy(estado_deteccion, "INFR:si");
                alertaInfrarojoGui = 1;
								actualizarMensajeEstadoPlataformaLCD("stop");
								
            } else {  // PD0 está en alto (sin objeto)
                GPIOB->ODR &= ~(1 << 0);         // Apaga el LED en PB0
                strcpy(estado_deteccion, "INFR:no");
                alertaInfrarojoGui = 0;
            }
            flag_deteccion = 1;  // Activar bandera para actualizar la LCD
        }
    }			
		//final c
}



void ajustarServo() {

			if (slider1_angle <= 180) {  // Ajustar el ángulo del primer servo cadera
        TIM5->CCR1 = (1600 + ((slider1_angle * 6400) / 180));
			}
			
			if (slider2_angle <= 180) {  // Ajustar el ángulo del segundo servo hombro
					TIM5->CCR2 = (1600 + ((slider2_angle * 6400) / 180));
			}
			
			if (slider3_angle <= 180) {  // Ajustar el ángulo del segundo servo codo
					TIM5->CCR3 = (1600 + ((slider3_angle * 6400) / 180));
			}
			if (botton_angle <= 180) {  // Ajustar el ángulo del segundo servo grepper
					TIM5->CCR4 = (1600 + ((botton_angle * 6400) / 180));
			}

	
}

void actualizarLCD_AnguloServo1(uint8_t angle) {
    char mensaje[16];
    sprintf(mensaje, "%3d", angle);  // Formatear el ángulo en una cadena como "Servo1: XXX°"
    actualizarMensajeLCD(mensaje);            // Enviar el mensaje a la LCD
}
void actualizarLCD_AnguloServo2(uint8_t angle) {
    char mensaje[16];
    sprintf(mensaje, "%3d", angle);  // Formato " XXX"                   
    actualizarMensaje2LCD(mensaje);  // Muestra el mensaje en la segunda línea
}

void actualizarLCD_AnguloServo3(uint8_t angle) {
    char mensaje[16];
    sprintf(mensaje, "%3d", angle);                     
    actualizarMensaje3LCD(mensaje); // Muestra el mensaje
}
void actualizarLCD_AnguloServo4(uint8_t angle) {
    char mensaje[16];
    sprintf(mensaje, "%3d", angle);  // Formato "XXX"                     
    actualizarMensaje4LCD(mensaje);  // Muestra el mensaje
}

void home(){
				GPIOB->ODR |= (1 << 0);  // Enciende un LED en PB0 para indicar que está inicializando (opcional)
    SysTick_ms(100);         // Espera breve

    // Coloca los servos en la posición de 0° (ajusta TIM5->CCR1-4 para cada servo en 0°)
    TIM5->CCR1 = 1600;  // Ajuste para 0° en el servo de cadera (PA0)
    TIM5->CCR2 = 1600;  // Ajuste para 0° en el servo de hombro (PA1)
    TIM5->CCR3 = 1600;  // Ajuste para 0° en el servo de codo (PA2)
    TIM5->CCR4 = 1600;  // Ajuste para 0° en el servo del greaper (PA3)

    GPIOB->ODR &= ~(1 << 0);  // Apaga el LED después de la inicialización
    SysTick_ms(100);          // Espera breve para finalizar la inicialización
}

void actualizarPoscionServoMotores(){
	 
	
					// Comprobar si el ángulo del servo 1 ha cambiado
					if (slider1_angle != last_angle) {
							last_angle = slider1_angle;           // Actualizar la última posición
							actualizarLCD_AnguloServo1(last_angle);  // Mostrar el nuevo ángulo en la LCD
					}
					// Verificar y actualizar el ángulo del servo 2
					if (slider2_angle != last_angle2) {
							last_angle2 = slider2_angle;
							actualizarLCD_AnguloServo2(last_angle2);
					}

					// Verificar y actualizar el ángulo del servo 3
					if (slider3_angle != last_angle3 ) {
							last_angle3 = slider3_angle;
							actualizarLCD_AnguloServo3(last_angle3);
					}
					
					// Verificar y actualizar el ángulo del servo 4
					if (botton_angle != last_botton ) {
							last_botton = botton_angle;
							actualizarLCD_AnguloServo4(last_botton);
					}

}
int main() {
	
    // Systick
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);
		
		settings();
		configuracionTimerCronometro();
		configuracionUART3comunicacionPc();
		configurarUART4BluetoothHc50();// pc10 uart4 TX --- pc11 uart4 RX
    //configuracion para servo motores  canal PA
		configuracionPWM();
    configurarSensorPD0();
		configuracionADCcny70();
		configuracionADCfotoCeldaLdr();
		// Configurar PWM y GPIOs para los motores DC en canal PB
    configuracionPWMmotoresDC();
		//Mostar incicialmente cronometro en 00:00
		actualizarMensajeCronometroLCD("00:00");
		actualizarMensajeEstadoPlataformaLCD("stop");
    // Bucle principal
    while (1) {		
					
			//actualizarCronometro();
			conversionADC();
			conversionADC_Ldr();
			ajustarServo();
								
				//verificar el estado del cronometro 
				if (flag_cronometro) {  // Si hubo un cambio en el cronómetro
            flag_cronometro = 0;
            sprintf(text, "%02d:%02d", minutos, segundos);  // Formatear el mensaje con minutos y segundos
            // setCursorLCD(2, 1);  // Posicionar el cursor en la columna 1 de la fila 2
						actualizarMensajeCronometroLCD(text);  // Actualizar la segunda línea con el cronómetro
						
				} 

				if(apagadoBrazoRobot==0){
													
						TIM5->CR1 &= ~(1 << 0);
				}
				if(apagadoBrazoRobot==1){
						configuracionPWM();
				}
					
				if(apagadoBrazoRobot==2){
					TIM5->CR1 &= ~(1 << 0); // Desactivar el contador (apagar PWM completamente) para bloqear servo motores
				}
				
				if(estadoDeteccionLdr == 1){
						actualizarPoscionServoMotores();
				//verificar estado de la plataforma para mover motores DC
						if(estadoPlataforma==1){
								avanzar();
								actualizarMensajeEstadoPlataformaLCD("Go!!");
								actualizarMensajeVelocidadMotorDC1LCD("100");
								actualizarMensajeVelocidadMotorDC2LCD("100");
						}else if(estadoPlataforma==2){
								girarIzquierda();
								actualizarMensajeEstadoPlataformaLCD("left");
						}else if(estadoPlataforma==3){
								girarDerecha();
								actualizarMensajeEstadoPlataformaLCD("righ");
						}else if(estadoPlataforma==4){
								reversa();
								actualizarMensajeEstadoPlataformaLCD("back");
								actualizarMensajeVelocidadMotorDC1LCD("50%");
								actualizarMensajeVelocidadMotorDC2LCD("50%");
						}else if(estadoPlataforma==5){
								detener();
								actualizarMensajeEstadoPlataformaLCD("stop");
								actualizarMensajeVelocidadMotorDC1LCD("0%");
								actualizarMensajeVelocidadMotorDC2LCD("0%");
						}else if(estadoPlataforma==6){
								velocidad35(seleccionMotorVelocidad);
								actualizarMensajeVelocidadMotorDC1LCD("35%");
						}else if(estadoPlataforma==7){
								velocidad55(seleccionMotorVelocidad);
								actualizarMensajeVelocidadMotorDC1LCD("55%");
						}else if(estadoPlataforma==8){
								velocidad70(seleccionMotorVelocidad);
								actualizarMensajeVelocidadMotorDC1LCD("70%");
						}else if(estadoPlataforma==9){
								velocidad90(seleccionMotorVelocidad);
								actualizarMensajeVelocidadMotorDC1LCD("90%");
						}else if(estadoPlataforma==10){
								velocidad35(seleccionMotorVelocidad);
								actualizarMensajeVelocidadMotorDC2LCD("35%");
						}else if(estadoPlataforma==11){
								velocidad55(seleccionMotorVelocidad);
								actualizarMensajeVelocidadMotorDC2LCD("55%");
						}else if(estadoPlataforma==12){
								velocidad70(seleccionMotorVelocidad);
								actualizarMensajeVelocidadMotorDC2LCD("70%");
						}else if(estadoPlataforma==13){
								velocidad90(seleccionMotorVelocidad);
								actualizarMensajeVelocidadMotorDC2LCD("90%");
						}		
				
					}else if(estadoDeteccionLdr == 0){
							TIM5->CCR1=0;
							TIM5->CCR2=0;
							TIM5->CCR3=0;
							TIM5->CCR4=0;
							
						}
					
				
				
				
			
				//pregunta por le estado de la captura del adc con el CNY70 
				if(alertCNYGui==1){
					detener();
				
				// Enviar mensaje por USART3 cuando se detecte un objeto
            char alert[] = "cnySi\n";
            for (int i = 0; i < strlen(alert); i++) {
                USART3->TDR = alert[i];  // Transmitir datos por UART
                while(((USART3->ISR & 0x80) >> 7) == 0){}  // Esperar a que se complete la transmisión
            }
				}else if(alertCNYGui==0){
								// Enviar mensaje por USART3 cuando no se detecte un objeto
								char alert[] = "cnyNo\n";								
								for (int i = 0; i < strlen(alert); i++) {
										USART3->TDR = alert[i];
										while(((USART3->ISR & 0x80) >> 7) == 0){}
								}
						
						}
				
				//pregunta por le estado de la interrupcion del infrarojo 
				if(alertaInfrarojoGui==1){
				// Enviar mensaje por USART3 cuando se detecte un objeto
            char alert[] = "infrarojoSi\n";
            for (int i = 0; i < strlen(alert); i++) {
                USART3->TDR = alert[i];  // Transmitir datos por UART
                while(((USART3->ISR & 0x80) >> 7) == 0){}  // Esperar a que se complete la transmisión
            }
				}else if(alertaInfrarojoGui==0){
								// Enviar mensaje por USART3 cuando no se detecte un objeto
								char alert[] = "infrarojoNo\n";								
								for (int i = 0; i < strlen(alert); i++) {
										USART3->TDR = alert[i];
										while(((USART3->ISR & 0x80) >> 7) == 0){}
								}
						
						}		
				
				
			// fin ciclo while 	
    }
}
