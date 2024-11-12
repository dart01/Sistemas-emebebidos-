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

uint8_t flag_cronometro = 0, cont = 0, cronometro_iniciado = 0;
int segundos = 0;
int minutos = 0;  // Variable para contar los minutos

// INA219 Address
#define INA219_ADDRESS_1 0x40
#define INA219_ADDRESS_2 0x41
#define INA219_ADDRESS_3 0x44

// INA219 Registers
#define INA219_REG_CONFIG         0x00
#define INA219_REG_SHUNT_VOLTAGE  0x01
#define INA219_REG_BUS_VOLTAGE    0x02
#define INA219_REG_POWER          0x03
#define INA219_REG_CURRENT        0x04
#define INA219_REG_CALIBRATION    0x05

// Variables
uint16_t shunt_voltage_raw;
uint16_t bus_voltage_raw;
float shunt_voltage;  // Voltaje de la resistencia shunt
float bus_voltage;    // Voltaje en el bus
float current_mA;
uint16_t current_raw;




void UART_Transmit(char *data) {
    while (*data) {
        while (!(USART3->ISR & USART_ISR_TXE)); // Esperar a que el registro de transmisión esté vacío
        USART3->TDR = *data++; // Enviar carácter actual
    }
    while (!(USART3->ISR & USART_ISR_TC)); // Esperar a que la transmisión termine
}

void enviarDatosUART(float voltaje1, float corriente1, float voltaje2, float corriente2, float voltaje3, float corriente3) {
    char buffer[50];

    // Enviar datos de cada motor con su identificador (M1, M2, M3)
    snprintf(buffer, sizeof(buffer), "M1 V:%.2f C:%.1f\n", voltaje1, corriente1);
    UART_Transmit(buffer);

    snprintf(buffer, sizeof(buffer), "M2 V:%.2f C:%.1f\n", voltaje2, corriente2);
    UART_Transmit(buffer);

    snprintf(buffer, sizeof(buffer), "M3 V:%.2f C:%.1f\n", voltaje3, corriente3);
    UART_Transmit(buffer);
}

typedef struct {
    float shunt_voltage;
    float bus_voltage;
    float current_mA;
} INA219_Data;



void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes) {
    uint8_t n;
    I2C1->CR2 &= ~(0x3FF << 0);
    I2C1->CR2 |= (Address << 1);
    I2C1->CR2 &= ~(1 << 10);
    I2C1->CR2 &= ~(0xFF << 16);
    I2C1->CR2 |= ((bytes + 1) << 16);
    I2C1->CR2 |= (1 << 25);
    I2C1->CR2 |= (1 << 13);

    while ((I2C1->ISR & (1 << 1)) == 0) {}

    I2C1->TXDR = Register;

    n = bytes;
    while (n > 0) {
        while ((I2C1->ISR & (1 << 1)) == 0) {}
        I2C1->TXDR = *Data;
        Data++;
        n--;
    }
    while ((I2C1->ISR & (1 << 5)) == 0) {}
}

// Función de lectura I2C1
void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes) {
    uint8_t n;
    I2C1->CR2 &= ~(0x3FF << 0);
    I2C1->CR2 |= (Address << 1);
    I2C1->CR2 &= ~(1 << 10);
    I2C1->CR2 &= ~(0xFF << 16);
    I2C1->CR2 |= (1 << 16);
    I2C1->CR2 &= ~(1 << 25);
    I2C1->CR2 |= (1 << 13);

    while ((I2C1->ISR & (1 << 1)) == 0) {}
    I2C1->TXDR = Register;

    while ((I2C1->ISR & (1 << 6)) == 0) {}

    I2C1->CR2 |= (1 << 10);
    I2C1->CR2 &= ~(0xFF << 16);
    I2C1->CR2 |= (bytes << 16);
    I2C1->CR2 |= (1 << 13);

    n = bytes;
    while (n > 0) {
        while ((I2C1->ISR & (1 << 2)) == 0) {}
        *Data = I2C1->RXDR;
        Data++;
        n--;
    }

    I2C1->CR2 |= (1 << 14);
    while ((I2C1->ISR & (1 << 5)) == 0) {}
}
// Leer mediciones de voltaje del INA219
void readINA219(uint8_t address, INA219_Data *data) {
    uint8_t buffer[2];

    // Leer voltaje de la resistencia shunt
    ReadI2C1(address, INA219_REG_SHUNT_VOLTAGE, buffer, 2);
    uint16_t shunt_voltage_raw = (buffer[0] << 8) | buffer[1];
    data->shunt_voltage = shunt_voltage_raw * 0.01;  // Conversión a mV

    // Leer voltaje del bus
    ReadI2C1(address, INA219_REG_BUS_VOLTAGE, buffer, 2);
    uint16_t bus_voltage_raw = (buffer[0] << 8) | buffer[1];
    data->bus_voltage = (bus_voltage_raw >> 3) * 0.004;  // Conversión a V (LSB = 4 mV)

    // Leer corriente
    ReadI2C1(address, INA219_REG_CURRENT, buffer, 2);
    uint16_t current_raw = (buffer[0] << 8) | buffer[1];
    data->current_mA = (float)current_raw * 0.1;  // Conversión a mA
}

// Configuración inicial del INA219
void configureINA219(uint8_t address) {
    uint8_t config_data[2] = {0x01, 0x9F};  // Configuración del registro de configuración
    WriteI2C1(address, INA219_REG_CONFIG, config_data, 2);
		
		uint8_t calibration_data[2] = {0x05, 0x00};  // Valor de calibración
    WriteI2C1(address, INA219_REG_CALIBRATION, calibration_data, 2);	

}

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

void configuracionComuniacionI2C(){
		//----------------------------------------------------------------------------
    //                        				I2C
    //----------------------------------------------------------------------------
		//comunicacion i2c configurada para los pinees PB8 y PB9
		RCC->AHB1ENR |= (1 << 1);  // Habilitar reloj GPIOB (PB9=I2C1_SDA y PB8=I2C1_SCL)
    GPIOB->MODER |= (1 << 19) | (1 << 17);  // Configurar PB9 y PB8 como función alternativa
    GPIOB->OTYPER |= (1 << 9) | (1 << 8);  // Configurar PB9 y PB8 como open-drain
    GPIOB->OSPEEDR |= (0b11 << 18) | (0b11 << 16);  // Configurar PB9 y PB8 como alta velocidad
    GPIOB->PUPDR |= (1 << 18) | (1 << 16);  // Pull-up para PB9 y PB8
    GPIOB->AFR[1] |= (1 << 6) | (1 << 2);  // Seleccionar AF4 para I2C1 en PB9 y PB8
    RCC->APB1ENR |= (1 << 21);  // Habilitar reloj I2C1
    I2C1->CR1 &= ~(1 << 0);  // Deshabilitar I2C1 para configurar
    I2C1->TIMINGR = 0x30420F13;  // Configuración de timing para 100 kHz
    I2C1->CR1 |= (1 << 0);  // Habilitar I2C1

    SysTick_ms(1000);
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
		setCursorLCD(1, 4);  // Fila 1, Columna 5
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
		
}

void actualizarMensaje3LCD(const char* mensaje) {
		setCursorLCD(1, 7);  // Fila 1, Columna 9
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}

void actualizarMensaje4LCD(const char* mensaje) {
		setCursorLCD(1, 10);  // Fila 1, Columna 13
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

void actualizarVoltajeServo1EnLCD(float voltaje) {
    char mensaje[8];  // Buffer para el mensaje, hasta 16 caracteres para el LCD
    snprintf(mensaje, sizeof(mensaje), "%.1f", voltaje);
    setCursorLCD(1, 13);  // Posicionar en la primera línea, primera columna
    
    // Escribir el mensaje en el LCD
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

// Mostrar corriente en la LCD
void actualizarCorrienteServo1EnLCD(float corriente) {
    char mensaje[8];
    snprintf(mensaje, sizeof(mensaje), "%.1f", corriente);
    limpiarLineaLCD(2);
    setCursorLCD(2, 7);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}


void actualizarVoltajeServo2EnLCD(float voltaje) {
    char mensaje[16];  // Buffer para el mensaje, hasta 16 caracteres para el LCD
		snprintf(mensaje, sizeof(mensaje), "%.1f", voltaje);
    setCursorLCD(2, 11);  // Posicionar en la primera línea, primera columna 
    // Escribir el mensaje en el LCD
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

// Mostrar corriente en la LCD
void actualizarCorrienteServo2EnLCD(float corriente) {
    char mensaje[16];
    snprintf(mensaje, sizeof(mensaje), "%.1f", corriente);
    
    setCursorLCD(2, 11);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}


void actualizarVoltajeServo3EnLCD(float voltaje) {
    char mensaje[16];  // Buffer para el mensaje, hasta 16 caracteres para el LCD
		snprintf(mensaje, sizeof(mensaje), "%.1f", voltaje);
    setCursorLCD(2, 11);  // Posicionar en la primera línea, primera columna 
    // Escribir el mensaje en el LCD
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

// Mostrar corriente en la LCD
void actualizarCorrienteServo3EnLCD(float corriente) {
    char mensaje[16];
    snprintf(mensaje, sizeof(mensaje), "%.1f", corriente);
    
    setCursorLCD(2, 14);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
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



extern "C" {
	//interrupcion de comunicacion usart3 para interfaz en matlab
    void USART3_IRQHandler(void) {
        if ((USART3->ISR & USART_ISR_RXNE) != 0) {  // Dato recibido
            uint8_t received_byte = USART3->RDR;    // Leer el byte recibido
            
            // Almacenar el byte recibido
            received_data[byte_count++] = received_byte;
            
            // Si se han recibido dos bytes (identificador y valor)
            if (byte_count >= 2) {
                uint8_t identifier = received_data[0];  // Primer byte es el identificador
                uint8_t angle = received_data[1];       // Segundo byte es el ángulo

                if (identifier == 1) {
                    slider1_angle = angle;  // Guardar ángulo para slider 1
                } else if (identifier == 2) {
                    slider2_angle = angle;  // Guardar ángulo para slider 2
                } else if (identifier == 3) {
                    slider3_angle = angle;  // Guardar ángulo para slider 3
                }else if (identifier == 4) {
											botton_angle = angle;
								}else if (identifier == 9) {
											cronometro_iniciado = 1 ;
								}
                byte_count = 0;  // Reiniciar contador para la próxima recepción
            }
        }
    }
		
		//interrupcion de comunicacion usart4 para modulo hc05
		void UART4_IRQHandler(void) {

					 static uint8_t buffer[7]; // Buffer para almacenar "id:valor"
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
								
									
											// Almacenar el ángulo según el identificador
											if (id == 1) {
													slider1_angle = angle;
											} else if (id == 2) {
													slider2_angle = angle;
											} else if (id == 3) {
													slider3_angle = angle;
											}else if (id == 4) {
													botton_angle = angle;
											}else if (id == 9) {
											cronometro_iniciado = 1 ;
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
                }
                flag_cronometro = 1;   // Activar la bandera para actualizar la LCD
							
							}
						}
				}
			}
		
		//final c
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
void actualizarCronometro(){
	// actualizacion cronometro display
				if (flag_cronometro) {  // Si hubo un cambio en el cronómetro
            flag_cronometro = 0;
            sprintf(text, "%02d:%02d", minutos, segundos);  // Formatear el mensaje con minutos y segundos
            // setCursorLCD(2, 1);  // Posicionar el cursor en la columna 1 de la fila 2
						actualizarMensaje2LCD(text);  // Actualizar la segunda línea con el cronómetro
						
				} 
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

int main() {
	
    // Systick
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);
		
		settings();
		configuracionTimerCronometro();
		configuracionUART3comunicacionPc();
		configurarUART4BluetoothHc50();// pc10 uart4 TX --- pc11 uart4 RX
    configuracionPWM();
    configuracionComuniacionI2C();
		// Configurar el INA219
    //configureINA219();
		configureINA219(INA219_ADDRESS_1);
		configureINA219(INA219_ADDRESS_2);
		configureINA219(INA219_ADDRESS_3);
		
		// Declaración de estructuras para almacenar datos de cada sensor
    INA219_Data sensor1_data, sensor2_data, sensor3_data;
	
		//Mostar incicialmente cronometro en 00:00
		actualizarMensajeCronometroLCD("0:00");
		
    // Bucle principal
    while (1) {		
					
			//actualizarCronometro();
				
			ajustarServo();
					
			// Leer datos de cada sensor
        readINA219(INA219_ADDRESS_1, &sensor1_data);
				readINA219(INA219_ADDRESS_2, &sensor2_data);
				readINA219(INA219_ADDRESS_3, &sensor3_data);
				actualizarVoltajeServo1EnLCD(sensor1_data.bus_voltage);
			SysTick_ms(200); 
				actualizarCorrienteServo1EnLCD(sensor1_data.current_mA);
			SysTick_ms(200); 
			actualizarCorrienteServo2EnLCD(sensor2_data.current_mA);
			SysTick_ms(200); 
			actualizarCorrienteServo3EnLCD(sensor3_data.current_mA);
			SysTick_ms(200); 
				// Enviar datos por UART
        enviarDatosUART(sensor1_data.bus_voltage, sensor1_data.current_mA, sensor2_data.bus_voltage, sensor2_data.current_mA,sensor3_data.bus_voltage, sensor3_data.current_mA);
				
				
			
				//verificar el estado del cronometro 
				if (flag_cronometro) {  // Si hubo un cambio en el cronómetro
            flag_cronometro = 0;
            sprintf(text, "%02d:%02d", minutos, segundos);  // Formatear el mensaje con minutos y segundos
            // setCursorLCD(2, 1);  // Posicionar el cursor en la columna 1 de la fila 2
						actualizarMensajeCronometroLCD(text);  // Actualizar la segunda línea con el cronómetro
						
				}	
			
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
        if (slider3_angle != last_angle3) {
            last_angle3 = slider3_angle;
            actualizarLCD_AnguloServo3(last_angle3);
        }
				
				// Verificar y actualizar el ángulo del servo 4
        if (botton_angle != last_botton) {
            last_botton = botton_angle;
            actualizarLCD_AnguloServo4(last_botton);
        }
				
				
				
    }
}
