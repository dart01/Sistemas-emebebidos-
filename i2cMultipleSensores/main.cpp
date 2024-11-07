#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
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

// Función para imprimir el valor del voltaje en el LCD
void actualizarVoltajeServo1EnLCD(float voltaje) {
    char mensaje[8];  // Buffer para el mensaje, hasta 16 caracteres para el LCD
    snprintf(mensaje, sizeof(mensaje), "V%.1f", voltaje);
    // Limpiar la primera línea del LCD y posicionar el cursor
    limpiarLineaLCD(1);
    setCursorLCD(1, 1);  // Posicionar en la primera línea, primera columna
    
    // Escribir el mensaje en el LCD
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

// Mostrar corriente en la LCD
void actualizarCorrienteServo1EnLCD(float corriente) {
    char mensaje[8];
    snprintf(mensaje, sizeof(mensaje), "C%.1f", corriente);
    limpiarLineaLCD(2);
    setCursorLCD(1, 6);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

void actualizarVoltajeServo2EnLCD(float voltaje) {
    char mensaje[16];  // Buffer para el mensaje, hasta 16 caracteres para el LCD
		snprintf(mensaje, sizeof(mensaje), "V%.1f", voltaje);
    setCursorLCD(1, 11);  // Posicionar en la primera línea, primera columna
    // Escribir el mensaje en el LCD
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

// Mostrar corriente en la LCD
void actualizarCorrienteServo2EnLCD(float corriente) {
    char mensaje[16];
    snprintf(mensaje, sizeof(mensaje), "C%.1f", corriente);
    limpiarLineaLCD(2);
    setCursorLCD(2, 1);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}


void actualizarVoltajeServo3EnLCD(float voltaje) {
    char mensaje[16];  // Buffer para el mensaje, hasta 16 caracteres para el LCD
		snprintf(mensaje, sizeof(mensaje), "V%.1f", voltaje);
    setCursorLCD(2, 6);  // Posicionar en la primera línea, primera columna
    
    // Escribir el mensaje en el LCD
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

// Mostrar corriente en la LCD
void actualizarCorrienteServo3EnLCD(float corriente) {
    char mensaje[16];
    snprintf(mensaje, sizeof(mensaje), "C:%.1f", corriente);
    setCursorLCD(2, 11);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
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

// Configuración inicial del INA219
void configureINA219(uint8_t address) {
    uint8_t config_data[2] = {0x01, 0x9F};  // Configuración del registro de configuración
    WriteI2C1(address, INA219_REG_CONFIG, config_data, 2);
		
		uint8_t calibration_data[2] = {0x20, 0x00};  // Valor de calibración
    WriteI2C1(address, INA219_REG_CALIBRATION, calibration_data, 2);	

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






//// Función para imprimir en UART
//void Print(char *data, int n) {
//    for (int j = 0; j < n; j++) {
//        USART3->TDR = *data;
//        data++;
//        while ((USART3->ISR & (1 << 7)) == 0) {}
//    }
//    USART3->TDR = 0x0D;  // Carriage return
//    while ((USART3->ISR & (1 << 7)) == 0) {}
//}


int main() {

    //----------------------------------------------------------------------------
    //                        				Systick
    //----------------------------------------------------------------------------
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);
// configurar LCD
	settings();
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

		
    // Configurar el INA219
    configureINA219(INA219_ADDRESS_1);
		configureINA219(INA219_ADDRESS_2);
		configureINA219(INA219_ADDRESS_3);
		
		 // Declaración de estructuras para almacenar datos de cada sensor
    INA219_Data sensor1_data, sensor2_data, sensor3_data;
		
		//INA219_Data sensor1_data, sensor2_data;
    while (1) {
         
				// Leer datos de cada sensor
        readINA219(INA219_ADDRESS_1, &sensor1_data);
				readINA219(INA219_ADDRESS_2, &sensor2_data);
        readINA219(INA219_ADDRESS_3, &sensor3_data);
			
       

       // Mostrar o enviar datos del sensor 1
       actualizarVoltajeServo1EnLCD(sensor1_data.bus_voltage);
       actualizarCorrienteServo1EnLCD(sensor1_data.current_mA);

        // Mostrar o enviar datos del sensor 2
        actualizarVoltajeServo2EnLCD(sensor2_data.bus_voltage);
        actualizarCorrienteServo2EnLCD(sensor2_data.current_mA);

//        // Mostrar o enviar datos del sensor 3
        actualizarVoltajeServo3EnLCD(sensor3_data.bus_voltage);
        actualizarCorrienteServo3EnLCD(sensor3_data.current_mA);

        // Delay para evitar actualización demasiado rápida en pantalla
        //SysTick_ms(1000);
    }
}

