


////// INA219 Address
////#define INA219_ADDRESS 0x40

////// INA219 Registers
////#define INA219_REG_CONFIG         0x00
////#define INA219_REG_SHUNT_VOLTAGE  0x01
////#define INA219_REG_BUS_VOLTAGE    0x02
////#define INA219_REG_POWER          0x03
////#define INA219_REG_CURRENT        0x04
////#define INA219_REG_CALIBRATION    0x05

////// Calibration values
////#define INA219_CALIBRATION_32V_2A {0x10, 0x00}
////#define CURRENT_DIVIDER_MA 10  // Divider for current to mA
////#define POWER_DIVIDER_MW 2     // Divider for power to mW

////// Variables
////uint16_t shunt_voltage_raw;
////uint16_t bus_voltage_raw;
////float shunt_voltage;  // Voltaje de la resistencia shunt
////float bus_voltage;    // Voltaje en el bus
////float current_mA;
////uint16_t current_raw;


//// Prototipos de funciones
//void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
//void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
//void configureINA219(void);
//void readINA219(void);
//void Print(char *data, int n);
//#define time 10

//// INA219 Address and Registers
//#define INA219_ADDRESS 0x40
//#define INA219_REG_CONFIG         0x00
//#define INA219_REG_SHUNT_VOLTAGE  0x01
//#define INA219_REG_BUS_VOLTAGE    0x02
//#define INA219_REG_CALIBRATION    0x05

//// Calibration values
//#define INA219_CALIBRATION_32V_2A {0x10, 0x00}
//#define CURRENT_DIVIDER_MA 10  // Divider for current to mA
//#define POWER_DIVIDER_MW 2     // Divider for power to mW

//// Variables
//uint16_t shunt_voltage_raw, bus_voltage_raw, current_raw;
//float shunt_voltage, bus_voltage, current_mA;
//char text[16];


//int main() {

//    //----------------------------------------------------------------------------
//    //                        				Systick
//    //----------------------------------------------------------------------------
//    SysTick->LOAD = 0x00FFFFFF;
//    SysTick->CTRL |= (0b101);
//// configurar LCD
//	settings();
//			//----------------------------------------------------------------------------
//    //                        				I2C
//    //----------------------------------------------------------------------------
//		//comunicacion i2c configurada para los pinees PB8 y PB9
//		RCC->AHB1ENR |= (1 << 1);  // Habilitar reloj GPIOB (PB9=I2C1_SDA y PB8=I2C1_SCL)
//    GPIOB->MODER |= (1 << 19) | (1 << 17);  // Configurar PB9 y PB8 como función alternativa
//    GPIOB->OTYPER |= (1 << 9) | (1 << 8);  // Configurar PB9 y PB8 como open-drain
//    GPIOB->OSPEEDR |= (0b11 << 18) | (0b11 << 16);  // Configurar PB9 y PB8 como alta velocidad
//    GPIOB->PUPDR |= (1 << 18) | (1 << 16);  // Pull-up para PB9 y PB8
//    GPIOB->AFR[1] |= (1 << 6) | (1 << 2);  // Seleccionar AF4 para I2C1 en PB9 y PB8
//    RCC->APB1ENR |= (1 << 21);  // Habilitar reloj I2C1
//    I2C1->CR1 &= ~(1 << 0);  // Deshabilitar I2C1 para configurar
//    I2C1->TIMINGR = 0x30420F13;  // Configuración de timing para 100 kHz
//    I2C1->CR1 |= (1 << 0);  // Habilitar I2C1

//    SysTick_ms(1000);

//		
//    // Configurar el INA219
//    configureINA219();

//    while (1) {
//         readINA219();  // Leer datos del sensor INA219
//        actualizarVoltajeEnLCD(bus_voltage);  // Mostrar el voltaje en el LCD
//        actualizarCorrienteEnLCD(current_mA);
//    }
//}

//// Configuración inicial del INA219
//void configureINA219(void) {
////    uint8_t config_data[2] = {0x01, 0x9F};  // Configuración del registro de configuración
////    WriteI2C1(INA219_ADDRESS, INA219_REG_CONFIG, config_data, 2);
////		
////		uint8_t calibration_data[2] = {0x20, 0x00};  // Valor de calibración
////    WriteI2C1(INA219_ADDRESS, INA219_REG_CALIBRATION, calibration_data, 2);	
//				uint8_t calibration_data[] = INA219_CALIBRATION_32V_2A;
//				WriteI2C1(INA219_ADDRESS, INA219_REG_CALIBRATION, calibration_data, 2);

//				uint8_t config_data[2] = {
//						(INA219_CONFIG_BVOLTAGERANGE_32V | INA219_CONFIG_GAIN_8_320MV |
//						 INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US |
//						 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS) >> 8,
//						(INA219_CONFIG_BVOLTAGERANGE_32V | INA219_CONFIG_GAIN_8_320MV |
//						 INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US |
//						 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS) & 0xFF
//				};
//				WriteI2C1(INA219_ADDRESS, INA219_REG_CONFIG, config_data, 2);
//}

//// Leer mediciones de voltaje del INA219
//void readINA219(void) {
////    uint8_t buffer[2];

////    // Leer voltaje de la resistencia shunt
////    ReadI2C1(INA219_ADDRESS, INA219_REG_SHUNT_VOLTAGE, buffer, 2);
////    shunt_voltage_raw = (buffer[0] << 8) | buffer[1];
////    shunt_voltage = shunt_voltage_raw * 0.01;  // Conversión a mV

////    // Leer voltaje del bus
////    ReadI2C1(INA219_ADDRESS, INA219_REG_BUS_VOLTAGE, buffer, 2);
////    bus_voltage_raw = (buffer[0] << 8) | buffer[1];
////    bus_voltage = (bus_voltage_raw >> 3) * 0.004;  // Conversión a V (LSB = 4 mV)

////		// Leer corriente
////    ReadI2C1(INA219_ADDRESS, INA219_REG_CURRENT, buffer, 2);
////    current_raw = (buffer[0] << 8) | buffer[1];
////    current_mA = (float)current_raw * 0.1;  // Conversión a mA
//	
//	uint8_t buffer[2];

//    // Shunt voltage
//    ReadI2C1(INA219_ADDRESS, INA219_REG_SHUNT_VOLTAGE, buffer, 2);
//    shunt_voltage_raw = (buffer[0] << 8) | buffer[1];
//    shunt_voltage = shunt_voltage_raw * 0.01;

//    // Bus voltage
//    ReadI2C1(INA219_ADDRESS, INA219_REG_BUS_VOLTAGE, buffer, 2);
//    bus_voltage_raw = (buffer[0] << 8) | buffer[1];
//    bus_voltage = (bus_voltage_raw >> 3) * 0.004;

//    // Current calculation
//    ReadI2C1(INA219_ADDRESS, INA219_REG_SHUNT_VOLTAGE, buffer, 2);
//    current_raw = (buffer[0] << 8) | buffer[1];
//    current_mA = current_raw / CURRENT_DIVIDER_MA;
//}


//void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes) {
////    uint8_t n;
////    I2C1->CR2 &= ~(0x3FF << 0);
////    I2C1->CR2 |= (Address << 1);
////    I2C1->CR2 &= ~(1 << 10);
////    I2C1->CR2 &= ~(0xFF << 16);
////    I2C1->CR2 |= ((bytes + 1) << 16);
////    I2C1->CR2 |= (1 << 25);
////    I2C1->CR2 |= (1 << 13);

////    while ((I2C1->ISR & (1 << 1)) == 0) {}

////    I2C1->TXDR = Register;

////    n = bytes;
////    while (n > 0) {
////        while ((I2C1->ISR & (1 << 1)) == 0) {}
////        I2C1->TXDR = *Data;
////        Data++;
////        n--;
////    }
////    while ((I2C1->ISR & (1 << 5)) == 0) {}

//		    I2C1->CR2 = (Address << 1) | ((bytes + 1) << 16) | (1 << 25) | (1 << 13);
//    while (!(I2C1->ISR & (1 << 1)));
//    I2C1->TXDR = Register;

//    while (bytes--) {
//        while (!(I2C1->ISR & (1 << 1)));
//        I2C1->TXDR = *Data++;
//    }
//    while (!(I2C1->ISR & (1 << 5)));
//}

//// Función de lectura I2C1
//void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes) {
////    uint8_t n;
////    I2C1->CR2 &= ~(0x3FF << 0);
////    I2C1->CR2 |= (Address << 1);
////    I2C1->CR2 &= ~(1 << 10);
////    I2C1->CR2 &= ~(0xFF << 16);
////    I2C1->CR2 |= (1 << 16);
////    I2C1->CR2 &= ~(1 << 25);
////    I2C1->CR2 |= (1 << 13);

////    while ((I2C1->ISR & (1 << 1)) == 0) {}
////    I2C1->TXDR = Register;

////    while ((I2C1->ISR & (1 << 6)) == 0) {}

////    I2C1->CR2 |= (1 << 10);
////    I2C1->CR2 &= ~(0xFF << 16);
////    I2C1->CR2 |= (bytes << 16);
////    I2C1->CR2 |= (1 << 13);

////    n = bytes;
////    while (n > 0) {
////        while ((I2C1->ISR & (1 << 2)) == 0) {}
////        *Data = I2C1->RXDR;
////        Data++;
////        n--;
////    }

////    I2C1->CR2 |= (1 << 14);
////    while ((I2C1->ISR & (1 << 5)) == 0) {}


//			 I2C1->CR2 = (Address << 1) | (1 << 16) | (1 << 13);
//    while (!(I2C1->ISR & (1 << 1)));
//    I2C1->TXDR = Register;
//    while (!(I2C1->ISR & (1 << 6)));

//    I2C1->CR2 = (Address << 1) | (bytes << 16) | (1 << 10) | (1 << 13);
//    while (bytes--) {
//        while (!(I2C1->ISR & (1 << 2)));
//        *Data++ = I2C1->RXDR;
//    }
//    I2C1->CR2 |= (1 << 14);
//}

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

























#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>



// LCD Commands
#define CD 0x01
#define EMS 0x06
#define DC 0x0F
#define FS 0x28
#define RAW1 0x80
#define RAW2 0xC0
#define RAW3 0x94
#define RAW4 0xD4



// Prototipos de funciones
void SysTick_Wait(uint32_t n);
void SysTick_ms(uint32_t x);
void LCD(unsigned char val);
void settingsLCD(unsigned char val);
void writeLCD(unsigned char val);
void settings(void);
void configureINA219(void);
void readINA219(void);
void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
void actualizarVoltajeEnLCD(float voltaje);
void actualizarCorrienteEnLCD(float corriente);

// Delay functions
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

// LCD Functions
void LCD(unsigned char val) {
    GPIOC->ODR |= 1UL << 8;  // PC8 (E)
    SysTick_ms(10);
    GPIOA->ODR = (GPIOA->ODR & ~0xF0) | ((val & 0x0F) << 4); // PA4-PA7 (D4-D7)
    SysTick_ms(1);
    GPIOC->ODR &= ~(1UL << 8); // PC8 (E)
    SysTick_ms(1);
}

void settingsLCD(unsigned char val) {
    GPIOC->ODR &= ~(1UL << 9); // PC9 (RS)
    LCD(val >> 4);
    LCD(val & 0x0F);
}

void writeLCD(unsigned char val) {
    GPIOC->ODR |= 1UL << 9; // PC9 (RS)
    LCD(val >> 4);
    LCD(val & 0x0F);
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



void actualizarVoltajeEnLCD(float voltaje, uint8_t linea) {
    char mensaje[16];
    snprintf(mensaje, sizeof(mensaje), ":%.1f V", voltaje);
    settingsLCD(linea);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

void actualizarCorrienteEnLCD(float corriente, uint8_t linea) {
    char mensaje[16];
    snprintf(mensaje, sizeof(mensaje), "C:%.1f mA", corriente);
    setCursorLCD(2, 1);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

void actualizarVoltaje2EnLCD(float voltaje) {
    char mensaje[16];
	
    snprintf(mensaje, sizeof(mensaje), "V:%.1f V", voltaje);
    setCursorLCD(2, 7);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

void actualizarCorriente2EnLCD(float corriente) {
    char mensaje[16];
    snprintf(mensaje, sizeof(mensaje), "Corriente: %.2f mA", corriente);
    settingsLCD(RAW2);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}
void actualizarVoltaje3EnLCD(float voltaje) {
    char mensaje[16];
    snprintf(mensaje, sizeof(mensaje), "Voltaje: %.2f V", voltaje);
    settingsLCD(RAW1);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

void actualizarCorrienteEn3LCD(float corriente) {
    char mensaje[16];
    snprintf(mensaje, sizeof(mensaje), "Corriente: %.2f mA", corriente);
    settingsLCD(RAW2);
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);
    }
}

void settings() {
    RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);  // GPIO A, B, C clocks

    // Configura pines PA4-PA7 para D4-D7 de la LCD
    GPIOA->MODER |= (1 << (4 * 2)) | (1 << (5 * 2)) | (1 << (6 * 2)) | (1 << (7 * 2));
    GPIOC->MODER |= (1 << (8 * 2)) | (1 << (9 * 2));

    settingsLCD(0x02);  // Modo de 4 bits
    settingsLCD(EMS);
    settingsLCD(DC);
    settingsLCD(FS);
    settingsLCD(CD);
}


#define INA219_ADDRESS_1 0x40
#define INA219_ADDRESS_2 0x41
#define INA219_ADDRESS_3 0x44
#define INA219_REG_CONFIG         0x00
#define INA219_REG_SHUNT_VOLTAGE  0x01
#define INA219_REG_BUS_VOLTAGE    0x02
#define INA219_REG_CALIBRATION    0x05

// Calibration values
#define INA219_CALIBRATION_32V_400mA {0x04, 0x00}
#define CURRENT_DIVIDER_MA 140 // Divider for current to mA
#define POWER_DIVIDER_MW 2     // Divider for power to mW

// Variables
uint16_t shunt_voltage_raw, bus_voltage_raw, current_raw;
float shunt_voltage, bus_voltage, current_mA;
char text[16];



// INA219 Configuration
void configureINA219(uint8_t address) {
    uint8_t calibration_data[] = INA219_CALIBRATION_32V_400mA;
    WriteI2C1(address, INA219_REG_CALIBRATION, calibration_data, 2);

    uint8_t config_data[2] = {
        (0x2000 | 0x1800 | 0x0400 | 0x0018 | 0x0007) >> 8,
        (0x2000 | 0x1800 | 0x0400 | 0x0018 | 0x0007) & 0xFF
    };
    WriteI2C1(address, INA219_REG_CONFIG, config_data, 2);
}

// INA219 Read Function
void readINA219(uint8_t address, float *bus_voltage, float *current_mA) {
    uint8_t buffer[2];

    // Leer voltaje en el shunt
    ReadI2C1(address, INA219_REG_SHUNT_VOLTAGE, buffer, 2);
    shunt_voltage_raw = (buffer[0] << 8) | buffer[1];
    shunt_voltage = shunt_voltage_raw * 0.01;

    // Leer voltaje en el bus
    ReadI2C1(address, INA219_REG_BUS_VOLTAGE, buffer, 2);
    bus_voltage_raw = (buffer[0] << 8) | buffer[1];
    *bus_voltage = (bus_voltage_raw >> 3) * 0.004;

    // Calcular corriente
    ReadI2C1(address, INA219_REG_SHUNT_VOLTAGE, buffer, 2);
    current_raw = (buffer[0] << 8) | buffer[1];
    *current_mA = current_raw / (float)CURRENT_DIVIDER_MA;
}

// I2C Write
void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes) {
    I2C1->CR2 = (Address << 1) | ((bytes + 1) << 16) | (1 << 25) | (1 << 13);
    while (!(I2C1->ISR & (1 << 1)));
    I2C1->TXDR = Register;

    while (bytes--) {
        while (!(I2C1->ISR & (1 << 1)));
        I2C1->TXDR = *Data++;
    }
    while (!(I2C1->ISR & (1 << 5)));
}

// I2C Read
void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes) {
    I2C1->CR2 = (Address << 1) | (1 << 16) | (1 << 13);
    while (!(I2C1->ISR & (1 << 1)));
    I2C1->TXDR = Register;
    while (!(I2C1->ISR & (1 << 6)));

    I2C1->CR2 = (Address << 1) | (bytes << 16) | (1 << 10) | (1 << 13);
    while (bytes--) {
        while (!(I2C1->ISR & (1 << 2)));
        *Data++ = I2C1->RXDR;
    }
    I2C1->CR2 |= (1 << 14);
}

// Main function
int main() {
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);

    settings();

    // Configurar I2C
    RCC->AHB1ENR |= (1 << 1);  // Habilitar reloj GPIOB
    GPIOB->MODER |= (1 << 19) | (1 << 17);  // PB9 y PB8 función alternativa
    GPIOB->OTYPER |= (1 << 9) | (1 << 8);  // Open-drain
    GPIOB->OSPEEDR |= (0b11 << 18) | (0b11 << 16);  // Alta velocidad
    GPIOB->PUPDR |= (1 << 18) | (1 << 16);  // Pull-up
    GPIOB->AFR[1] |= (1 << 6) | (1 << 2);  // AF4 para I2C1

    RCC->APB1ENR |= (1 << 21);  // Habilitar I2C1
    I2C1->CR1 &= ~(1 << 0);  // Deshabilitar I2C1 para configurar
    I2C1->TIMINGR = 0x30420F13;  // Configuración de timing para 100 kHz
    I2C1->CR1 |= (1 << 0);  // Habilitar I2C1

    SysTick_ms(1000);
    configureINA219(INA219_ADDRESS_1);
    configureINA219(INA219_ADDRESS_2);
    configureINA219(INA219_ADDRESS_3);

    float bus_voltage1, current_mA1, bus_voltage2, current_mA2, bus_voltage3, current_mA3;

    while (1) {
//        readINA219();
//        actualizarVoltajeEnLCD(bus_voltage);
//        actualizarCorrienteEnLCD(current_mA);
			readINA219(INA219_ADDRESS_1, &bus_voltage1, &current_mA1);
        actualizarVoltajeEnLCD(bus_voltage1, RAW1);
        actualizarCorrienteEnLCD(current_mA1, RAW2);
			
				readINA219(INA219_ADDRESS_2, &bus_voltage2, &current_mA2);
        actualizarVoltajeEnLCD(bus_voltage2, RAW3);
        actualizarCorrienteEnLCD(current_mA2, RAW4);
			readINA219(INA219_ADDRESS_3, &bus_voltage3, &current_mA3);
    }
}












//#include <stdio.h>
//#include "stm32f7xx.h"
//#include <string.h>
//#include <stdint.h>
//#include <stdlib.h>

//#define INA219_ADDRESS_1 0x40
//#define INA219_ADDRESS_2 0x41
//#define INA219_ADDRESS_3 0x44
//#define INA219_REG_CONFIG         0x00
//#define INA219_REG_SHUNT_VOLTAGE  0x01
//#define INA219_REG_BUS_VOLTAGE    0x02
//#define INA219_REG_CALIBRATION    0x05

// Calibration values
//#define INA219_CALIBRATION_32V_400mA {0x04, 0x00}
//#define CURRENT_DIVIDER_MA 140 // Divider for current to mA
//#define POWER_DIVIDER_MW 2     // Divider for power to mW

// LCD Commands
//#define CD 0x01
//#define EMS 0x06
//#define DC 0x0F
//#define FS 0x28
//#define RAW1 0x80
//#define RAW2 0xC0
//#define RAW3 0x94
//#define RAW4 0xD4

// Variables
//uint16_t shunt_voltage_raw, bus_voltage_raw, current_raw;
//float shunt_voltage, bus_voltage, current_mA;
//char text[16];

// Prototipos de funciones
//void SysTick_Wait(uint32_t n);
//void SysTick_ms(uint32_t x);
//void LCD(unsigned char val);
//void settingsLCD(unsigned char val);
//void writeLCD(unsigned char val);
//void settings(void);
//void configureINA219(uint8_t address);
//void readINA219(uint8_t address, float *bus_voltage, float *current_mA);
//void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
//void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
//void actualizarVoltajeEnLCD(float voltaje, uint8_t linea);
//void actualizarCorrienteEnLCD(float corriente, uint8_t linea);

// Delay functions
//void SysTick_Wait(uint32_t n) {
//    SysTick->LOAD = n - 1;
//    SysTick->VAL = 0;
//    while (((SysTick->CTRL & 0x00010000) >> 16) == 0);
//}

//void SysTick_ms(uint32_t x) {
//    for (uint32_t i = 0; i < x; i++) {
//        SysTick_Wait(16000);
//    }
//}

// LCD Functions
//void LCD(unsigned char val) {
//    GPIOC->ODR |= 1UL << 8;  // PC8 (E)
//    SysTick_ms(10);
//    GPIOA->ODR = (GPIOA->ODR & ~0xF0) | ((val & 0x0F) << 4); // PA4-PA7 (D4-D7)
//    SysTick_ms(1);
//    GPIOC->ODR &= ~(1UL << 8); // PC8 (E)
//    SysTick_ms(1);
//}

//void settingsLCD(unsigned char val) {
//    GPIOC->ODR &= ~(1UL << 9); // PC9 (RS)
//    LCD(val >> 4);
//    LCD(val & 0x0F);
//}

//void writeLCD(unsigned char val) {
//    GPIOC->ODR |= 1UL << 9; // PC9 (RS)
//    LCD(val >> 4);
//    LCD(val & 0x0F);
//}

//void actualizarVoltajeEnLCD(float voltaje, uint8_t linea) {
//    char mensaje[16];
//    snprintf(mensaje, sizeof(mensaje), "Voltaje: %.2f V", voltaje);
//    settingsLCD(linea);
//    for (int i = 0; mensaje[i] != '\0'; i++) {
//        writeLCD(mensaje[i]);
//    }
//}

//void actualizarCorrienteEnLCD(float corriente, uint8_t linea) {
//    char mensaje[16];
//    snprintf(mensaje, sizeof(mensaje), "Corriente: %.2f mA", corriente);
//    settingsLCD(linea);
//    for (int i = 0; mensaje[i] != '\0'; i++) {
//        writeLCD(mensaje[i]);
//    }
//}

//void settings() {
//    RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);  // GPIO A, B, C clocks

//    // Configura pines PA4-PA7 para D4-D7 de la LCD
//    GPIOA->MODER |= (1 << (4 * 2)) | (1 << (5 * 2)) | (1 << (6 * 2)) | (1 << (7 * 2));
//    GPIOC->MODER |= (1 << (8 * 2)) | (1 << (9 * 2));

//    settingsLCD(0x02);  // Modo de 4 bits
//    settingsLCD(EMS);
//    settingsLCD(DC);
//    settingsLCD(FS);
//    settingsLCD(CD);
//}

// INA219 Configuration
//void configureINA219(uint8_t address) {
//    uint8_t calibration_data[] = INA219_CALIBRATION_32V_400mA;
//    WriteI2C1(address, INA219_REG_CALIBRATION, calibration_data, 2);

//    uint8_t config_data[2] = {
//        (0x2000 | 0x1800 | 0x0400 | 0x0018 | 0x0007) >> 8,
//        (0x2000 | 0x1800 | 0x0400 | 0x0018 | 0x0007) & 0xFF
//    };
//    WriteI2C1(address, INA219_REG_CONFIG, config_data, 2);
//}

// INA219 Read Function
//void readINA219(uint8_t address, float *bus_voltage, float *current_mA) {
//    uint8_t buffer[2];

//    // Leer voltaje en el shunt
//    ReadI2C1(address, INA219_REG_SHUNT_VOLTAGE, buffer, 2);
//    shunt_voltage_raw = (buffer[0] << 8) | buffer[1];
//    shunt_voltage = shunt_voltage_raw * 0.01;

//    // Leer voltaje en el bus
//    ReadI2C1(address, INA219_REG_BUS_VOLTAGE, buffer, 2);
//    bus_voltage_raw = (buffer[0] << 8) | buffer[1];
//    *bus_voltage = (bus_voltage_raw >> 3) * 0.004;

//    // Calcular corriente
//    ReadI2C1(address, INA219_REG_SHUNT_VOLTAGE, buffer, 2);
//    current_raw = (buffer[0] << 8) | buffer[1];
//    *current_mA = current_raw / (float)CURRENT_DIVIDER_MA;
//}

// I2C Write
//void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes) {
//    I2C1->CR2 = (Address << 1) | ((bytes + 1) << 16) | (1 << 25) | (1 << 13);
//    while (!(I2C1->ISR & (1 << 1)));
//    I2C1->TXDR = Register;

//    while (bytes--) {
//        while (!(I2C1->ISR & (1 << 1)));
//        I2C1->TXDR = *Data++;
//    }
//    while (!(I2C1->ISR & (1 << 5)));
//}

// I2C Read
//void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes) {
//    I2C1->CR2 = (Address << 1) | (1 << 16) | (1 << 13);
//    while (!(I2C1->ISR & (1 << 1)));
//    I2C1->TXDR = Register;
//    while (!(I2C1->ISR & (1 << 6)));

//    I2C1->CR2 = (Address << 1) | (bytes << 16) | (1 << 10) | (1 << 13);
//    while (bytes--) {
//        while (!(I2C1->ISR & (1 << 2)));
//        *Data++ = I2C1->RXDR;
//    }
//    I2C1->CR2 |= (1 << 14);
//}

// Main function
//int main() {
//    SysTick->LOAD = 0x00FFFFFF;
//    SysTick->CTRL |= (0b101);

//    settings();

//    // Configurar I2C
//    RCC->AHB1ENR |= (1 << 1);  // Habilitar reloj GPIOB
//    GPIOB->MODER |= (1 << 19) | (1 << 17);  // PB9 y PB8 función alternativa
//    GPIOB->OTYPER |= (1 << 9) | (1 << 8);  // Open-drain
//    GPIOB->OSPEEDR |= (0b11 << 18) | (0b11 << 16);  // Alta velocidad
//    GPIOB->PUPDR |= (1 << 18) | (1 << 16);  // Pull-up
//    GPIOB->AFR[1] |= (1 << 6) | (1 << 2);  // AF4 para I2C1

//    RCC->APB1ENR |= (1 << 21);  // Habilitar I2C1
//    I2C1->CR1 &= ~(1 << 0);  // Deshabilitar I2C1 para configurar
//    I2C1->TIMINGR = 0x30420F13;  // Configuración de timing para 100 kHz
//    I2C1->CR1 |= (1 << 0);  // Habilitar I2C1

//    SysTick_ms(1000);
//    configureINA219(INA219_ADDRESS_1);
//    configureINA219(INA219_ADDRESS_2);
//    configureINA219(INA219_ADDRESS_3);

//    float bus_voltage1, current_mA1, bus_voltage2, current_mA2, bus_voltage3, current_mA3;

//    while (1) {
//        readINA219(INA219_ADDRESS_1, &bus_voltage1, &current_mA1);
//        actualizarVoltajeEnLCD(bus_voltage1, RAW1);
//        actualizarCorrienteEnLCD(current_mA1, RAW2);

//        readINA219(INA219_ADDRESS_2, &bus_voltage2, &current_mA2);
//        actualizarVoltajeEnLCD(bus_voltage2, RAW3);
//        actualizarCorrienteEnLCD(current_mA2, RAW4);

//        readINA219(INA219_ADDRESS_3, &bus_voltage3, &current_mA3);
//        // Asigna líneas para el tercer sensor en caso de que tengas más espacio en el LCD

//        SysTick_ms(1000);
//    }
//}
