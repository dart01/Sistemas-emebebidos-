#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>
#include <stdlib.h>  // Para la funci�n atoi
#define BUFFER_SIZE 3
uint8_t flag = 0;
//char bufferUART[10];   // Buffer para almacenar los datos recibidos por UART
uint8_t bufferIndex = 0;
int valorEntero = 0;   // Variable para almacenar el valor entero recibido
int movimiento = 0;
char d;
char bufferUART[BUFFER_SIZE];  // Buffer para almacenar los datos recibidos


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

void ajustarServo(uint32_t angle) {
    // Ajustar el PWM seg�n el �ngulo
    TIM5->CCR1 = (int)((1939.48 * (float)angle / 180.0) + 1600.0);
}


extern "C" {
    void USART3_IRQHandler(void) {
        if (USART3->ISR & USART_ISR_RXNE) {  // Si hay un dato recibido por UART
            char receivedChar = USART3->RDR;  // Leer el dato recibido
            
            // Almacena el car�cter recibido en el buffer
            bufferUART[bufferIndex++] = receivedChar;

            // Si ya se han recibido los dos caracteres, procesamos el comando
            if (bufferIndex >= BUFFER_SIZE) {
                bufferUART[BUFFER_SIZE] = '\0';  // Agregar el terminador de cadena

                // Comparar los comandos recibidos
               // Comparar los comandos recibidos (de menor a mayor)
								if (strcmp(bufferUART, "000") == 0) {
											TIM5->CCR1 = 1600;  // �ngulo 0�
									} else if (strcmp(bufferUART, "002") == 0) {
											TIM5->CCR1 = 1689;  // �ngulo 2.5�
									} else if (strcmp(bufferUART, "005") == 0) {
											TIM5->CCR1 = 1778;  // �ngulo 5�
									} else if (strcmp(bufferUART, "007") == 0) {
											TIM5->CCR1 = 1867;  // �ngulo 7.5�
									} else if (strcmp(bufferUART, "010") == 0) {
											TIM5->CCR1 = 1956;  // �ngulo 10�
									} else if (strcmp(bufferUART, "012") == 0) {
											TIM5->CCR1 = 2044;  // �ngulo 12.5�
									} else if (strcmp(bufferUART, "015") == 0) {
											TIM5->CCR1 = 2133;  // �ngulo 15�
									} else if (strcmp(bufferUART, "017") == 0) {
											TIM5->CCR1 = 2222;  // �ngulo 17.5�
									} else if (strcmp(bufferUART, "020") == 0) {
											TIM5->CCR1 = 2311;  // �ngulo 20�
									} else if (strcmp(bufferUART, "022") == 0) {
											TIM5->CCR1 = 2400;  // �ngulo 22.5�
									} else if (strcmp(bufferUART, "025") == 0) {
											TIM5->CCR1 = 2489;  // �ngulo 25�
									} else if (strcmp(bufferUART, "027") == 0) {
											TIM5->CCR1 = 2578;  // �ngulo 27.5�
									} else if (strcmp(bufferUART, "030") == 0) {
											TIM5->CCR1 = 2667;  // �ngulo 30�
									} else if (strcmp(bufferUART, "032") == 0) {
											TIM5->CCR1 = 2756;  // �ngulo 32.5�
									} else if (strcmp(bufferUART, "035") == 0) {
											TIM5->CCR1 = 2844;  // �ngulo 35�
									} else if (strcmp(bufferUART, "037") == 0) {
											TIM5->CCR1 = 2933;  // �ngulo 37.5�
									} else if (strcmp(bufferUART, "040") == 0) {
											TIM5->CCR1 = 3022;  // �ngulo 40�
									} else if (strcmp(bufferUART, "042") == 0) {
											TIM5->CCR1 = 3111;  // �ngulo 42.5�
									} else if (strcmp(bufferUART, "045") == 0) {
											TIM5->CCR1 = 3200;  // �ngulo 45�
									} else if (strcmp(bufferUART, "047") == 0) {
											TIM5->CCR1 = 3289;  // �ngulo 47.5�
									} else if (strcmp(bufferUART, "050") == 0) {
											TIM5->CCR1 = 3378;  // �ngulo 50�
									} else if (strcmp(bufferUART, "052") == 0) {
											TIM5->CCR1 = 3467;  // �ngulo 52.5�
									} else if (strcmp(bufferUART, "055") == 0) {
											TIM5->CCR1 = 3556;  // �ngulo 55�
									} else if (strcmp(bufferUART, "057") == 0) {
											TIM5->CCR1 = 3644;  // �ngulo 57.5�
									} else if (strcmp(bufferUART, "060") == 0) {
											TIM5->CCR1 = 3733;  // �ngulo 60�
									} else if (strcmp(bufferUART, "062") == 0) {
											TIM5->CCR1 = 3822;  // �ngulo 62.5�
									} else if (strcmp(bufferUART, "065") == 0) {
											TIM5->CCR1 = 3911;  // �ngulo 65�
									} else if (strcmp(bufferUART, "067") == 0) {
											TIM5->CCR1 = 4000;  // �ngulo 67.5�
									} else if (strcmp(bufferUART, "070") == 0) {
											TIM5->CCR1 = 4089;  // �ngulo 70�
									} else if (strcmp(bufferUART, "072") == 0) {
											TIM5->CCR1 = 4178;  // �ngulo 72.5�
									} else if (strcmp(bufferUART, "075") == 0) {
											TIM5->CCR1 = 4267;  // �ngulo 75�
									} else if (strcmp(bufferUART, "077") == 0) {
											TIM5->CCR1 = 4356;  // �ngulo 77.5�
									} else if (strcmp(bufferUART, "080") == 0) {
											TIM5->CCR1 = 4444;  // �ngulo 80�
									} else if (strcmp(bufferUART, "082") == 0) {
											TIM5->CCR1 = 4533;  // �ngulo 82.5�
									} else if (strcmp(bufferUART, "085") == 0) {
											TIM5->CCR1 = 4622;  // �ngulo 85�
									} else if (strcmp(bufferUART, "087") == 0) {
											TIM5->CCR1 = 4711;  // �ngulo 87.5�
									} else if (strcmp(bufferUART, "090") == 0) {
											TIM5->CCR1 = 4800;  // �ngulo 90�
									} else if (strcmp(bufferUART, "092") == 0) {
											TIM5->CCR1 = 4889;  // �ngulo 92.5�
									} else if (strcmp(bufferUART, "095") == 0) {
											TIM5->CCR1 = 4978;  // �ngulo 95�
									} else if (strcmp(bufferUART, "097") == 0) {
											TIM5->CCR1 = 5067;  // �ngulo 97.5�
									} else if (strcmp(bufferUART, "100") == 0) {
											TIM5->CCR1 = 5156;  // �ngulo 100�
									} else if (strcmp(bufferUART, "102") == 0) {
											TIM5->CCR1 = 5244;  // �ngulo 102.5�
									} else if (strcmp(bufferUART, "105") == 0) {
											TIM5->CCR1 = 5333;  // �ngulo 105�
									} else if (strcmp(bufferUART, "107") == 0) {
											TIM5->CCR1 = 5422;  // �ngulo 107.5�
									} else if (strcmp(bufferUART, "110") == 0) {
											TIM5->CCR1 = 5511;  // �ngulo 110�
									} else if (strcmp(bufferUART, "112") == 0) {
											TIM5->CCR1 = 5600;  // �ngulo 112.5�
									} else if (strcmp(bufferUART, "115") == 0) {
											TIM5->CCR1 = 5689;  // �ngulo 115�
									} else if (strcmp(bufferUART, "117") == 0) {
											TIM5->CCR1 = 5778;  // �ngulo 117.5�
									} else if (strcmp(bufferUART, "120") == 0) {
											TIM5->CCR1 = 5867;  // �ngulo 120�
									} else if (strcmp(bufferUART, "122") == 0) {
											TIM5->CCR1 = 5956;  // �ngulo 122.5�
									} else if (strcmp(bufferUART, "125") == 0) {
											TIM5->CCR1 = 6044;  // �ngulo 125�
									} else if (strcmp(bufferUART, "127") == 0) {
											TIM5->CCR1 = 6133;  // �ngulo 127.5�
									} else if (strcmp(bufferUART, "130") == 0) {
											TIM5->CCR1 = 6222;  // �ngulo 130�
									} else if (strcmp(bufferUART, "132") == 0) {
											TIM5->CCR1 = 6311;  // �ngulo 132.5�
									} else if (strcmp(bufferUART, "135") == 0) {
											TIM5->CCR1 = 6400;  // �ngulo 135�
									} else if (strcmp(bufferUART, "137") == 0) {
											TIM5->CCR1 = 6489;  // �ngulo 137.5�
									} else if (strcmp(bufferUART, "140") == 0) {
											TIM5->CCR1 = 6578;  // �ngulo 140�
									} else if (strcmp(bufferUART, "142") == 0) {
											TIM5->CCR1 = 6667;  // �ngulo 142.5�
									} else if (strcmp(bufferUART, "145") == 0) {
											TIM5->CCR1 = 6756;  // �ngulo 145�
									} else if (strcmp(bufferUART, "147") == 0) {
											TIM5->CCR1 = 6844;  // �ngulo 147.5�
									} else if (strcmp(bufferUART, "150") == 0) {
											TIM5->CCR1 = 6933;  // �ngulo 150�
									} else if (strcmp(bufferUART, "152") == 0) {
											TIM5->CCR1 = 7022;  // �ngulo 152.5�
									} else if (strcmp(bufferUART, "155") == 0) {
											TIM5->CCR1 = 7111;  // �ngulo 155�
									} else if (strcmp(bufferUART, "157") == 0) {
											TIM5->CCR1 = 7200;  // �ngulo 157.5�
									} else if (strcmp(bufferUART, "160") == 0) {
											TIM5->CCR1 = 7289;  // �ngulo 160�
									} else if (strcmp(bufferUART, "162") == 0) {
											TIM5->CCR1 = 7378;  // �ngulo 162.5�
									} else if (strcmp(bufferUART, "165") == 0) {
											TIM5->CCR1 = 7467;  // �ngulo 165�
									} else if (strcmp(bufferUART, "167") == 0) {
											TIM5->CCR1 = 7556;  // �ngulo 167.5�
									} else if (strcmp(bufferUART, "170") == 0) {
											TIM5->CCR1 = 7644;  // �ngulo 170�
									} else if (strcmp(bufferUART, "172") == 0) {
											TIM5->CCR1 = 7733;  // �ngulo 172.5�
									} else if (strcmp(bufferUART, "175") == 0) {
											TIM5->CCR1 = 7822;  // �ngulo 175�
									} else if (strcmp(bufferUART, "177") == 0) {
											TIM5->CCR1 = 7911;  // �ngulo 177.5�
									} else if (strcmp(bufferUART, "180") == 0) {
											TIM5->CCR1 = 8000;  // �ngulo 180�
									}



                // Reiniciar el �ndice del buffer para la pr�xima recepci�n
                bufferIndex = 0;
            }

        }
    }
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

int main() {
    // Configuraci�n de GPIOs, UART, PWM (como antes)...
    // GPIOs fuera del while
    RCC->AHB1ENR |= (1 << 1);

    GPIOB->MODER &= ~((0b11 << 0) | (0b11 << 14));
    GPIOB->MODER |= ((1 << 0) | (1 << 14));

    GPIOB->OTYPER &= ~((1 << 0) | (1 << 7));
    GPIOB->OSPEEDR |= (((1 << 1) | (1 << 0) | (1 << 15) | (1 << 14)));
    GPIOB->PUPDR &= ~((0b11 << 0) | (0b11 << 14));

    // Systick
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);
	
		configuracionUART3comunicacionPc();
    configuracionPWM();
    

    // Bucle principal
    while (1) {
        GPIOB->ODR |= (1 << 0);  // Enciende el LED en PB0
        SysTick_ms(100);   // Esperar 1 segundo
        GPIOB->ODR &= ~(1 << 0);  // Apaga el LED en PB0
        SysTick_ms(100);   // Esperar 1 segundo
				 // Ahora puedes usar el valor entero recibido
        
    }
}

