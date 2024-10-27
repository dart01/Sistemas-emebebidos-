#include <stdio.h>
#include <stm32f7xx.h>
#include <string.h>


uint8_t flag_cronometro = 0, flag_deteccion = 0, cont = 0, cronometro_iniciado = 0;
char text[16];
char estado_deteccion[12] = "INFR: no";
unsigned char d;       // Variable para datos recibidos por UART
int segundos = 0;
int minutos = 0;  // Variable para contar los minutos
uint16_t digital;
float voltaje;

char text2[8];
char estado_plataforma[6] = "stop";
int movimiento=0;
int alertaInfrarojoGui=0;//bandera para modificar esatdo de deteccion en la GUI
int alertCNYGui=0;//bandera para modificar esatdo de deteccion en la GUI
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

void actualizarMensaje2LCD(char* mensaje) {
		setCursorLCD(2, 1);  // Fila 2, Columna 1
    //settingsLCD(0xC0);  // Posicionar el cursor en la segunda línea
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en la LCD
    }
}



// Ahora para escribir el valor en la columna 8, fila 1:
void actualizarMensajeLCDEnColumna(char* mensaje) {
    setCursorLCD(1, 9);  // Fila 1, Columna 8
    for (int i = 0; mensaje[i] != '\0'; i++) {
        writeLCD(mensaje[i]);  // Escribir el mensaje en el LCD
        SysTick_ms(2);  // Agregar un retraso pequeño para asegurar que el LCD procese
    }
}

void actualizarMensajeLCDmovimiento(char* mensaje) {	
    setCursorLCD(2, 9);  // Fila 2, Columna 8
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
    // Motor 1 (PD0, PD1) - Motor derecho para elante
    // Motor 1 (PD0, PD1)
    GPIOD->ODR |= (1 << 0);  // PD0 = 1
    GPIOD->ODR &= ~(1 << 1); // PD1 = 0
    
    // Motor 2 (PD2, PD3) - Motor izquiero quieto 
    // Detener Motor 2 (PD2, PD3)
    GPIOD->ODR &= ~((1 << 2) | (1 << 3)); // PD2 = 0, PD3 = 0
}

// Girar a la derecha (Motor izquierdo adelante, motor derecho atrás)
void girarDerecha() {
    // Motor 1 (PD0, PD1) - Motor derecho quieto
    // Detener Motor 1 (PD0, PD1)
    GPIOD->ODR &= ~((1 << 0) | (1 << 1)); // PD0 = 0, PD1 = 0
    // Motor 2 (PD2, PD3) - Motor izquierdo adelante
    GPIOD->ODR |= (1 << 2);  // PD2 = 1
    GPIOD->ODR &= ~(1 << 3); // PD3 = 0
}

// Detener ambos motores
void detenerMotor() {
    // Detener Motor 1 (PD0, PD1)
    GPIOD->ODR &= ~((1 << 0) | (1 << 1)); // PD0 = 0, PD1 = 0
    // Detener Motor 2 (PD2, PD3)
    GPIOD->ODR &= ~((1 << 2) | (1 << 3)); // PD2 = 0, PD3 = 0
}


// Manejo de la interrupción externa EXTI2 (PD2) para sensor infrarojo
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

void configuracionTimerCronometro(){
		// Configuración del Timer3 (para contar 1 segundo)
    RCC->APB1ENR |= (1<<1); // Habilitar el reloj de TIM3 
    TIM3->PSC = 16000 - 1;  // Prescaler para contar en ms (16 MHz / 16,000 = 1 kHz)
    TIM3->ARR = 100 - 1;    // 100 ms por interrupción
    TIM3->DIER |= (1<<0);   // Habilitar interrupciones de actualización
    TIM3->CR1 |= (1<<0);    // Iniciar el temporizador
    NVIC_EnableIRQ(TIM3_IRQn);  // Habilitar IRQ para TIM3
}

void configuracionADCcny70(){
		//ADC
    GPIOC->MODER |= (0b11<<0); // PC0 (ADC123_IN10) modo analógico		
    RCC->APB2ENR |= (1<<9); // Activar reloj del ADC2
    ADC2->CR2 |= ((1<<10)|(1<<0)); // EOC y habilitar ADC
    ADC2->CR1 &= ~(0b11<<24); // Limpiar bits de resolución
    ADC2->CR1 |= (1<<24); // Resolución de 10 bits
    ADC2->SMPR1 |= (1<<0); // 15 ciclos ADCCLK en canal 10 (PC0)
    ADC2->SQR3 &= ~(0b11111<<0); // Limpiar secuencia regular
    ADC2->SQR3 |= (0b1010<<0); // Canal 10 en primera conversión

}

void conversionADC(){
		// configuracion led usuario PB7
		GPIOB->MODER &= ~(0b11<<14);// configuracion led usuario PB7
    GPIOB->MODER |= (1<<14); 
		GPIOB->OSPEEDR |= ((1<<15)|(1<<14));
		// Iniciar conversión ADC y esperar a que termine
    ADC2->CR2 |= (1<<30); // Iniciar conversión A/D  
    while(((ADC2->SR & (1<<1)) >> 1) == 0){} // Esperar final conversión  
    ADC2->SR &= ~(1<<1); // Limpiar bit EOC  
    digital = ADC2->DR;  
    voltaje = (float)digital * (3.3 / 1023.0); // Calcular voltaje  

		if(voltaje>=2){
			detenerMotor();
			GPIOB->ODR |= (1 << 7);  // Encender LED en PB7
			// Mostrar el valor en la columna 8 de la fila 1 del LCD
			actualizarMensajeLCDEnColumna("CNY:si");// Mostrar el valor en la pantalla LCD en columna 8
			alertCNYGui=1;//bandera para modificar esatdo de deteccion en la GUI
		}else{
			GPIOB->ODR &= ~(1 << 7);  // Apagar el LED en PB7
			actualizarMensajeLCDEnColumna("CNY:no");// Mostrar el valor en la pantalla LCD en columna 8
			alertCNYGui=0;//bandera para modificar esatdo de deteccion en la GUI
		}

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

extern "C" {
	
	//imterrupcion externa  para sensor infrarojo PA0
	void EXTI0_IRQHandler(void) {
      if (EXTI->PR & (1 << 0)) { // Verifica si la interrupción fue generada por PA0
        EXTI->PR |= (1 << 0);  // Limpia la interrupción para PA0

        // Cambia el estado de detección según el pin PA0
        if ((GPIOA->IDR & (1 << 0)) == 0) { // Si se detecta un objeto (PA0 en bajo)
           
            GPIOB->ODR |= (1 << 0);  // Enciende el LED en PB0
						detenerMotor();
						// Actualiza la pantalla LCD con "INFR: si"
						strcpy(estado_deteccion, "INFR:si");
						alertaInfrarojoGui=1;//bandera para modificar esatdo de deteccion en la GUI
					
										
					
				} else {  // Si no hay detección (PA0 en alto)
           
            GPIOB->ODR &= ~(1 << 0);  // Apaga el LED en PB0
						// Actualiza la pantalla LCD con "s1: no"
						strcpy(estado_deteccion, "INFR:no");
						alertaInfrarojoGui=0;//bandera para modificar esatdo de deteccion en la GUI
					
              }
				}
				flag_deteccion = 1;  // Activar la bandera para actualizar la LCD
    }
  
	
	//interrupucion de comunicacion USART3 
	void USART3_IRQHandler(void){ //Receive interrupt
    if (USART3->ISR & USART_ISR_RXNE) { // Si se ha recibido un dato
            d = USART3->RDR;  // Leer el dato recibido   
//						 // Evaluar el dato recibido y mover el motor en consecuencia
            switch(d) {
                case 'W':
										movimiento=1;
                    moverMotoresAdelante();
                    break;
                case 'S':
										movimiento=2;										
                    moverMotoresAtras();
                    break;
                case 'A':
										//actualizarMensajeLCDmovimiento("Left");
										girarIzquierda();
										movimiento=3;
                    break;
                case 'D':
										//actualizarMensajeLCDmovimiento("Right");
										girarDerecha();
										movimiento=4;                   
                    break;
								case 'C':  // Comando para iniciar cronómetro
										cronometro_iniciado = 1;  // Iniciar cronómetro
										break;
								case 'X':  // Comando para iniciar cronómetro
										 detenerMotor(); 
										 break;
								case 'R':  // Reiniciar temporizador
										TIM3->CNT = 0;  // Reiniciar el contador del temporizador
										TIM3->CR1 &= ~(1 << 0);  // Detener el temporizador
										actualizarMensaje2LCD("00:00");
								break;
							
                default:
                    detenerMotor();  // Detener el motor si no es un comando válido
                      TIM3->CR1 |= (1<<0);  // Asegurarse de que el temporizador está habilitado
										break;
            }
				
				}
    }
		
		//timer
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
		
		//interrupcion de comunicacion usart4 para modulo hc05
		void UART4_IRQHandler(void) {
				if (UART4->ISR & USART_ISR_RXNE) {  // Verificar si hay datos recibidos
						d = UART4->RDR;      // Leer dato recibido
									// Evaluar el dato recibido y mover el motores
							switch(d) {
									case 'W':
											movimiento=1;
											moverMotoresAdelante();
											break;
									case 'S':
											moverMotoresAtras();
											movimiento=2;
											break;
									case 'A':
											movimiento=3;
											girarIzquierda();											
											break;
									case 'D':
											movimiento=4;
											girarDerecha();									
											break;
									case 'C':  // Comando para iniciar cronómetro
											cronometro_iniciado = 1;  // Iniciar cronómetro 
											break;
									default:
											detenerMotor();  // Detener el motor si no es un comando válido											
											TIM3->CR1 |= (1<<0);  // Asegurarse de que el temporizador está habilitado
											break;
						}
				}

		}


	}

int main(){
		settings(); 
	  configurarMotor();//configuracion de pines para los motores
		configuracionUART3comunicacionPc();
		configurarUART4BluetoothHc50();// pc10 uart4 TX --- pc11 uart4 RX
		configurarSensorPA0();// sensor infrarojo configurado en el PA0 como interrupcion externa
    configuracionTimerCronometro();
		configuracionADCcny70();
		// Mostrar inicialmente "CNY: no" en la pantalla
		actualizarMensajeLCD("INFR:no");
		// Mostrar inicialmente "INFR: no" en la pantalla
		actualizarMensajeLCDEnColumna("CNY:no");
		//MOstar incicialmente cronometro en 00:00
		actualizarMensaje2LCD("00:00");
	
		actualizarMensajeLCDmovimiento(estado_plataforma);
		
	while(1){
		conversionADC();
			if (flag_deteccion) {  // Si hubo un cambio en el estado de detección
            flag_deteccion = 0;
            actualizarMensajeLCD(estado_deteccion);  // Actualizar la primera línea con el estado del sensor
        }

        if (flag_cronometro) {  // Si hubo un cambio en el cronómetro
            flag_cronometro = 0;
            sprintf(text, "%02d:%02d", minutos, segundos);  // Formatear el mensaje con minutos y segundos
            // setCursorLCD(2, 1);  // Posicionar el cursor en la columna 1 de la fila 2
						actualizarMensaje2LCD(text);  // Actualizar la segunda línea con el cronómetro
						
				}  
				
				
				// consulta por el estado del movimiento de la plataforma en ambos canales UART
				if(movimiento==1){
					actualizarMensajeLCDmovimiento("Run!!!");
				}else if(movimiento==2){
					actualizarMensajeLCDmovimiento("Back!!");
				}else if(movimiento==3){
					actualizarMensajeLCDmovimiento("Left!!");
				}else if(movimiento==4){
					actualizarMensajeLCDmovimiento("Right!");
				}else{
					actualizarMensajeLCDmovimiento("stop!!");
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
				
						
						
				//pregunta por le estado de la captura del adc con el CNY70 
				if(alertCNYGui==1){
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
   }
 }