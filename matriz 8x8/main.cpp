//Ejemplo Hola mundo con led de usuario de la tarjeta
//Fabián Barrera Prieto
//Universidad ECCI
//STM32F767ZIT6U
//operation 'or' (|) for set bit and operation 'and' (&) for clear bit

#include <stdio.h>
#include "stm32f7xx.h"
#define MICROSECONDS_PER_SECOND 1000000

//int desplazamiento[8] ={0x0001, 0x0002, 0x0004, 0x0008, 0x0040, 0x0080, 0x00100, 0x00200};// derecha a izquierda
int desplazamiento[8] ={0x200, 0x100, 0x80, 0x40, 0x08, 0x04, 0x02, 0x0001};//izquierda a derecha
// Desplazamiento para la segunda matriz
// Desplazamiento para la segunda matriz
int desplazamiento2[8] = { 0b10000000, // Desplazamiento para la columna 7
													 0b01000000, // Desplazamiento para la columna 6
                           0b00100000, // Desplazamiento para la columna 5
                           0b00010000, // Desplazamiento para la columna 4
                           0b00001000, // Desplazamiento para la columna 3
                           0b00000100, // Desplazamiento para la columna 2
                           0b00000010, // Desplazamiento para la columna 1
                           0b00000001  // Desplazamiento para la columna 0
};

int A[] =     {0b111111111,
							 0b000000000, 
							 0b111100110, 
							 0b111100110, 	
               0b111100110, 
				       0b111100110, 
							 0b000000000, 
						   0b111111111,
};
int A2[] =    {0b11111111,
							 0b00000000, 
							 0b11100110, 
							 0b11100110, 	
               0b11100110, 
				       0b11100110, 
							 0b00000000, 
						   0b11111111,
};
int B[] =     {0b100011001,
							 0b000000000, 
							 0b011100110, 
							 0b011100110, 	
               0b011100110, 
				       0b011100110, 
							 0b000000000, 
						   0b111111111
};
int B2[] =    {0b10011001,
							 0b00000000, 
							 0b01100110, 
							 0b01100110, 	
               0b01100110, 
				       0b01100110, 
							 0b00000000, 
						   0b11111111
};
int C[] =     {0b111111111,
							 0b011111110, 
							 0b011111110, 
							 0b011111110, 	
               0b011111110, 
				       0b011111110, 
							 0b000000000, 
						   0b111111111
};
int C2[] =     {0b11111111,
							 0b01111110, 
							 0b01111110, 
							 0b01111110, 	
               0b01111110, 
				       0b01111110, 
							 0b00000000, 
						   0b11111111
};
int D[] =     {0b100000001,
							 0b000000000, 
							 0b001111100, 
							 0b011111110, 	
               0b011111110, 
				       0b011111110, 
							 0b000000000, 
						   0b111111111
};
int D2[] =    {0b10000001,
							 0b00000000, 
							 0b00111100, 
							 0b01111110, 	
               0b01111110, 
				       0b01111110, 
							 0b00000000, 
						   0b11111111
};
int E[] =     {0b111111111,
							 0b011100110, 
							 0b011100110, 
							 0b011100110, 	
               0b011100110, 
				       0b011100110, 
							 0b000000000, 
						   0b11111111
};
int E2[] =    {0b11111111,
							 0b01100110, 
							 0b01100110, 
							 0b01100110, 	
               0b01100110, 
				       0b01100110, 
							 0b00000000, 
						   0b11111111
};
int F[] =     {0b111111111,
							 0b111110110, 
							 0b111110110, 
							 0b111110110, 	
               0b111110110, 
				       0b111110110, 
							 0b000000000, 
						   0b111111111
};
int F2[] =    {0b11111111,
							 0b11110110, 
							 0b11110110, 
							 0b11110110, 	
               0b11110110, 
				       0b11110110, 
							 0b00000000, 
						   0b11111111
};
int G[] =     {0b111111111,
							 0b000001110, 
							 0b011101110, 
							 0b011101110, 	
               0b011111110, 
				       0b011111110, 
							 0b000000000, 
						   0b111111111
};
int H[] =     {0b111111111,
							 0b000000000, 
							 0b111100111, 
							 0b111100111, 	
               0b111100111, 
				       0b111100111, 
							 0b000000000, 
						   0b111111111
};
int I[] =     {0b111111111,
							 0b011111110, 
							 0b011111110, 
							 0b000000000, 	
               0b000000000, 
				       0b011111110, 
							 0b011111110, 
						   0b111111111
};
int I2[] =    {0b11111111,
							 0b01111110, 
							 0b01111110, 
							 0b00000000, 	
               0b00000000, 
				       0b01111110, 
							 0b01111110, 
						   0b11111111
};
int J[] =     {0b111111111,
							 0b111111110, 
							 0b111111110, 
							 0b000000000, 	
               0b011111110, 
				       0b000001110, 
							 0b111111110, 
						   0b111111111
};
int K[] =     {0b111111111,
							 0b011111110, 
							 0b101111101, 
							 0b110111011, 	
               0b111010111, 
				       0b111101111, 
							 0b000000000, 
						   0b111111111
};
int L[] =     {0b111111111,
							 0b111111111, 
							 0b011111111, 
							 0b011111111, 	
               0b011111111, 
				       0b011111111, 
							 0b000000000, 
						   0b111111111
};
int M[] =     {0b111111111,
							 0b111111111, 
							 0b000000000, 
							 0b111111101, 	
               0b111111011, 
				       0b111111101, 
							 0b000000000, 
						   0b111111111
};
int M2[] =    {0b11111111,
							 0b11111111, 
							 0b00000000, 
							 0b11111101, 	
               0b11111011, 
				       0b11111101, 
							 0b00000000, 
						   0b11111111
};
int N[] =     {0b111111111,
							 0b000000000, 
							 0b100111111, 
							 0b111001111, 	
               0b111000111, 
				       0b111111001, 
							 0b000000000, 
						   0b111111111
};
int O[] =     {0b111111111,
							 0b000000000, 
							 0b011111110, 
							 0b011111110, 	
               0b011111110, 
				       0b011111110, 
							 0b000000000, 
						   0b111111111
};
int O2[] =     {0b11111111,
							 0b00000000, 
							 0b01111110, 
							 0b01111110, 	
               0b01111110, 
				       0b01111110, 
							 0b00000000, 
						   0b11111111
};
int P[] =     {0b111111111,
							 0b111111111, 
							 0b111110000, 
							 0b111110110, 	
               0b111110110, 
				       0b111110110, 
							 0b000000000, 
						   0b111111111
};
int Q[] =     {0b111111111,
							 0b101111111, 
							 0b110000000, 
							 0b110111110, 	
               0b110101110, 
				       0b110111110, 
							 0b110000000, 
						   0b111111111
};
int R[] =     {0b11111111,
							 0b11111111, 
							 0b001110000, 
							 0b110110110, 	
               0b111000110, 
				       0b111100110, 
							 0b000000000, 
						   0b111111111
};
int S[] =     {0b111111111,
							 0b000000110, 
							 0b011110110, 
							 0b011110110, 	
               0b011110110, 
				       0b011110110, 
							 0b011110000, 
						   0b111111111
};
int T[] =     {0b111111110,
							 0b111111110, 
							 0b111111110, 
							 0b000000000, 	
               0b000000000, 
				       0b111111110, 
							 0b111111110, 
						   0b111111110
};
int U[] =     {0b111111111,
							 0b000000000, 
							 0b011111111, 
							 0b011111111, 	
               0b011111111, 
				       0b011111111, 
							 0b000000000, 
						   0b111111111
};
int W[] =     {0b111111111,
							 0b000000000, 
							 0b011111111, 
							 0b000000000, 	
               0b000000000, 
				       0b011111111, 
							 0b000000000, 
						   0b111111111
};
int X[] =     {0b111111111,
							 0b011111110, 
							 0b001111100, 
							 0b110010011, 	
               0b111000111, 
				       0b110010011, 
							 0b001111100, 
						   0b011111110
};

int ruta[100]={ //a
							 0b111111111,
							 0b000000000, 
							 0b111100110, 
							 0b111100110, 	
               0b111100110, 
				       0b111100110, 
							 0b000000000, 
						   0b111111111,
								//d
							 0b100000001,
							 0b000000000, 
							 0b001111100, 
							 0b011111110, 	
               0b011111110, 
				       0b011111110, 
							 0b000000000, 
								//a
							 0b111111111,
							 0b000000000, 
							 0b111100110, 
							 0b111100110, 	
               0b111100110, 
				       0b111100110, 
							 0b000000000, 
								//r
							 0b111111111, 
							 0b001110000, 
							 0b110110110, 	
               0b111000110, 
				       0b111100110, 
							 0b000000000, 
						   0b111111111,
							 
							 
							 //a
							 0b000000000, 
							 0b111100110, 
							 0b111100110, 	
               0b111100110, 
				       0b111100110, 
							 0b000000000, 
						   0b111111111,
				       //p 
							 0b111110000, 
							 0b111110110, 	
               0b111110110, 
				       0b111110110, 
							 0b000000000, 
						   0b111111111,
							 0b111111111,
							 0b111111111,
							 //a
							 0b111111111,
							 0b111111111,
							 0b111111111,
							 0b000000000, 
							 0b111100110, 
							 0b111100110, 	
               0b111100110, 
				       0b111100110, 
							 0b000000000, 
						   0b111111111,
							 
							 //m
							 0b111111111, 
							 0b000000000, 
							 0b111111101, 	
               0b111111011, 
				       0b111111101, 
							 0b000000000,
							 0b111111111,
							 //i
							 0b111111111,
							 0b011111110, 
							 0b011111110, 
							 0b000000000, 	
               0b000000000, 
				       0b011111110, 
							 0b011111110,
							 //x
							 0b111111111,
							 0b011111110, 
							 0b001111100, 
							 0b110010011, 	
               0b111000111, 
				       0b110010011, 
							 0b001111100, 
						   0b011111110,
							 0b111111111,
							 //o
							 0b111111111,
							 0b000000000, 
							 0b011111110, 
							 0b011111110, 	
               0b011111110, 
				       0b011111110, 
							 0b000000000, 
							 0b111111111,//espacio
							 //r
							 0b001110000, 
							 0b110110110, 	
               0b111000110, 
				       0b111100110, 
							 0b000000000, 
						   0b111111111,
							 //p							
							 0b111110000, 
							 0b111110110, 	
               0b111110110, 
				       0b111110110, 
							 0b000000000, 
						   0b111111111
};




//void Delay (uint32_t time)
//{
//	//while (time--);  
//	for (int t=0;t<time;t++);

//}
void SysTick_Wait(uint32_t n){
	SysTick->LOAD = n - 1; //15999
	SysTick->VAL = 0; //Clean the value of Systick counter
	while ((SysTick->CTRL & 0x00010000) == 0); //Check the count flag until it's 1 
}

void SysTick_ms(uint32_t x){
	for (uint32_t i = 0; i < x; i++){//x ms
		SysTick_Wait(16000); //1ms
	}
}

void letra(void);

int main(){

	RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3) |(1<<4) |(1<<5)|(1<<6); //Enable the GPIOB clock and GPIOC and GPIOD (user led LD1 is connected to PB0)
	
	
	GPIOA->MODER &= ~(0Xffff<<0); 
	GPIOA->MODER |= (1<<0) |(1<<2)|(1<<4)|(1<<6)|(1<<8)|(1<<10)|(1<<12)|(1<<14);
	GPIOB->MODER &= ~(0xff<<0); //clear (00) pin PB0(bits 1:0) and set as Input (00) for default 
	GPIOB->MODER |= (1<<0) |(1<<2)|(1<<4)|(1<<6)|(1<<8)|(1<<10)|(1<<12)|(1<<14); //pin PB0(bits 1:0) as Output (01)
	
	GPIOC->MODER &= ~(0xffff<<0); //clear (00) pin PD0(bits 1:0) and set as Input (00) for default 
	GPIOC->MODER |= (1<<0) |(1<<2) |(1<<4)|(1<<6)|(1<<8)|(1<<10)|(1<<12)|(1<<14) |(1<<16) |(1<<18) |(1<<20)|(1<<22)|(1<<24)|(1<<26)|(1<<28)|(1<<30);
	
	GPIOD->MODER &= ~(0xff<<0); //clear (00) pin PB0(bits 1:0) and set as Input (00) for default 
	GPIOD->MODER |= (1<<0) |(1<<2)|(1<<4)|(1<<6)|(1<<8)|(1<<10)|(1<<12)|(1<<14)|(1<<16)|(1<<18);
	
	GPIOE->MODER &= ~(0xff<<0); //clear (00) pin PD0(bits 1:0) and set as Input (00) for default 
	GPIOE->MODER |= (1<<0) |(1<<2) |(1<<4)|(1<<6)|(1<<8)|(1<<10)|(1<<12)|(1<<14) |(1<<16) |(1<<18);
	
	GPIOF->MODER &= ~(0xffff<<0); //clear (00) pin PB0(bits 1:0) and set as Input (00) for default 
	GPIOF->MODER |= (1<<0) |(1<<2)|(1<<4)|(1<<6)|(1<<8)|(1<<10)|(1<<12)|(1<<14)|(1<<16)|(1<<18);
	
	GPIOG->MODER &= ~(0xff<<0); //clear (00) pin PB0(bits 1:0) and set as Input (00) for default 
	GPIOG->MODER |= (1<<0) |(1<<2)|(1<<4)|(1<<6)|(1<<8)|(1<<10)|(1<<12)|(1<<14)|(1<<16)|(1<<18);
	
	GPIOA->OTYPER	&= ~(1<<0) &~(1<<1) & ~(1<<2) & ~(1<<3) & ~(1<<4) & ~(1<<5) & ~(1<<6) & ~(1<<7) &~(1<<8) & ~(1<<9) & ~(1<<10) & ~(1<<11) & ~(1<<12) & ~(1<<13) & ~(1<<14) &~(1<<15) ;
	GPIOB->OTYPER &= ~(1<<0) &~(1<<1) &~(1<<2) &~(1<<3) &~(1<<4) &~(1<<5) &~(1<<6 )&~(1<<7) &~(1<<8);  // clear (0) pin PB0 (bit 0) --> Output push pull (HIGH or LOW)
	GPIOC->OTYPER	&= ~(1<<0) &~(1<<1) & ~(1<<2) & ~(1<<3) & ~(1<<4) & ~(1<<5) & ~(1<<6) & ~(1<<7) &~(1<<8) & ~(1<<9) & ~(1<<10) & ~(1<<11) & ~(1<<12) & ~(1<<13) & ~(1<<14) &~(1<<15) ;
	GPIOD->OTYPER	&= ~(1<<0) &~(1<<1) & ~(1<<2) & ~(1<<3) & ~(1<<4) & ~(1<<5) & ~(1<<6) & ~(1<<7) &~(1<<8) & ~(1<<9) & ~(1<<10) & ~(1<<11) & ~(1<<12) & ~(1<<13) & ~(1<<14) &~(1<<15) ;
	GPIOE->OTYPER	&= ~(1<<0) &~(1<<1) & ~(1<<2) & ~(1<<3) & ~(1<<4) & ~(1<<5) & ~(1<<6) & ~(1<<7) &~(1<<8) & ~(1<<9) & ~(1<<10) & ~(1<<11) & ~(1<<12) & ~(1<<13) & ~(1<<14) &~(1<<15) ;
	GPIOF->OTYPER &= ~(1<<0) &~(1<<1) &~(1<<2)&~(1<<3)&~(1<<4)&~(1<<5)&~(1<<6)&~(1<<7)&~(1<<8)&~(1<<9);
	GPIOG->OTYPER &= ~(1<<0) &~(1<<1) &~(1<<2)&~(1<<3)&~(1<<4)&~(1<<5)&~(1<<6)&~(1<<7)&~(1<<8)&~(1<<9);
	
	GPIOA->OSPEEDR |= (0xffff<<0);
	GPIOB->OSPEEDR |= (0xffff<<0);//(0b11<<0)  // Pin PB0 (bits 1:0) as Very High Speed (11)
	//GPIOB->OSPEEDR |= ((1<<15)|(1<<14));	// Pin PB7 (bits 15:14) as Very High Speed (11)
	GPIOC->OSPEEDR |= (0xffff<<0);
	GPIOD->OSPEEDR |= (0Xff<<0);
	GPIOE->OSPEEDR |= (0Xfff<<0);
	GPIOF->OSPEEDR |= (0Xfff<<0);
	GPIOG->OSPEEDR |= (0Xfff<<0);
	
	GPIOA->PUPDR &= ~(0xff<<0);
	GPIOB->PUPDR &= ~(0b11<<0); //~((1<<1)|(1<<0)) // Pin PB0 (bits 1:0) are 0:0 --> no pull up or pull down
	//GPIOC->PUPDR &= ~(0b11<<26); //~((1<<27)|(1<<26)) // Pin PC13 (bits 27:26) are 0:0 --> no pull up or pull down
	//GPIOC->PUPDR |= (1<<27); // Pin PC_13 (bits 27:26) are 1:0 --> pull down for default
	GPIOC->PUPDR &= ~(0xffff<<0); //~((1<<27)|(1<<26)) // Pin PC13 (bits 27:26) are 0:0 --> no pull up or pull down
	GPIOD->PUPDR &= ~(0xff<<0);
	GPIOE->PUPDR &= ~(0xff<<0);
	GPIOF->PUPDR &= ~(0xfff<<0);
	GPIOG->PUPDR &= ~(0xfff<<0);
	
	//Systick
	SysTick->LOAD = 0x00FFFFFF; //Initializing with the maximum value to 24 bits
	SysTick->CTRL |= (0b101); //Clock source is processor clock (AHB) and counter enable
	
	
	while(1){
				// "puerto: pin Display" mtriz 1
				//pc0=13, PC1=3 ,PC2=4 ,PC3=10,PC6=6 ,PC7=11 ,PC8=15 ,PC9=16 ;
				//PF0=9, PF1=14 , PF2=8 , PF3=12 , PF4=1 , PF6=7 , PF7= 2, PF8=5
			//	for(int i=100; i>=0; i--){         //Recorrido de tabla
     // for(int k=0;k<25;k++){  	
//				for (int columna = 0; columna<= 8; columna++) {	
//					
//					//matriz 1
//						GPIOF->ODR = P[columna];					
//						GPIOC->ODR = desplazamiento[columna];
//					//SysTick_ms(1);
//					
//					// matriz 2
//						GPIOG->ODR = R[columna];					
//						GPIOD->ODR = desplazamiento2[columna];
//					SysTick_ms(1);
//					
//					// matriz 3
//						GPIOA->ODR = O2[columna];					
//						GPIOE->ODR = desplazamiento2[columna];	
//					  SysTick_ms(1);
 int total_desplazamiento=24;
			for (int i = 0; i < total_desplazamiento; i++) {  // 'total_desplazamiento' es el número total de columnas a desplazar.
    for (int columna = 0; columna < 8; columna++) {  // Itera sobre cada columna.
        // Matriz 1
        if (i < 8) {
            // Inicialmente despliega sólo la primera matriz, el resto están apagadas.
            GPIOF->ODR = P[columna];  // Encender filas de la primera matriz.
            GPIOC->ODR = desplazamiento[columna];  // Encender columnas de la primera matriz.
        } else if (i < 16) {
            // Desplaza desde la matriz 2 a la matriz 1.
            GPIOF->ODR = R[columna];  // Encender filas de la segunda matriz.
            GPIOC->ODR = desplazamiento2[columna] << (i - 8);  // Desplazar el contenido desde la segunda matriz.
        } else {
            // Desplaza desde la matriz 3 a la matriz 2.
            GPIOF->ODR = O2[columna];  // Encender filas de la tercera matriz.
            GPIOC->ODR = desplazamiento2[columna] << (i - 16);  // Desplazar el contenido desde la tercera matriz.
        }

        // Matriz 2
        if (i >= 8 && i < 16) {
            // Mostrar lo que está en la matriz 2.
            GPIOG->ODR = R[columna];
            GPIOD->ODR = desplazamiento2[columna] >> (i - 8);
        } else if (i >= 16 && i < 24) {
            // Desplazar desde la matriz 3 a la matriz 2.
            GPIOG->ODR = O2[columna];
            GPIOD->ODR = desplazamiento2[columna] << (i - 16);
        }

        // Matriz 3
        if (i >= 16 && i < 24) {
            // Mostrar lo que está en la matriz 3.
            GPIOA->ODR = O2[columna];
            GPIOE->ODR = desplazamiento2[columna] >> (i - 16);
        }

        SysTick_ms(5);  // Esperar un poco entre cada paso.
    }
}
					
				
				
			
	  
  }
}
//}
	
	//void letra(void){
	//	while(1){
			

   
//		for(int i=99; i>=0; i--){         //Recorrido de tabla
//			
//      for(int k=0;k<20;k++){  	
//				for (int matriz = 2; matriz >= 0; matriz--) { // Recorrido de matrices en orden inverso
//				for (int columna = 0; columna<= 8; columna++) {	
//            switch (matriz) {
//                case 2:
//                    GPIOA->ODR = ruta[columna+i];
//                    GPIOE->ODR = desplazamiento2[columna];
//								
//                    break;
//                case 1:
//                    GPIOG->ODR = ruta[columna+i];
//                    GPIOD->ODR = desplazamiento2[columna];
//											
//								break;
//                case 0:
//                    GPIOF->ODR = ruta[columna+i];
//                    GPIOC->ODR = desplazamiento[columna];
//                   
//								break;
//            }
//						Delay(100);
//            //SysTick_ms(2);
//        }
//    }
//}
			///////////////////////////////	 
		
					
	
		
	//}

