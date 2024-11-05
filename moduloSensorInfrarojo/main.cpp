#include <stdio.h>
#include <stm32f7xx.h>
#include <string.h>

int alertaInfrarojoGui = 0;  // bandera para modificar estado de detección en la GUI
char estado_deteccion[12] = "INFR: no";
uint8_t flag_deteccion = 0;

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
    EXTI->RTSR &= ~(1 << 0);       // Deshabilitar borde de subida

    // Habilitar interrupción en el NVIC para EXTI0
    NVIC_EnableIRQ(EXTI0_IRQn);
}

extern "C" {
    // Interrupción externa para sensor infrarrojo PD0
    void EXTI0_IRQHandler(void) {
        if (EXTI->PR & (1 << 0)) {  // Verifica si la interrupción fue generada por PD0
            EXTI->PR |= (1 << 0);   // Limpia la interrupción para PD0

            // Cambia el estado de detección según el pin PD0
            if ((GPIOD->IDR & (1 << 0)) == 0) {  // Si PD0 está en bajo (objeto detectado)
                GPIOB->ODR |= (1 << 0);          // Enciende el LED en PB0
                strcpy(estado_deteccion, "INFR:si");
                alertaInfrarojoGui = 1;
            } else {  // PD0 está en alto (sin objeto)
                GPIOB->ODR &= ~(1 << 0);         // Apaga el LED en PB0
                strcpy(estado_deteccion, "INFR:no");
                alertaInfrarojoGui = 0;
            }
            flag_deteccion = 1;  // Activar bandera para actualizar la LCD
        }
    }
}

int main() {
    configurarSensorPD0();

    while (1) {
        // Aquí puedes añadir lógica adicional si es necesario
    }
}
