/*
 * GccApplication3.c
 *
 * Created: 8. 09. 2025 14:43:31
 * Author : Jan Petric
 */ 

#include "sam.h"

#define LED_PIN     22      // LED PD22
#define SWITCH_PIN  2       // SW0 PA2

void Define_GPIO(void) {

	// Enable peripheral clocks PIOA,PIOD
	PMC->PMC_PCER0 = (1 << ID_PIOA) | (1 << ID_PIOD);

	// Configure LED
	PIOD->PIO_PER = (1 << LED_PIN);  // Enable PIO control
	PIOD->PIO_OER = (1 << LED_PIN);  // Configure as output
	PIOD->PIO_CODR = (1 << LED_PIN); // Initialize LED output

	// Configure switch
	PIOA->PIO_PER = (1 << SWITCH_PIN); // Enable PIO control
	PIOA->PIO_ODR = (1 << SWITCH_PIN); // Configure as input
	PIOA->PIO_PUER = (1 << SWITCH_PIN); // Enable pull-up resistor
}

int main(void) {

	SystemInit();
	Define_GPIO();

	while (1) {
		
		if (PIOA->PIO_PDSR & (1 << SWITCH_PIN)) {
			PIOD->PIO_SODR = (1 << LED_PIN);
			}
		else {
			PIOD->PIO_CODR = (1 << LED_PIN);
		}

	}
}
