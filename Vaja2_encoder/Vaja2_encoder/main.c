/*
 * Vaja2_encoder.c
 *
 * Created: 8. 09. 2025 14:56:30
 * Author : Jan Petric
 */ 
#include "stdio.h"
#include "sam.h"
#include "math.h"

// UART0
#define UART0_RX 9  // UART0 RX line PA9
#define UART0_TX 10 // UART0 TX line PA10
#define BAUDRATE 115200

void delay(uint32_t ms) {
	for (uint32_t k = 0; k < ms; k++) {
		for (volatile uint32_t i = 0; i < 10000; i++) {;}
	}
}

void DefineUART0(void){
	
	// Enabling clock for UART0
	PMC->PMC_PCER0 |= (1 << ID_UART0) | (1 << ID_PIOA);
	
	// Disable PIO control over PA9 & 10
	PIOA->PIO_PDR |= (1 << UART0_RX) | (1 << UART0_TX);
	
	// Configuring PIO controller to enable I/O line operations of UART0
	PIOA->PIO_ABCDSR[0] &= ~((1 << UART0_RX) | (1 << UART0_TX));
	PIOA->PIO_ABCDSR[1] &= ~((1 << UART0_RX) | (1 << UART0_TX));
	
	// Reset UART0 RX & TX
	UART0->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX;
	
	// Disable UART0 RX & TX
	UART0->UART_CR = UART_CR_RXDIS | UART_CR_TXDIS;
	
	// Defining baud rate
	uint32_t baud_rate = (SystemCoreClock / (16 * BAUDRATE));
	UART0->UART_BRGR = baud_rate;
	
	// Defining mode of operation
	UART0->UART_MR = (UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL);
		
	// Enabling UART0 RX & TX
	UART0->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
}

void UART_send_char(char ch) {
	while (!(UART0->UART_SR & UART_SR_TXRDY)); // Wait TX ready
	UART0->UART_THR = ch;
}

void UART_send_string(const char *str){
		while (*str) {
			UART_send_char(*str++);
		}
}

void UART_send_number(int value){
	int len = 0;
	int temp = value;
	int arr[10];
	
	//Get length of value
		
	if(value == 0){
		UART_send_char(48);
	}else{
		while (temp > 0) {
			arr[len] = temp % 10;
			temp /= 10;
			len++;
		}
		
		for(int i = (len - 1);i >= 0; i--){
			UART_send_char(arr[i] + 48);			
		}
	}			
}

void spi_init(void) {
	// Enable clock for SPI peripheral
	PMC->PMC_PCER0 |= (1 << ID_SPI);

	// Configure pins for SPI (Peripheral A)
	PIOA->PIO_PDR |= (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14); // Disable PIO control
	PIOA->PIO_ABSR &= ~((1 << 11) | (1 << 12) | (1 << 13) | (1 << 14)); // Select Peripheral A

	// Reset and configure SPI in Master Mode
	SPI->SPI_CR = SPI_CR_SWRST;      // Reset SPI
	SPI->SPI_CR = SPI_CR_SPIEN;      // Enable SPI

	SPI->SPI_MR = SPI_MR_MSTR         // Master mode
	| SPI_MR_MODFDIS;     // Disable mode fault detection

	// Configure Chip Select 0 (8-bit, CPOL=0, NCPHA=1)
	SPI->SPI_CSR[SPI_NPCS0] = SPI_CSR_BITS_8_BIT    // 8-bit data
	| SPI_CSR_SCBR(42)      // Baud rate divider (Peripheral clock / 42)
	| SPI_CSR_NCPHA;        // Data captured on rising edge

	// SPI is ready to use
}

int main(void)
{
    
    SystemInit();
	DefineUART0();
			
	while (1) 
    {	
		UART_send_string("Encoder position: ");
		UART_send_number(255);
		UART_send_string("\r\n");
		delay(100);
	}
}