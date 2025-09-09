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

// SPI
#define SPI_MISO 12 // SPI master in/slave out	-> PA12 -> EXT1 PIN 17
#define SPI_MOSI 13 // SPI master out/slave in	-> PA13 -> EXT1 PIN 16
#define SPI_SCK 14  // SPI clock				-> PA14 -> EXT1 PIN 18
#define SPI_SS_A 14 // SPI Slave select A		-> PB14 -> EXT1 PIN 15

// Improvised delay
void delay(uint32_t ms) {
	for (uint32_t k = 0; k < ms; k++) {
		for (volatile uint32_t i = 0; i < 10000; i++) {;}
	}
}

// Functions for UART
void Define_UART0(void){
	
	// Enabling clock for UART0
	PMC->PMC_PCER0 |= (1 << ID_UART0) | (1 << ID_PIOA);
	
	// Disable PIO control over PA9 & 10
	PIOA->PIO_PDR |= (1 << UART0_RX) | (1 << UART0_TX);
	
	// Configuring PIO controller to enable I/O line operations for UART0
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

void UART_send_number(uint16_t value){
	int len = 0;
	uint16_t temp = value;
	uint16_t arr[30];
	
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

// Functions for SPI
void Define_SPI(void) {
	// Enable clock for SPI
	PMC->PMC_PCER0 |= (1 << ID_SPI);//| (1 << ID_PIOB);

	// Disabling PIO control over SPI pins
	PIOA->PIO_PDR |= (1 << SPI_MISO) | (1 << SPI_MOSI) | (1 << SPI_SCK);
	PIOB->PIO_PDR |= (1 << SPI_SS_A);
		
	// Configuring PIO controller to enable I/O line operations for SPI on PIOA
	PIOA->PIO_ABCDSR[0] &= ~((1 << SPI_MISO) | (1 << SPI_MOSI) | (1 << SPI_SCK));
	PIOA->PIO_ABCDSR[1] &= ~((1 << SPI_MISO) | (1 << SPI_MOSI) | (1 << SPI_SCK));
	
	// Configuring PIO controller to enable I/O line operations for SPI on PIOB
	//PIOB->PIO_ABCDSR[0] |= (1 << SPI_SS_A);
	//PIOB->PIO_ABCDSR[1] &= ~(1 << SPI_SS_A);
	
	// Set CS as output, start high
	PIOB->PIO_OER = (1 << SPI_SS_A);
	PIOB->PIO_SODR = (1 << SPI_SS_A);

	// Reset SPI
	SPI->SPI_CR = SPI_CR_SWRST;
		
	// Configuring SPI in master mode, ???disable fault detection -> only master???, activate CS0 for SPI
	SPI->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS;// | SPI_MR_PCS(0xE);

	// Configuring Chip Select 0 (16-bit, baud rate = peripheral clock / 42 -> 2MHz, data capture on RE)
	SPI->SPI_CSR[0] = SPI_CSR_BITS_16_BIT | SPI_CSR_SCBR(42) | SPI_CSR_NCPHA;
	
	// Enable SPI
	SPI->SPI_CR = SPI_CR_SPIEN;
}

uint16_t SPI_transfer(uint16_t data)
{
	// Wait for TX buffer to be empty
	while (!(SPI->SPI_SR & SPI_SR_TDRE));

	// Write data to SPI TDR (PCS=0 for CS0)
	SPI->SPI_TDR = SPI_TDR_TD(data) | SPI_TDR_PCS(0);

	// Wait for received data
	while (!(SPI->SPI_SR & SPI_SR_RDRF));

	// Read received data
	return (uint16_t)(SPI->SPI_RDR & SPI_RDR_RD_Msk);
}

uint16_t read_angle(void)
{
	// Start AS5050 cycle
	PIOB->PIO_CODR = (1 << SPI_SS_A);
	
	SPI_transfer(0x0000);	// Dummy
	uint16_t raw = SPI_transfer(0x0000);

	// End AS5050 cycle
	PIOB->PIO_SODR = (1 << SPI_SS_A);

	return raw & 0x0FFF;
}

int main(void)
{
    
    SystemInit();
	Define_UART0();
	Define_SPI();
	
			
	while (1) 
    {	
		uint16_t angle = read_angle();
		
		UART_send_string("Encoder position: ");
		UART_send_number(angle);
		UART_send_string("\r\n");
		delay(100);		
	}
}