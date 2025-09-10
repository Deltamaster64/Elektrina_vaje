
#include "asf.h"
#include "stdio.h"
#include "math.h"
#include "spi.h"

// UART0
#define UART0_RX 9  // UART0 RX line PA9
#define UART0_TX 10 // UART0 TX line PA10
#define BAUDRATE 115200

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
	uint16_t arr[8];
	
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

void UART_send_float(float value, uint8_t decimals) {
	char arr[16];
	
	// Format the float into string with given decimal places
	sprintf(arr, "%.*f", decimals, value);
	
	UART_send_string(arr);
}

// SPI
#define SPI_CLK_SPEED 1000000 // 1 MHz

void Define_SPI(void){
	// Enable peripheral clock for SPI
	sysclk_enable_peripheral_clock(ID_SPI);

	// Configure SPI pins -> pio_configure(PIO#,periferal function,bit mask,pull-up/pull-down...);
	pio_configure(PIOA, PIO_PERIPH_A, PIO_PA12A_MISO, PIO_DEFAULT);  // MISO
	pio_configure(PIOA, PIO_PERIPH_A, PIO_PA13A_MOSI, PIO_DEFAULT);  // MOSI
	pio_configure(PIOA, PIO_PERIPH_A, PIO_PA14A_SPCK, PIO_DEFAULT);  // SCK
	
	// Configure chip select output & set it high	
	pio_set_output(PIOB, PIO_PB14, 1, 0, 0);

	// Disable & reset SPI
	spi_disable(SPI);
	spi_reset(SPI);

	spi_set_master_mode(SPI); // ATSAM4E is master of com.
	spi_disable_mode_fault_detect(SPI); // Only one master so no need for fault detect
	spi_set_clock_polarity(SPI, 0, 0); // SPI,NPCS index,0 (idle clock 0 -> mode1 on AS5050)
	spi_set_clock_phase(SPI, 0, 1); // SPI,NPCS index,1 (data on second RE)
	spi_set_bits_per_transfer(SPI, 0, SPI_CSR_BITS_16_BIT); // SPI,NPCS index,nr. of bits in each transfer
	spi_set_baudrate_div(SPI, 0, sysclk_get_cpu_hz() / SPI_CLK_SPEED);
	spi_set_peripheral_chip_select_value(SPI, spi_get_pcs(0));

	// Enable SPI
	spi_enable(SPI);
}

uint16_t SPI_transfer(uint16_t data){
	uint16_t rx;

	spi_write(SPI, data, 0, 0);
	spi_read(SPI, &rx, SPI_TIMEOUT);

	return rx;
}

uint16_t AS5050_read_angle(void){
	uint16_t raw;

	pio_clear(PIOB, PIO_PB14);          // CS LOW
	raw = SPI_transfer(0xFFFF);       // Send dummy word
	pio_set(PIOB, PIO_PB14);            // CS HIGH

	return raw & 0x03FF;                // Extract 10-bit angle
}

int main (void)
{
	sysclk_init();
	board_init();
	Define_UART0();
	Define_SPI();
	
	while(1){
		// Read position
		uint16_t raw_position = AS5050_read_angle();
		float angle = ((float)raw_position / 1023) * 360;
		
		// Display position
		UART_send_string("Encoder position: ");
		UART_send_number(raw_position);
		UART_send_string(" -> ");
		UART_send_float(angle,1);
		UART_send_string(" deg\r\n");
		delay_ms(100);
	}
}
