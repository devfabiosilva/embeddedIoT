// 21/05/2015 (qua)
#include <avr/pgmspace.h>
//#include "pgmstrings.h"

#define UART_EOT 0x04
#define UART_ACK 0x06
#define UART_NACK 0x15
#define SEND_EOT 0x02
// keys
#define UART_0_KEY '0'
#define UART_1_KEY '1'
#define UART_2_KEY '2'
#define UART_3_KEY '3'
#define UART_4_KEY '4'
#define UART_5_KEY '5'
#define UART_6_KEY '6'
#define UART_7_KEY '7'
#define UART_8_KEY '8'
#define UART_9_KEY '9'
#define UART_PLUS_MINUS_KEY 0x3A
#define UART_DOT_KEY 0x3B
#define UART_UP_PLUS_KEY 0x3C
#define UART_DOWN_MINUS_KEY 0x3D
#define UART_RIGHT_MULTIPLY_KEY 0x3E
#define UART_LEFT_DIVIDE_KEY 0x3F
#define UART_ENTER_KEY 0x40
#define UART_MENU_KEY 0x41
#define UART_RETURN_KEY 0x42
#define UART_CLEAR_KEY 0x43
#define UART_MF_KEY 0x44
#define UART_BEEP_ON_OFF_KEY 0x45
// end keys

// EEPROM address X and Y
// Where X and Y are forbidden for UART user
// Flag L |EE|PP|x|L|DD|
// Flag H |N|T|x|SS|En|F|R|
// Flag HL N|T|x|SS|En|F|R|EE|PP|x|L|DD
// Defaut values:
//
//     x --> Don't care value equals ZERO
//     Can read Date and Time DD=0x01
//     Can NOT access LCD panel L=0
//     Can read only pins information PP=0x01
//     Can read only EEPROM memory EE=0x01
//     Can NOT RESET system R=0x0
//     Can NOT read FLASH F=0x0
//     UART is ENABLED En=0x1
//     Can NOT read and write SRAM SS=0x00
//     Two wire access enabled (read and write) T=0x0
//     Can read sensors N=0x1
// Flag HL N|T|x|SS|En|F|R|EE|PP|x|L|DD
//         1|0|0|00|1 |0|0|01|01|0|0|01
//     Flag=0b1100010001010011
//     Flag=0x8451
//     Flag=33873
// 3ff
// 3fe
// 3fd
// 3fc [3]
// 3fb [2]
// 3fa [1]
// 3f9 [0]
#define FLAG_H_ADDR 0x03FF
#define FLAG_L_ADDR 0x03FE
#define FLAG_H_DEFAULT_VALUE 0x84
#define FLAG_L_DEFAULT_VALUE 0x51
//#define UART_CONF_EEPROM_ADDRESS 0x03FC
#define UART_CONF_EEPROM_ADDRESS 0x03FD
#define PASSWORD_ADDRESS_OFFSET 0x3F9
//#define MAX_EEPROM_MEMORY_ACCESS 0x03FC
#define MAX_EEPROM_MEMORY_ACCESS (PASSWORD_ADDRESS_OFFSET-1)
#define UART_FLASH_MEMORY_READ_ONLY 0x02
#define UART_EEPROM_READ_ONLY 0x40
#define UART_EEPROM_WRITE_ONLY 0x80
#define UART_PIN_READ_ONLY 0x10
#define UART_PIN_WRITE_ONLY 0x20
#define UART_DATE_TIME_READ_ONLY 0x01
#define UART_DATE_TIME_WRITE_ONLY 0x02
//#define UART_LATCH_READ_ONLY 0x04
//#define UART_LATCH_WRITE_ONLY 0x08
#define UART_LCD_ENABLE 0x04
#define UART_RESET_ENABLE 0x01
#define UART_ENABLE 0x04
#define UART_SRAM_READ_ONLY 0x08
#define UART_SRAM_WRITE_ONLY 0x10
#define UART_TWO_WIRE_ACCESS 0x40
#define UART_READ_SENSOR_ENABLE 0x80

#define UART_LCD_MODE 0xFF
// F --> Flash program access 0 -> access denied 1-> Read only
// EE --> EEPROM memory access 0-> access denied 1-> Read only 2-> write only 3-> read and write
// PP --> Pins information access 0-> access denied 1-> Read only 2-> write only 3-> read and write
// DD --> Date and time access 0-> access denied 1-> Read only 2-> write only 3-> read and write
// L --> access LCD panel 0 --> access denied 1 --> access ok
// R --> R -> 0 user can not reset system 1-> User can reset system
// En --> 0-> UART disabled 1-> UART ENABLED
// SS --> SRAM access 0 --> access denied 1- Read only 2- Write only 3- Read and write
// T --> Two wire access 0 --> access denied 1- read/write access
// N --> Read sensor 0 --> access denied 1 --> Read ok
#define UART_BEEP_TEST 'b' //
#define UART_RESET_MCU 'z' //
#define UART_READ_TIME 'i' //
#define UART_READ_DATE 't' //
#define UART_READ_EEPROM 'r'//
#define UART_READ_SRAM 'm' //
#define UART_READ_PIN 'p' //
#define UART_I2C_TRANSMIT_SEND_STOP 'o' //
#define UART_I2C_TRANSMIT_SEND_START 's' //
#define UART_I2C_TRANSMIT_SEND_CMD 'd' //
#define UART_I2C_RECEIVE_DATA 'v' //
#define UART_SEND_AUTHOR 'h' //
#define UART_SEND_AUTHOR_EMAIL 'l' //
#define UART_SEND_AUTHOR_USER 'S' //
#define UART_READ_SENSOR 'N' //
#define UART_READ_FLASH 'L'//
#define UART_HARDWARE_VERSION 'W' //
#define UART_FIRMWARE_VERSION 'V' //
#define UART_ENTER_LCD_MODE 'c' //
#define UART_CLEAR_SCREEN 'n' //
#define UART_EXIT_LCD_MODE 'O' //
//#define UART_LCD_CLEAR_LINE 'C'// <-- BUG TILL E IS FORBIDDEN
#define UART_LCD_CLEAR_LINE 'I'// Fixed 07/28/2015
#define UART_LCD_RETURN_HOME 'H' //
#define UART_SET_CURSOR 'U' //
#define UART_LCD_CONTROL 'u' //

#define UART_WRITE_EEPROM 'R' //
#define UART_WRITE_SRAM 'M' //
#define UART_LCD_DATA 'a' //
#define UART_WRITE_TIME 'T' //
#define UART_WRITE_DATE 'F' //
#define UART_WRITE_PIN 'P' //

#define UART_SENSOR_T1 0x01
#define UART_SENSOR_T2 0x02
#define UART_SENSOR_B1 0x04
#define UART_SENSOR_B2 0x08
#define UART_STRING_FROM_PROGRAM 0x01
#define UART_STRING_FROM_SRAM 0x00

#define UART_2X_SPEED_ENABLE 0x40

#define UART_PARITY_MODE_MASK ((~(1<<UPM00))&(~(1<<UPM01)))
#define UART_PARITY_MODE_DISABLE 0x00
#define UART_PARITY_MODE_EVEN_PARITY (1<<UPM01)
#define UART_PARITY_MODE_ODD_PARITY (1<<UPM01)|(1<<UPM00)
#define UART_DEFAULT_SPEED (UART_PARITY_MODE_DISABLE|0x05)

//added 02/26/2016
#define UART_LED_TX_OFF (1<<PB3)
#define UART_LED_TX_ON ~(UART_LED_TX_OFF)
#define UART_LED_RX_OFF (1<<PB1)
#define UART_LED_RX_ON ~(UART_LED_RX_OFF)

/*
#define UART_LED_TX_ON 0x08//(1<<PB3)
#define UART_LED_TX_OFF ~(UART_LED_TX_ON)
#define UART_LED_RX_ON 0x02//(1<<PB1)
#define UART_LED_RX_OFF ~(UART_LED_RX_ON) */
// end addded



extern void UART_Transmit( unsigned char data ) asm ("asm_UART_TRANSMIT");
extern void UART_println(const char *s, unsigned char from) asm ("asm_UART_PRINTLN");
//static const unsigned char UART_MAP_MODE[] PROGMEM={UART_PARITY_MODE_DISABLE, UART_PARITY_MODE_EVEN_PARITY, UART_PARITY_MODE_ODD_PARITY}; 
//#define UART_SEND_KEY 'k'

// until E is forbiden

// end security

void init_UART(unsigned char speed_index_and_mode)
{
	//speed_index & 0x0F in bps
	// 0 -> 300, 1->600, 2->1200, 3->2400, 4->4800, 5->9600, 6->14400, 7->19200, 8->28800
	//9-> 38400, 10-> 50000, 11-> 76800, 12-> 230400, 13-> 250000, 14-> 500000, 15-> 1000000
	// xabb|mmmm --> a-> enable 2x speed --> bb set parity (0 disable), (1 reserved), (2 even parity), 3 (odd parity)
	//x-> don't care
	static const unsigned char UART_SPEED_L[] PROGMEM=
		{
			0x04, 0x82, 0x41, 0xA0, 0xCF, 0x67, 0x4F, 0x44, 0x33, 0x22, 0x19, 0x13, 0x0C, 0x03, 0x01, 0x00 
		};
	static const unsigned char UART_SPEED_H[] PROGMEM=
		{
			0x0D, 0x06, 0x03, 0x01
		};

	unsigned char a=(speed_index_and_mode&0x0F);

	if (a<4) UBRR0H=(unsigned char)pgm_read_byte_near(UART_SPEED_H + a);
	else UBRR0H=0x00;

	UBRR0L=(unsigned char)pgm_read_byte_near(UART_SPEED_L + a);

//	UBRR0H=0x00;
//	UBRR0L=0x67; // 9600 bounds for normal transmission and 19200 for 2x transmission
	if (speed_index_and_mode&UART_2X_SPEED_ENABLE)
		UCSR0A|=(1<<U2X0);
	else
		UCSR0A&=~(1<<U2X0);
//	UCSR0C=(1<<UPM01)|(1<<UPM00)|(1<<UCSZ01)|(1<<UCSZ00); // 8-bit format and odd parity
	a=(UCSR0C&UART_PARITY_MODE_MASK)|(speed_index_and_mode&0x30);
	//a|=(speed_index_and_mode&0x30);
	UCSR0C=a;
	UCSR0B=(1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); // RX interrupt enable and RX and TX anabled
}

void UART_flush()
{
	unsigned char dummy;
	while (UCSR0A & (1<<RXC0)) dummy = UDR0;
}

unsigned char read_SRAM(unsigned int address)
{
	asm volatile(
		"push r26" "\n\t"
		"push r27" "\n\t"
		"movw r26, %0" "\n\t"
		"inc r27" "\n\t"
		"ld %0, X" "\n\t"
		"pop r27" "\n\t"
		"pop r26"
	:"=r"(address));
}
void write_SRAM(unsigned int address, unsigned char value)
{
	asm volatile(
		"push r26" "\n\t"
		"push r27" "\n\t"
		"movw r26, %0" "\n\t"
		"inc r27" "\n\t"
		"st X, %1" "\n\t"
		"pop r27" "\n\t"
		"pop r26" "\n\t"
	//:"=&r"(address):"r"(value)
	:
	:"r"(address), "r"(value)
	);
}
unsigned char read_PROGRAM_MEMORY(unsigned int address)
{
	uint8_t result_b;
	asm volatile(
		"push r30" "\n\t"
		"push r31" "\n\t"
		"movw r30, %0" "\n\t"
		"lpm %0, Z" "\n\t"
		"pop r31" "\n\t"
		"pop r30" "\n\t"
	:"=r"(result_b)
	:"r"(address));
	return result_b;
}
/*
unsigned char UART_receive()
{
	return UDR0;
}
*/
void UART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
	//added 02/26/2016
	PORTB|=(UART_LED_RX_OFF);
	PORTB^=(UART_LED_TX_OFF);
}

void UART_Transmit_BCD(unsigned char dataa)
{
	UART_Transmit((dataa>>4)+0x30);
	UART_Transmit((dataa&0x0F)+0x30);
}
void UART_println(const char *s, unsigned char from)
{
	static char buffer[65];
	static unsigned char i,a;
	// if (from)  // Fatal  Fixed 08/26/2015
	if (from&0x01)
		strcpy_P(buffer, s);
	else
		strcpy(buffer, s);
	a=strlen(buffer);
	for (i=0;(i < a); i++) UART_Transmit(buffer[i]);
	if (from&SEND_EOT) UART_Transmit(UART_EOT);
}

unsigned char check_uart_error()
{

	static const char ERR_FRAME_ERROR[] PROGMEM="Frame";
	static const char ERR_PARITY_ERROR[] PROGMEM="Parity";
	static const char ERR_DATA_OVR[] PROGMEM="Data overrun";
	static const char ERR_STR_A[] PROGMEM="ERROR: ";
	unsigned char a=(UCSR0A&((1<<FE0)|(1<<DOR0)|(1<<UPE0)));
	uint16_t address;

	if (a)
	{
		UART_flush(); // Fixed 28/08/2015
		UART_Transmit(UART_NACK);
		UART_Transmit(a);
		UART_println(ERR_STR_A, UART_STRING_FROM_PROGRAM);
		if (a&(1<<UPE0))
			address=(uint16_t)&ERR_PARITY_ERROR;
		else if (a&(1<<DOR0))
			address=(uint16_t)&ERR_DATA_OVR;
		else
			address=(uint16_t)&ERR_FRAME_ERROR;
		UART_println((const char *)address, UART_STRING_FROM_PROGRAM|SEND_EOT);
	}
	return a;
}

