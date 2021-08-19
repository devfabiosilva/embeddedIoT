// 18/04/2014
//PD5 | PD4 | PD3 | PD2 | PD1 | PD0 // old
//PD7 | PD6 | PD5 | PD4 | PD3 | PD2 // new
//E   | RS  | DB7 | DB6 | DB5 | DB4
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/pgmspace.h>
//#include <avr/pgmspace.h>
//#define F_CPU 16000000
// 19/04/2015 (dom)
static unsigned char _displaycontrol=0;
//unsigned char botao=0;
// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_FUNCTIONSET 0x20
#define LCD_2LINE 0x08
#define LCD_DISPLAYCONTROL 0x08
#define LCD_ENTRYMODESET 0x04
#define LCD_SETDDRAMADDR 0x80
#define LCD_RETURNHOME 0x02

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

//#define MAX_CUSTOM_CHAR 0x38 //56
#define MAX_CUSTOM_CHAR 0x40 //64
#define SET_CGRAM_ADDR 0x40

/////
//#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00
extern void pulsar() asm ("asm_PULSAR");
extern void lcdData(unsigned char Data) asm ("asm_LCD_DATA");
extern void sub_data(unsigned char data_a, unsigned char or_a, unsigned char and_a) asm ("asm_SUB_DATA");
extern void SetCursor(unsigned char row, unsigned char col) asm ("asm_SETCURSOR");
void replicate_char(char s, unsigned char n);

void pulsar()
{

	PORTD &=~(1<<PD7);
	_delay_us(15);
	PORTD |=(1<<PD7);
	_delay_us(15);
	PORTD &=~(1<<PD7);
	_delay_us(200);
}

void sub_data(unsigned char data_a, unsigned char or_a, unsigned char and_a)
{
	asm volatile
	(
		"mov r0, %0" "\n\t"
		"lsr %0" "\n\t"
		"lsr %0" "\n\t"
		"or %0, %1" "\n\t"
		"and %0, %2" "\n\t"
		"out 0x0b, %0" "\n\t"
		"call asm_PULSAR" "\n\t"
		"mov %0, r0" "\n\t"
		"lsl %0" "\n\t"
		"lsl %0" "\n\t"
		"or %0, %1" "\n\t"
		"and %0, %2" "\n\t"
		"out 0x0b, %0" "\n\t"
		"call asm_PULSAR" "\n\t"
		"mov %0, r0" "\n\t" // Added for replicate_char function loop (07/20/2015)
	:
	:"r"(data_a), "r"(or_a), "r"(and_a)
	);
}
void lcdcmd(unsigned char Data)
{
	sub_data(Data, 0x00, 0x3C);
}
void lcdData(unsigned char Data)
{

	asm volatile
	(
		"push r22" "\n\t" // 07/20/2015 For replicate_char loop
		"ldi r22, 0x40" "\n\t"
		"ldi r20, 0x7C" "\n\t"
		"call asm_SUB_DATA" "\n\t"
		"pop r22" "\n\t" // 07/20/2015 For replicate_char loop
	:
	:"r"(Data)
	);
}

void display_on(void)
{
	_displaycontrol |= (LCD_DISPLAYCONTROL | LCD_DISPLAYON);
	lcdcmd(_displaycontrol);
}
void cursor_on(void)
{
	_displaycontrol |= (LCD_DISPLAYCONTROL | LCD_CURSORON|LCD_BLINKON);
	lcdcmd(_displaycontrol);
}

void cursor_off(void)
{
	_displaycontrol &=~(LCD_CURSORON|LCD_BLINKON);
	lcdcmd(_displaycontrol);
}
void clear_display()
{
	lcdcmd(LCD_CLEARDISPLAY);
	_delay_ms(2);
}

void init_lcd(void)
{
	_delay_ms(50);
	PORTD=0x0C;
	pulsar();
	_delay_ms(5);

	//PORTD=0x0C; // is it really necessary?
	pulsar();
	_delay_ms(5);

	//PORTD=0x0C; // is it really necessary ? 
	pulsar();
	_delay_us(150);

	PORTD=0x08;
	pulsar();

	lcdcmd(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS);
	lcdcmd(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF | LCD_BLINKOFF | LCD_CURSOROFF);
	clear_display();
	lcdcmd(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);

}

void SetCursor(unsigned char row, unsigned char col)
{
	static const unsigned char row_offset[] PROGMEM={0x00, 0x40 ,0x14, 0x54};
	//lcdcmd(LCD_SETDDRAMADDR|pgm_read_byte_near(row_offset+row-1)|(col-1));
	lcdcmd(LCD_SETDDRAMADDR|(pgm_read_byte_near(row_offset+row-1)+(col-1)));// Fixed 07/28/2015

}

void lcd_return_home()
{
	lcdcmd(LCD_RETURNHOME);
	_delay_ms(2);
}

void print_lcd2(const char *s)
{
	unsigned char i,a=strlen(s);
	//for (i=0;(i < strlen(s)); i++)
	for (i=0;(i < a); i++)
	{ 
		if (s[i]>0x0F) lcdData(s[i]);
		else lcdData(s[i]-1);
	}
}

void lcd_clear_line(unsigned char k)
{
	SetCursor(k, 1);
	replicate_char(0x20,0x14);
	SetCursor(k,1);
}

void load_custom_char()
{
// 0- á
// 1- ã
// 2- Ã
// 3- Í
// 4-°
// 5 - ;)
// 6 - í
// 7 - ç <-- retirado
	unsigned char i;
	static const unsigned char custom_char[] PROGMEM=
			{
			        0x0d, 0x12, 0x00, 0x0e, 0x01, 0x0f, 0x11, 0x0f, // ã
		 	        //0x0d, 0x16, 0x0e, 0x01, 0x0f, 0x11, 0x0f, 0x0f, // ã  | 0
		                0x0d, 0x12, 0x00, 0x0e, 0x11, 0x1f, 0x11, 0x11, // Ã  | 1
			 	0x02, 0x04, 0x00, 0x04 ,0x0c, 0x04, 0x04, 0x0e, // í  | 2
			        0x02, 0x04, 0x0e, 0x04, 0x04, 0x04, 0x04, 0x0e, // Í  | 3
				0x02, 0x04, 0x00, 0x0e, 0x01, 0x0f, 0x11, 0x0f, // á  | 4
			 	0x0c, 0x12, 0x12, 0x12, 0x0c, 0x00, 0x00, 0x00, // °  | 5
			 	0x1b, 0x1b, 0x00, 0x04, 0x00, 0x11, 0x0e, 0x00, // ;) | 6
			};

	lcdcmd(SET_CGRAM_ADDR);
	//for (i=0; i<MAX_CUSTOM_CHAR; i++) lcdData(custom_char[i]);
	for (i=0; i<MAX_CUSTOM_CHAR; i++) lcdData((unsigned char)pgm_read_byte_near(custom_char + i));
	SetCursor(1,1);
}
void replicate_char(char s, unsigned char n)
{
	asm volatile
	(
		"replicate_char_VOLTAR:" "\n\t"
			"call asm_LCD_DATA" "\n\t"
			"dec %1" "\n\t"
			"brne replicate_char_VOLTAR" "\n\t"
		:
		:"r"(s), "r"(n)
	);
}

void print_lcd2_P(const char *s)
{
	char buffer[21];
	strcpy_P(buffer, s);
	print_lcd2(buffer);
}

