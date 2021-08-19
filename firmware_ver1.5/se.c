/*
 * se.c
 * This file is part of se.fw
 *
 * 2015 - Fábio Pereira da Silva
 * LICENSE: MIT
 */

#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include "display_ver23.h"
#include "rtc.h"
//#include "timer_event.h"
//#include "dow.h"
#include <stdlib.h> 
#include <math.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include "pgmstrings.h"
#include "uartcmd.h"
//#include "sha1b.h"
#include "mhd.h"
//#define MAX 53
#define MAX 0x19 // 25
//#define MAX 0x32

#define TIMER0_1440_US 0xA6+0x010
//#define TIMER0_1200_US 0xBA
#define TIMER_4000_US 0x00

#define TIMER1_ENABLE 0x80
#define TIMER1_DISABLE ~(TIMER1_ENABLE)

#define SELECT_LCD 0x00
#define SELECT_KBD_OR_NONE 0x04
#define SELECT_PINOS 0x10 
#define SELECT_AUX_REG 0x14

#define CHARGER1_OFF 0x04
#define CHARGER2_OFF 0x08
#define SPK_ON 0x40
#define LCD_ON 0x80

#define PINO1 0x04
#define PINO2 0x08
#define PINO3 0x10
#define PINO4 0x20
#define PINO5 0x40
#define PINO6 0x80

#define IS_KBD_KEY_PRESSED 0x01
#define IS_MENU 0x80
#define MENU_KEY 0x41
#define RETURN_KEY 0x42
#define PLUS_MINUS_KEY 0x3A
#define DOT_KEY 0x3B
#define UP_PLUS_KEY 0x3C
#define DOWN_MINUS_KEY 0x3D
#define RIGHT_MULTIPLY_KEY 0x3E
#define LEFT_DIVIDE_KEY 0x3F
#define ENTER_KEY 0x40
#define CLEAR_KEY 0x43
#define MF_KEY 0x44
#define BEEP_ON_OFF 0x45
#define BEEP_ENABLE 0x40
/////////
#define ABOUT_MENU 0xB0
#define CALCULATOR_MENU 0xA8
#define BASIC_CALC_MENU 0xA9
#define SCIENTIFIC_MENU 0xAA
#define DATE_TIME_MENU 0x88
#define DATE_TIME_ADJUST 0x89
#define DATE_TIME_WEEK_DAY 0x8A
#define ALARM_SETUP 0x8B
#define SENSOR_MENU 0xA0
#define OTHER_MENU 0x98
#define OTHER_MENU_LANGUAGE 0x99

#define OTHER_MENU_TEMP_MEASUREMENT 0xF0
#define OTHER_MENU_UART_FOR_IOT 0xF1
#define OTHER_MENU_SECURITY 0xF2
#define OTHER_MENU_SET_PASSWORD 0xF3
#define OTHER_MENU_CONFIGURE_PORT_UART 0xF4
#define OTHER_MENU_CONFIGURE_SPEED_UART 0xF5
#define OTHER_MENU_CONFIGURE_PARITY_UART 0xF6
#define OTHER_MENU_CONFIGURE_DEF 0xF7
#define OTHER_MENU_CONFIGURE_UART2X 0xF8
#define OTHER_MENU_CONFIGURE_UART_SECURITY 0xF9
#define OTHER_MENU_CONFIGURE_UART_SEC_PERM 0xFA
#define OTHER_MENU_CONFIGURE_DEF_SECURITY 0xFB

#define OTHER_MENU_UART_FOR_IOT_PASSWD 0xFC
#define OTHER_MENU_CONFIGURE_UART 0xFD
#define MIN_UART_LOCK_INTERVAL OTHER_MENU_UART_FOR_IOT
#define MAX_UART_LOCK_INTERVAL OTHER_MENU_CONFIGURE_UART

/////
#define PINO_MENU 0xB8
#define PINO_MENU_P1 0xB9
#define PINO_MENU_P2 0xBA
#define PINO_MENU_P3 0xBB
#define PINO_MENU_P4 0xBC
#define PINO_MENU_P5 0xBD
#define PINO_MENU_P6 0xBE
/////////////////////////
// about sensors input //
/////////////////////////
//#define TEMP1_SENSOR (1<<REFS0)
#define TEMP1_SENSOR ((1<<REFS1)|(1<<REFS0))
#define TEMP2_SENSOR ((1<<REFS1)|(1<<REFS0)|(1<<MUX3))
#define B1_SENSOR ((1<<REFS1)|(1<<REFS0)|(1<<MUX0))
#define B2_SENSOR ((1<<REFS1)|(1<<REFS0)|(1<<MUX1))

// end sensor input

//////////////////
// LED's pilots //
//////////////////
#define B1_LED_SIGNAL 0x10
#define B2_LED_SIGNAL 0x20
// END LED PILOTS

static unsigned char pseudo_latch_pinos, pseudo_latch_aux;
static double t1, t2, b1, b2;//, d_tmp_var;

static const double charged_voltage=1.415, uncharged_voltage=1.35, high_voltage=1.55, low_voltage=0.5;
static const double inv_100_charged_voltage= 70.67137809;//69.444444;// 100/charged_voltage=1.44
static const double pi_inv_180=0.0174532925199433;
static const double deg_inv_pi=57.295779513082321;

///////////////////////////
// EEPROM memory address //
///////////////////////////
#define PINO1_OFFSET_ADDR 0x02
#define PINO2_OFFSET_ADDR 0x15
#define PINO3_OFFSET_ADDR 0x28
#define PINO4_OFFSET_ADDR 0x3B
#define PINO5_OFFSET_ADDR 0x4E
#define PINO6_OFFSET_ADDR 0x61

// END EEPROM memory address

/*
	UART Serial communication for IoT

*/
//#define BAUD 9600                           // define baud
//#define UBRR ((F_CPU)/(BAUD*16UL)-1)

#define UART_TIME_OUT 0x80 // the bit 8 of uart_cmd_count_max are used to flag timeout
#define UART_READY_STREAM_DATA 0x40 // the uart bit 7 is flag to be ready to receive stream data
#define UART_COMMAND_MAX_ENTRY 0x09
//static char uart_command[UART_COMMAND_MAX_ENTRY]={0x00};
static unsigned char uart_command[UART_COMMAND_MAX_ENTRY]={0x00};
static unsigned char uart_cmd_count, uart_cmd_count_max=0;

// end UART for IoT
//////////////////
// PINS MONITOR //
//////////////////
static struct PINOS_t
{
	// command b7:b3--> reserved | b2:b0-->command
	unsigned char command,hh1,mm1,hh2,mm2, hh3,mm3,ss3,hh4,mm4,ss4,hh5,mm5,ss5, opbat1, opbat2, gpv;
	// gpv = general purpose var
	unsigned int countdown1, countdown2, countdown3;
	signed char d, e;

}pino1,pino2,pino3,pino4,pino5,pino6;

// END PINS MONITOR

#define MAX_INPUT 0x11


double toFarenheit(double *address)
{
	asm
	(
		"ld r22, Y" "\n\t"
		"ldd r23, Y+1" "\n\t"
		"ldd r24, Y+2" "\n\t"
		"ldd r25, Y+3" "\n\t"
		"ldi r18, 0x66" "\n\t"
		"ldi r19, 0x66" "\n\t"
		"ldi r20, 0xE6" "\n\t"
		"ldi r21, 0x3F" "\n\t"
		"call __mulsf3" "\n\t"
		"ldi r18, 0x00" "\n\t"
		"ldi r19, 0x00" "\n\t"
		"ldi r20, 0x00" "\n\t"
		"ldi r21, 0x42" "\n\t"
		"call __addsf3" "\n\t"
		//"ret" "\n\t"
	:
	:"y"(address)
	);
}

#define to_uint_16(high, low) \
(__extension__					\
	(							\
		{						\
			unsigned char __high=(unsigned char)(high); \
			unsigned char __low=(unsigned char)(low);	\
			uint16_t __result_a;						\
			__asm__										\
			(											\
				"mov %B0, %2" "\n\t"					\
				"mov %A0, %1" "\n\t"					\
				:"=r"(__result_a)						\
				:"r"(__low),"r"(__high)					\
			);											\
			__result_a;									\
		}												\
	)													\
)
#define compare_int_16_ovf(value) 						\
(__extension__											\
	(													\
		{												\
			uint32_t __value=(uint32_t)(value); 		\
			uint8_t __result_a;							\
			__asm__										\
			(											\
				"cli" "\n\t"							\
				"ldi r24, 0x01" "\n\t"					\
				"cp %A1, r1" "\n\t"						\
				"cpc %B1, r1" "\n\t"					\
				"cpc %C1, r24" "\n\t"					\
				"cpc %D1, r1" "\n\t"					\
				"brcc OVF_SAIR" "\n\t"					\
				"ldi r24, 0x00" "\n\t"					\
			"OVF_SAIR:" "\n\t"							\
				"sei" "\n\t"							\
				:"=r"(__result_a)						\
				:"r"(__value)							\
			);											\
			__result_a;									\
		}												\
	)													\
)
#define hi_uint_16(value_d)								\
(__extension__											\
	(													\
		{												\
			uint32_t __value_d=(uint32_t)(value_d);		\
			uint8_t __result_d;							\
			__asm__										\
			(											\
				"mov %0, %B1" "\n\t"					\
			:"=r"(__result_d)							\
			:"r"(__value_d)								\
			);											\
			__result_d;									\
		}												\
	)													\
)

static char input[MAX_INPUT];
static unsigned char pos_input=0;

struct DT_t
{
	char year[5], month[3], day[3], hh[3], mm[3], ss[3];

}DT;

// END INPUT METHOD
extern unsigned char temporizador=0x00;
//static unsigned char code[MAX]={0};
static unsigned char n,n2,p;//temporizador=0;
static char conv[13];
//static unsigned long key;
static uint32_t key;
static unsigned char cr_start=0;
//static unsigned char cr_timer=0;
static unsigned char code_cr=0x00;;//=0xff; // valor inicial (nenhum -> 0xff);
static unsigned char tmp_gpr; // temporary general purpose;

//static uint32_t scan_cr_timeout=0x0000;

/////////////////////////
// variables for MENU //
///////////////////////
static unsigned char menu=0x00;

///////////////////////////////////
// TIME VARIABLES AND CONSTANTS //
/////////////////////////////////

static unsigned char hours, minutes;//, seconds; // current time
static unsigned char hoursA, minutesA; // Alarm variables

// END TIME VARIABLES AND CONSTANTS

//////////////////////////////////////////
// BASICS CALC VARIABLES AND CONSTANTS //
////////////////////////////////////////

#define BAS_CALC_DIGIT 10
static double A, B, C;
static char bas_calc_op;
static unsigned char pos_bas_calc, bas_calc_status=0;
static char bas_calc_char[BAS_CALC_DIGIT];
 

// END BASIC CALC AND CONSTANTS

////////////////////////////////////////
// SCIENTIFC CONSTANTS AND VARIABLES //
//////////////////////////////////////

// variable A is used as X in scientific calculator
// variable bas_calc_char is used in scientific_calculator
//static char function_name[10];
static char function_name[9];
static unsigned char function_name_op=0;

// END SCIENTIFC CONSTANTS AND VARIABLES

////////////////////////////////
// BEGIN BILINGUAL INTERFACE //
//////////////////////////////

static char language;

// END BILINGUAL INTERFACE

///////////////////////
// BEGIN PROTOTYPES //
/////////////////////

void show_menu_calculator();
void prepare_lcd();
void clean_UART_STREAM();
void beepAlarm();
unsigned char EEPROM_read(unsigned int uiAddress);
//void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char valid_time(unsigned char hh, unsigned char mm, unsigned char ss);
unsigned char is_BCD(unsigned char value);
unsigned char BCD_to_uint8(unsigned char valuea);
unsigned char valid_date(unsigned char day, unsigned char month, unsigned int year);
int getDay(int y, int m, int d);
void set_none();
void exit_menu();
void set_latch_pinos();

// end prototypes

extern void EEPROM_write(unsigned int uiAddress, unsigned char ucData) asm ("asm_EEPROM_WRITE");
extern unsigned char is_BCD(unsigned char value) asm ("asm_IS_BCD");
extern void turn_on_timer_2(unsigned char time2) asm ("asm_TURN_ON_TIMER_2");
extern unsigned char EEPROM_read(unsigned int uiAddress) asm ("asm_EEPROM_READ");
extern void fill_null_char(const uint16_t address, unsigned char size);
void show_other_menu();

unsigned char read_write_temporizador(unsigned char value_k)
{
	asm volatile
	(
		//"push r16" "\n\t"
		"lds r0, 0x00B1" "\n\t" // save state of timer 2
		"sts 0x00B1, r1" "\n\t" // pause timer2 (if enabled)
		"sts 0x0081, r1" "\n\t" // pause timer 1
		"st X, %0" "\n\t"
		"ldi r26, 0x05" "\n\t"
		"sts 0x0081, r26" "\n\t" // continue timer 1
		"sts 0x00B1, r0" "\n\t" // continue timer 2 (if enabled)
		//"pop r16"
	:"=r"(value_k)
	:"x"(&temporizador)
	);
	return value_k;
}

void turn_off_timer0()
{
	TCCR0B=0x00;
	TIMSK0=0x00;
	TCNT0=0x00;
}
void turn_on_timer0(unsigned char start)
{
	TCCR0B=0x00;
	TCNT0=start;
	TIMSK0=(1<<TOIE0);
	TCCR0B=(1<<CS02); // prescale by fs/256 (counter 0)
}

void turn_on_timer_2(unsigned char time2)
{

	TCCR1B=0x00;
	asm volatile
	(
		"ld r25, X" "\n\t"
		"or r25, %1" "\n\t"
		"st X, r25" "\n\t"
		"ldi %1, 0x01" "\n\t"
		"sts 0x0070, %1" "\n\t" // TIMSK2 interrupt enabled
		"ldi %1, 0x07" "\n\t" // fs/1024 timer 2 on
		"sts 0x00B1, %1" "\n\t"
	://"+x"(&temporizador)
	:"x"(&temporizador), "r"(time2)
	);
	TCCR1B=(1<<CS12)|(1<<CS10);
}

unsigned char validate_uart_access(uint16_t address, unsigned char what_c)
{

	asm volatile
	(
		"call asm_EEPROM_READ" "\n\t"
		"and %0, %1" "\n\t"
		"brne validate_uart_access_SAIR" "\n\t"
		"ldi r24, 0x15" "\n\t" //  NACK
		"call asm_UART_TRANSMIT" "\n\t"
		"ld r24, X" "\n\t"
//		"andi r24, 0x01" "\n\t"
		"sbrc r24, 0" "\n\t"
		"adiw r30, 0x0E" "\n\t" // SizeOf(ACCESS_DENIED[0])
		"movw r24, r30" "\n\t"
		//"ldi r22, 0x01" "\n\t"
		"ldi r22, 0x03" "\n\t"
		"call asm_UART_PRINTLN" "\n\t"
		"clr r24" "\n\t" // False result
	"validate_uart_access_SAIR:" "\n\t"
	:
	:"r"(address),"r"(what_c), "x"(&language),"z"(&ACCESS_DENIED)
	);
}

//////////////////////////////
// BEGIN INTERRUPT VECTORS //
////////////////////////////

ISR(PCINT0_vect)
{

	cli();
	if (cr_start==0x01)
	{
		if (PINB & (1<<PB0))
		{
			//cli();
			PCICR=0x00;
			turn_on_timer0(TIMER0_1440_US);
			cr_start=0x10;
			key|=0x01;
			//sei();
		}
	}
	else if (cr_start==0)
	{
		if (PINB & (1<<PB0))
		{
			//cli();
			PCICR=0x00;
			turn_on_timer0(TIMER_4000_US);
			//sei();
		}
	}
}

ISR(TIMER0_OVF_vect)
{

	cli();
	if (cr_start==0x10)
	{

		key^=(PINB&(1<<PB0));
		key<<=1;

		p++;
		//cli();
		if (p<MAX)
		{
			cr_start=0x01;
			PCICR=(1<<PCIE0);
		}
		else cr_start=0x02;
		turn_off_timer0();
		//PCICR=(1<<PCIE0);
		//sei();
	}
	else if (cr_start==0)
	{
		if (PINB & (1<<PB0)) cr_start=0x04;
		else
		{
			//restart_pin_interrupt();

			//cli();
			turn_off_timer0();
			PCICR=(1<<PCIE0);
			//sei();

		}
	}
	else if (cr_start&0x04)
	{
		if (PINB & (1<<PB0))
			cr_start=0x08;
		else
		{
			//restart_pin_interrupt();
			cr_start=0x00;
			//cli();
			turn_off_timer0();
			PCICR=(1<<PCIE0);
			//sei();
		}
	}
	else if (cr_start&0x08)
	{
		if (PINB&(1<<PB0)) cr_start=0x00;
		else
		{
			cr_start=0x01;
			p=0;
			key=0x0000;
		}
		//cli();
		PCICR=(1<<PCIE0);
		turn_off_timer0();
		//sei();
	}
}

ISR(TIMER1_OVF_vect)
{
	TIMSK2=0x00;
	temporizador|=TIMER1_ENABLE;
	TIMSK2=(1<<TOIE1);
	if (uart_cmd_count_max & UART_READY_STREAM_DATA) uart_cmd_count_max|=UART_TIME_OUT;
}
ISR(TIMER2_OVF_vect)
{
	//cli();
	asm volatile (
		"sts 0x0081, r1" "\n\t" // stop timer 1 for a while
		"ld r0, X" "\n\t"
		"push r25" "\n\t"
		"mov r25, r0" "\n\t"
		"andi r25, 0x1F" "\n\t" // TIMER_EVENT
		"breq eval_timer_erro_1" "\n\t"
		"dec r25" "\n\t"
		"andi r25, 0x1F" "\n\t"
		"mov r1, r25" "\n\t"
	"eval_timer_erro_1:" "\n\t"
		"mov r25, r0" "\n\t"
		"andi r25, 0x60" "\n\t" // LOOP_EVENT
		"breq eval_timer_erro_2" "\n\t"
		"subi r25, 0x20" "\n\t" // -0x20
		"andi r25, 0x60" "\n\t" // LOOP_EVENT
		//"or r1, r25" "\n\t"
	"eval_timer_erro_2:" "\n\t"
		//"tst r1" "\n\t"
		"or r1, r25" "\n\t" // Or r25 and test at same time. I dont need tst
		"brne eval_timer_sair" "\n\t"
		"sts 0x00B1, r1" "\n\t" // HERE r1 is equal 0 (timer 2 stopped)
		"sts 0x0070, r1" "\n\t" // Interrupt (overflow disabled)
		"sts 0x00B2, r1" "\n\t" // counter =0
	"eval_timer_sair:" "\n\t"
		"mov r25, r0" "\n\t"
		"andi r25, 0x80" "\n\t"
		"or r25, r1" "\n\t"
		"st X, r25" "\n\t"
		"ldi r25, 0x05" "\n\t"
		"sts 0x0081, r25" "\n\t" // running timer 1 again
		"pop r25" "\n\t"
		//"clr r1" "\n\t" // After used r1 then set r1 to (Zero) (I don't need it in interrupt)
		//"sts X, %0" "\n\t"
	:
	:"x"(&temporizador)
	//:"y"(temporizador)
	);
}
void sub_uart_lock()
{
	UART_Transmit(UART_NACK);
	UART_println(ERR_UART_SECURUTY_LOCK ,UART_STRING_FROM_PROGRAM|SEND_EOT);
	clean_UART_STREAM();
	UART_flush();
	//code_cr=0x00; // Added 07/27/2015
	//cr_start=0x00; // Added 07/27/2015
	exit_menu(); // Added 07/24/2015
	_delay_us(600);
	beepAlarm();
		//show_other_menu();
}
static uint16_t Memory;
//static char tmp[7]; //000.00 Fixed for transmit sensors Data
static char tmp[12]; // 000.0000 FIXED (08/25/2015)

ISR(USART_RX_vect)
{

	cli(); // Fixed 27/07/2015
//	PORTB&=(UART_LED_RX_ON & UART_LED_TX_ON); // Added 02/26/2016
	PORTB&=(UART_LED_RX_ON); // Added 02/26/2016
	if (check_uart_error())
		clean_UART_STREAM();
	else if ((menu>=MIN_UART_LOCK_INTERVAL)&&(menu<=MAX_UART_LOCK_INTERVAL))
	//if (is_lock_interval())
	{

		sub_uart_lock();
	}
	else
	{
		//cli();
		if (EEPROM_read(FLAG_H_ADDR)&UART_ENABLE)
		{
			if (uart_cmd_count < UART_COMMAND_MAX_ENTRY)
			{
				if ((uart_cmd_count_max) & (UART_TIME_OUT))
				{
					UART_Transmit(UART_NACK);
					UART_println(ERR_TIME_OUT, UART_STRING_FROM_PROGRAM|SEND_EOT);
					clean_UART_STREAM();
				}
				else
				{
					uart_command[uart_cmd_count]=UDR0;
					if(uart_command[0]<=UART_BEEP_ON_OFF_KEY)
					{
						if(uart_command[0]>=UART_0_KEY)
						{
							if ((menu>=MIN_UART_LOCK_INTERVAL)&&(menu<=MAX_UART_LOCK_INTERVAL))
							//if (is_lock_interval())
							{

								sub_uart_lock();
							}
							else
							{
								cr_start=0x04;
								code_cr=uart_command[0];
								UART_Transmit(UART_ACK);
							}
						}
						else
						{
							UART_Transmit(UART_NACK);
							UART_println(ERR_INVALID_UART_COMMAND, UART_STRING_FROM_PROGRAM|SEND_EOT);
						}
						clean_UART_STREAM();
					}
					else
					{
						switch (uart_command[0])
						{
							case UART_BEEP_TEST:

								UART_Transmit(UART_ACK);
								clean_UART_STREAM();
								beepAlarm();
								break;

							case UART_RESET_MCU:
/*
								if (EEPROM_read(FLAG_H_ADDR)&UART_RESET_ENABLE)
								{
									UART_Transmit(UART_ACK);
									asm volatile("jmp 0x0000");
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
								}
								break;
*/
								if (validate_uart_access(FLAG_H_ADDR, UART_RESET_ENABLE))
								{
									UART_Transmit(UART_ACK);
									asm volatile("jmp 0x0000");
								}
								clean_UART_STREAM();
								break;

							case UART_READ_TIME:
								if (rtc_disable)
								{
									UART_Transmit(UART_NACK);
									UART_println(ERR_I2C_BUSY, UART_STRING_FROM_PROGRAM|SEND_EOT);
								}
								else
								{
									//if (EEPROM_read(FLAG_L_ADDR)& UART_DATE_TIME_READ_ONLY)
									if (validate_uart_access(FLAG_L_ADDR, UART_DATE_TIME_READ_ONLY))
									{
										UART_Transmit(UART_ACK);
										TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN); // send stop
										UART_Transmit_BCD(read_from_rtc(RTC_HOURS_REG)&0x3F);
										UART_Transmit_BCD(read_from_rtc(RTC_MINUTES_REG)&0x7F);
										UART_Transmit_BCD(read_from_rtc(RTC_SECONDS_REG)&0x7F);
									}
/*									else
									{
										UART_Transmit(UART_NACK);
										UART_println(ACCESS_DENIED[(language&0x01)],UART_STRING_FROM_PROGRAM);
									}
*/
								}
								clean_UART_STREAM();
								break;

							case UART_READ_DATE:
								if (rtc_disable)
								{
									UART_Transmit(UART_NACK);
									UART_println(ERR_I2C_BUSY, UART_STRING_FROM_PROGRAM|SEND_EOT);
								}
								else
								{
									//if (EEPROM_read(FLAG_L_ADDR)&UART_DATE_TIME_READ_ONLY)
									if (validate_uart_access(FLAG_L_ADDR, UART_DATE_TIME_READ_ONLY))
									{
										if (read_from_rtc(RTC_MONTHS_CENTURY_REG)&IS_19XX)
											strcpy_P(tmp,CENTURY_19XX);
										else
											strcpy_P(tmp,CENTURY_20XX);
										UART_Transmit(UART_ACK);
										UART_println(tmp, UART_STRING_FROM_SRAM);

										UART_Transmit_BCD(read_from_rtc(RTC_YEARS_REG));

										UART_Transmit_BCD(read_from_rtc(RTC_MONTHS_CENTURY_REG)&0x1F);

										UART_Transmit_BCD(read_from_rtc(RTC_DAYS_REG)&0x3F);

										UART_Transmit_BCD(read_from_rtc(RTC_WEEKDAYS_REG)&0x07);

									}
/*
									else
									{
										UART_Transmit(UART_NACK);
										UART_println(ACCESS_DENIED[language&0x01],UART_STRING_FROM_PROGRAM);
									}
*/
								}
								clean_UART_STREAM();
								break;

							case UART_READ_EEPROM:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_EEPROM_READ_ONLY)
								{
									//if (uart_cmd_count < 3) // Fixed 23/07/2015
									if (uart_cmd_count < 2)
									{
										uart_cmd_count++;
										uart_cmd_count_max|=UART_READY_STREAM_DATA;
									}
									else
									{
										//Memory.access.high=uart_command[1];
										//Memory.access.low=uart_command[2];
										//Memory=(uart_command[1]<<8)|(uart_command[2]);
										Memory=to_uint_16(uart_command[1], uart_command[2]);
										//if (Memory.memory > MAX_EEPROM_MEMORY_ACCESS) // last 2 bytes is forbidden to UART user
										if (Memory > MAX_EEPROM_MEMORY_ACCESS) // last 2 bytes is forbidden to UART user
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_MEMORY_OVF, UART_STRING_FROM_PROGRAM);
											UART_println(ERR_MEMORY_OVF_EEPROM, UART_STRING_FROM_PROGRAM);
										}
										else
										{
											UART_Transmit(UART_ACK);
											//UART_Transmit(EEPROM_read(Memory.memory));
											UART_Transmit(EEPROM_read(Memory));
											//UART_Transmit_ByteToASCII(EEPROM_read(Memory.memory));
										}
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[language&0x01], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
								// [eeprom_read][high address][low address]
*/
								//if (uart_cmd_count < 3) // Fixed 23/07/2015
								if (uart_cmd_count < 2)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
//								if (EEPROM_read(FLAG_L_ADDR)&UART_EEPROM_READ_ONLY)
									if (validate_uart_access(FLAG_L_ADDR, UART_EEPROM_READ_ONLY))
									{
										Memory=to_uint_16(uart_command[1], uart_command[2]);

										if (Memory > MAX_EEPROM_MEMORY_ACCESS) // last 2 bytes is forbidden to UART user
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_MEMORY_OVF, UART_STRING_FROM_PROGRAM);
											UART_println(ERR_MEMORY_OVF_EEPROM, UART_STRING_FROM_PROGRAM|SEND_EOT);
										}
										else
										{
											UART_Transmit(UART_ACK);
											//UART_Transmit(EEPROM_read(Memory.memory));
											UART_Transmit(EEPROM_read(Memory));
											//UART_Transmit_ByteToASCII(EEPROM_read(Memory.memory));
										}
									}
									clean_UART_STREAM();
								}

								break;

							case UART_READ_SRAM:
/*
								if (EEPROM_read(FLAG_H_ADDR)&UART_SRAM_READ_ONLY)
								{
									//if (uart_cmd_count < 3) //Fixed 23/07/2015
									if (uart_cmd_count < 2)
									{
										uart_cmd_count++;
										uart_cmd_count_max|=UART_READY_STREAM_DATA;
									}
									else
									{
										//Memory.access.high=uart_command[1];
										//Memory.access.low=uart_command[2];
										//Memory=(uart_command[1]<<8)|(uart_command[2]);
										Memory=to_uint_16(uart_command[1], uart_command[2]);
										//if (Memory.memory > 2047)
										if (Memory > 2047)
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_MEMORY_OVF, UART_STRING_FROM_PROGRAM);
											UART_println(ERR_MEMORY_OVF_SRAM, UART_STRING_FROM_PROGRAM);
										}
										else
										{
											UART_Transmit(UART_ACK);
											//UART_Transmit(read_SRAM(Memory.memory));
											UART_Transmit(read_SRAM(Memory));
											//UART_Transmit_ByteToASCII(read_SRAM(Memory.memory))

										}
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/

									//if (uart_cmd_count < 3) //Fixed 23/07/2015
								if (uart_cmd_count < 2)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
//								if (EEPROM_read(FLAG_H_ADDR)&UART_SRAM_READ_ONLY)
									if (validate_uart_access(FLAG_H_ADDR, UART_SRAM_READ_ONLY))
									{
										//Memory.access.high=uart_command[1];
										//Memory.access.low=uart_command[2];
										//Memory=(uart_command[1]<<8)|(uart_command[2]);
										Memory=to_uint_16(uart_command[1], uart_command[2]);
										if (Memory > 2047)
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_MEMORY_OVF, UART_STRING_FROM_PROGRAM);
											UART_println(ERR_MEMORY_OVF_SRAM, UART_STRING_FROM_PROGRAM|SEND_EOT);
										}
										else
										{
											UART_Transmit(UART_ACK);
											//UART_Transmit(read_SRAM(Memory.memory));
											UART_Transmit(read_SRAM(Memory));
											//UART_Transmit_ByteToASCII(read_SRAM(Memory.memory))
										}
									}
									clean_UART_STREAM();
								}
								break;

							case UART_READ_FLASH:
/*
								if (EEPROM_read(FLAG_H_ADDR)&UART_FLASH_MEMORY_READ_ONLY)
								{
									//if (uart_cmd_count < 3) //Fixed 23/07/2015
									if (uart_cmd_count < 2)
									{
										uart_cmd_count++;
										uart_cmd_count_max|=UART_READY_STREAM_DATA;
									}
									else
									{
										//Memory.access.high=uart_command[1];
										//Memory.access.low=uart_command[2];
										//Memory=(uart_command[1]<<8)|(uart_command[2]);
										Memory=to_uint_16(uart_command[1], uart_command[2]);
										//if (Memory.memory > 32767)
										if (Memory > (32767-512))
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_MEMORY_OVF, UART_STRING_FROM_PROGRAM);
											UART_println(ERR_MEMORY_OVF_FLASH, UART_STRING_FROM_PROGRAM);
										}
										else
										{
											UART_Transmit(UART_ACK);
											//UART_Transmit(read_PROGRAM_MEMORY(Memory.memory));
											UART_Transmit(read_PROGRAM_MEMORY(Memory));
											//UART_Transmit_ByteToASCII(read_PROGRAM_MEMORY(Memory.memory));
										}
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[language&0x01], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
								if (uart_cmd_count < 2)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
//								if (EEPROM_read(FLAG_H_ADDR)&UART_FLASH_MEMORY_READ_ONLY)
									if (validate_uart_access(FLAG_H_ADDR, UART_FLASH_MEMORY_READ_ONLY))
									{
										//Memory.access.high=uart_command[1];
										//Memory.access.low=uart_command[2];
										//Memory=(uart_command[1]<<8)|(uart_command[2]);
										Memory=to_uint_16(uart_command[1], uart_command[2]);
										//if (Memory.memory > 32767)
										if (Memory > (32767-512))
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_MEMORY_OVF, UART_STRING_FROM_PROGRAM);
											UART_println(ERR_MEMORY_OVF_FLASH, UART_STRING_FROM_PROGRAM|SEND_EOT);
										}
										else
										{
											UART_Transmit(UART_ACK);
											//UART_Transmit(read_PROGRAM_MEMORY(Memory.memory));
											UART_Transmit(read_PROGRAM_MEMORY(Memory));
											//UART_Transmit_ByteToASCII(read_PROGRAM_MEMORY(Memory.memory));
										}
									}
									clean_UART_STREAM();
								}
								break;

							case UART_READ_PIN:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_PIN_READ_ONLY)
								{
									//if (uart_cmd_count<1)
									//{
									//	uart_cmd_count++;
									//	uart_cmd_count_max|=UART_READY_STREAM_DATA;
									//}
									//else
									//{
										UART_Transmit(UART_ACK);
										UART_Transmit(pseudo_latch_pinos);
										//UART_Transmit_ByteToASCII(pseudo_latch_pinos);
										clean_UART_STREAM();
									//}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
								//if (EEPROM_read(FLAG_L_ADDR)&UART_PIN_READ_ONLY)
								if (validate_uart_access(FLAG_L_ADDR, UART_PIN_READ_ONLY))
								{
									UART_Transmit(UART_ACK);
									UART_Transmit(pseudo_latch_pinos);
								}
								clean_UART_STREAM();
								break;

							case UART_I2C_TRANSMIT_SEND_STOP:
/*
								if (EEPROM_read(FLAG_H_ADDR)&UART_TWO_WIRE_ACCESS)
								{
									TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
									UART_Transmit(UART_ACK);
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
								}
*/
								//if (EEPROM_read(FLAG_H_ADDR)&UART_TWO_WIRE_ACCESS)
								if (validate_uart_access(FLAG_H_ADDR, UART_TWO_WIRE_ACCESS))
								{
									TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
									UART_Transmit(UART_ACK);
								}
								rtc_disable=0x00;
								clean_UART_STREAM();
								break;

							case UART_I2C_TRANSMIT_SEND_START:
/*
								if (EEPROM_read(FLAG_H_ADDR)&UART_TWO_WIRE_ACCESS)
								{
									rtc_disable=0x01;
									TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
									check_twcr();
									UART_Transmit(UART_ACK);
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									rtc_disable=0x00;
								}
*/
								//if (EEPROM_read(FLAG_H_ADDR)&UART_TWO_WIRE_ACCESS)
								if (validate_uart_access(FLAG_H_ADDR, UART_TWO_WIRE_ACCESS))
								{
									rtc_disable=0x01;
									TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
									check_twcr();
									UART_Transmit(UART_ACK);
								}
								else
									rtc_disable=0x00;
								clean_UART_STREAM();
								break;

							case UART_I2C_TRANSMIT_SEND_CMD:
/*
								if (EEPROM_read(FLAG_H_ADDR)&UART_TWO_WIRE_ACCESS)
								{
									if (uart_cmd_count<1)
									{
										uart_cmd_count++;
										uart_cmd_count_max|=UART_READY_STREAM_DATA;
									}
									else
									{
										UART_Transmit(UART_ACK);
										send_cmd(uart_command[0x01]);
										clean_UART_STREAM();
									}
									rtc_disable=0x01;
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									rtc_disable=0x00;
								}
*/
								//if (EEPROM_read(FLAG_H_ADDR)&UART_TWO_WIRE_ACCESS)

								if (uart_cmd_count<1)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									if (validate_uart_access(FLAG_H_ADDR, UART_TWO_WIRE_ACCESS))
									{
										rtc_disable=0x01;
										UART_Transmit(UART_ACK);
										send_cmd(uart_command[0x01]);
									}
									else
										rtc_disable=0x00;
									clean_UART_STREAM();
								}
								break;

							case UART_I2C_RECEIVE_DATA:
/*
								if (EEPROM_read(FLAG_H_ADDR)&UART_TWO_WIRE_ACCESS)
								{
									rtc_disable=0x01;
									UART_Transmit(UART_ACK);
									UART_Transmit(receive_data());
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									rtc_disable=0x00;
								}
*/
//								if (EEPROM_read(FLAG_H_ADDR)&UART_TWO_WIRE_ACCESS)
								if (validate_uart_access(FLAG_H_ADDR, UART_TWO_WIRE_ACCESS))
								{
									rtc_disable=0x01;
									UART_Transmit(UART_ACK);
									UART_Transmit(receive_data());
								}
								else
									rtc_disable=0x00;
								clean_UART_STREAM();
								break;

							case UART_SEND_AUTHOR:

								UART_Transmit(UART_ACK);
								UART_println(AUTHOR_NAME, UART_STRING_FROM_PROGRAM|SEND_EOT);
								clean_UART_STREAM();

								break;

							case UART_SEND_AUTHOR_EMAIL:

								UART_Transmit(UART_ACK);
								UART_println(AUTHOR_EMAIL, UART_STRING_FROM_PROGRAM|SEND_EOT);
								clean_UART_STREAM();

								break;

							case UART_SEND_AUTHOR_USER:

								UART_Transmit(UART_ACK);
								UART_println(USER, UART_STRING_FROM_PROGRAM|SEND_EOT);
								clean_UART_STREAM();

								break;

							case UART_READ_SENSOR:
/*
								if (EEPROM_read(FLAG_H_ADDR)&UART_READ_SENSOR_ENABLE)
								{
									if (uart_cmd_count<1)
									{
										uart_cmd_count++;
										uart_cmd_count_max|=UART_READY_STREAM_DATA;
									}
									else
									{
										//tmp[0]=UART_NACK;
	
										switch (uart_command[1])
										{
											case UART_SENSOR_T1:
												UART_println(dtostrf(t1,1,1, tmp), UART_STRING_FROM_SRAM);
												UART_Transmit(UART_ACK);
	//											tmp[0]=UART_ACK;
												break;
											case UART_SENSOR_T2:
												UART_println(dtostrf(t2,1,1, tmp), UART_STRING_FROM_SRAM);
												UART_Transmit(UART_ACK);
	//											tmp[0]=UART_ACK;
												break;

											case UART_SENSOR_B1:
												UART_println(dtostrf(b1,1,1, tmp), UART_STRING_FROM_SRAM);
												UART_Transmit(UART_ACK);
												//tmp[0]=UART_ACK;
												break;

											case UART_SENSOR_B2:
												UART_println(dtostrf(b2,1,1, tmp), UART_STRING_FROM_SRAM);
												UART_Transmit(UART_ACK);
												//tmp[0]=UART_ACK;
												break;
											default:
												UART_Transmit(UART_NACK);
												//tmp[0]=UART_NACK;
										}
										//UART_Transmit(tmp[0]);
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
								if (uart_cmd_count<1)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									//if (EEPROM_read(FLAG_H_ADDR)&UART_READ_SENSOR_ENABLE)
									if (validate_uart_access(FLAG_H_ADDR, UART_READ_SENSOR_ENABLE))
									{
										uart_command[2]=UART_ACK;
										tmp[0]=0x00; // Fixed Null string (08/25/2015)
										switch (uart_command[1])
										{
											//uart_command[2]=UART_ACK; Fatal 08/15/2015
											case UART_SENSOR_T1:
												//UART_Transmit(UART_ACK);
												//UART_println(dtostrf(t1,1,1, tmp), UART_STRING_FROM_SRAM);
												//dtostrf(t1,1,1, tmp);
												if (language&0x80)
													dtostrf(t1, 1, 4, tmp); // Fixed 08/25/2015
												else
													dtostrf(toFarenheit(&t1), 1, 4, tmp);
	//											UART_Transmit(UART_ACK);
	//											tmp[0]=UART_ACK;
												break;
											case UART_SENSOR_T2:
												//UART_Transmit(UART_ACK);
												//UART_println(dtostrf(t2,1,1, tmp), UART_STRING_FROM_SRAM);
												//dtostrf(t2,1,1, tmp);
												if (language&0x80)
													dtostrf(t2,1,4, tmp); // Fixed 08/25/2015
												else
													dtostrf(toFarenheit(&t2), 1, 4, tmp);
	//											UART_Transmit(UART_ACK);
	//											tmp[0]=UART_ACK;
												break;

											case UART_SENSOR_B1:
												//UART_Transmit(UART_ACK);
												//UART_println(dtostrf(b1,1,1, tmp), UART_STRING_FROM_SRAM);
												//dtostrf(b1,1,1, tmp);
												dtostrf(b1,1,4, tmp); // Fixed 08/25/2015
//												UART_Transmit(UART_ACK);
												//tmp[0]=UART_ACK;
												break;

											case UART_SENSOR_B2:
												//UART_Transmit(UART_ACK);
												//UART_println(dtostrf(b2,1,1, tmp), UART_STRING_FROM_SRAM);
												//dtostrf(b2,1,1, tmp);
												dtostrf(b2,1,4, tmp); // Fixed 08/25/2015
//												UART_Transmit(UART_ACK);
												//tmp[0]=UART_ACK;
												break;
											default:
												uart_command[2]=UART_NACK;
												//tmp[0]=UART_NACK;
												//UART_Transmit(UART_NACK);
												//tmp[0]=UART_NACK;
										}
										UART_Transmit(uart_command[2]);
										UART_println(tmp, UART_STRING_FROM_SRAM|SEND_EOT);
									}
									clean_UART_STREAM();
								}
								break;

							case UART_HARDWARE_VERSION:

								UART_Transmit(UART_ACK);
								UART_println(HW_VERSION ,UART_STRING_FROM_PROGRAM|SEND_EOT);
								clean_UART_STREAM();
								break;

							case UART_FIRMWARE_VERSION:
								UART_Transmit(UART_ACK);
								UART_println(FW_VERSION ,UART_STRING_FROM_PROGRAM|SEND_EOT);
								clean_UART_STREAM();
								break;

							case UART_ENTER_LCD_MODE:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								{
									menu=UART_LCD_MODE;
									cr_start=0;
									prepare_lcd();
									clear_display();
									cursor_off();
									print_lcd2_P(LCD_MODE_MSG);
									set_none();
									UART_Transmit(UART_ACK);
									UART_println(LCD_MODE_MSG, UART_STRING_FROM_PROGRAM);

								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
								}
*/
								//if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								if (validate_uart_access(FLAG_L_ADDR, UART_LCD_ENABLE))
								{
									menu=UART_LCD_MODE;
									cr_start=0;
									prepare_lcd();
									clear_display();
									cursor_off();
									print_lcd2_P(LCD_MODE_MSG);
									set_none();
									UART_Transmit(UART_ACK);
									UART_println(LCD_MODE_MSG, UART_STRING_FROM_PROGRAM|SEND_EOT);

								}
								clean_UART_STREAM();
								break;

							case UART_CLEAR_SCREEN:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								{
									if (menu==UART_LCD_MODE)
									{
										prepare_lcd();
										clear_display();
										set_none();
										UART_Transmit(UART_ACK);
									}
									else
										UART_Transmit(UART_NACK);
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
								}
*/
								//if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								if (validate_uart_access(FLAG_L_ADDR, UART_LCD_ENABLE))
								{
									if (menu==UART_LCD_MODE)
									{
										prepare_lcd();
										clear_display();
										set_none();
										UART_Transmit(UART_ACK);
									}
									else
										UART_Transmit(UART_NACK);
								}
								clean_UART_STREAM();
								break;

							case UART_LCD_CLEAR_LINE:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								{
									if (menu==UART_LCD_MODE)
									{
										if (uart_cmd_count < 1)
										{
											uart_cmd_count++;
											uart_cmd_count_max|=UART_READY_STREAM_DATA;
										}
										else
										{
											// lin
											if ((uart_command[1]<1)||(uart_command[1]>4)) UART_Transmit(UART_NACK);
											else
											{
												prepare_lcd();
												lcd_clear_line(uart_command[1]);
												set_none();
												UART_Transmit(UART_ACK);
											}
											//clear_display();
											clean_UART_STREAM();
										}
									}
									else
									{
										UART_Transmit(UART_NACK);
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
								if (uart_cmd_count < 1)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									// lin
									if (validate_uart_access(FLAG_L_ADDR, UART_LCD_ENABLE))
									{
										if (menu==UART_LCD_MODE)
										{
											if ((uart_command[1]<1)||(uart_command[1]>4)) UART_Transmit(UART_NACK);
											else
											{
												prepare_lcd();
												lcd_clear_line(uart_command[1]);
												set_none();
												UART_Transmit(UART_ACK);
											}
										}
										else
											UART_Transmit(UART_NACK);
									}
									clean_UART_STREAM();
								}
								break;

							case UART_LCD_RETURN_HOME:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								{
									if (menu==UART_LCD_MODE)
									{
										prepare_lcd();
										lcd_return_home();
										set_none();
										UART_Transmit(UART_ACK);
									}
									else
										UART_Transmit(UART_NACK);
									clean_UART_STREAM();
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
//								if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								if (validate_uart_access(FLAG_L_ADDR, UART_LCD_ENABLE))
								{
									if (menu==UART_LCD_MODE)
									{
										prepare_lcd();
										lcd_return_home();
										set_none();
										UART_Transmit(UART_ACK);
									}
									else
										UART_Transmit(UART_NACK);
								}
								clean_UART_STREAM();
								break;

							case UART_EXIT_LCD_MODE:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								{
									if (menu==UART_LCD_MODE)
									{
										menu=0;
										temporizador&=TIMER1_DISABLE;
										cr_start=0;
										prepare_lcd();
										clear_display();
										cursor_off();
										load_custom_char();
										print_lcd2_P(LCD_MODE_OFF_MSG);
										set_none();
									}
									UART_Transmit(UART_ACK);
									UART_println(LCD_MODE_OFF_MSG, UART_STRING_FROM_PROGRAM);
									//}
									//else
									//	UART_Transmit(UART_NACK);
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
								}
*/
								//if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								if (validate_uart_access(FLAG_L_ADDR, UART_LCD_ENABLE))
								{
									if (menu==UART_LCD_MODE)
									{
										menu=0;
										temporizador&=TIMER1_DISABLE;
										cr_start=0;
										code_cr=0x00;
										prepare_lcd();
										clear_display();
										cursor_off();
										load_custom_char();
										print_lcd2_P(LCD_MODE_OFF_MSG);
										set_none();
									}
									UART_Transmit(UART_ACK);
									UART_println(LCD_MODE_OFF_MSG, UART_STRING_FROM_PROGRAM|SEND_EOT);
								}
								clean_UART_STREAM();

								break;

							case UART_SET_CURSOR:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								{
									if (menu==UART_LCD_MODE)
									{

										if (uart_cmd_count<1)
										{
											uart_cmd_count++;
											uart_cmd_count_max|=UART_READY_STREAM_DATA;
										}
										else
										{
											// ccccc|rrr
											prepare_lcd();
											SetCursor(uart_command[1]&0x07, uart_command[1]>>3);
											set_none();
											clean_UART_STREAM();
										}
										UART_Transmit(UART_ACK);
									}
									else
									{
										UART_Transmit(UART_NACK);
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
								if (uart_cmd_count<1)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									if (validate_uart_access(FLAG_L_ADDR, UART_LCD_ENABLE))
									{
										//if (menu==UART_LCD_ENABLE)// Fatal 27/07/2015
										if (menu==UART_LCD_MODE)
										{
/*
											// ccccc|rrr
											prepare_lcd();
											SetCursor(uart_command[1]&0x07, uart_command[1]>>3);
											set_none();
											UART_Transmit(UART_ACK);
*/
											prepare_lcd();
											asm volatile
											(
												"ld r22, X" "\n\t"
												"mov r24, r22" "\n\t"
												"andi r24, 0x07" "\n\t"
												"lsr r22" "\n\t"
												"lsr r22" "\n\t"
												"lsr r22" "\n\t"
												"call asm_SETCURSOR" "\n\t"
											:
											:"x"(&uart_command[1])
											);
											set_none();
											UART_Transmit(UART_ACK);
										}
										else
											UART_Transmit(UART_NACK);
									}
									clean_UART_STREAM();
								}
								break;

							case UART_LCD_CONTROL:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								{
									if (menu==UART_LCD_MODE)
									{
										if (uart_cmd_count<1)
										{
											uart_cmd_count++;
											uart_cmd_count_max|=UART_READY_STREAM_DATA;
										}
										else
										{
											prepare_lcd();
											lcdcmd(LCD_DISPLAYCONTROL|uart_command[1]);
											set_none();
											UART_Transmit(UART_ACK);
											clean_UART_STREAM();
										}

									}
									else
									{
										UART_Transmit(UART_NACK);
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/

								if (uart_cmd_count<1)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									if (validate_uart_access(FLAG_L_ADDR, UART_LCD_ENABLE))
									{
										if (menu==UART_LCD_MODE)
										{
											prepare_lcd();
											lcdcmd(LCD_DISPLAYCONTROL|uart_command[1]);
											set_none();
											UART_Transmit(UART_ACK);
										}
										else
											UART_Transmit(UART_NACK);
									}
									clean_UART_STREAM();
								}

								break;

							case UART_LCD_DATA:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_LCD_ENABLE)
								{
									if (menu==UART_LCD_MODE)
									{
										if (uart_cmd_count<1)
										{
											uart_cmd_count++;
											uart_cmd_count_max|=UART_READY_STREAM_DATA;
										}
										else
										{
											prepare_lcd();
											lcdData(uart_command[1]);
											set_none();
											UART_Transmit(UART_ACK);
											clean_UART_STREAM();
										}

									}
									else
									{
										UART_Transmit(UART_NACK);
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
								if (uart_cmd_count<1)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									if (validate_uart_access(FLAG_L_ADDR, UART_LCD_ENABLE))
									{
										if (menu==UART_LCD_MODE)
										{
											prepare_lcd();
											lcdData(uart_command[1]);
											set_none();
											UART_Transmit(UART_ACK);
										}
										else
											UART_Transmit(UART_NACK);
									}
									clean_UART_STREAM();
								}
								break;

							case UART_WRITE_EEPROM:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_EEPROM_WRITE_ONLY)
								{
									// [cmd] [h address] [l address] [data]
									if (uart_cmd_count<3)
									{
										uart_cmd_count++;
										uart_cmd_count_max|=UART_READY_STREAM_DATA;
									}
									else
									{
										//UART_Transmit(UART_ACK);
										//Memory.access.high=uart_command[1];
										//Memory.access.low=uart_command[2];
										//Memory=(uart_command[1]<<8)|(uart_command[2]);
										Memory=to_uint_16(uart_command[1], uart_command[2]);
										//if (Memory.memory>MAX_EEPROM_MEMORY_ACCESS) UART_Transmit(UART_NACK);
										if (Memory>MAX_EEPROM_MEMORY_ACCESS) UART_Transmit(UART_NACK);
										else
										{
											//EEPROM_write(Memory.memory, uart_command[3]);
											EEPROM_write(Memory, uart_command[3]);
											UART_Transmit(UART_ACK);
										}
										clean_UART_STREAM();

									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
								if (uart_cmd_count<3)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									if (validate_uart_access(FLAG_L_ADDR, UART_EEPROM_WRITE_ONLY))
									{
										//UART_Transmit(UART_ACK);
										//Memory.access.high=uart_command[1];
										//Memory.access.low=uart_command[2];
										//Memory=(uart_command[1]<<8)|(uart_command[2]);
										Memory=to_uint_16(uart_command[1], uart_command[2]);
										//if (Memory.memory>MAX_EEPROM_MEMORY_ACCESS) UART_Transmit(UART_NACK);
										if (Memory>MAX_EEPROM_MEMORY_ACCESS)
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_MEMORY_OVF, UART_STRING_FROM_PROGRAM);
											UART_println(ERR_MEMORY_OVF_EEPROM, UART_STRING_FROM_PROGRAM|SEND_EOT);
										}
										else
										{
											//EEPROM_write(Memory.memory, uart_command[3]);
											EEPROM_write(Memory, uart_command[3]);
											UART_Transmit(UART_ACK);
										}
									}
									clean_UART_STREAM();
								}

								break;

							case UART_WRITE_SRAM:
/*
							//if (EEPROM_read(FLAG_H_ADDR)&UART_EEPROM_WRITE_ONLY) //FATAL Fixed 07/25/2015
								if (EEPROM_read(FLAG_H_ADDR)&UART_SRAM_WRITE_ONLY)
								{
									// [cmd] [h address] [l address] [data]
									if (uart_cmd_count<3)
									{
										uart_cmd_count++;
										uart_cmd_count_max|=UART_READY_STREAM_DATA;
									}
									else
									{
										UART_Transmit(UART_ACK);
										//Memory.access.high=uart_command[1];
										//Memory.access.low=uart_command[2];
										//write_SRAM(Memory.memory, uart_command[3]);
										//Memory=(uart_command[1]<<8)|(uart_command[2]);
										//write_SRAM(Memory, uart_command[3]);

										write_SRAM(to_uint_16(uart_command[1], uart_command[2]), uart_command[3]);
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
								if (uart_cmd_count<3)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									if (validate_uart_access(FLAG_H_ADDR, UART_SRAM_WRITE_ONLY))
									{
										Memory=to_uint_16(uart_command[1], uart_command[2]);
										if (Memory > 2047)
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_MEMORY_OVF, UART_STRING_FROM_PROGRAM);
											UART_println(ERR_MEMORY_OVF_SRAM, UART_STRING_FROM_PROGRAM|SEND_EOT);
										}
										else
										{
											UART_Transmit(UART_ACK);
											//write_SRAM(to_uint_16(uart_command[1], uart_command[2]), uart_command[3]);
											write_SRAM(Memory, uart_command[3]);
										}
									}
									else
										UART_Transmit(UART_NACK);
									clean_UART_STREAM();
								}

								break;

							case UART_WRITE_TIME:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_DATE_TIME_WRITE_ONLY)
								{
									// [cmd] [h1] [h0] [m1] [m0] [s1] [s0] // NOT IN BCD (Old)
									// [cmd] [h] [m] [s] IN BCD (New) (26/08/2015)
								if (uart_cmd_count<6)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
										if (rtc_disable)
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_I2C_BUSY, UART_STRING_FROM_PROGRAM);
										}
										else
										{

//											tmp[0]=(uart_command[1]>='0')&&(uart_command[1]<='2'); //2
//											tmp[1]=(uart_command[2]>='0')&&(uart_command[2]<='3'); //3
//											tmp[2]=(uart_command[3]>='0')&&(uart_command[3]<='5'); //5
//											tmp[3]=(uart_command[4]>='0')&&(uart_command[4]<='9'); //9
//											tmp[4]=(uart_command[5]>='0')&&(uart_command[5]<='5'); //5
//											tmp[5]=(uart_command[6]>='0')&&(uart_command[6]<='9'); //9
//											if (tmp[0]&&tmp[1]&&tmp[2]&&tmp[3]&&tmp[4]&&tmp[5])

											tmp[0]=uart_command[1];
											tmp[1]=uart_command[2];
											tmp[2]=0x00;
											tmp[3]=(unsigned char)atoi(tmp);
											tmp[0]=uart_command[3];
											tmp[1]=uart_command[4];
											tmp[4]=(unsigned char)atoi(tmp);
											tmp[0]=uart_command[5];
											tmp[1]=uart_command[6];
											tmp[5]=(unsigned char)atoi(tmp);
											if (valid_time(tmp[3],tmp[4],tmp[5]))
											{
												write_to_rtc(((uart_command[1]<<4)|(uart_command[2]&0x0F)), RTC_HOURS_REG);
												write_to_rtc(((uart_command[3]<<4)|(uart_command[4]&0x0F)), RTC_MINUTES_REG);
												write_to_rtc(((uart_command[5]<<4)|(uart_command[6]&0x0F)), RTC_SECONDS_REG);
												UART_Transmit(UART_ACK);
											}
											else
											{
												UART_Transmit(UART_NACK);
												UART_println(ERR_TIME2, UART_STRING_FROM_PROGRAM);
											}
										}
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
								if (uart_cmd_count<3)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									if (validate_uart_access(FLAG_L_ADDR, UART_DATE_TIME_WRITE_ONLY))
									{
										if (rtc_disable)
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_I2C_BUSY, UART_STRING_FROM_PROGRAM|SEND_EOT);
										}
										else
										{
/*
											tmp[0]=uart_command[1];
											tmp[1]=uart_command[2];
											tmp[2]=0x00;
											tmp[3]=(unsigned char)atoi(tmp);
											tmp[0]=uart_command[3];
											tmp[1]=uart_command[4];
											tmp[4]=(unsigned char)atoi(tmp);
											tmp[0]=uart_command[5];
											tmp[1]=uart_command[6];
											//tmp[5]=(unsigned char)atoi(tmp);
											//if (valid_time(tmp[3],tmp[4],tmp[5])) // Save space (26/08/2015)
*/
											//if (valid_time(tmp[3],tmp[4],(unsigned char)atoi(tmp)))
											if (valid_time(BCD_to_uint8(uart_command[1]), BCD_to_uint8(uart_command[2]), BCD_to_uint8(uart_command[3])))
											{
/*
												write_to_rtc(((uart_command[1]<<4)|(uart_command[2]&0x0F)), RTC_HOURS_REG);
												write_to_rtc(((uart_command[3]<<4)|(uart_command[4]&0x0F)), RTC_MINUTES_REG);
												write_to_rtc(((uart_command[5]<<4)|(uart_command[6]&0x0F)), RTC_SECONDS_REG);
												UART_Transmit(UART_ACK);
*/
												write_to_rtc(uart_command[1], RTC_HOURS_REG);
												write_to_rtc(uart_command[2], RTC_MINUTES_REG);
												write_to_rtc(uart_command[3], RTC_SECONDS_REG);
												UART_Transmit(UART_ACK);
											}
											else
											{
												UART_Transmit(UART_NACK);
												UART_println(ERR_TIME2, UART_STRING_FROM_PROGRAM|SEND_EOT);
											}
										}
									}
									clean_UART_STREAM();
								}
								//clean_UART_STREAM(); Fatal Fixed 08/27/2015
								break;

							case UART_WRITE_DATE:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_DATE_TIME_WRITE_ONLY)
								{
									// [cmd] [y1] [y0] [m0] [d0] in BCD
									// y1 =[19 or 20], y0=[0..99], m0=[1..12], d0=[0..31]
									if (uart_cmd_count<5)
									{
										uart_cmd_count++;
										uart_cmd_count_max|=UART_READY_STREAM_DATA;
									}
									else
									{
										if (rtc_disable)
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_I2C_BUSY, UART_STRING_FROM_PROGRAM);
										}
										else
										{

											if ((uart_command[1]==0x19)||(uart_command[1]==0x20))
											{
												if (uart_command[1]==0x19) tmp[0]=IS_19XX;
												else tmp[0]=0x00;

												if (is_BCD(uart_command[2]))
												{
													//Memory.memory=(unsigned int)(100*BCD_to_uint8(uart_command[1])+BCD_to_uint8(uart_command[2])); // year
													Memory=(unsigned int)(100*BCD_to_uint8(uart_command[1])+BCD_to_uint8(uart_command[2])); // year
													tmp[1]=BCD_to_uint8(uart_command[3]); // month
													tmp[2]=BCD_to_uint8(uart_command[4]); // day
													//if (valid_date(tmp[2],tmp[1],Memory.memory))
													if (valid_date(tmp[2],tmp[1],Memory))
													{
														write_to_rtc(tmp[0]|uart_command[3],RTC_MONTHS_CENTURY_REG);
														write_to_rtc(uart_command[2], RTC_YEARS_REG);
														write_to_rtc(uart_command[4], RTC_DAYS_REG);
														//write_to_rtc(getDay(Memory.memory, (unsigned int)tmp[3],(unsigned int)tmp[2]), RTC_WEEKDAYS_REG);
														write_to_rtc(getDay(Memory, (unsigned int)tmp[1],(unsigned int)tmp[2]), RTC_WEEKDAYS_REG);
														UART_Transmit(UART_ACK);
													}
													else
													{
														UART_Transmit(UART_NACK);
														UART_println(ERR_INVALID, UART_STRING_FROM_PROGRAM);
														UART_println(ERR_DATE, UART_STRING_FROM_PROGRAM);
													}
												}
												else
												{
													UART_Transmit(UART_NACK);
													UART_println(ERR_BCD_FORMAT, UART_STRING_FROM_PROGRAM);
												}
											}
											else
											{
												UART_Transmit(UART_NACK);
												UART_println(ERR_INVALID, UART_STRING_FROM_PROGRAM);
												UART_println(ERR_YEAR_INTERVAL, UART_STRING_FROM_PROGRAM);
											}

										}
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/

								//if (uart_cmd_count<5)
								if (uart_cmd_count<4) // Fixed (08/26/2015)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									if (validate_uart_access(FLAG_L_ADDR, UART_DATE_TIME_WRITE_ONLY))
									{
										if (rtc_disable)
										{
											UART_Transmit(UART_NACK);
											UART_println(ERR_I2C_BUSY, UART_STRING_FROM_PROGRAM|SEND_EOT);
										}
										else
										{

											if ((uart_command[1]==0x19)||(uart_command[1]==0x20))
											{
												if (uart_command[1]==0x19) tmp[0]=IS_19XX;
												else tmp[0]=0x00;

												if (is_BCD(uart_command[2]))
												{
													//Memory.memory=(unsigned int)(100*BCD_to_uint8(uart_command[1])+BCD_to_uint8(uart_command[2])); // year
													Memory=(unsigned int)(100*BCD_to_uint8(uart_command[1])+BCD_to_uint8(uart_command[2])); // year
													tmp[1]=BCD_to_uint8(uart_command[3]); // month
													tmp[2]=BCD_to_uint8(uart_command[4]); // day
													//if (valid_date(tmp[2],tmp[1],Memory.memory))
													if (valid_date(tmp[2],tmp[1],Memory))
													{
														write_to_rtc(tmp[0]|uart_command[3],RTC_MONTHS_CENTURY_REG);
														write_to_rtc(uart_command[2], RTC_YEARS_REG);
														write_to_rtc(uart_command[4], RTC_DAYS_REG);
														//write_to_rtc(getDay(Memory.memory, (unsigned int)tmp[3],(unsigned int)tmp[2]), RTC_WEEKDAYS_REG);
														write_to_rtc(getDay(Memory, (unsigned int)tmp[1],(unsigned int)tmp[2]), RTC_WEEKDAYS_REG);
														UART_Transmit(UART_ACK);
													}
													else
													{
														UART_Transmit(UART_NACK);
														UART_println(ERR_INVALID, UART_STRING_FROM_PROGRAM);
														UART_println(ERR_DATE, UART_STRING_FROM_PROGRAM|SEND_EOT);
													}
												}
												else
												{
													UART_Transmit(UART_NACK);
													UART_println(ERR_BCD_FORMAT, UART_STRING_FROM_PROGRAM|SEND_EOT);
												}
											}
											else
											{
												UART_Transmit(UART_NACK);
												UART_println(ERR_INVALID, UART_STRING_FROM_PROGRAM);
												UART_println(ERR_YEAR_INTERVAL, UART_STRING_FROM_PROGRAM|SEND_EOT);
											}

										}
									}
									clean_UART_STREAM();
								}
								break;

							case UART_WRITE_PIN:
/*
								if (EEPROM_read(FLAG_L_ADDR)&UART_PIN_WRITE_ONLY)
								{
									if (uart_cmd_count<1)
									{
										uart_cmd_count++;
										uart_cmd_count_max|=UART_READY_STREAM_DATA;
									}
									else
									{
										UART_Transmit(UART_ACK);
										pseudo_latch_pinos=uart_command[1];
										set_latch_pinos();
										clean_UART_STREAM();
									}
								}
								else
								{
									UART_Transmit(UART_NACK);
									UART_println(ACCESS_DENIED[(language&0x01)], UART_STRING_FROM_PROGRAM);
									clean_UART_STREAM();
								}
*/
								if (uart_cmd_count<1)
								{
									uart_cmd_count++;
									uart_cmd_count_max|=UART_READY_STREAM_DATA;
								}
								else
								{
									if (validate_uart_access(FLAG_L_ADDR, UART_PIN_WRITE_ONLY))
									{
										UART_Transmit(UART_ACK);
										pseudo_latch_pinos=uart_command[1];
										set_latch_pinos();
									}
									clean_UART_STREAM();
								}
								break;
							default:
								uart_cmd_count++;
								uart_cmd_count_max|=UART_READY_STREAM_DATA;
								//teste();
						}
					}
				}
			}
			else
			{
				UART_Transmit(UART_NACK);
				UART_println(ERR_INVALID_UART_COMMAND, UART_STRING_FROM_PROGRAM|SEND_EOT);
				clean_UART_STREAM();
			}
		}
		else
		{
			UART_Transmit(UART_NACK);
			UART_println(ERR_UART_DISABLE, UART_STRING_FROM_PROGRAM|SEND_EOT);
			clean_UART_STREAM();
		}
	}
	//UART_flush(); // Fixed 07/23/2015
	uart_cmd_count_max&=~(UART_TIME_OUT);
	//sei();
	// added 02/26/2016
	PORTB|=((UART_LED_TX_OFF)|(UART_LED_RX_OFF));
}

// END INTERRUPT VECTORS

uint32_t load_dword_from_flash(const uint32_t addr[], unsigned char idx)
{
// FIXME I've tried to use pgm_read_dword_near() function but unfortunately it didn't work then I've made this function
// This function works fine !!! If you find any other solution send me an email: fabioegel@gmail.com
	uint32_t __result;
	asm volatile
	(
		"add %2, %2" "\n\t"
		"add %2, %2" "\n\t"
		//"mov r0, %2" "\n\t"
		"add r30, %2" "\n\t"
		"adc r31, r1" "\n\t"
		"lpm r22, Z+" "\n\t"
		"lpm r23, Z+" "\n\t"
		"lpm r24, Z+" "\n\t"
		"lpm r25, Z" "\n\t"

	:"=r"(__result)
	:"z"(addr), "r"(idx)
	);
	//return __result;
}
// FIXME I've tried to use pgm_read_byte_near() function but unfortunately it didn't work then I've made this function
// This function works fine !!! If you find any other solution send me an email: fabioegel@gmail.com
#define load_byte_from_flash(addr, idx) \
(__extension__({                         \
    uint16_t __addr16 = (uint16_t)(addr);   \
    uint8_t __result;                    \
	uint8_t __idx=(uint8_t)(idx);			\
    __asm__                                 \
    (                                       \
		"add r30, %2" "\n\t"				\
		"adc r31, r1" "\n\t"				\
		"lpm %0, Z" "\n\t"					\
		:"=r"(__result)						\
		:"z"(__addr16), "r"(__idx)			\
    );                                      \
    __result;                               \
}))


/*
uint8_t load_byte_from_flash(const uint8_t addr[], unsigned char idx)
{
// FIXME I've tried to use pgm_read_dword_near() function but unfortunately it didn't work then I've made this function
// This function works fine !!! If you find any other solution send me an email: fabioegel@gmail.com
	uint8_t __result;
	asm volatile
	(
		"add r30, %2" "\n\t"
		"adc r31, r1" "\n\t"
		"lpm %0, Z" "\n\t"		
	:"=r"(__result)
	:"z"(addr), "r"(idx)
	);
	return __result;
}
*/
//////////////////////////////
// floating point routines //
////////////////////////////

void lcd_print_float_to_string(double x)
{
	//char conversion[2*BAS_CALC_DIGIT];
	static char conversion[2*BAS_CALC_DIGIT];
	dtostrf(x,1,8, conversion);
	print_lcd2(conversion);
}

// end floating point routines

////////////////////
// MENU ROUTINES //
//////////////////

unsigned char menu_kbd_interval(unsigned char min, unsigned char max)
{
	return ((code_cr > min) && (code_cr < max));
}

// END MENU ROUTINES

//////////////////////
// EEPROM routines //
////////////////////

void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	//cli();
	//unsigned char a=SREG;
	//cli();
	// Wait for completion of previous write
	while(EECR & (1<<EEPE));
	// Set up address and Data Registers 
	EEAR = uiAddress;
	EEDR = ucData;
	// Write logical one to EEMPE
	EECR |= (1<<EEMPE);
	//Start eeprom write by setting EEPE
	EECR |= (1<<EEPE);
	//sei();
	//SREG=a;
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
	// Wait for completion of previous write
	//unsigned char a=SREG;
	//cli();
	while(EECR & (1<<EEPE));
	// Set up address register
	EEAR = uiAddress;
	// Start eeprom read by writing EERE
	EECR |= (1<<EERE);
	// Return data from Data Register
	//SREG=a;
	return EEDR;

}

void readLanguageEEPROM()
{
	// language=(EEPROM_read(0x0000)&0x01);
	// acxxxxxb
	// where x -> don't care
	// b =0 --> Portuguese language. b=1 --> English language
	// a=0 --> °F a=1 --> °C
	// c --> 1- Beep ON 0 --> Beep OFF
	language=(EEPROM_read(0x0000));
}
void writeLanguageEEPROM()
{
	//unsigned char k;
	//k=EEPROM_read(0x0000);
	//k|=language;
	EEPROM_write(0x0000, language);
}

void readAlarmEEPROM()
{
	// in BCD
	hoursA=EEPROM_read(0x0001);
	minutesA=EEPROM_read(0x0002);
}
void writeAlarmEEPROM()
{
	// in BCD
	EEPROM_write(0x0001, hoursA);
	EEPROM_write(0x0002, minutesA);
}

void writePinoEEPROM(struct PINOS_t pino_eeprom, unsigned char address)
{

	EEPROM_write(address+1,pino_eeprom.command);
	EEPROM_write(address+2, pino_eeprom.hh1);
	EEPROM_write(address+3, pino_eeprom.mm1);
	EEPROM_write(address+4, pino_eeprom.hh2);
	EEPROM_write(address+5, pino_eeprom.mm2);
	EEPROM_write(address+6, pino_eeprom.hh3);
	EEPROM_write(address+7, pino_eeprom.mm3);
	EEPROM_write(address+8, pino_eeprom.ss3);
	EEPROM_write(address+9, pino_eeprom.hh4);
	EEPROM_write(address+10, pino_eeprom.mm4);
	EEPROM_write(address+11, pino_eeprom.ss4);
	EEPROM_write(address+12, pino_eeprom.hh5);
	EEPROM_write(address+13, pino_eeprom.mm5);
	EEPROM_write(address+14, pino_eeprom.ss5);
	EEPROM_write(address+15, pino_eeprom.opbat1);
	EEPROM_write(address+16, pino_eeprom.opbat2);
	EEPROM_write(address+17, pino_eeprom.gpv);
	EEPROM_write(address+18, pino_eeprom.d);
	EEPROM_write(address+19, pino_eeprom.e);

}
//void readPinoEEPROM(PINOS_t &pino_e, unsigned char address_a)
void readPinoEEPROM(struct PINOS_t *pino_e, unsigned char address_a) // C compatible
{
	pino_e->command=EEPROM_read(address_a+1);
	pino_e->hh1=EEPROM_read(address_a+2);
	pino_e->mm1=EEPROM_read(address_a+3);
	pino_e->hh2=EEPROM_read(address_a+4);
	pino_e->mm2=EEPROM_read(address_a+5);
	pino_e->hh3=EEPROM_read(address_a+6);
	pino_e->mm3=EEPROM_read(address_a+7);
	pino_e->ss3=EEPROM_read(address_a+8);
	pino_e->hh4=EEPROM_read(address_a+9);
	pino_e->mm4=EEPROM_read(address_a+10);
	pino_e->ss4=EEPROM_read(address_a+11);
	pino_e->hh5=EEPROM_read(address_a+12);
	pino_e->mm5=EEPROM_read(address_a+13);
	pino_e->ss5=EEPROM_read(address_a+14);
	pino_e->opbat1=EEPROM_read(address_a+15);
	pino_e->opbat2=EEPROM_read(address_a+16);
	pino_e->gpv=EEPROM_read(address_a+17);
	pino_e->d=EEPROM_read(address_a+18);
	pino_e->e=EEPROM_read(address_a+19);

//
	n=(pino_e->command&0x07);
	if (n==0x02) pino_e->countdown1=(3600*pino_e->hh3+60*pino_e->mm3+pino_e->ss3);
	else if (n==0x03)
	{
		pino_e->countdown2=0x00;
		pino_e->countdown3=0x00;
	}
//
}

void read_password_EEPROM(unsigned char passwd_a[])
{
	unsigned char tmp_d[HASH_BLOCK];
	tmp_d[0]=EEPROM_read(PASSWORD_ADDRESS_OFFSET);
	tmp_d[1]=EEPROM_read(PASSWORD_ADDRESS_OFFSET+1);
	tmp_d[2]=EEPROM_read(PASSWORD_ADDRESS_OFFSET+2);
	tmp_d[3]=EEPROM_read(PASSWORD_ADDRESS_OFFSET+3);
	memcpy(passwd_a, tmp_d, sizeof(tmp_d));
}

void write_password_EEPROM(unsigned char passwd_b[])
{
	/*
00001818 <_Z21write_password_EEPROMPh>:
    1818:	cf 93       	push	r28
    181a:	df 93       	push	r29
    181c:	ec 01       	movw	r28, r24
    181e:	89 ef       	ldi	r24, 0xF9	; 249
    1820:	93 e0       	ldi	r25, 0x03	; 3
    1822:	68 81       	ld	r22, Y
    1824:	0e 94 a1 0a 	call	0x1542	; 0x1542 <_Z12EEPROM_writejh>
    1828:	8a ef       	ldi	r24, 0xFA	; 250
    182a:	93 e0       	ldi	r25, 0x03	; 3
    182c:	69 81       	ldd	r22, Y+1	; 0x01
    182e:	0e 94 a1 0a 	call	0x1542	; 0x1542 <_Z12EEPROM_writejh>
    1832:	8b ef       	ldi	r24, 0xFB	; 251
    1834:	93 e0       	ldi	r25, 0x03	; 3
    1836:	6a 81       	ldd	r22, Y+2	; 0x02
    1838:	0e 94 a1 0a 	call	0x1542	; 0x1542 <_Z12EEPROM_writejh>
    183c:	8c ef       	ldi	r24, 0xFC	; 252
    183e:	93 e0       	ldi	r25, 0x03	; 3
    1840:	6b 81       	ldd	r22, Y+3	; 0x03
    1842:	0e 94 a1 0a 	call	0x1542	; 0x1542 <_Z12EEPROM_writejh>
    1846:	df 91       	pop	r29
    1848:	cf 91       	pop	r28
    184a:	08 95       	ret

	EEPROM_write(PASSWORD_ADDRESS_OFFSET, passwd_b[0]);
	EEPROM_write(PASSWORD_ADDRESS_OFFSET+1, passwd_b[1]);
	EEPROM_write(PASSWORD_ADDRESS_OFFSET+2, passwd_b[2]);
	EEPROM_write(PASSWORD_ADDRESS_OFFSET+3, passwd_b[3]);
	*/
	// 52 Bytes
	const unsigned char HI=(PASSWORD_ADDRESS_OFFSET>>8), LO=(PASSWORD_ADDRESS_OFFSET&0x00FF);
	asm volatile
	(
		"movw r24, %1" "\n\t"
		"subi %1, (~(0x04)+1)" "\n\t" // Working only with low byte. WARNING! ONLY if address is in interval (3FB-300)
	"WRITE_PASSWORD_VOLTAR:" "\n\t"
		"ld r22, X+" "\n\t"
		"call asm_EEPROM_WRITE" "\n\t"
		"inc r24" "\n\t" // Working only with low byte. WARNING !!! ONLY if address is in interval (3FB-300)
		"cpse r24, %1" "\n\t"
		"rjmp WRITE_PASSWORD_VOLTAR" "\n\t"
	:
	:"x"(passwd_b), "r"(LO), "r"(HI)
	);
}
// End EEPROM routines

// Code-Encoding routines

unsigned char is_BCD(unsigned char value)
{
/*
00001240 <asm_IS_BCD>:
    1240:	90 e0       	ldi	r25, 0x00	; 0
    1242:	9c 01       	movw	r18, r24
    1244:	2f 70       	andi	r18, 0x0F	; 15
    1246:	30 70       	andi	r19, 0x00	; 0
    1248:	2a 30       	cpi	r18, 0x0A	; 10
    124a:	31 05       	cpc	r19, r1
    124c:	4c f4       	brge	.+18     	; 0x1260 <asm_IS_BCD+0x20>
    124e:	9c 01       	movw	r18, r24
    1250:	20 7f       	andi	r18, 0xF0	; 240
    1252:	30 70       	andi	r19, 0x00	; 0
    1254:	81 e0       	ldi	r24, 0x01	; 1
    1256:	20 3a       	cpi	r18, 0xA0	; 160
    1258:	31 05       	cpc	r19, r1
    125a:	1c f0       	brlt	.+6      	; 0x1262 <asm_IS_BCD+0x22>
    125c:	80 e0       	ldi	r24, 0x00	; 0
    125e:	08 95       	ret
    1260:	80 e0       	ldi	r24, 0x00	; 0
    1262:	08 95       	ret

	if ((value&0x0F)<0x0A)
		return ((value&0xF0)<0xA0);
	else
		return 0;
*/

	asm volatile
	(
		"mov r0, %0" "\n\t"
		"andi %0, 0x0F" "\n\t"
		"cpi %0, 0x0A" "\n\t"
		"brge erro_is_BCD" "\n\t"
		"mov %0, r0" "\n\t"
		"swap %0" "\n\t" // to become signal is always positive
		"andi %0, 0x0F" "\n\t" // and and signal is positive
		"cpi %0, 0x0A" "\n\t"
		"brge erro_is_BCD" "\n\t"
		"ldi %0, 0x01" "\n\t"
		"ret" "\n\t"
	"erro_is_BCD:" "\n\t"
		"ldi %0, 0x00" "\n\t"
		:"=r"(value)
	);
}
unsigned char BCD_to_uint8(unsigned char valuea)
{
/*

00001264 <_Z12BCD_to_uint8h>:
    1264:	cf 93       	push	r28
    1266:	c8 2f       	mov	r28, r24
    1268:	0e 94 20 09 	call	0x1240	; 0x1240 <_Z6is_BCDh>
    126c:	88 23       	and	r24, r24
    126e:	59 f0       	breq	.+22     	; 0x1286 <_Z12BCD_to_uint8h+0x22>
    1270:	9c 2f       	mov	r25, r28
    1272:	92 95       	swap	r25
    1274:	9f 70       	andi	r25, 0x0F	; 15
    1276:	99 0f       	add	r25, r25
    1278:	89 2f       	mov	r24, r25
    127a:	88 0f       	add	r24, r24
    127c:	88 0f       	add	r24, r24
    127e:	89 0f       	add	r24, r25
    1280:	cf 70       	andi	r28, 0x0F	; 15
    1282:	8c 0f       	add	r24, r28
    1284:	01 c0       	rjmp	.+2      	; 0x1288 <_Z12BCD_to_uint8h+0x24>
    1286:	8f ef       	ldi	r24, 0xFF	; 255
    1288:	cf 91       	pop	r28
    128a:	08 95       	ret



	if (is_BCD(valuea))
		return ((valuea>>4)*10+(valuea&0x0F));
	else
		return 0xFF;
*/
	asm volatile(
		"mov r25, %0" "\n\t"
		"call asm_IS_BCD" "\n\t"
		"tst %0" "\n\t"
		"breq erro" "\n\t"
		"mov %0, r25" "\n\t"
		"swap r25" "\n\t"
		"andi r25, 0x0F" "\n\t"
		"mov r0, r25" "\n\t"
		"ldi r25, 0x0A" "\n\t"
		"mul r0, r25" "\n\t"
		"andi %0, 0x0F" "\n\t"
		"add %0, r0" "\n\t"
		// "clr r1" "\n\t" // I don't need it r1 is always 0 in mul operation
		//"rjmp sair" "\n\t"
		"ret" "\n\t"
		"erro:"
		//"ldi %0, 0xFF" "\n\t" // Fatal Fixed (08/27/2015)
		"ldi %0, 0x7F" "\n\t"
	:"=r"(valuea));
//PS: I don't need to clear r1 because max value of MAX(mul r0, r25)= 99 (0x0063), r1 is always 0. Then I save one clock cycle and 2 bytes in program memory
}

char *strupr_b(char *s)
{
/*
0000619e <strupr>:
    619e:	dc 01       	movw	r26, r24
    61a0:	6c 91       	ld	r22, X
    61a2:	61 56       	subi	r22, 0x61	; 97
    61a4:	6a 31       	cpi	r22, 0x1A	; 26
    61a6:	08 f0       	brcs	.+2      	; 0x61aa <strupr+0xc>
    61a8:	60 5e       	subi	r22, 0xE0	; 224
    61aa:	6f 5b       	subi	r22, 0xBF	; 191
    61ac:	6d 93       	st	X+, r22
    61ae:	c1 f7       	brne	.-16     	; 0x61a0 <strupr+0x2>
    61b0:	08 95       	ret

*/

	asm volatile(
		"push %0"				"\n\t"
		"movw r26, %0" 		"\n\t"
	"strupr_b_voltar:" 			"\n\t"
		"ld r22, X" 			"\n\t"
		"mov %0, r22"			"\n\t"
		"andi %0, 0xF0" 		"\n\t"
		"breq strupr_b_lcd_char" "\n\t"
		"subi r22, 0x61" 		"\n\t"
		"cpi r22, 0x1A" 		"\n\t"
		"brcs strupr_b_1" 		"\n\t"
		"subi r22, 0xE0" 		"\n\t"
	"strupr_b_1:" 				"\n\t"
		"subi r22, 0xBF" 		"\n\t"
	"strupr_b_2:"				"\n\t"
		"st X+, r22" 			"\n\t"
		"rjmp strupr_b_voltar" 	"\n\t"
	"strupr_b_lcd_char:"		"\n\t"
		"tst r22"				"\n\t"
		"breq strupr_b_sair"	"\n\t"
		"dec r22"				"\n\t"
		"ori r22, 0x01"		"\n\t"
		"inc r22"				"\n\t"
		"rjmp strupr_b_2"		"\n\t"
	"strupr_b_sair:"			"\n\t"
		"pop %0"				"\n\t"
	:"=r"(s)
	);
}
// End Code-Encoding routines


////////////////////////////
// input sensor routines //
//////////////////////////

void read_sensor(unsigned char sensor)
{
	ADMUX=sensor;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));

	if (sensor==TEMP1_SENSOR)
	{
		t1=(double)ADCW;
		//k1=0.3016,to1=30.2138
		//t1*=k1;
		//t1-=to1;
		t1*=(-0.4296875);
		//t1+=260.2;
		t1+=249.4;
	}
	else if (sensor==TEMP2_SENSOR)
	{
		t2=(double)ADCW;
		//t2*=1.011950579;
		t2*=1.01036376;
		//t2-=270.3348382;
		t2-=323.3348382; //adjust
	}
	else if (sensor==B1_SENSOR)
	{
		b1=(double)ADCW;
		b1*=0.0015644027;
	}
	else if (sensor==B2_SENSOR)
	{
		b2=(double)ADCW;
		b2*=0.0015644027;
	}
}
void read_all_sensor()
{
	read_sensor(TEMP1_SENSOR);
	read_sensor(TEMP2_SENSOR);
	read_sensor(B1_SENSOR);
	read_sensor(B2_SENSOR);
}

void clean_UART_STREAM()
{
	uart_command[0]=0x00;
	uart_cmd_count=0x00;
	uart_cmd_count_max=0x00;
}

void decodificar()
{

	//static unsigned char i,j=0;
	static unsigned char i;
	/*
	static const uint32_t CONV2[] PROGMEM={
				0x332b54c0,0x332cd53e,0x33332ac4,0x332b2ac2,0x3334d540,0x332ad540,
				0x3332d542, 0x334ad53e,0x334d2abe,0x3354ad3e,0x335352c2,0x3354ab40,
				0x332cab40,0x332d2ac0,0x332b52c2,0x3352acbe,0x3354b4be,
				0x3FD6EE, 0x3FED5E, 0x3FDE9E, 0x3FDCBE, 0x3FEE6F};
	*/
	static const uint32_t CONV2[] PROGMEM={
				0x3fe40, 0x3fee0, 0x3ffe0, 0x3fe60, 0x3ff60, 0x3fe20, 0x3ffa0, 0x3fd22, 0x3fda2, 0x3fc4a,
				0x3fcca, 0x3fc02, 0x3fec0, 0x3fea0, 0x3fe48, 0x3fc82, 0x3fc0a, 0x3fcaa, 0x3fec8, 0x3fed2,
				0x3fdc2, 0x3fd62};
	static const uint8_t CR_KEY[] PROGMEM={
					'1','2','3','4','5','6','7','8','9','0',
					UP_PLUS_KEY, DOWN_MINUS_KEY,RIGHT_MULTIPLY_KEY, LEFT_DIVIDE_KEY,
					ENTER_KEY, PLUS_MINUS_KEY, DOT_KEY, MENU_KEY, RETURN_KEY, CLEAR_KEY, MF_KEY, BEEP_ON_OFF};

	// added to UART for IoT
	if (cr_start==2)
	{
		if (code_cr==0x00)
		{
			for (i=0; i<sizeof(CR_KEY);i++)
			{
				if ((load_dword_from_flash(CONV2,i))==(key))
				{
					//code_cr=CR_KEY[i];
					//code_cr=pgm_read_byte_near(CR_KEY+i);
					code_cr=load_byte_from_flash(CR_KEY, i);
					break;
				}
			}
		}
		if (code_cr==0x00) code_cr=0x01;
	}

}
void set_none()
{
	unsigned char key_latch_none;
	_delay_us(150);
	//n=(PINB & (1<<PB0)); // evita interrupcao
	//PORTB=(SELECT_KBD_OR_NONE | n);
	//key_latch_none=(PINB & (1<<PB0));
	key_latch_none=(PINB & ((1<<PB0)|(UART_LED_TX_OFF)|(UART_LED_RX_OFF))); //added 02/26/2016
	PORTB=(SELECT_KBD_OR_NONE|key_latch_none);
	_delay_us(150);

}
void set_latch_aux()
{
	unsigned char key_latch_aux;
	PORTD=pseudo_latch_aux;
	set_none();
	//n=(PINB & (1<<PB0));
	//PORTB=(SELECT_AUX_REG | n);
	//key_latch_aux=(PINB & (1<<PB0));
	key_latch_aux=(PINB & ((1<<PB0)|(UART_LED_TX_OFF)|(UART_LED_RX_OFF))); //added 02/26/2016
	PORTB=(SELECT_AUX_REG|key_latch_aux);
	set_none();
}
void set_latch_pinos()
{
	unsigned char key_latch_pinos;
	PORTD=pseudo_latch_pinos;
	set_none();
	//n=(PINB & (1<<PB0));
	//key_latch_pinos=(PINB & (1<<PB0));
	//PORTB=(SELECT_PINOS | n);
	key_latch_pinos=(PINB & ((1<<PB0)|(UART_LED_TX_OFF)|(UART_LED_RX_OFF))); //added 02/26/2016
	PORTB=(SELECT_PINOS | key_latch_pinos);
	set_none();
}
unsigned char check_pinc()
{
	// check if pinc3 input is active: 0 active, 1 non active
	_delay_us(5);
	return ((PINC & (1<<PC3))==0);
}
void read_kbd()
{
	// uses pseudo_latch_aux bit 1 (not used in output latch) as flag and n for sweep keyboard
	// This function should be called after decodificar() function

	//if (cr_start==0) fixed
	if (code_cr<0x02)
		if (!(pseudo_latch_aux & IS_KBD_KEY_PRESSED))
		{
			set_none();
			for (n=0; n<22; n++)
			{
				PORTD=(n<<2);
				if (check_pinc())
				{
					code_cr=n+0x30;
					tmp_gpr=n;
					cr_start=0x02;
					pseudo_latch_aux|=IS_KBD_KEY_PRESSED;
					break;
				}
			}
		}
	/*
	n=pseudo_latch_aux & IS_KBD_KEY_PRESSED;
	if ((!(n)) && (code_cr==0xFF))
	{
		set_none();
		for (n=0; n<22; n++)
		{
			PORTD=(n<<2);
			if (check_pinc())
			{
				code_cr=n+0x30;
				tmp_gpr=n;
				cr_start=0x02;
				pseudo_latch_aux|=IS_KBD_KEY_PRESSED;
				break;
			}
		}
	}
	*/
}
void insert_input(unsigned char exception)
{
	unsigned char a=(exception&0x01);
	//if (((code_cr>='0') && (code_cr<='9'))||(exception))
	if (((code_cr>='0') && (code_cr<='9'))||(a))
	{
		if (pos_input < (MAX_INPUT-3))
		{
//			if (exception)
			if (a)
				input[pos_input]='-';
			else
				input[pos_input]=code_cr;
			pos_input++;
			input[pos_input]='\0';
		}
		prepare_lcd();
		SetCursor(3,1);
		if (exception&0x02)
			replicate_char('*',pos_input);
		else
			print_lcd2(input);
		SetCursor(3,1);
		set_none();
	}
}

void fill_null_char(const uint16_t address, unsigned char size)
{
	asm volatile
	(
		"clr r25" "\n\t"
		"mov r24, %1" "\n\t"
		"add r24, r26" "\n\t"
		"adc r25, r27" "\n\t"
		//"adc r25, r1" "\n\t"
	"FILL_NULL_CHAR_VOLTAR:"
		"st X+, r1" "\n\t"
		"cp r26, r24" "\n\t"
		"cpc r27, r25" "\n\t"
//		"cpc r27, r1" "\n\t"
		"brne FILL_NULL_CHAR_VOLTAR"
	:
	:"x"(address), "r"(size)
	);
}

void sub_clear_input()
{
	lcd_clear_line(4);
	lcd_clear_line(3);
}
void clear_input(unsigned char clear_display)
{
	char i;
	//DT_t *DT_a;
/*
	DT.year[0]='\0';
	DT.year[4]='\0';
	DT.month[0]='\0';
	DT.month[4]='\0';
	DT.day[0]='\0';
	DT.day[2]='\0';
	DT.hh[0]='\0';
	DT.hh[2]='\0';
	DT.mm[0]='\0';
	DT.mm[2]='\0';
	DT.ss[0]='\0';
	DT.ss[2]='\0';
*/
	fill_null_char((uint16_t)(&DT),sizeof(DT));
	pos_input=0;

	//for (i=0;i<MAX_INPUT; i++) input[i]=0x00;
	fill_null_char((uint16_t)(&input), sizeof(input));
	if (clear_display)
	{
		prepare_lcd();
		//lcd_clear_line(4);
		//lcd_clear_line(3);
		sub_clear_input();
		set_none();
	}
}
void beepAlarm()
{
/*
	set_none();
	pseudo_latch_aux|=SPK_ON;
	set_latch_aux();
	_delay_ms(200);
	pseudo_latch_aux&=~SPK_ON;
	set_latch_aux();
*/
	set_none();
	pseudo_latch_aux|=SPK_ON;
	set_latch_aux();
	turn_on_timer_2(0x0C);//196 ms
}
void beep()
{
	//if (pseudo_latch_aux&0x02) beepAlarm();
	if (language&BEEP_ENABLE) beepAlarm();
}

void prepare_lcd()
{
	unsigned char key_prepare_lcd;
	set_none();
	PORTD=0x00;
	_delay_us(150);
	//n=(PINB & (1<<PB0));
	//PORTB=(SELECT_LCD | n);
	//key_prepare_lcd=(PINB & (1<<PB0));
	key_prepare_lcd=(PINB & ((1<<PB0)|(UART_LED_TX_OFF)|(UART_LED_RX_OFF))); //added 02/26/2016
	PORTB=(SELECT_LCD | key_prepare_lcd);
	_delay_us(150);
}

/////////////////////////////
// Date and time routines //
///////////////////////////

void print_time_date()
{
	lcdData((n>>4)+0x30);
	lcdData((n&0x0F)+0x30);
}

void get_week_day(unsigned char value)
{
// Get months from value 1-7 means 1-sun , 2-mon ... and so on
// value is stored to (conv) variable
/*	char week[3][6];
	if (language)
	{
		strcpy(week[0], "Sun");
		strcpy(week[1], "Mon");
		strcpy(week[2], "Tue");
		strcpy(week[3], "Wed");
		strcpy(week[4], "Thu");
		strcpy(week[5], "Fri");
		strcpy(week[6], "Sat");

	}
	else
	{
		strcpy(week[0], "Dom");
		strcpy(week[1], "Seg");
		strcpy(week[2], "Ter");
		strcpy(week[3], "Qua");
		strcpy(week[4], "Qui");
		strcpy(week[5], "Sex");
		strcpy(week[6], "Sab");
	}

	strcpy(conv, week[(value&0x07)]);
*/

	if (language&0x01)
	{
		/*
		switch (value & 0x07)
		{
			case 0: strcpy(conv,"Sun");break;
			case 1: strcpy(conv, "Mon");break;
			case 2: strcpy(conv, "Tue");break;
			case 3: strcpy(conv, "Wed");break;
			case 4: strcpy(conv, "Thu");break;
			case 5: strcpy(conv, "Fri");break;
			case 6: strcpy(conv, "Sat");
		}
		*/
		strcpy_P(conv,WEEKDAY_EN[(value&0x07)]);

	}
	else
	{
		/*
		switch (value & 0x07)
		{
			case 0: strcpy(conv,"Dom");break;
			case 1: strcpy(conv, "Seg");break;
			case 2: strcpy(conv, "Ter");break;
			case 3: strcpy(conv, "Qua");break;
			case 4: strcpy(conv, "Qui");break;
			case 5: strcpy(conv, "Sex");break;
			case 6: strcpy(conv, "Sab");
		}
		*/
		strcpy_P(conv,WEEKDAY_BR[(value&0x07)]);
	}
}

void get_month_day(unsigned char val)
{

	// Added to FIX FATAL @ 10/01/2015

	unsigned char f=BCD_to_uint8(val&0x1F)-1;

	(language&0x01)?strcpy_P(conv, MONTH_EN[f]):strcpy_P(conv, MONTH_BR[f]);

}

int getDay(int y, int m, int d)
{
    int weekday  = (d += m < 3 ? y-- : y - 2 , 23 * m / 9 + d + 4 + y / 4 - y / 100 + y / 400) % 7;
    return weekday;
//	(d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7  Sakamoto's formula
}

unsigned char is_common_year(unsigned int year)
{
/*
// 0 is leap year
// 1 is common year
	if (year%4)
		return 1;
	else
		if (year%100) return 0;
		else if (year%400) return 1;
		else return 0;
*/

	asm volatile
	(
		"movw r18, %0" "\n\t"
		"andi %0, 0x03" "\n\t"
		"brne IS_COMMOM_YEAR" "\n\t"
		"movw %0, r18" "\n\t"
		"ldi r22, 0x64" "\n\t"
		"ldi r23, 0x00" "\n\t"
		"call __udivmodhi4" "\n\t"
		"adiw %0, 0x00" "\n\t"
		"brne IS_LEAP_YEAR" "\n\t"
		"movw %0, r18" "\n\t"
		"ldi r22, 0x90" "\n\t"
		"ldi r23, 0x01" "\n\t"
		"call __udivmodhi4" "\n\t"
		"adiw %0, 0x00" "\n\t"
		"brne IS_COMMOM_YEAR" "\n\t"
	"IS_LEAP_YEAR:" "\n\t"
		"ldi %0, 0x00" "\n\t"
		"ret" "\n\t"
	"IS_COMMOM_YEAR:" "\n\t"
		"ldi r24, 0x01" "\n\t"
	:
	:"r"(year)
	);
}

unsigned char valid_date(unsigned char day, unsigned char month, unsigned int year)
{
	unsigned char k_day=0;
/*
 FIXME please debug it. I am analyzing the behavior of passing year. I'm not sure if the algorithm below is correct.
 if you find a bug, please correct and send me a message at fabioegel@gmail.com
*/
	// Sakamoto's formula is valid only 30/09/1752 and above
	if ((year > 1752)&&(month>0))
	{
		//if ((month < 8)&&(month>0))
		if (month < 8)
		{
			if (month==2)
			{
				if (is_common_year(year)) k_day = 28;
				else k_day=29;
			}
			else if (month&0x01) k_day=31;
			else k_day=30;
		}
		//else if (month>0)
		else if (month < 13)
		{
			if (month&0x01) k_day=30;
			else k_day=31;
		}
	}

	if (day <= k_day) return day;
	else return 0;
}
unsigned char valid_time(unsigned char hh, unsigned char mm, unsigned char ss)
{
/*
	return ((hh<24)&&(mm<60)&&(ss<60));
00001d72 <_Z10valid_timehhh>:
    1d72:	88 31       	cpi	r24, 0x18	; 24
    1d74:	28 f4       	brcc	.+10     	; 0x1d80 <_Z10valid_timehhh+0xe>
    1d76:	6c 33       	cpi	r22, 0x3C	; 60
    1d78:	28 f4       	brcc	.+10     	; 0x1d84 <_Z10valid_timehhh+0x12>
    1d7a:	81 e0       	ldi	r24, 0x01	; 1
    1d7c:	4c 33       	cpi	r20, 0x3C	; 60
    1d7e:	18 f0       	brcs	.+6      	; 0x1d86 <_Z10valid_timehhh+0x14>
    1d80:	80 e0       	ldi	r24, 0x00	; 0
    1d82:	08 95       	ret
    1d84:	80 e0       	ldi	r24, 0x00	; 0
    1d86:	08 95       	ret
*/
	unsigned char result=hh;
	asm volatile
	(
		"cpi %0, 0x18" "\n\t"
		"brcc valid_time_ERRO" "\n\t"
		"cpi %1, 0x3C" "\n\t"
		"brcc valid_time_ERRO" "\n\t"
		"ldi %0, 0x01" "\n\t"
		"cpi %2, 0x3C" "\n\t"
		"brcs valid_time_SAIR" "\n\t"
	"valid_time_ERRO:" "\n\t"
		"clr %0" "\n\t"
	"valid_time_SAIR:"
	:"=r"(result)
	:"r"(mm), "r"(ss)
	);
	return result;
}


// end of date and time routines

void about()
{
	char buffer[18];
	menu=ABOUT_MENU;
	prepare_lcd();
	clear_display();
	//print_lcd2("SOBRE (Vers");
	//lcdData(0x01);
	//print_lcd2("o):");
	/*
	if (language)
		strcpy_PF(conv, ABOUT[1]);
		print_lcd2("ABOUT");
	else
		strcpy_PF(conv, ABOUT[0]);
		print_lcd2("SOBRE");
	*/
	strcpy_P(buffer, ABOUT[language&0x01]);
	print_lcd2(buffer);
	SetCursor(2,1);
	strcpy_P(buffer,FW_VERSION);
	//print_lcd2("FW: 1.5a 20140921");
	print_lcd2(buffer);
	SetCursor(3,1);
	strcpy_P(buffer, HW_VERSION);
	print_lcd2(buffer);
	SetCursor(4,1);
	//lcdData('F');
	//lcdData(0x00); // á
	strcpy_P(buffer, MSG_LINUX);
	print_lcd2(buffer);
	//lcdData(0x05); // ;)
	set_none();

}
void show_initial_menu()
{
	//char str_tmp[9][3];
	char str_tmp[4][9];

	if (language&0x01)
	{
		/*
		strcpy(str_tmp[0], "1-Time");
		strcpy(str_tmp[1], "2-Pins");
		strcpy(str_tmp[2], "5-Other");
		strcpy(str_tmp[3], "6-About");
		*/
		strcpy_P(str_tmp[0], STR_MENU[0]);
		strcpy_P(str_tmp[1], STR_MENU[1]);
		strcpy_P(str_tmp[2], STR_MENU[2]);
		strcpy_P(str_tmp[3], STR_MENU[3]);
	}
	else
	{
		/*
		strcpy(str_tmp[0], "1-Hora");
		strcpy(str_tmp[1], "2-Pinos");
		strcpy(str_tmp[2], "5-Outros");
		strcpy(str_tmp[3], "6-Sobre");
		*/
		strcpy_P(str_tmp[0], STR_MENU[4]);
		strcpy_P(str_tmp[1], STR_MENU[5]);
		strcpy_P(str_tmp[2], STR_MENU[6]);
		strcpy_P(str_tmp[3], STR_MENU[7]);
	}
	menu=IS_MENU;
	prepare_lcd();
	clear_display();
	SetCursor(1,4);
	//print_lcd2("*** MENU ***");
	replicate_char('*',3);
	//strcpy_P(conv, STR_MENU_TITLE);
	//print_lcd2(conv);
	print_lcd2_P(STR_MENU_TITLE);
	replicate_char('*',3);
	SetCursor(2,1);
	//print_lcd2("1-Hora");
	print_lcd2(str_tmp[0]);
	SetCursor(2,11);
	//print_lcd2("4-Calc.");
	print_lcd2_P(STR_MENU[9]);
	SetCursor(3,1);
	//print_lcd2("2-Pinos");
	print_lcd2(str_tmp[1]);
	SetCursor(3,11);
	//print_lcd2("5-Outros");
	print_lcd2(str_tmp[2]);
	SetCursor(4,1);
	//print_lcd2("3-Sensor");
	print_lcd2_P(STR_MENU[8]);
	SetCursor(4,11);
	print_lcd2(str_tmp[3]);
	set_none();
}

void show_set_date_time()
{
	//char str_tmp_set_date_time[3][20];
	//char str_tmp_set_date_time[25][4];
	menu=DATE_TIME_MENU;

	clear_input(0);
	prepare_lcd();
	clear_display();
	cursor_off();
	if (language&0x01)
	{
		/*
		strcpy(str_tmp_set_date_time[0], "DATE/TIME ADJUST");
		strcpy(str_tmp_set_date_time[1], "1-Date/time");
		strcpy(str_tmp_set_date_time[2], "2-Weekday");
		strcpy(str_tmp_set_date_time[3], "3-Alarm");
		*/

		//print_lcd2("DATE/TIME ADJUST");
		print_lcd2_P(DATE_TIME_ADJUST_EN);
		SetCursor(2,1);
		//print_lcd2("1-Date/time");
		print_lcd2_P(DATE_TIME_EN);
		SetCursor(3,1);
		//print_lcd2("2-Weekday");
		print_lcd2_P(DATE_TIME_WEEKDAY_EN);
		SetCursor(4,1);
		//print_lcd2("3-Alarm");
		print_lcd2_P(DATE_TIME_ALARM_EN);
	}
	else
	{
		/*
		strcpy(str_tmp_set_date_time[0],"AJUST. DATA E HORA");
		strcpy(str_tmp_set_date_time[1],"1-Data/hora");
		strcpy(str_tmp_set_date_time[2],"2-dia da semana");
		strcpy(str_tmp_set_date_time[3],"3-Alarme");
		*/
		//print_lcd2("AJUST. DATA E HORA");
		print_lcd2_P(DATE_TIME_ADJUST_BR);
		SetCursor(2,1);
		//print_lcd2("1-Data/hora");
		print_lcd2_P(DATE_TIME_BR);
		SetCursor(3,1);
		//print_lcd2("2-Dia da semana");
		print_lcd2_P(DATE_TIME_WEEKDAY_BR);
		SetCursor(4,1);
		//print_lcd2("3-Alarme");
		print_lcd2_P(DATE_TIME_ALARM_BR);
	}
	/*	
	//print_lcd2("AJUST. DATA E HORA:");
	print_lcd2(str_tmp_set_date_time[0]);
	SetCursor(2,1);
	//print_lcd2("1-Data/hora");
	print_lcd2(str_tmp_set_date_time[1]);
	SetCursor(3,1);
	//print_lcd2("2-Dia da semana");
	print_lcd2(str_tmp_set_date_time[2]);
	SetCursor(4,1);
	//print_lcd2("3-Alarme");
	print_lcd2(str_tmp_set_date_time[3]);
	*/
	set_none();
}

void set_time_date()
{

	prepare_lcd();
	clear_display();
	//if (language)
		//print_lcd2("Type");
	//	print_lcd2_P(TYPE_STR_EN);
	//else
		//print_lcd2("Digite");
	//	print_lcd2_P(TYPE_STR_BR);
	print_lcd2_P(TYPE_STR[language&0x01]);
	lcdData(':');
	SetCursor(2,1);
	n=read_from_rtc(RTC_YEARS_REG);
	menu=read_from_rtc(RTC_MONTHS_CENTURY_REG); // uses 'menu' as temporary register WARNING! ITs NOT a GOOD IDEA FOR PROGRAMMERS, but a good idea to save memory

	if (menu&IS_19XX) print_lcd2_P(CENTURY_19XX); //print_lcd2("19");
	else print_lcd2_P(CENTURY_20XX); //print_lcd2("20");
	print_time_date();
	lcdData('/');
	n=menu; // it's faster than code above in comment. I don't need read again i2c.
	menu=DATE_TIME_ADJUST; // YES !!! Now, menu is doing its work.
	n&=0x1F;
	print_time_date();
	lcdData('/');
	n=read_from_rtc(RTC_DAYS_REG);
	n&=0x3F;
	print_time_date();
	lcdData(' ');
	n=read_from_rtc(RTC_HOURS_REG);
	n&=0x3F;
	print_time_date();
	lcdData(':');
	n=read_from_rtc(RTC_MINUTES_REG);
	n&=0x7F;
	print_time_date();
	lcdData(':');
	n=read_from_rtc(RTC_SECONDS_REG);
	n&=0x7F;
	print_time_date();
	SetCursor(3,1);
	cursor_on();
	set_none();

}
void find_weekday()
{
	//char str_tmp_weekday[9];
	menu=DATE_TIME_WEEK_DAY;
	prepare_lcd();
	clear_display();
	if (language&0x01)
		print_lcd2_P(WEEKDAY_MENU_STR_EN);
		//strcpy_P(str_tmp_weekday,TYPE_STR_EN);
	else
		print_lcd2_P(WEEKDAY_MENU_STR_BR);
		//strcpy(str_tmp_weekday,"Digite");
		//strcpy_P(str_tmp_weekday,TYPE_STR_BR);
	SetCursor(2,1);
	//print_lcd2("Digite: yyyymmdd");
	//print_lcd2(str_tmp_weekday);
	print_lcd2_P(TYPE_STR[language&0x01]);
	//print_lcd2(": yyyymmdd");
	//print_lcd2_P(WEEKDAY_MENU_STR_YYYYMMDD);
	print_lcd2(": ");
	replicate_char('y',4);
	replicate_char('m',2);
	replicate_char('d',2);
	SetCursor(3,1);
	cursor_on();
	set_none();
}

unsigned char is_time_date_valid_input(unsigned char wide_year)
{
	// if wide_year is not zero, then computes a wide range date from year 0000 to 9999
	// WARNING I don't know if is valid for Sakamoto's Formula
	// code below is added to is_time_date_valid_input() to save memory

	DT.year[0]=input[0];
	DT.year[1]=input[1];
	DT.year[2]=input[2];
	DT.year[3]=input[3];
	DT.year[4]=0x00;
	for (n=0; n<2; n++)
	{
		DT.month[n]=input[(n+4)];
		DT.day[n]=input[(n+6)];
		DT.hh[n]=input[(n+8)];
		DT.mm[n]=input[(n+10)];
		DT.ss[n]=input[(n+12)];
	}
	DT.month[2]='\0';
	DT.day[2]='\0';
	DT.hh[2]='\0';
	DT.mm[2]='\0';
	DT.ss[2]='\0';
	prepare_lcd();
	lcd_clear_line(4);
	n2=(((input[0]=='2')&&(input[1]=='0'))||((input[0]=='1')&&(input[1]=='9')));
	n2|=wide_year;
	if (n2)
		if (valid_date((unsigned char)atoi(DT.day), (unsigned char)atoi(DT.month), atoi(DT.year)))
		{
			if (valid_time((unsigned char)atoi(DT.hh),(unsigned char)atoi(DT.mm), (unsigned char)atoi(DT.ss)))
			{
				n2=0x01;
				//print_lcd2("OK!");
				print_lcd2_P(STATUS_OK);
			}
			else
			{

				n2=0x00;
				//print_lcd2("Invalid time");
				print_lcd2_P(ERR_INVALID);
				print_lcd2_P(ERR_TIME[1]);
				//print_lcd2("Hora inv");
				//lcdData(0x00);
				//print_lcd2("lida");
			}
		}
		else
		{
			n2=0x00;
			//print_lcd2("Invalid date");
			print_lcd2_P(ERR_INVALID);
			print_lcd2_P(ERR_DATE);
			//print_lcd2("Data inv");
			//lcdData(0x00);
			//print_lcd2("lida");
		}
	else
	{
		n2=0x00;
		//print_lcd2("Invalid year");
		print_lcd2_P(ERR_INVALID);
		print_lcd2_P(ERR_YEAR);
		//print_lcd2("Ano inv");
		//lcdData(0x00);
		//print_lcd2("lido");
	}
	SetCursor(3,1);
	set_none();
	return (n2);
}
/*
double toFarenheit(double x)
{
	double k=1.8;// 9/5
	k*=x;
	return (k+32.0);
}

double toCelsius(double x)
{
	double k=0.5555555555;
	k*=(x-32.0);
	return k;
}
*/
void show_pino_menu()
{
	menu=PINO_MENU;
	clear_input(0);
	prepare_lcd();
	clear_display();
	cursor_off();
	if (language&0x01)
		//print_lcd2("PINS CONF.");
		print_lcd2_P(PINS_CONF_MENU_STR_EN);
	else
		//print_lcd2("CONF. DE PINOS");
		print_lcd2_P(PINS_CONF_MENU_STR_BR);
	SetCursor(3,1);
	//print_lcd2("1-P1   3-P3   5-P5");
	print_lcd2_P(PINS_CONF_MENU_STR_A);
	SetCursor(4,1);
	//print_lcd2("2-P2   4-P4   6-P6");
	print_lcd2_P(PINS_CONF_MENU_STR_B);
	set_none();
}
void set_pino_menu(unsigned char pino_menu_a)
{
	//char str_tmp_pino_menu[7];
	unsigned char a=(language&0x01);
	menu=pino_menu_a;
	prepare_lcd();
	clear_display();
	/*
	if (language)
	{
		print_lcd2("Pin P");
		strcpy(str_tmp_pino_menu, "Type");
	}
	else
	{
		print_lcd2("Pino P");
		strcpy(str_tmp_pino_menu, "Digite");
	}
	*/
	print_lcd2_P(PIN_P_STR[a]);
	//if (!(language&0x01)) lcdData('o');
	//print_lcd2(" P");
	//lcdData(':');
	//n=(pino_menu_a&0x07)+0x30;
	//lcdData(n);
	lcdData((pino_menu_a&0x07)+0x30);
	SetCursor(2,1);
	//print_lcd2("Digite");
	//print_lcd2(str_tmp_pino_menu);
	print_lcd2_P(TYPE_STR[a]);
	lcdData(':');
	SetCursor(3,1);
	cursor_on();
	clear_input(1); // added 05, 14 2015
	//set_none();
}
void set_pino_print_error()
{
	prepare_lcd();
	lcd_clear_line(4);
	print_lcd2(conv);
	SetCursor(3,1);
	set_none();
}

unsigned char set_pino_read_input(unsigned char index)
{
	//unsigned char k_a;
	//k_a=((input[index]-0x30)<<4);
	//k_a+=(input[index+1]-0x30);
	//return k_a; // result in BCD
	return ((input[index]<<4)|(input[index+1]&0x0F));
	
}

//unsigned char sub_set_pino_temperature(unsigned char index)
// bugfix at 11/04/2015 @ 15:59
signed int sub_set_pino_temperature(unsigned char index)
{
	//PINOS_t pino_tmp_b;
	DT.year[0]=input[index];
	DT.year[1]=input[index+1];
	DT.year[2]=input[index+2];
	DT.year[3]=input[index+3];
	//pino_tmp_b.gpv=(signed char)atoi(DT.year);
	signed int tmp_a=(signed int)atoi(DT.year);
	//static double d_tmp_var; //Saving memory
	//n2=1;
	if (input[1]=='1')
	{
		//if ((pino_tmp_b.gpv > 185)||(pino_tmp_b.gpv < -40)) n2=0;
		if ((tmp_a>185)||(tmp_a<-40)) n2=0;
		else
		{

			//d_tmp_var=toCelsius((double)tmp_a); //Original
			//tmp_a=(signed int)d_tmp_var; //Original

			//d_tmp_var=(double)tmp_a;
			//tmp_a=(signed int)d_tmp_var;
/*
    240e:	be 01       	movw	r22, r28
    2410:	88 27       	eor	r24, r24
    2412:	77 fd       	sbrc	r23, 7
    2414:	80 95       	com	r24
    2416:	98 2f       	mov	r25, r24
    2418:	0e 94 aa 33 	call	0x6754	; 0x6754 <__floatsisf>
    241c:	dc 01       	movw	r26, r24
    241e:	cb 01       	movw	r24, r22
    2420:	80 93 07 02 	sts	0x0207, r24
    2424:	90 93 08 02 	sts	0x0208, r25
    2428:	a0 93 09 02 	sts	0x0209, r26
    242c:	b0 93 0a 02 	sts	0x020A, r27
    2430:	00 00       	nop
    2432:	00 00       	nop
    242c:	0e 94 bb 33 	call	0x6776	; 0x6776 <__fixsfsi>
    2430:	dc 01       	movw	r26, r24
    2432:	cb 01       	movw	r24, r22
    2434:	ec 01       	movw	r28, r24
*/
			//d_tmp_var=(double)tmp_a;
			asm volatile
			(
				"movw r22, %0" "\n\t"
				"clr r24" "\n\t"
				"sbrc r23, 7" "\n\t"
				"com r24" "\n\t"
				"mov r25, r24" "\n\t"
				"call __floatsisf" "\n\t"
				"ldi r18, 0x00" "\n\t"
				"ldi r19, 0x00" "\n\t"
				"ldi r20, 0x00" "\n\t"
				"ldi r21, 0x42" "\n\t"
				"call __subsf3" "\n\t"
				"ldi r18, 0xE4" "\n\t"
				"ldi r19, 0x38" "\n\t"
				"ldi r20, 0x0E" "\n\t"
				"ldi r21, 0x3F" "\n\t"
				"call __mulsf3" "\n\t"
				"call __fixsfsi" "\n\t"
				//"movw %0, r24" "\n\t" // Fixed (27/08/2015)
				"movw %0, r22" "\n\t"
			:
			:"r"(tmp_a)
			);
		}
	}
	else
	{
		if ((tmp_a >85) || (tmp_a < -40)) n2=0;
	}

	if (!(n2))
	{
		//strcpy(conv, "Err:ovrflw");
		strcpy_P(conv, ERR_OVRFLW);
		set_pino_print_error();
	}

	return tmp_a;
}
void set_pino()
{
	struct PINOS_t pino_tmp;
	//1; //FLAG 1- no error --> 0 - error or input==default
	unsigned char k;
	k=input[0];
	n2=(k<'8');

	
	pino_tmp.command=0;
/*
	pino_tmp.hh1=0;
	pino_tmp.mm1=0;
	pino_tmp.hh2=0;
	pino_tmp.mm2=0;
	pino_tmp.hh3=0;
	pino_tmp.mm3=0;
	pino_tmp.ss3=0;
	pino_tmp.hh4=0;
	pino_tmp.mm4=0;
	pino_tmp.ss4=0;
*/

	if (k=='1')
	{
		pino_tmp.command=0x01;
		if (input[1]=='1')
		{
			k=set_pino_read_input(2);
			if (k>0x23)
			{
				n2=0;
				//strcpy(conv, "Err:time");
				strcpy_P(conv, ERR_TIME2);
				set_pino_print_error();
			}
			else
				pino_tmp.hh1=k;

			k=set_pino_read_input(4);
			if (k>0x59)
			{
				n2=0;
				//strcpy(conv, "Err:minute");
				strcpy_P(conv, ERR_MINUTE);
				set_pino_print_error();
			} else
				pino_tmp.mm1=k;
			pino_tmp.command|=0x10;//0x80; // enable turn on
		}
		else if (input[1]>'1')
		{
			n2=0;
			//strcpy(conv,"Err:E/D");
			strcpy_P(conv, ERR_E_D);
			set_pino_print_error();
		}

		if (input[6]=='1')
		{
			k=set_pino_read_input(7);
			if (k>0x23)
			{
				n2=0;
				//strcpy(conv,"Err:time");
				strcpy_P(conv, ERR_TIME2);
				set_pino_print_error();
			}
			else
				pino_tmp.hh2=k;

			k=set_pino_read_input(9);
			if (k>0x59)
			{
				n2=0;
				//strcpy(conv, "Err:minute");
				strcpy_P(conv, ERR_MINUTE);
				set_pino_print_error();
			}
			else
				pino_tmp.mm2=k;
			pino_tmp.command|=0x08;//0x10; // enable turn off
		}
		else if (input[6]>'1')
		{
			n2=0;
			//strcpy(conv, "Err:E/D");
			strcpy_P(conv, ERR_E_D);
			set_pino_print_error();
		}
	}
	else if (k=='2')
	{
		pino_tmp.command=0x02;
		//n=(input[1]=='1');		
//		if ((input[1]=='1')||(input[1]=='0'))
		if ((input[1]=='1')||(input[1]=='0'))
		{
			//pino_tmp.command|=0x08; // turn on at hh3 mm3 ss3
			if (input[1]=='1') pino_tmp.command|=0x08;

			DT.hh[0]=input[2];
			DT.hh[1]=input[3];
			DT.hh[3]=0x00;
			pino_tmp.hh3=(unsigned char)atoi(DT.hh);
			if (pino_tmp.hh3>17)
			{
				n2=0;
				//strcpy(conv, "Err:time");
				strcpy_P(conv, ERR_TIME2);
				set_pino_print_error();
			}
			else
			{
				DT.mm[0]=input[4];
				DT.mm[1]=input[5];
				DT.mm[2]=0x00;
				pino_tmp.mm3=(unsigned char)atoi(DT.mm);
				if (pino_tmp.mm3>59)
				{
					n2=0;
					//strcpy(conv, "Err:minute");
					strcpy_P(conv, ERR_MINUTE);
					set_pino_print_error();
				}
				else
				{
					DT.ss[0]=input[6];
					DT.ss[1]=input[7];
					DT.ss[2]=0x00;
					pino_tmp.ss3=(unsigned char)atoi(DT.ss);
					if (pino_tmp.ss3>59)
					{
						n2=0;
						//strcpy(conv, "Err:sec.");
						strcpy_P(conv, ERR_SEC);
						set_pino_print_error();
					}
					else
					{
						pino_tmp.countdown1=(3600*pino_tmp.hh3+60*pino_tmp.mm3+pino_tmp.ss3);
					}
				}
			}
		}
		else// if (input[1]>'1')
		{
			n2=0;
			//strcpy(conv, "Err:ON-OFF");
			strcpy_P(conv, ERR_ON_OFF);
			set_pino_print_error();
		}
	}
	else if (k=='3')
	{
		pino_tmp.command=0x03;
		DT.hh[0]=input[1];
		DT.hh[1]=input[2];
		DT.hh[2]=0x00;
		pino_tmp.hh4=(unsigned char)atoi(DT.hh);
		if (atoi(DT.hh)>17)
		{
			n2=0;
			//strcpy(conv, "Err:time");
			strcpy_P(conv, ERR_TIME2);
			set_pino_print_error();
		}
		else
		{
			DT.mm[0]=input[3];
			DT.mm[1]=input[4];
			DT.mm[2]=0x00;
			pino_tmp.mm4=(unsigned char)atoi(DT.mm);
			if (pino_tmp.mm4>59)
			{
				n2=0;
				//strcpy(conv, "Err:minute");
				strcpy_P(conv, ERR_MINUTE);
				set_pino_print_error();
			}
			else
			{
				DT.ss[0]=input[5];
				DT.ss[1]=input[6];
				DT.ss[2]=0x00;
				pino_tmp.ss4=(unsigned char)atoi(DT.ss);
				if ((pino_tmp.ss4)>59)
				{
					n2=0;
					//strcpy(conv, "Err:sec.");
					strcpy_P(conv, ERR_SEC);
					set_pino_print_error();
				}
			}
		}

		DT.hh[0]=input[7];
		DT.hh[1]=input[8];
		DT.hh[2]=0x00;
		pino_tmp.hh5=(unsigned char)atoi(DT.hh);
		if (pino_tmp.hh5>17)
		{
			n2=0;
			//strcpy(conv,"Err:time");
			strcpy_P(conv, ERR_TIME2);
			set_pino_print_error();
		}
		else
		{
			DT.mm[0]=input[9];
			DT.mm[1]=input[10];
			DT.mm[2]=0x00;
			pino_tmp.mm5=(unsigned char)atoi(DT.mm);

			if (pino_tmp.mm5>59)
			{
				n2=0;
				//strcpy(conv, "Err:minute");
				strcpy_P(conv, ERR_MINUTE);
				set_pino_print_error();
			}
			else
			{
				DT.ss[0]=input[11];
				DT.ss[1]=input[12];
				DT.ss[2]=0x00;
				pino_tmp.ss5=(unsigned char)atoi(DT.ss);
				if (pino_tmp.ss5>59)
				{
					n2=0;
					//strcpy(conv, "Err:sec.");
					strcpy_P(conv, ERR_SEC);
					set_pino_print_error();
				}
			}
		}
		
	}
	else if ((k=='4') || (k=='5'))
	{
		// ZYYXX|ccc
		pino_tmp.command=(k-0x30);
		if ((input[2]>'2')||(input[7]>'2'))
		{
			n2=0;
			//strcpy(conv,"Err:x2/y2");
			strcpy_P(conv, ERR_X2_Y2);
			set_pino_print_error();
		}
		else
		{
			if (input[1]>'1')
			{
				n2=0;
				//strcpy(conv, "Err:C/F");
				strcpy_P(conv, ERR_C_F);
				set_pino_print_error();
			}
			else
			{
				pino_tmp.d=sub_set_pino_temperature(3);
				pino_tmp.e=sub_set_pino_temperature(8);
				//pino_tmp.gpv=0x00;
				pino_tmp.command|=((input[2]-0x30)<<3);
				pino_tmp.command|=((input[7]-0x30)<<5);
			}
		}
	}
	else if ((k=='6')||(k=='7'))
	{
		pino_tmp.command=(k-0x30);
		if ((input[1]>'1')||(input[3]>'1'))
		{
			n2=0;
			//strcpy(conv, "Err.:E/D");
			strcpy_P(conv, ERR_E_D);
			set_pino_print_error();
		}
		else
		{

			//
			pino_tmp.command|=((input[3]-0x30)<<4);
			n=(input[1]-0x30)<<5;
			pino_tmp.command|=n;
			//
			if ((input[2]>'3')||(input[4]>'3'))
			{
				n2=0x00;
				//strcpy(conv, "Err.:BAT1/2");
				strcpy_P(conv, ERR_BAT_1_2);
				set_pino_print_error();
			}
			else
			{
				if (k=='6')
				{
					pino_tmp.opbat1=((input[2]-0x30)<<4);
					pino_tmp.opbat1|=(input[4]-0x30);
				}
				else
				{
					pino_tmp.opbat2=((input[2]-0x30)<<4);
					pino_tmp.opbat2|=(input[4]-0x30);
				}
			}
		}

	}
	else if (k>'7')
	{
		n2=0;
		//strcpy(conv, "Err:codif.");
		strcpy_P(conv, ERR_CODE);
		set_pino_print_error();
	}

	if (n2)
	{
		prepare_lcd();
		lcd_clear_line(4);
		/*
		if (language)
			print_lcd2("Writing");
		else
			print_lcd2("Gravando");
		*/
		print_lcd2_P(WRITING_MSG[(language&0x01)]);
		//print_lcd2("...");
		replicate_char('.',3);
		/// adicionado
		cli();
		///
		if (menu==PINO_MENU_P1)
		{
			pino1=pino_tmp;
			writePinoEEPROM(pino1,PINO1_OFFSET_ADDR);
		}
		else if (menu==PINO_MENU_P2)
		{
			pino2=pino_tmp;
			writePinoEEPROM(pino2,PINO2_OFFSET_ADDR);
		}
		else if (menu==PINO_MENU_P3)
		{
			pino3=pino_tmp;
			writePinoEEPROM(pino3,PINO3_OFFSET_ADDR);
		}
		else if (menu==PINO_MENU_P4)
		{
			pino4=pino_tmp;
			writePinoEEPROM(pino4,PINO4_OFFSET_ADDR);
		}
		else if (menu==PINO_MENU_P5)
		{
			pino5=pino_tmp;
			writePinoEEPROM(pino5,PINO5_OFFSET_ADDR);
		}
		else if (menu==PINO_MENU_P6)
		{
			pino6=pino_tmp;
			writePinoEEPROM(pino6,PINO6_OFFSET_ADDR);
		}
		/// adicionado
		sei();
		///
		lcd_clear_line(4);
		//print_lcd2("OK!");
		print_lcd2_P(STATUS_OK);
		SetCursor(3,1);
		set_none();
	}
}
void show_sensor_menu()
{
	//char str_tmp_sensor_menu[8];
	menu=SENSOR_MENU;

	/*
	if (language)
		strcpy(str_tmp_sensor_menu,"no bat.");
	else
		strcpy(str_tmp_sensor_menu, "s/ bat.");
	*/
	prepare_lcd();
	clear_display();
	dtostrf(b1,1,2, conv); // Exemplo B1:1.42V (98.3%) | B2: 0.32V (s/ bat.)
	print_lcd2("B1:");
	print_lcd2(conv);
	print_lcd2("V (");
	if (b1 > low_voltage)
	{
		b1*=inv_100_charged_voltage;
		dtostrf(b1,1,1, conv);
		print_lcd2(conv);
		lcdData('%');
	}
	else print_lcd2_P(STR_SENSOR[language&0x01]); //print_lcd2(str_tmp_sensor_menu);//print_lcd2("s/ bat.");
	lcdData(')');

	SetCursor(2,1);
	dtostrf(b2,1,2, conv);
	print_lcd2("B2:");
	print_lcd2(conv);
	print_lcd2("V (");
	if (b2 >  low_voltage)
	{
		b2*=inv_100_charged_voltage;
		dtostrf(b2,1,1,conv);
		print_lcd2(conv);
		lcdData('%');
	}
	else print_lcd2_P(STR_SENSOR[language&0x01]);// print_lcd2(str_tmp_sensor_menu);//print_lcd2("s/ bat.");
	lcdData(')'); // Exemplo T1:32.8°C, 128.8°F
	SetCursor(3,1);
	dtostrf(t1,1,1, conv);
	print_lcd2("T1:");
	print_lcd2(conv);
	lcdData(0x05);
	print_lcd2("C, ");
	dtostrf(toFarenheit(&t1),1,1,conv);
	print_lcd2(conv);
	lcdData(0x05);
	lcdData('F');

	SetCursor(4,1);
	dtostrf(t2,1,1, conv);
	print_lcd2("T2:");
	print_lcd2(conv);
	lcdData(0x05);
	print_lcd2("C, ");
	dtostrf(toFarenheit(&t2),1,1,conv);
	print_lcd2(conv);
	lcdData(0x05);
	lcdData('F');
	set_none();
}
void set_alarm()
{
	//char str_tmp_set_alarm[7];
	menu=ALARM_SETUP;
	prepare_lcd();
	clear_display();
	//print_lcd2("ALARM");
	print_lcd2_P(ALARM_STR[language&0x01]);
	/*
	if (language)
		strcpy(str_tmp_set_alarm,"Type");
	else
	{
		lcdData('E');
		strcpy(str_tmp_set_alarm, "Digite");		
	}
	*/
	//if (!(language)) lcdData('E');
	//print_lcd2("ALARME:");
	lcdData(':');
	SetCursor(2,1);
	//print_lcd2("Digite:chhmm:");
	//print_lcd2(str_tmp_set_alarm);
	print_lcd2_P(TYPE_STR[language&0x01]);
	//print_lcd2(":chhmm:");
	//print_lcd2_P(ALARM_TYPE_STR);
	print_lcd2(":c");
	replicate_char('h',2);
	replicate_char('m',2);
	lcdData(':');
	if (hoursA&0x80) lcdData('1');
	else lcdData('0');
	n=hoursA&0x3F;
	print_time_date();
	n=minutesA;
	print_time_date();
	SetCursor(3,1);
	cursor_on();
	set_none();
}
//////////////////////////
// CALCULADORA BASICA ///
////////////////////////
/*
	CALCULADORA BASICA:

	Para a calculadora basica temos as seguintes variaveis:
	A = primeiro valor (float)
	B = segundo valor (float)
	C = resultado do valor A*B ou A+B ou A-C ou A/B
	bas_calc_op -> (char) armazena a operacao a ser feita + - / *
	bas_calc_status -> (unsigned char) x3 x2 x1 x0
		onde:
			x0 = 1 -> A tem valor definido
			x1 = 1 -> B tem valor definido
			x2 = 1 -> ponto definido
	bas_calc_char[15] -> contem os strings de 13 digitos, incluindo o ponto mais o sinal
	pos_bas_calc -> (char) posicao do bas_calc_char
*/

void clear_bas_calc_char()
{
	bas_calc_char[0]=' ';
	bas_calc_char[1]='0';
	bas_calc_char[2]=0x0;
	pos_bas_calc=0;
	bas_calc_status&=~(0x4);//11111011
}

void clear_all_bas_calc()
{
	A=0; B=0;
	bas_calc_status=0;
	clear_bas_calc_char();
}
void bas_calc_sign(unsigned char x)
{
	if (x==UP_PLUS_KEY) bas_calc_op='+';
	else if (x==DOWN_MINUS_KEY) bas_calc_op='-';
	else if (x==RIGHT_MULTIPLY_KEY) bas_calc_op='*';
	else if (x==LEFT_DIVIDE_KEY) bas_calc_op='/';
}
void scan_basic_and_scientific_command()
{
	if (code_cr==CLEAR_KEY)
	{
		clear_all_bas_calc();
		lcd_clear_line(2);
		lcd_clear_line(1);
	}
	else if (code_cr==DOT_KEY)
	{
		if ((bas_calc_status&4)==0)
		{
			if (pos_bas_calc<BAS_CALC_DIGIT)
			{
				bas_calc_status|=0x4;
				pos_bas_calc++;
				bas_calc_char[pos_bas_calc]='.';
				bas_calc_char[pos_bas_calc+1]=0x0;
				prepare_lcd();
				lcd_clear_line(2);
				print_lcd2(bas_calc_char);
				set_none();
			}
		}
	}
	else if ((code_cr>='0')&&(code_cr<='9'))
	{
		if (pos_bas_calc<BAS_CALC_DIGIT)
		{
			pos_bas_calc++;
			bas_calc_char[pos_bas_calc]=code_cr;
			bas_calc_char[pos_bas_calc+1]=0x0;
			prepare_lcd();
			lcd_clear_line(2);
			print_lcd2(bas_calc_char);
			set_none();
		}
	}
	else if (code_cr==PLUS_MINUS_KEY)
	{
		if (bas_calc_char[0]==' ') bas_calc_char[0]='-';
		else bas_calc_char[0]=' ';
		prepare_lcd();
		lcd_clear_line(2);
		print_lcd2(bas_calc_char);
		set_none();
	}
	else if (code_cr==MF_KEY)
	{
		A=C;
		bas_calc_status|=0x01;
		prepare_lcd();
		lcd_clear_line(3);
		lcd_print_float_to_string(A);
		lcd_clear_line(2);
		//print_lcd2("A<-C");
		print_lcd2_P(MATH_A_C_STR);
		set_none();
	}
}
void basic_calculator()
{

	if (code_cr==RETURN_KEY)
	{
		clear_all_bas_calc();
		show_menu_calculator();
	}
	else if ((code_cr==UP_PLUS_KEY)||(code_cr==DOWN_MINUS_KEY)||(code_cr==RIGHT_MULTIPLY_KEY)||(code_cr==LEFT_DIVIDE_KEY))
	{
		if (!(bas_calc_status&0x01)) A=atof(bas_calc_char); //adicionado
		bas_calc_status|=1;
		bas_calc_status&=~(0x2);
		//A=atof(bas_calc_char); // tirado
		bas_calc_sign(code_cr);
		B=0;
		prepare_lcd();
		lcd_clear_line(2);
		//print_lcd2("A ");
		print_lcd2_P(MATH_A_STR);
		lcdData(bas_calc_op);
		lcd_clear_line(3);
		clear_bas_calc_char();
		print_lcd2(bas_calc_char);
		set_none();
	}
	else if (code_cr==ENTER_KEY)
	{
		if ((bas_calc_status&0x03)==0x01)
		{
			B=atof(bas_calc_char);
			bas_calc_status|=0x2;
		}
		if ((bas_calc_status&0x03)==0x03)
		{
			if (bas_calc_op=='+') C=A+B;
			else if (bas_calc_op=='-') C=A-B;
			else if (bas_calc_op=='*') C=A*B;
			else if (bas_calc_op=='/') C=A/B;

			prepare_lcd();
			lcd_clear_line(2);
			//print_lcd2("A ");
			print_lcd2_P(MATH_A_STR);
			lcdData(bas_calc_op);
			//print_lcd2(" B = C");
			print_lcd2_P(MATH_B_EQUALS_C_STR);
			lcd_clear_line(3);
			lcd_print_float_to_string(C);
			set_none();
			clear_all_bas_calc();
		}
	
	}
	scan_basic_and_scientific_command();
}

// FIM DE CALCULADORA BASICA

/////////////////////////////
// CALCULADORA CIENTIFICA //
///////////////////////////

void print_scientific_function_name()
{
	// prints scientific function name in scientific calculator

	//if (function_name_op==23) function_name_op--;
	if (function_name_op==24) function_name_op--;
	bas_calc_status&=~(0x01);
// new functions
/*

 1- exp
 2- ln

	log/log(c)

 3- sqrt
 4- cbrt
 5- sin
 6- cos
 7- tan
 8- sind
 9- cosd
10- tand
11- asin
12- acos
13- atan
14- asind
15- acosd
16- atand
17- A<-C
18- hyp(C,A)
29- mod(C,A)
20- sinh
21- cosh
22- tanh

*/
	switch (function_name_op)
	{
		/*
		case 0:strcpy(function_name,"exp");break;
		case 1:strcpy(function_name,"ln");break;
		case 2:strcpy(function_name, "ln(x)/C"); break;
		case 3:strcpy(function_name,"sqrt");break;
		case 4:strcpy(function_name,"cbrt");break;
		case 5:strcpy(function_name,"sin");break;
		case 6:strcpy(function_name,"cos");break;
		case 7:strcpy(function_name,"tan");break;
		case 8:strcpy(function_name,"sind");break;
		case 9:strcpy(function_name,"cosd");break;
		case 10:strcpy(function_name,"tand");break;
		case 11:strcpy(function_name,"asin");break;
		case 12:strcpy(function_name,"acos");break;
		case 13:strcpy(function_name,"atan");break;
		case 14:strcpy(function_name,"asind");break;
		case 15:strcpy(function_name,"acosd");break;
		case 16:strcpy(function_name, "atand"); break;
		case 17:strcpy(function_name,"C<-A"); break;
		case 18:strcpy(function_name,"A<-C"); break;
		case 19:strcpy(function_name, "hyp(C,A)"); break;
		case 20:strcpy(function_name,"mod(C,A)"); break;
		case 21:strcpy(function_name,"sinh"); break;
		case 22:strcpy(function_name, "cosh"); break;
		case 23:strcpy(function_name, "tanh");
		*/
		case 0:strcpy_P(function_name,MATH_EXP_STR);break;
		case 1:strcpy_P(function_name,MATH_LN_STR);break;
		case 2:strcpy_P(function_name, MATH_LN_X_C_STR); break;
		case 3:strcpy_P(function_name, MATH_SQRT_STR);break;
		case 4:strcpy_P(function_name,MATH_CBRT_STR);break;
		case 5:strcpy_P(function_name, MATH_SIN_STR);break;
		case 6:strcpy_P(function_name,MATH_COS_STR);break;
		case 7:strcpy_P(function_name,MATH_TAN_STR);break;
		case 8:strcpy_P(function_name,MATH_SIND_STR);break;
		case 9:strcpy_P(function_name,MATH_COSD_STR);break;
		case 10:strcpy_P(function_name,MATH_TAND_STR);break;
		case 11:strcpy_P(function_name,MATH_ASIN_STR);break;
		case 12:strcpy_P(function_name,MATH_ACOS_STR);break;
		case 13:strcpy_P(function_name,MATH_ATAN_STR);break;
		case 14:strcpy_P(function_name,MATH_ASIND_STR);break;
		case 15:strcpy_P(function_name,MATH_ACOSD_STR);break;
		case 16:strcpy_P(function_name, MATH_ATAND_STR); break;
		case 17:strcpy_P(function_name,MATH_C_A_STR); break;
		case 18:strcpy_P(function_name,MATH_A_C_STR); break;
		case 19:strcpy_P(function_name, MATH_HYP_STR); break;
		case 20:strcpy_P(function_name,MATH_MOD_STR); break;
		case 21:strcpy_P(function_name,MATH_SINH_STR); break;
		case 22:strcpy_P(function_name, MATH_COSH_STR); break;
		case 23:strcpy_P(function_name, MATH_TANH_STR);
	}
	prepare_lcd();
	lcd_clear_line(4);
	//print_lcd2("f(x)=");
	print_lcd2_P(MATH_F_X_STR);
	print_lcd2(function_name);
	set_none();

}
void scientific_calculator()
{
	if (code_cr==RETURN_KEY)
	{
		clear_all_bas_calc();
		show_menu_calculator();
	}
	else
	{
		if (code_cr==UP_PLUS_KEY)
		{
			function_name_op++;
			print_scientific_function_name();
		}
		else if (code_cr==DOWN_MINUS_KEY)
		{
			if (function_name_op>0) function_name_op--;
			print_scientific_function_name();
		}
		else if (code_cr==ENTER_KEY)
		{

			if ((bas_calc_status&0x01)==0) A=atof(bas_calc_char);
			switch (function_name_op)
			{
				case 0: C=exp(A); break;
				case 1: C=log(A); break;
				case 2: C=log(A)/C; break;
				case 3: C=sqrt(A); break;
				case 4: C=cbrt(A); break;
				case 5: C=sin(A); break;
				case 6: C=cos(A); break;
				case 7: C=tan(A); break;
				case 8: C=sin(pi_inv_180*A); break;// sind
				case 9: C=cos(pi_inv_180*A); break;// cosd
				case 10: C=tan(A*pi_inv_180); break;// tand
				case 11: C=asin(A); break;
				case 12: C=acos(A); break;
				case 13: C=atan(A); break;
				case 14: C=deg_inv_pi*asin(A); break;
				case 15: C=deg_inv_pi*acos(A); break;
				case 16: C=deg_inv_pi*atan(A); break;
				case 17: C=A; break;
				case 18: A=C; break;
				case 19: C=hypot(C,A); break;
				case 20: C=fmod(C,A); break;
				case 21: C=sinh(A); break;
				case 22: C=cosh(A); break;
				case 23: C=tanh(A);
			}
			prepare_lcd();
			lcd_clear_line(2);
			print_lcd2(function_name);
			//print_lcd2("(x) =");
			print_lcd2_P(MATH_X_EQUAL);
			lcd_clear_line(3);
			lcd_print_float_to_string(C);
			set_none();
			clear_all_bas_calc();
		}

		scan_basic_and_scientific_command();
	}
}
// FIM DE CALCULADORA CIENTIFICA

void show_menu_calculator()
{
	//char str_tmp_menu_calculator[17][3];
	//char str_tmp_menu_calculator[4][20];
//	char str_tmp_menu_calculator[4][17];
	unsigned char a=(language&0x01);
	menu=CALCULATOR_MENU;
	clear_all_bas_calc();
	prepare_lcd();
	clear_display();
	//print_lcd2("CALCULADORA");
	//print_lcd2("CALCULA");
	print_lcd2_P(CALCULA_STR);
/*
	if (language)
	{
		//print_lcd2("TOR");
		print_lcd2_P(CALCULA_STR2);
		//strcpy(str_tmp_menu_calculator[0],"Default calc.");
		strcpy_P(str_tmp_menu_calculator[0],DEF_CALC_EN);
		//strcpy(str_tmp_menu_calculator[1], "Scientific calc.");
		strcpy_P(str_tmp_menu_calculator[1], SCI_CALC_EN);
		//strcpy(str_tmp_menu_calculator[2],"DEFAULT CALC.");
		//strcpy(str_tmp_menu_calculator[3], "SCIENTIFIC CALC.");
		strcpy(str_tmp_menu_calculator[2],str_tmp_menu_calculator[0]);
		strcpy(str_tmp_menu_calculator[3],str_tmp_menu_calculator[1]);
		strupr(str_tmp_menu_calculator[2]);
		strupr(str_tmp_menu_calculator[3]);
	}
	else
	{
		//print_lcd2("DORA"); strupr
		print_lcd2_P(CALCULA_STR3);
		//strcpy(str_tmp_menu_calculator[0],"Calc. padrao");
		strcpy_P(str_tmp_menu_calculator[0],DEF_CALC_BR);
		//str_tmp_menu_calculator[10][0]=0x01; //ã
		//str_tmp_menu_calculator[0][10]=0x01; //ã
		//strcpy(str_tmp_menu_calculator[1], "Calc. cientifica");
		strcpy_P(str_tmp_menu_calculator[1], SCI_CALC_BR);
		strcpy(str_tmp_menu_calculator[2],str_tmp_menu_calculator[0]);
		strcpy(str_tmp_menu_calculator[3],str_tmp_menu_calculator[1]);
		strupr(str_tmp_menu_calculator[2]);
		strupr(str_tmp_menu_calculator[3]);
		str_tmp_menu_calculator[0][10]=0x01; //ã
		//str_tmp_menu_calculator[11][1]=0x06; //í
		str_tmp_menu_calculator[1][11]=0x06; //í
		//strcpy(str_tmp_menu_calculator[2],"CALC. PADRAO");
		//str_tmp_menu_calculator[10][2]=0x02;// Ã
		str_tmp_menu_calculator[2][10]=0x02;// Ã
		//strcpy(str_tmp_menu_calculator[3], "CALC. CIENTIFICA");
		//str_tmp_menu_calculator[11][3]=0x03; //Í
		str_tmp_menu_calculator[3][11]=0x03; //Í
	}
*/
	print_lcd2_P(CALCULA_STR2[a]);
	SetCursor(2,1);
	//print_lcd2("1-Calc. padr");
	print_lcd2("1-");
	//print_lcd2(str_tmp_menu_calculator[0]);
	print_lcd2_P(DEF_CALC[a]);
	//lcdData(0x01); //ã
	//print_lcd2("o");
	SetCursor(3,1);
	print_lcd2("2-");
	//print_lcd2(str_tmp_menu_calculator[1]);
	print_lcd2_P(SCI_CALC[a]);
	//print_lcd2("2-Calc. cient");
	//lcdData(0x06); // í
	//print_lcd2("fica");
	//n=menu_kbd_interval('0','3');
	//if (n)
/*
	if (menu_kbd_interval('0','3'))
	{
		clear_display();
		if (code_cr=='1')
		{
			menu=BASIC_CALC_MENU;
			print_lcd2(str_tmp_menu_calculator[2]);
			//print_lcd2("CALC. PADR");
			//lcdData(0x02);
			//lcdData('O');
		}
		else if (code_cr=='2') 
		{
			menu=SCIENTIFIC_MENU;
			print_lcd2(str_tmp_menu_calculator[3]);
			//print_lcd2("CALC. CIENT");
			//lcdData(0x03);
			//print_lcd2("FICA");
		}
	}
*/
	set_none();
}
void sub_show_menu_calculator()
{
	unsigned char a=(language&0x01);
	char buffer[20];
	prepare_lcd();
	if (code_cr=='1')
	{
		menu=BASIC_CALC_MENU;
		clear_display();
		strcpy_P(buffer, DEF_CALC[a]);
		print_lcd2(strupr_b(buffer));
	}
	else if (code_cr=='2') 
	{
		menu=SCIENTIFIC_MENU;
		clear_display();
		strcpy_P(buffer, SCI_CALC[a]);
		print_lcd2(strupr_b(buffer));
	}
	set_none();
}
//unsigned char count_down_timer(unsigned int &countdown_a)
unsigned char count_down_timer(unsigned int *countdown_a) // C compatible
{
/*
00003910 <_Z16count_down_timerRj>:
    3910:	fc 01       	movw	r30, r24
    3912:	80 81       	ld	r24, Z
    3914:	91 81       	ldd	r25, Z+1	; 0x01
    3916:	00 97       	sbiw	r24, 0x00	; 0
    3918:	19 f0       	breq	.+6      	; 0x3920 <_Z16count_down_timerRj+0x10>
    391a:	01 97       	sbiw	r24, 0x01	; 1
    391c:	91 83       	std	Z+1, r25	; 0x01
    391e:	80 83       	st	Z, r24
    3920:	81 e0       	ldi	r24, 0x01	; 1
    3922:	20 81       	ld	r18, Z
    3924:	31 81       	ldd	r19, Z+1	; 0x01
    3926:	21 15       	cp	r18, r1
    3928:	31 05       	cpc	r19, r1
    392a:	09 f0       	breq	.+2      	; 0x392e <_Z16count_down_timerRj+0x1e>
    392c:	80 e0       	ldi	r24, 0x00	; 0
    392e:	08 95       	ret

	if (countdown_a) countdown_a--;
	return (countdown_a==0);
*/
	//unsigned char result_k;
/* Qua 27 Jun 2018 21:53:10 -03 
	asm volatile
	(
		"adiw %0, 0x00" "\n\t"
		"breq count_down_SAIR_VERDADEIRO" "\n\t"
		"sbiw %0, 0x01" "\n\t"
		"std Z+1, %B0" "\n\t"
		"st Z, %A0" "\n\t"
		//"adiw %0, 0x00" "\n\t" // I don't need this to test (08/26/2015)
		"breq count_down_SAIR_VERDADEIRO" "\n\t"
		"ldi %0, 0x00" "\n\t"
		"ret" "\n\t"
	"count_down_SAIR_VERDADEIRO:" "\n\t"
		"ldi %0, 0x01" "\n\t"
	:
	:"r"(countdown_a)
	); */
	asm volatile
	(
		"ld r24, Z" "\n\t"
		"ldd r25, Z+1" "\n\t"
		"adiw 24, 0x00" "\n\t"
		"breq count_down_SAIR_VERDADEIRO" "\n\t"
		"sbiw r24, 0x01" "\n\t"
		"std Z+1, r25" "\n\t"
		"st Z, r24" "\n\t"
		//"adiw %0, 0x00" "\n\t" // I don't need this to test (08/26/2015)
		"breq count_down_SAIR_VERDADEIRO" "\n\t"
		"ldi r24, 0x00" "\n\t"
		"ret" "\n\t"
	"count_down_SAIR_VERDADEIRO:" "\n\t"
		"ldi r24, 0x01" "\n\t"
	:
	//:"r"(countdown_a)
	:"z"(countdown_a)
	);
}

void battery_monitor(unsigned char CHARGERX_B, unsigned char BX_LED_SIGNAL, double bx_b)
{

// this function is:
// read sensors in batery 1 and battery 2
// charge battery 1 or battery 2 if necessary
// Show to user if battery 1 or 2 are low, charged or charging by red and green leds


	if (bx_b < low_voltage)
	{
		//if (!(pseudo_latch_aux&BX_LED_SIGNAL))
		if ((pseudo_latch_aux&(CHARGERX_B|BX_LED_SIGNAL))!=(CHARGERX_B|BX_LED_SIGNAL))
		{
			//pseudo_latch_aux|=BX_LED_SIGNAL;
			pseudo_latch_aux|=(CHARGERX_B|BX_LED_SIGNAL);
			set_latch_aux();
			//beep();
		}
	}
	else if (bx_b>high_voltage)
	{
		//if (!(pseudo_latch_aux&CHARGER1_OFF))
		//{
			pseudo_latch_aux|=(CHARGERX_B|BX_LED_SIGNAL);  // turn off transistor key and turn off red and green leds
			set_latch_aux();
			//beep();
		//}
	}
	else if (bx_b < uncharged_voltage)
	{
	//	if (pseudo_latch_aux&CHARGER1_OFF)
	//	{
			pseudo_latch_aux&=~(CHARGERX_B); // turns on CHARGER 1
			pseudo_latch_aux^=BX_LED_SIGNAL; // blinks red led
			set_latch_aux();
	//	}
	}
	else if (bx_b> charged_voltage)
	{
		n=(!(pseudo_latch_aux&CHARGERX_B));
		if (n | (pseudo_latch_aux&BX_LED_SIGNAL))
		{
			//n=~(BX_LED_SIGNAL);
			pseudo_latch_aux|=CHARGERX_B;
			pseudo_latch_aux&=~(BX_LED_SIGNAL); // turns on green led
			set_latch_aux();
			//beep();
		}
	}
	else
	{
		if (!(pseudo_latch_aux&CHARGERX_B))
		{
			pseudo_latch_aux^=BX_LED_SIGNAL;
			set_latch_aux();
			//beep();
		}
		else if ((pseudo_latch_aux&(BX_LED_SIGNAL|CHARGERX_B))==(BX_LED_SIGNAL|CHARGERX_B))
		{
			pseudo_latch_aux&=~(BX_LED_SIGNAL);
			set_latch_aux();
		}
	}
}


//void sub_monitor_pins_battery(PINOS_t &pin_b, unsigned char CHARGERX, double bx, unsigned char pino_k)
void sub_monitor_pins_battery(struct PINOS_t *pin_b, unsigned char CHARGERX, double bx, unsigned char pino_k) // C compatible
{
	
	//xxxYX|ccc
	// opbat_n= 00xx|00yy
	// xx or yy turn on or turn off when: xx=00 disabled;  xx=01 low battery; xx=10 charging; xx=11 charged
	unsigned char opbatx, ledx;
	/// CHARGERX 
	if (CHARGERX&CHARGER2_OFF)
	{
		ledx=B2_LED_SIGNAL;
		opbatx=pin_b->opbat2;
	}
	else
	{
		ledx=B1_LED_SIGNAL;
		opbatx=pin_b->opbat1;
	}
	if (pin_b->command&0x20)
		switch (opbatx&0x30)
		{
			case 0x10:
			{
				if ((bx < uncharged_voltage)&&(bx > low_voltage))
				{
					if ((pseudo_latch_pinos&pino_k)==0)
					{
						pseudo_latch_pinos|=pino_k;
						set_latch_pinos();
						beep();
					}
				}
				break;
			}
			case 0x20:
			{
				if ((pseudo_latch_aux&CHARGERX)==0)
				{
					if ((pseudo_latch_pinos&pino_k)==0)
					{
						pseudo_latch_pinos|=pino_k;
						set_latch_pinos();
						beep();
					}
				}
				break;
			}
			case 0x30:
			{
				if (((pseudo_latch_aux&CHARGERX)==CHARGERX)&&((pseudo_latch_aux&ledx)==0))
				{
					if ((pseudo_latch_pinos&pino_k)==0)
					{
						pseudo_latch_pinos|=pino_k;
						set_latch_pinos();
						beep();
					}
				}
				break;
			}
			default:
			{
				if (bx < low_voltage)
				{
					if ((pseudo_latch_pinos&pino_k)==0)
					{
						pseudo_latch_pinos|=pino_k;
						set_latch_pinos();
						beep();
					}
				}
			}
	}

	if (pin_b->command&0x10)
		switch (opbatx&0x03)
		{
			case 0x01:
			{
				if ((bx < uncharged_voltage)&&(bx > low_voltage))
				{
					if (pseudo_latch_pinos&pino_k)
					{
						pseudo_latch_pinos&=~(pino_k);
						set_latch_pinos();
						beep();
					}
				}
				break;
			}
			case 0x02:
			{
				if ((pseudo_latch_aux&CHARGERX)==0)
				{
					if (pseudo_latch_pinos&pino_k)
					{
						pseudo_latch_pinos&=~(pino_k);
						set_latch_pinos();
						beep();
					}
				}
				break;
			}
			case 0x03:
			{
				if (((pseudo_latch_aux&CHARGERX)==CHARGERX)&&((pseudo_latch_aux&ledx)==0))
				{
					if (pseudo_latch_pinos&pino_k)
					{
						pseudo_latch_pinos&=~(pino_k);
						set_latch_pinos();
						beep();
					}
				}
				break;
			}
			default:
			{
				if (bx < low_voltage)
				{
					if (pseudo_latch_pinos&pino_k)
					{
						pseudo_latch_pinos&=~(pino_k);
						set_latch_pinos();
						beep();
					}
				}
			}
		}
}
/*
void sub_monitor_pins_temperature(PINOS_t &pin, unsigned char PINO, double temperature)
{
	// ZYYXX|ccc
	// XX 00 --> disabled 01 --> t1<d1 10 --> t1>d1
	//if (!(pin.gpv&0x01))
	if ((pin.gpv&0x01)==0)
	{
		// gpv xxxx xxx1 --> is started
		if (pin.command &0x08)
		{
			d_tmp_var=(double)pin.d;
			if (temperature<d_tmp_var)
			{
				pin.gpv|=0x01; // is started
				//if (pin.gpv==0x02) pin.gpv&=~(0x02);//is turn off started? if yes then turn off (hysteresis guaranted)
				pin.gpv&=~(0x02);
				pseudo_latch_pinos|=PINO;
				set_latch_pinos();
				beep();
			}
		}
		else if (pin.command&0x10)
		{
			d_tmp_var=(double)pin.d;
			if (temperature>d_tmp_var)
			{
				pin.gpv|=0x01; // is started
				//if (pin.gpv==0x02) pin.gpv&=~(0x02);//is turn off started? if yes then turn off (hysteresis guaranted)
				pin.gpv&=~(0x02);
				pseudo_latch_pinos|=PINO;
				set_latch_pinos();
				beep();
			}
		}
	}
////////////////////////////////////
	//if (!(pin.gpv&0x02))
	if ((pin.gpv&0x02)==0)
	{
		// gpv xxxx xx1x --> is started
		if (pin.command &0x20)
		{
			d_tmp_var=(double)pin.e;
			if (temperature<d_tmp_var)
			{
				pin.gpv|=0x02; // is started
				//if (pin.gpv==0x01) pin.gpv&=~(0x01);//is turn off started? if yes then turn off (hysteresis guaranted)
				pin.gpv&=~(0x01);
				pseudo_latch_pinos&=~(PINO);
				set_latch_pinos();
				beep();
			}
		}
		else if (pin.command&0x40)
		{
			d_tmp_var=(double)pin.e;
			if (temperature>d_tmp_var)
			{
				pin.gpv|=0x02; // is started
				//if (pin.gpv==0x01) pin.gpv&=~(0x01);//is turn off started? if yes then turn off (hysteresis guaranted)
				pin.gpv&=~(0x01);
				pseudo_latch_pinos&=~(PINO);
				set_latch_pinos();
				beep();
			}
		}
	}
}
*/
//void sub_monitor_pins_temperature(PINOS_t &pin, unsigned char PINO, double temperature)
void sub_monitor_pins_temperature(struct PINOS_t *pin, unsigned char PINO, double temperature) // C compatible
{
	// ZYYXX|ccc
	// XX 00 --> disabled 01 --> t1<d1 10 --> t1>d1
	//if (!(pin.gpv&0x01))
	//if ((pin.gpv&0x01)==0)
	//{
		// gpv xxxx xxx1 --> is started
	double d_tmp_var;
	if (pin->command &0x08)
	{
		d_tmp_var=(double)pin->d;
		if (temperature<d_tmp_var)
		{
			//if (pin.gpv==0x02) pin.gpv&=~(0x02);//is turn off started? if yes then turn off (hysteresis guaranted)
			if ((pin->gpv&0x01)==0)
			{
				pin->gpv|=0x01; // is started
				//pin.gpv&=~(0x02);
				if ((pseudo_latch_pinos&PINO)==0) beep();
				pseudo_latch_pinos|=PINO;
				set_latch_pinos();
				//beep();
			}
		} else pin->gpv&=~(0x01);
	}
	else if (pin->command&0x10)
	{
		d_tmp_var=(double)pin->d;
		if (temperature>d_tmp_var)
		{
			//pin.gpv|=0x01; // is started
			//if (pin.gpv==0x02) pin.gpv&=~(0x02);//is turn off started? if yes then turn off (hysteresis guaranted)
			if ((pin->gpv&0x01)==0)
			{
				pin->gpv|=0x01; // is started
				//pin.gpv&=~(0x02);
				if ((pseudo_latch_pinos&PINO)==0) beep();
				pseudo_latch_pinos|=PINO;
				set_latch_pinos();
				//beep();
			}
		} else pin->gpv&=~(0x01);
	}
	//}
////////////////////////////////////
	//if (!(pin.gpv&0x02))
	//if ((pin.gpv&0x02)==0)
	//{
	// gpv xxxx xx1x --> is started
	if (pin->command &0x20)
	{
		d_tmp_var=(double)pin->e;
		if (temperature<d_tmp_var)
		{
			//if (pin.gpv==0x01) pin.gpv&=~(0x01);//is turn off started? if yes then turn off (hysteresis guaranted)
			if ((pin->gpv&0x02)==0)
			{
				pin->gpv|=0x02; // is started
				//pin.gpv&=~(0x01);
				if (pseudo_latch_pinos&PINO) beep();
				pseudo_latch_pinos&=~(PINO);
				set_latch_pinos();
				//beep();
			}

		} else pin->gpv&=~(0x02);
	}
	else if (pin->command&0x40)
	{
		d_tmp_var=(double)pin->e;
		if (temperature>d_tmp_var)
		{
			if ((pin->gpv&0x02)==0)
			{
				pin->gpv|=0x02; // is started
				//if (pin.gpv==0x01) pin.gpv&=~(0x01);//is turn off started? if yes then turn off (hysteresis guaranted)
				//pin.gpv&=~(0x01);
				if (pseudo_latch_pinos&PINO) beep();
				pseudo_latch_pinos&=~(PINO);
				set_latch_pinos();
				//beep();
			}
		} else pin->gpv&=~(0x02);
	}
	//}
}
void exit_menu()
{
	menu=0x00;
	prepare_lcd();
	clear_display();
	cursor_off(); // Added 27/07/2015
	//temporizador=1;
	//temporizador|=TIMER1_ENABLE;
	read_write_temporizador(temporizador|TIMER1_ENABLE);
	set_none();
}
//void monitor_pins(PINOS_t &pin_c, unsigned char PINO_C)
void monitor_pins(struct PINOS_t *pin_c, unsigned char PINO_C) // C compatible
{
	n=(pin_c->command&0x07);
	if (n==1)
	{
		// 0tsXY|ccc <-- command | s is started turn on and t is started turn off
		n=((hours==pin_c->hh1)&&(minutes==pin_c->mm1));
		if (pin_c->command&0x10) // turn on at hh1:mm1
		{
			//n2
			if ((n)&&((pin_c->command&0x20)==0)) //  turn on is  not started yet
			{
				pseudo_latch_pinos|=PINO_C;
				set_latch_pinos();
				pin_c->command|=0x20; // is started !!!
				beep();
			}
		}
		n2=((hours==pin_c->hh2)&&(minutes==pin_c->mm2));
		if (pin_c->command&0x08) // turns off at hh2:mm2
		{
			//n2&&=((pino1.command&0x40)==0);
			if ((n2)&&((pin_c->command&0x40)==0)) //turn off is not started yet
			{
				pseudo_latch_pinos&=~(PINO_C);
				set_latch_pinos();
				pin_c->command|=0x40; // is started;
				beep();
			}

		}
		//if ((pin_c.command&0x20)&&(!(n))) pin_c.command&=~(0x20);// turn on is ready again
		//if ((pin_c.command&0x40)&&(!(n2))) pin_c.command&=~(0x40); // turn off is ready again.
		if (pin_c->command&0x20)
		{
			if (!(n)) pin_c->command&=~(0x20);
		}

		if (pin_c->command&0x40)
		{
			if (!(n2)) pin_c->command&=~(0x40);
		}
	}
	else if (n==2)
	{
		// 000sX|ccc <-- command s=1-->  is started
		// if s=1 <-- timer running else time NOT running
		//n2=(pin_c.command&0x10==0);
		if ((pin_c->command&0x10)==0)
		{
			//timer_tmp=pin_c.countdown1;
			//if (count_down_timer(pin_c.countdown1))
			if (count_down_timer(&pin_c->countdown1)) // C compatible
			{
				//if (pin_c.command&0x04) // turns on
				if (pin_c->command&0x08)
					pseudo_latch_pinos|=PINO_C;
				else
					pseudo_latch_pinos&=~(PINO_C); //turns off
				pin_c->command|=0x10;
				set_latch_pinos();
				beep();
			}
			//pin_c.countdown1=timer_tmp;
		}
	}
	else if (n==3)
	{
		// 000sX|ccc <-- command s=1 --> turn on countdown is started
		// command s=1 --> s=0 --> turn off countdown is started
		if (pin_c->command&0x10)
		{
			//timer_tmp=pin_c.countdown2;
			//if (count_down_timer(pin_c.countdown2))
			if (count_down_timer(&pin_c->countdown2)) // C compatible
			{
				//pin_c.countdown2=(unsigned int)(3600*pin_c.hh4+60*pin_c.mm4+pin_c.ss4);
				pin_c->countdown3=(unsigned int)(3600*pin_c->hh5+60*pin_c->mm5+pin_c->ss5);
				pin_c->command&=~(0x10);
				pseudo_latch_pinos|=PINO_C;
				set_latch_pinos();
				beep();
			}
			//pin_c.countdown2=timer_tmp;
		}
		else
		{
			//timer_tmp=pin_c.countdown3;
			//if (count_down_timer(pin_c.countdown3))
			if (count_down_timer(&pin_c->countdown3)) // C compatible
			{
				//pin_c.countdown3=(unsigned int)(3600*pin_c.hh5+60*pin_c.mm5+pin_c.ss5);
				pin_c->countdown2=(unsigned int)(3600*pin_c->hh4+60*pin_c->mm4+pin_c->ss4);
				pin_c->command|=(0x10);
				pseudo_latch_pinos&=~(PINO_C);
				set_latch_pinos();
				beep();
			}
			//pin_c.countdown3=timer_tmp;
		}
	}
	else if (n==4)
		sub_monitor_pins_temperature(pin_c, PINO_C, t1); // C compatible
	else if (n==5)
		sub_monitor_pins_temperature(pin_c, PINO_C, t2); // C compatible
	else if (n==6)
		sub_monitor_pins_battery(pin_c, CHARGER1_OFF, b1, PINO_C); // C compatible
	else if (n==7)
		sub_monitor_pins_battery(pin_c, CHARGER2_OFF, b2, PINO_C); // C compatible
}
void display_battery_status(unsigned char CHARGERX_C, unsigned char BX_LED_SIGNAL_B, double bx_c)
{
	if (bx_c > low_voltage)
	{
		// FIXED may 20 2015 (Wed)
		if (pseudo_latch_aux&CHARGERX_C)
			strcpy_P(conv, ONE_HUNDRED_STR);
		else
			dtostrf(bx_c*inv_100_charged_voltage,1,1,conv);
		print_lcd2(conv);
		lcdData('%');
		// Exemplo B1:100.0% B2:100.0%
	}
	else lcdData('x');//print_lcd2("x");
}
void show_other_menu()
{
	//char str_tmp_other_menu[9];
	unsigned char a=(language&0x01);
	menu=OTHER_MENU;
	prepare_lcd();
	clear_display();
	cursor_off();
	/*
	if (language)
	{
		print_lcd2("OTHER CONF.");
		//strcpy(str_tmp_other_menu, "Language");
	}
	else
	{
		print_lcd2("OUTRAS CONF.");
		//strcpy(str_tmp_other_menu, "Idioma");
	}
	*/
	print_lcd2_P(OTHER_CONF_STR[a]);
	//SetCursor(3,1);
	SetCursor(2,1);
	//print_lcd2("1-Language/Idioma");
	//print_lcd2("1-");
	print_lcd2_P(LANGUAGE_STR);
	SetCursor(3,1);
	print_lcd2("2-");
	print_lcd2_P(TEMP_MEASURE[a]);
	SetCursor(4,1);
	print_lcd2("3-");
	print_lcd2_P(UART_FOR_IOT_MENU_STR);
	//print_lcd2(str_tmp_other_menu);
	set_none();
}
void show_other_menu_language()
{
	char buffer[16];
	menu=OTHER_MENU_LANGUAGE;
	prepare_lcd();
	clear_display();
	//if (language&0x01)
	strcpy_P(buffer, LANGUAGE_STR);
		//print_lcd2("LANGUAGE/IDIOMA");
	print_lcd2(strupr_b(buffer));
	//else
	//	print_lcd2("IDIOMA");
	SetCursor(3,1);
	//print_lcd2("1-<US-ENGLISH>");
	print_lcd2_P(LANG[1]);
	SetCursor(4,1);
	//print_lcd2("2-<BR-PORT.>");
	print_lcd2_P(LANG[0]);
	set_none();
}
void show_temp_measure()
{
	char buffer[16];
	menu=OTHER_MENU_TEMP_MEASUREMENT;
	prepare_lcd();
	clear_display();
	strcpy_P(buffer,TEMP_MEASURE[language&0x01]);
	print_lcd2(strupr_b(buffer));
	SetCursor(3,1);
	print_lcd2("1-");
	lcdData(0x05); // °
	lcdData('C');
	SetCursor(4,1);
	print_lcd2("2-");
	lcdData(0x05);
	lcdData('F');
	set_none();
}
//show_uart_for_iot_configure_port();
//show_uart_for_iot_configure_security();
/*
void show_menu_password()
{
	char buffer[10];
	menu=OTHER_MENU_UART_FOR_IOT_PASSWD;
//	prepare_lcd();
//	clear_display();
	strcpy_P(buffer, PASSWORD_STR[(language&0x01)]);
	print_lcd2(strupr_b(buffer));
	SetCursor(3,1);
	cursor_on();
//	set_none();
}

void show_uart_menu_for_iot()
{
	// UART for IoT
	// 1-Configure port
	// 2-Security attributes
	// 3-Set PASSWORD
	unsigned char a=(language&0x01);
	//char buffer[20];
	menu=OTHER_MENU_CONFIGURE_UART;
	prepare_lcd();
	cursor_off();
	clear_display();
	//strcpy_P(buffer, UART_FOR_IOT_MENU_STR);
	//print_lcd2(strupr_b(buffer));
	print_lcd2_P(UART_FOR_IOT_MENU_STR);
	SetCursor(2,1);
	print_lcd2("1-");
	print_lcd2_P(SET_CONFIGURE_PORT[a]);
	SetCursor(3,1);
	print_lcd2("2-");
	print_lcd2_P(SET_SECURITY[a]);
	SetCursor(4,1);
	print_lcd2("3-");
	print_lcd2_P(SET_PASSWORD[a]);
	set_none();
}
*/
unsigned char check_user_password()
{
	static unsigned char tmp_c[HASH_BLOCK];
	static unsigned char tmp_rom[HASH_BLOCK];
	//unsigned char a=strlen(input);
	clear_passwd(passwd);
	read_password_EEPROM(tmp_rom);
	memcpy(passwd, input, strlen(input));
	memcpy(tmp_c, digest(passwd), HASH_BLOCK);
	return (memcmp(tmp_rom, tmp_c, HASH_BLOCK)==0);
}
/*
void sub_show_uart_menu_for_iot(unsigned char lang)
{
	menu=OTHER_MENU_CONFIGURE_UART;
	cursor_off();
	print_lcd2_P(UART_FOR_IOT_MENU_STR);
	SetCursor(2,1);
	print_lcd2("1-");
	print_lcd2_P(SET_CONFIGURE_PORT[lang]);
	SetCursor(3,1);
	print_lcd2("2-");
	print_lcd2_P(SET_SECURITY[lang]);
	SetCursor(4,1);
	print_lcd2("3-");
	print_lcd2_P(SET_PASSWORD[lang]);
}
*/
void show_uart_menu_for_iot(unsigned char login)
{
	// UART for IoT
	// 1-Configure port
	// 2-Security attributes
	// 3-Set PASSWORD
	unsigned char a=(language&0x01);
	static unsigned char tmp_a[HASH_BLOCK];
	static char buffer[10];
	clear_input(0);
	prepare_lcd();
	clear_display();
	read_password_EEPROM(tmp_a);
	if (is_null_pass(tmp_a)||login)
	{
		//sub_show_uart_menu_for_iot(a);
		menu=OTHER_MENU_CONFIGURE_UART;
		cursor_off();
		print_lcd2_P(UART_FOR_IOT_MENU_STR);
		SetCursor(2,1);
		print_lcd2("1-");
		print_lcd2_P(SET_CONFIGURE_PORT[a]);
		SetCursor(3,1);
		print_lcd2("2-");
		print_lcd2_P(SET_SECURITY[a]);
		SetCursor(4,1);
		print_lcd2("3-");
		print_lcd2_P(SET_PASSWORD[a]);
	}
	else
	{
		menu=OTHER_MENU_UART_FOR_IOT_PASSWD;
		strcpy_P(buffer, PASSWORD_STR[a]);
		print_lcd2(strupr_b(buffer));
		SetCursor(3,1);
		cursor_on();
	}
	set_none();
}
void show_uart_for_iot_configure_port()
{
	// CONFIGURE PORT
	// 1-Speed    4-UART2x
	// 2-Parity
	// 3-Load Defaut

	// CONFIGURAR PORTA
	// 1-Veloc.   4-UART2x
	// 2-Paridade
	// 3-Padrão
	char buffer[20];
	unsigned char a=(language&0x01);
	menu=OTHER_MENU_CONFIGURE_PORT_UART;
	prepare_lcd();
	cursor_off();
	clear_display();
	strcpy_P(buffer, SET_CONFIGURE_PORT[a]);
	print_lcd2(strupr_b(buffer));
	SetCursor(2,1);
	print_lcd2("1-");
	print_lcd2_P(UART_SPEED_CONFIGURATION_STR[a]);
	SetCursor(2,11);
	print_lcd2("4-");
	print_lcd2_P(UART_UART2X_STR);
	SetCursor(3,1);
	print_lcd2("2-");
	print_lcd2_P(UART_PARITY_CONFIGURATION_STR[a]);
	SetCursor(4,1);
	print_lcd2("3-");
	print_lcd2_P(UART_DEFAULT_CONFIGURATION_STR[a]);
	set_none();
}

void show_uart_for_iot_configure_speed()
{
	unsigned char a=(language&0x01);
	char buffer[20];
	clear_input(0);
	menu=OTHER_MENU_CONFIGURE_SPEED_UART;
	prepare_lcd();
	clear_display();
	strcpy_P(buffer, UART_SPEED_CONFIGURATION_STR[a]);
	print_lcd2(strupr_b(buffer));
	SetCursor(2,1);
	print_lcd2_P(UART_SPEED_CONFIGURATION_STR[a]);
	lcdData(':');
	utoa(EEPROM_read(UART_CONF_EEPROM_ADDRESS)&0x0F, buffer, 10);
	print_lcd2(buffer);
	SetCursor(3,1);
	cursor_on();
	set_none();
}

void show_uart_for_iot_configure_parity()
{
	char buffer[20];
	//unsigned char a=(language&0x01); //??? Fixed??? 08/31/2015
	unsigned char a=(language&0x01);
	//menu=OTHER_MENU_CONFIGURE_PARITY_UART;
	//menu=
	prepare_lcd();
	clear_display();
	strcpy_P(buffer, UART_PARITY_CONFIGURATION_STR[a]);
	//print_lcd2_P(strupr_b(buffer)); 
	print_lcd2(strupr_b(buffer));
	SetCursor(2,1);
//1-[X]
	print_lcd2("1-");
	print_lcd2_P(INDEX_MARK);
	print_lcd2_P(UART_PARITY_NONE[a]);
	SetCursor(3,1);
	print_lcd2("2-");
	print_lcd2_P(INDEX_MARK);
	print_lcd2_P(UART_PARITY_EVEN[a]);
	SetCursor(4,1);
	print_lcd2("3-");
	print_lcd2_P(INDEX_MARK);
	print_lcd2_P(UART_PARITY_ODD[a]);
/*
	a=0x01;
	switch (EEPROM_read(UART_CONF_EEPROM_ADDRESS)&0x30)
	{
		case UART_PARITY_MODE_ODD_PARITY: a++;
		case UART_PARITY_MODE_EVEN_PARITY: a++;
		default: a++;
	}
	SetCursor(a,4);
*/
/*
    4d9e:	8d ef       	ldi	r24, 0xFD	; 253
    4da0:	93 e0       	ldi	r25, 0x03	; 3
    4da2:	0e 94 b2 0a 	call	0x1564	; 0x1564 <asm_EEPROM_READ>
    4da6:	80 73       	andi	r24, 0x30	; 48
    4da8:	80 32       	cpi	r24, 0x20	; 32
    4daa:	11 f4       	brne	.+4      	; 0x4db0 <_Z34show_uart_for_iot_configure_parityv+0xe8>
    4dac:	83 e0       	ldi	r24, 0x03	; 3
    4dae:	01 c0       	rjmp	.+2      	; 0x4db2 <_Z34show_uart_for_iot_configure_parityv+0xea>
    4db0:	84 e0       	ldi	r24, 0x04	; 4
    4db2:	64 e0       	ldi	r22, 0x04	; 4
    4db4:	0e 94 ca 06 	call	0xd94	; 0xd94 <asm_SETCURSOR>
BUGFIX FIXME else is not compiling then I've tried asm
	menu=(EEPROM_read(UART_CONF_EEPROM_ADDRESS)&0x30); // use menu as temporary register.

	if (menu==UART_PARITY_MODE_EVEN_PARITY)
	{
		a=0x03;
		//SetCursor(3,4);
	}
	else if (menu==UART_PARITY_MODE_ODD_PARITY)
	{
		a=0x04;
		//SetCursor(4,4);
	}
	else
		a=0x02;
*/
	const unsigned char UART_CONF_HI=(UART_CONF_EEPROM_ADDRESS>>8);
	const unsigned char UART_CONF_LO=(UART_CONF_EEPROM_ADDRESS&0xFF);
	const unsigned char K_EVEN=UART_PARITY_MODE_EVEN_PARITY;
	const unsigned char K_ODD=UART_PARITY_MODE_ODD_PARITY;

	asm volatile
	(
		"call asm_EEPROM_READ" "\n\t"
		"andi %0, 0x30" "\n\t" // Fixed 10/02/2015
		"cp %0, %3" "\n\t"
		"brne IF_IS_EVEN" "\n\t"
		"ldi %0, 0x04" "\n\t"
		"rjmp SAIR_IF" "\n\t"
	"IF_IS_EVEN:" "\n\t"
		"cp %0, %2"	"\n\t"
		"brne IF_ELSE" "\n\t"
		"ldi %0, 0x03" "\n\t"
		"rjmp SAIR_IF" "\n\t"
    "IF_ELSE:" "\n\t"
		"ldi %0, 0x02" "\n\t"
	"SAIR_IF:" "\n\t"
		"ldi r22, 0x04" "\n\t"
		"call asm_SETCURSOR" "\n\t"
	:
	:"r"(UART_CONF_LO), "r"(UART_CONF_HI), "r"(K_EVEN),"r"(K_ODD)
	);
	//SetCursor(a,4);

	lcdData('X');
	set_none();
	menu=OTHER_MENU_CONFIGURE_PARITY_UART; // now menu is doing it's work !
}

void show_uart_for_iot_configure_default(unsigned char what_menu)
{
	unsigned char a=(language&0x01);
	char buffer[20];
	//menu=OTHER_MENU_CONFIGURE_DEF;
	menu=what_menu;
	prepare_lcd();
	clear_display();
	strcpy_P(buffer, UART_DEFAULT_CONFIGURATION_STR[a]);
	print_lcd2(strupr_b(buffer));
	SetCursor(2,1);
	print_lcd2_P(UART_DEFAULT_CONF_QUESTION_STR[a]);
	print_lcd2_P(UART_DEFAULT_CONFIGURATION_STR[a]);
	//lcdData((unsigned char)pgm_read_byte_near(&QUESTION_STR));
	lcdData('?');
	SetCursor(3,1);
	print_lcd2("1-");
	print_lcd2_P(YES_STR[a]);
	SetCursor(4,1);
	print_lcd2("2-");
	print_lcd2_P(NO_STR[a]);
	set_none();
//teste();
}

void show_uart_for_iot_configure_uart2x()
{
	unsigned char a=(language&0x01);
	menu=OTHER_MENU_CONFIGURE_UART2X;
	prepare_lcd();
	clear_display();
	print_lcd2_P(UART_UART2X_STR);
	SetCursor(2,1);
	print_lcd2("1-");
	print_lcd2_P(UART_UART2X_ENABLED_STR[a]);
	SetCursor(3,1);
	print_lcd2("2-");
	print_lcd2_P(UART_UART2X_DISABLE_STR[a]);
	//Status: Desligado
	SetCursor(4,1);
	print_lcd2_P(UART_UART2X_STATUS_STR);
	if (EEPROM_read(UART_CONF_EEPROM_ADDRESS)&UART_2X_SPEED_ENABLE)
		print_lcd2_P(UART_UART2X_ENABLED_STR[a]);
	else
		print_lcd2_P(UART_UART2X_DISABLE_STR[a]);
	set_none();
}
void show_uart_for_iot_configure_security()
{
	// Security attributes
	//
	// 1- Permission
	// 2- Default
	unsigned char a=(language&0x01);
	char buffer[20];
	menu=OTHER_MENU_CONFIGURE_UART_SECURITY;
	prepare_lcd();
	clear_display();
	cursor_off();
	strcpy_P(buffer, SET_SECURITY[a]);
	print_lcd2(strupr_b(buffer));
	SetCursor(3,1);
	print_lcd2("1-");
	print_lcd2_P(UART_ROOT_ACCESS_PERMISSION_STR[a]);
	SetCursor(4,1);
	print_lcd2("2-");
	print_lcd2_P(UART_DEFAULT_CONFIGURATION_STR[a]);
	set_none();
}
void show_uart_for_iot_configure_sec_permission()
{
	// 0xFFFF,65536
	unsigned char a=(language&0x01);
	char buffer[20];
	unsigned int Flag;
	clear_input(0);
	menu=OTHER_MENU_CONFIGURE_UART_SEC_PERM;
	//Flag=((EEPROM_read(FLAG_H_ADDR)<<8)|EEPROM_read(FLAG_L_ADDR));
	Flag=to_uint_16(EEPROM_read(FLAG_H_ADDR),EEPROM_read(FLAG_L_ADDR));
	//Flag.access.low=EEPROM_read(FLAG_L_ADDR);
	prepare_lcd();
	clear_display();
	strcpy_P(buffer, UART_ROOT_ACCESS_PERMISSION_STR[a]);
	print_lcd2(strupr_b(buffer));
	SetCursor(4,1);
	print_lcd2(utoa(Flag, buffer, 16));
	lcdData(',');
	print_lcd2(utoa(Flag, buffer, 10));
	SetCursor(3,1);
	cursor_on();
	set_none();
}

void show_set_password()
{
	unsigned char a=(language&0x01);
	unsigned char tmp_b[HASH_BLOCK];
	char buffer[14];
	menu=OTHER_MENU_SET_PASSWORD;
	clear_input(0);
	read_password_EEPROM(tmp_b);
	prepare_lcd();
	clear_display();
	strcpy_P(buffer, SET_PASSWORD[a]);
	//print_lcd2_P(strupr_b(buffer));
	print_lcd2(strupr_b(buffer)); // Fixed 07/13/2015
	SetCursor(2,1);
	if (is_null_pass(tmp_b))
	{
		conv[0]=0x02; // for flag 1 -> old pass, 2 -> new pass and finally RETYPE PASS
		print_lcd2_P(NEW_PASSWORD_STR[a]);
	}
	else
	{
		conv[0]=0x01;
		print_lcd2_P(OLD_PASSWORD_STR[a]);
	}
	lcdData(':');
	SetCursor(3,1);
	cursor_on();
	set_none();
}
void dynamic()
{
	//n=menu_kbd_interval('0','7');
	if (menu_kbd_interval('0','7'))
	{
		switch (code_cr)
		{
			case '1': pseudo_latch_pinos^=PINO1; break;
			case '2': pseudo_latch_pinos^=PINO2; break;
			case '3': pseudo_latch_pinos^=PINO3; break;
			case '4': pseudo_latch_pinos^=PINO4; break;
			case '5': pseudo_latch_pinos^=PINO5; break;
			case '6': pseudo_latch_pinos^=PINO6;
		}
		//PORTD=pseudo_latch_pinos;
		set_latch_pinos();
		//beep();
	}
	n=hoursA&0x3F;
	if ((hours==n)&&(minutes==minutesA)) hoursA&=0xBF;
}
int main(void)
{
	uint32_t Memory_b;
	// Configurando o controle interno do MCU
	DDRD=((1<<PD7) | (1<<PD6) | (1<<PD5) | (1<<PD4) | (1<<PD3) | (1<<PD2)); //set all as output pins
	//DDRB=((1<<PB4)|(1<<PB2)); // set PB4 and PB2 as output pins
	DDRB=((1<<PB4)|(1<<PB2)|(1<<PB1)|(1<<PB3)); // set PB4 and PB2 as output pins. Now (02/26/2016) as a LED_TX and LED_RX

	DIDR0=((1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D)); // disable digital input/output pins. Set them to analogical pins
	//ADMUX|=(1<<REFS0); // set Avcc with output capacitor pin.
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // set factor to 128 for all conversions

	//PCICR=(1<<PCIE1); // ativa interrupção em todos os pinos PCI23:8
	PCICR=(1<<PCIE0); // ativa interrupção em todos os pinos PCI7:0
	//PCMSK1=(1<<PCINT8);//mascara a interrupção todos os pinos PCI23:9 menos o pino 0
	PCMSK0=(1<<PCINT0);//mascara a interrupção todos os pinos PCI7:0 menos o pino 0

	//UCSR0A=(1<<RXCIE0); // UART Tx complete interrupt enabled;

	//TCCR0B|=(1<<CS01)|(1<<CS00); // prescale by fs/64 the counter 0
	//TCCR0B|=(1<<CS02); // prescale by fs/256 (counter 0)
	//TCCR1B|=(1<<CS12)|(1<<CS10); // preescale by fs/1024 the counter 1 (16 bit)
	TCCR1B=(1<<CS12)|(1<<CS10); // preescale by fs/1024 the counter 1 (16 bit)
	//TCCR1B|=(1<<CS12); // preescale by fs/256 the counter 1 (16 bit)
	// Fim de configuração do controle interno

	// PORTC = 0b111000		
	PORTC=0x38; // Set pull-up resistor in PC3, PC4 (future use) and PC5 (future use)

	// About 2-wire:

	TWBR=0xFF; // 2-wire frequency about 7.78 kHz @ 16 MHz CPU clock
	TWSR=(1<<TWPS0);

	// End 2-wire

	set_none();
	pseudo_latch_aux=(CHARGER2_OFF|CHARGER1_OFF|B1_LED_SIGNAL|B2_LED_SIGNAL);
	pseudo_latch_aux&=~(LCD_ON); // adicionado em depuracao;

	set_latch_aux();
	pseudo_latch_pinos=0x00;
	set_latch_pinos();
	pseudo_latch_aux|=LCD_ON;
	set_latch_aux();
	_delay_ms(100);
	readLanguageEEPROM();// load a preset language (English / Portuguese)
	prepare_lcd();

	init_lcd();
	load_custom_char();
	display_on();
	clear_display();
	lcd_return_home();
	SetCursor(2,1);
	/*
	if (language)
		print_lcd2("Starting");
	else
		print_lcd2("Iniciando");
	*/
	print_lcd2_P(LANG_STARTUP[language&0x01]);
	//print_lcd2("...");
	replicate_char('.',3);
	set_none();
	n=read_from_rtc(RTC_DAYS_REG);
	n&=0x3F;
	if (n==0x00)
	{
		write_to_rtc(n, RTC_SECONDS_REG);

		write_to_rtc(n, RTC_MINUTES_REG);

		write_to_rtc(n, RTC_HOURS_REG);

		write_to_rtc(0x06, RTC_DAYS_REG);

		write_to_rtc((IS_19XX|0x07), RTC_MONTHS_CENTURY_REG);
		write_to_rtc(0x81, RTC_YEARS_REG); // fixed
		//n=getDay(1981,7,6); // Monday (1)
		write_to_rtc(0x01, RTC_WEEKDAYS_REG);
	}

	read_from_rtc(RTC_SECONDS_REG);
	read_from_rtc(RTC_MINUTES_REG);
	read_from_rtc(RTC_HOURS_REG);
	read_from_rtc(RTC_DAYS_REG);
	read_from_rtc(RTC_WEEKDAYS_REG);
	read_from_rtc(RTC_MONTHS_CENTURY_REG);
	read_from_rtc(RTC_YEARS_REG);
	//pseudo_latch_aux|=0x02; // sounds beep at startup
	//pseudo_latch_aux|=(language&0x02);
	beep();
	PORTB|=((UART_LED_TX_OFF)|(UART_LED_RX_OFF)); // turns off led tx and rx (02/26/2016)
	TIMSK1=(1<<TOIE1); // An approx 1 second is call for update a temperature sensor and sense the voltage of the charger.

	// read EEPROM event
	readAlarmEEPROM();
	/*readPinoEEPROM(pino1, PINO1_OFFSET_ADDR);
	readPinoEEPROM(pino2, PINO2_OFFSET_ADDR);
	readPinoEEPROM(pino3, PINO3_OFFSET_ADDR);
	readPinoEEPROM(pino4, PINO4_OFFSET_ADDR);
	readPinoEEPROM(pino5, PINO5_OFFSET_ADDR);
	readPinoEEPROM(pino6, PINO6_OFFSET_ADDR);*/

	readPinoEEPROM(&pino1, PINO1_OFFSET_ADDR);
	readPinoEEPROM(&pino2, PINO2_OFFSET_ADDR);
	readPinoEEPROM(&pino3, PINO3_OFFSET_ADDR);
	readPinoEEPROM(&pino4, PINO4_OFFSET_ADDR);
	readPinoEEPROM(&pino5, PINO5_OFFSET_ADDR);
	readPinoEEPROM(&pino6, PINO6_OFFSET_ADDR);

	// end EEPROM event

	// Discard firt sensors read
	//read_sensor(TEMP1_SENSOR);
	//read_sensor(TEMP2_SENSOR);
	//read_sensor(B1_SENSOR);
	//read_sensor(B2_SENSOR);
	read_all_sensor();
	// End of discard
	clear_input(0);
	init_UART(EEPROM_read(UART_CONF_EEPROM_ADDRESS)); // initializing UART
	sei(); // set all interrupts

	for (;;)
	{

		_delay_ms(4);
		decodificar(); // decodifica o controle remoto
		read_kbd();


		//n=(cr_start==2) && (code_cr!=0xff);
		//if (n)
		//if (n)
		//if (((cr_start&0x06)>0)&&(menu<0xFF)) // 0xFF UART access LCD
		//if (cr_start&0x02) // depuração
		//if ((code_cr!=0xFF)&&(menu!=0xFF))
		if ((code_cr>0x01)&&(menu!=0xFF))
		{
/*
			cli();
			prepare_lcd();
				lcd_clear_line(3);
				//ultoa(key, conv, 16);
				ultoa(code_cr, conv, 16);
				print_lcd2(conv);
				//code_cr=0xFE;
			set_none();
			sei();
*/
			if ((code_cr==MENU_KEY)&&(menu==0x00)) show_initial_menu();
			else if (code_cr==RETURN_KEY)
			{
				if (menu==IS_MENU)
				{
					exit_menu();
/*					
					menu=0x00;
					prepare_lcd();
					clear_display();
					temporizador=1;
					set_none();
*/
				}

			}
			else if (code_cr==BEEP_ON_OFF)
			{
				//uses pseudo_latch_aux bit 1 as flag
				language^=BEEP_ENABLE;
				writeLanguageEEPROM();
				//pseudo_latch_aux^=0x02;

			}

			if (menu==IS_MENU)
			{
					if (code_cr=='1') show_set_date_time();
					else if (code_cr=='2') show_pino_menu();
					else if (code_cr=='3') show_sensor_menu();
					else if (code_cr=='4') show_menu_calculator();
					else if (code_cr=='5') show_other_menu();
					else if (code_cr=='6') about();
			}
			else if (menu==DATE_TIME_MENU)
			{
				if (code_cr==RETURN_KEY) show_initial_menu();
				else
				{
					switch (code_cr)
					{
						case '1': set_time_date();break;
						case '2': find_weekday(); break;
						case '3': set_alarm();
					}

				}
			}
			else if (menu==DATE_TIME_ADJUST)
			{
				insert_input(0);
				if (code_cr==RETURN_KEY) show_set_date_time();
				else if (code_cr==ENTER_KEY)
				{

					if (is_time_date_valid_input(0))
					{
						n=(DT.ss[0]-0x30)<<4;
						n|=(DT.ss[1]-0x30);
						write_to_rtc(n, RTC_SECONDS_REG);

						n=(DT.mm[0]-0x30)<<4;
						n|=(DT.mm[1]-0x30);
						write_to_rtc(n, RTC_MINUTES_REG);

						n=(DT.hh[0]-0x30)<<4;
						n|=(DT.hh[1]-0x30);						

						write_to_rtc(n, RTC_HOURS_REG);

						n=(DT.day[0]-0x30)<<4;
						n|=(DT.day[1]-0x30);						

						write_to_rtc(n, RTC_DAYS_REG);

						n=(DT.month[0]-0x30)<<4;
						n|=(DT.month[1]-0x30);
						if ((input[0]=='1')&&(input[1]=='9')) n|=0x80;
						write_to_rtc(n, RTC_MONTHS_CENTURY_REG);

						n=getDay(atoi(DT.year),atoi(DT.month),atoi(DT.day));
						write_to_rtc(n, RTC_WEEKDAYS_REG);

						n=(DT.year[2]-0x30)<<4;
						n|=(DT.year[3]-0x30);
						write_to_rtc(n, RTC_YEARS_REG);
					}
					clear_input(0);
				}
				else if (code_cr==CLEAR_KEY)
				{
					clear_input(1);
				}

			}
			else if (menu==DATE_TIME_WEEK_DAY)
			{
				insert_input(0);
				if  (code_cr==RETURN_KEY) show_set_date_time();
				else if (code_cr==ENTER_KEY)
				{
					if (is_time_date_valid_input(1))
					{
						prepare_lcd();
						lcd_clear_line(4);
						get_week_day((unsigned char)getDay(atoi(DT.year), atoi(DT.month),atoi(DT.day)));
						print_lcd2(conv);
						SetCursor(3,1);
						set_none();
					}
					clear_input(0);
				}
				else if (code_cr==CLEAR_KEY)
				{

					clear_input(1);

				}
			}
			else if (menu==ALARM_SETUP)
			{
				insert_input(0);
				if (code_cr==RETURN_KEY) show_set_date_time();
				else if (code_cr==ENTER_KEY)
				{
					prepare_lcd();
					lcd_clear_line(4);

					if (input[0]=='1')
					{
						DT.hh[0]=input[1];
						DT.hh[1]=input[2];
						DT.mm[0]=input[3];
						DT.mm[1]=input[4];
						// FIXED may 14 2015 Thu -> Turn-on off setup input
						//
						//hoursA=(unsigned char)atoi(DT.hh);
						//minutesA=(unsigned char)atoi(DT.mm);
						//if (valid_time(hoursA, minutesA, 0x00))
						if (valid_time((unsigned char)atoi(DT.hh), (unsigned char)atoi(DT.mm), 0x00))
						{
							if (strlen(DT.hh)>1)
							{
								n=(input[1]-0x30)<<4;
								//n&=0x3F;
								hoursA=n|(input[2]-0x30);
							}
							if (strlen(DT.mm)>1)
							{
								n=(input[3]-0x30)<<4;
								minutesA=n|(input[4]-0x30);
							}
							hoursA|=0xC0;
							/// 
							cli();
							writeAlarmEEPROM();
							sei();
							//print_lcd2("OK!");
							print_lcd2_P(STATUS_OK);
						} else print_lcd2_P(ERR_ALARM_CONF_STR); //print_lcd2("Err:alarm conf.");
						//clear_input(0);
					}
					else if (input[0]=='0') //else if (input[1]=='0')
					{
						hoursA&=0x3F;
						cli();
						writeAlarmEEPROM();
						sei();
						//print_lcd2("OK!");
						print_lcd2_P(STATUS_OK);
					} else 
						print_lcd2_P(ERR_INPUT_STR);//print_lcd2("Err:input");
					SetCursor(3,1);
					set_none();
					clear_input(0);
				}
				else if (code_cr==CLEAR_KEY)
				{
					clear_input(1);
				}
			}
			else if (menu==PINO_MENU)
			{
				if (code_cr==RETURN_KEY) show_initial_menu();
				else
				{
					if (code_cr=='1') set_pino_menu(PINO_MENU_P1);
					else if (code_cr=='2') set_pino_menu(PINO_MENU_P2);
					else if (code_cr=='3') set_pino_menu(PINO_MENU_P3);
					else if (code_cr=='4') set_pino_menu(PINO_MENU_P4);
					else if (code_cr=='5') set_pino_menu(PINO_MENU_P5);
					else if (code_cr=='6') set_pino_menu(PINO_MENU_P6);

				}

			}
			else if ((menu==PINO_MENU_P1)||(menu==PINO_MENU_P2)||(menu==PINO_MENU_P3)||(menu==PINO_MENU_P4)||(menu==PINO_MENU_P5)|(menu==PINO_MENU_P6))
			{
				n=(code_cr==PLUS_MINUS_KEY);
				//n2=n&&((pos_input==2)||(pos_input==7));
				n2=n&&((pos_input==3)||(pos_input==8));
				//insert_input(code_cr==PLUS_MINUS_KEY);
				insert_input(n2&&((input[0]=='4')||(input[0]=='5')));
				//if (is_return_key()) show_pino_menu();
				if (code_cr==RETURN_KEY) show_pino_menu();
				else if (code_cr==ENTER_KEY)
				{
					// decodificando
					set_pino();
					clear_input(0);
					// fim de decodificação
				}
				else if (code_cr==CLEAR_KEY)
					clear_input(1);
			}
			else if (menu==SENSOR_MENU)
			{
				//if (is_return_key()) show_initial_menu();
				dynamic();
				if (code_cr==RETURN_KEY) show_initial_menu();
				//else
				//{
				//	temporizador=0;
				//	show_sensor_menu();
				//}
			}
			else if (menu==CALCULATOR_MENU)
			{
				//if (is_return_key()) show_initial_menu();
				if (code_cr==RETURN_KEY) show_initial_menu();
				else sub_show_menu_calculator();
			}
			else if (menu==BASIC_CALC_MENU) //else if (menu==CALCULATOR_MENU)
			{
				basic_calculator();
			}
			else if (menu==SCIENTIFIC_MENU)
			{
				scientific_calculator();
			}
			else if (menu==OTHER_MENU)
			{
				if (code_cr==RETURN_KEY) show_initial_menu();
				else if (code_cr=='1') show_other_menu_language();
				else if (code_cr=='2') show_temp_measure();
				else if (code_cr=='3') show_uart_menu_for_iot(0);
			}
			else if (menu==OTHER_MENU_LANGUAGE)
			{
				if (code_cr==RETURN_KEY) show_other_menu();
				else if ((code_cr=='1')||(code_cr=='2'))
				{
					//n=((code_cr-0x30)&0x01); // 0 or 1
					//if (n!=language)
					//if (n!=(language&0x01))
					if ((code_cr&0x01)^(language&0x01))
					{
						//language=n;
						//language&=0xFE;
						//language|=n;
						//cli();
						language^=0x01;
						writeLanguageEEPROM();
						//sei();
					}
					//show_other_menu_language();
					show_initial_menu();
				}
			}
			else if (menu==OTHER_MENU_TEMP_MEASUREMENT)
			{
				if (code_cr==RETURN_KEY) show_other_menu();
				else if ((code_cr=='1')||(code_cr=='2'))
				{
					//n=((code_cr-0x30)&0x01); // 0 or 1
					//if (n!=language)
					//if ((code_cr-0x30)&0x01) n=0x80;
					if (code_cr&0x01) n=0x80;
					else n=0x00;
					//if (n!=(language&0x80))
					if (n^(language&0x80))
					{
						//language=n;
						//language&=0x7F;
						//language|=n;
						language^=0x80;
						//cli();
						writeLanguageEEPROM();
						//sei();
					}
					//show_other_menu_language();
					//show_initial_menu();
					exit_menu();
				}
			}
			else if (menu==OTHER_MENU_CONFIGURE_UART)
			{
				if (code_cr==RETURN_KEY) show_other_menu();
				else if (code_cr=='1') show_uart_for_iot_configure_port();
				else if (code_cr=='2') show_uart_for_iot_configure_security();
				else if (code_cr=='3') show_set_password();
			}
			else if (menu==OTHER_MENU_CONFIGURE_PORT_UART)
			{
				if (code_cr==RETURN_KEY) show_uart_menu_for_iot(1);
				else if (code_cr=='1') show_uart_for_iot_configure_speed();
				else if (code_cr=='2') show_uart_for_iot_configure_parity();
				else if (code_cr=='3') show_uart_for_iot_configure_default(OTHER_MENU_CONFIGURE_DEF);
				else if (code_cr=='4') show_uart_for_iot_configure_uart2x();
			}
			else if (menu==OTHER_MENU_CONFIGURE_SPEED_UART)
			{
				insert_input(0);
				if (code_cr==RETURN_KEY) show_uart_for_iot_configure_port();
				else if (code_cr==ENTER_KEY)
				{
					n=(unsigned char)atoi(input);
					prepare_lcd();
					SetCursor(4,1);
					if (n>0x0F)
						print_lcd2_P(ERR_OVRFLW);
					else
					{
						n2=((EEPROM_read(UART_CONF_EEPROM_ADDRESS)&0xF0)|n);
						//EEPROM_write(UART_CONF_EEPROM_ADDRESS, ((EEPROM_read(UART_CONF_EEPROM_ADDRESS)&0xF0)|n));
						init_UART(n2);
						EEPROM_write(UART_CONF_EEPROM_ADDRESS, n2);
						print_lcd2_P(STATUS_OK);
						SetCursor(2,1);
						print_lcd2_P(UART_SPEED_CONFIGURATION_STR[language&0x01]);
						lcdData(':');
						utoa(n, input, 10);
						print_lcd2(input);
					}
					SetCursor(3,1);
					set_none();
					clear_input(0);
				}
				else if (code_cr==CLEAR_KEY) clear_input(1);
			}
			else if (menu==OTHER_MENU_CONFIGURE_PARITY_UART)
			{

				if (code_cr==RETURN_KEY) show_uart_for_iot_configure_port();
				else if (menu_kbd_interval('0','4')) //Fixed 13/07/2015 (menu_kbd_interval(0,4))
				{
					//1-[ ]
					prepare_lcd();
					SetCursor(2,4);
					lcdData(' ');
					if (code_cr=='1')
					{
						SetCursor(2,4);
						n=UART_PARITY_MODE_DISABLE;
						lcdData('X');
					}
					SetCursor(3,4);
					lcdData(' ');
					if (code_cr=='2')
					{
						SetCursor(3,4);
						n=UART_PARITY_MODE_EVEN_PARITY;
						lcdData('X');
					}
					SetCursor(4,4);
					lcdData(' ');
					if (code_cr=='3')
					{
						SetCursor(4,4);
						n=UART_PARITY_MODE_ODD_PARITY;
						lcdData('X');
					}
					set_none();
					n2=EEPROM_read(UART_CONF_EEPROM_ADDRESS);
					if ((n2&0x30)^n)
					{
						n2&=(~(0x30));
						n2|=n;
						EEPROM_write(UART_CONF_EEPROM_ADDRESS, n2);
						init_UART(n2);
					}
				}

			}
			else if (menu==OTHER_MENU_CONFIGURE_DEF)
			{
				if ((code_cr==RETURN_KEY)||(code_cr=='2')) show_uart_for_iot_configure_port();
				else if (code_cr=='1')
				{
					if (EEPROM_read(UART_CONF_EEPROM_ADDRESS)^UART_DEFAULT_SPEED)
						EEPROM_write(UART_CONF_EEPROM_ADDRESS, UART_DEFAULT_SPEED);
					init_UART(UART_DEFAULT_SPEED);
					show_uart_for_iot_configure_port();
				}
			}
			else if (menu==OTHER_MENU_CONFIGURE_UART2X)
			{
				if (code_cr==RETURN_KEY) show_uart_for_iot_configure_port();
				else
				{
					n=EEPROM_read(UART_CONF_EEPROM_ADDRESS);
					if (code_cr=='1')
					{
						if ((n&UART_2X_SPEED_ENABLE)==0x00)
							EEPROM_write(UART_CONF_EEPROM_ADDRESS, (n|UART_2X_SPEED_ENABLE));
						prepare_lcd();
						lcd_clear_line(4);
						print_lcd2_P(UART_UART2X_STATUS_STR);
						print_lcd2_P(UART_UART2X_ENABLED_STR[language&0x01]);
						set_none();

					}
					else if (code_cr=='2')
					{
						if (n&UART_2X_SPEED_ENABLE)
							EEPROM_write(UART_CONF_EEPROM_ADDRESS, (n&(~UART_2X_SPEED_ENABLE)));
						prepare_lcd();
						lcd_clear_line(4);
						print_lcd2_P(UART_UART2X_STATUS_STR);
						print_lcd2_P(UART_UART2X_DISABLE_STR[language&0x01]);
						set_none();
					}
				}
			}
			else if (menu==OTHER_MENU_CONFIGURE_UART_SECURITY)
			{
				if (code_cr==RETURN_KEY) show_uart_menu_for_iot(1);
				else if (code_cr=='1') show_uart_for_iot_configure_sec_permission();
				else if (code_cr=='2') show_uart_for_iot_configure_default(OTHER_MENU_CONFIGURE_DEF_SECURITY);
			}
			else if (menu==OTHER_MENU_CONFIGURE_UART_SEC_PERM)
			{
				if (code_cr==RETURN_KEY) show_uart_for_iot_configure_security();
				else
				{
					insert_input(0);
					if (code_cr==ENTER_KEY)
					{
						prepare_lcd();
						Memory_b=strtoul(input, NULL, 10);
						lcd_clear_line(4);
						//if ((Memory_b)>(0xFFFF))
						if (compare_int_16_ovf(Memory_b))
							print_lcd2_P(ERR_OVRFLW);
						else
						{
							//EEPROM_write(FLAG_H_ADDR, (unsigned char)(Memory_b>>8));
							//EEPROM_write(FLAG_L_ADDR, (unsigned char)(Memory_b&0xFF));
							EEPROM_write(FLAG_H_ADDR, (uint8_t)hi_uint_16(Memory_b));
							EEPROM_write(FLAG_L_ADDR, (uint8_t)Memory_b);
							SetCursor(4,1);
							print_lcd2(utoa(Memory_b, input, 16));
							lcdData(',');
							print_lcd2(utoa(Memory_b, input, 10));
						}
						clear_input(0);
						SetCursor(3,1);
						set_none();
					}
					else if (code_cr==CLEAR_KEY) clear_input(1);//clear_input(0);
				}
			}
			else if (menu==OTHER_MENU_CONFIGURE_DEF_SECURITY)
			{
				if ((code_cr==RETURN_KEY)||(code_cr=='2')) show_uart_for_iot_configure_security();
				else if (code_cr=='1')
				{
					EEPROM_write(FLAG_H_ADDR, FLAG_H_DEFAULT_VALUE);
					EEPROM_write(FLAG_L_ADDR, FLAG_L_DEFAULT_VALUE);
					show_uart_for_iot_configure_security();
				}
			}
			else if (menu==OTHER_MENU_UART_FOR_IOT_PASSWD)
			{
				if (code_cr==RETURN_KEY) show_other_menu();
				else if (code_cr==CLEAR_KEY) clear_input(1);
				else
				{
					if (pos_input<(MAX_BLOCK-1))
					{
						insert_input(0x02);
					}
					if (code_cr==ENTER_KEY)
					{
						prepare_lcd();
						if (check_user_password())
						{
							clear_display();
							show_uart_menu_for_iot(1);
						}
						else
						{
							lcd_clear_line(4);
							print_lcd2_P(ACCESS_DENIED[(language&0x01)]);
							SetCursor(3,1);
						}
						set_none();
					}
				}
			}
			else if (menu==OTHER_MENU_SET_PASSWORD)
			{
				if (code_cr==RETURN_KEY) show_uart_menu_for_iot(1);
				else if (code_cr==CLEAR_KEY) clear_input(1);
				else
				{
					//cli(); // Fixed 07/18/2015
					//insert_input(0x02);
					// Fixed again (10/03/2015)
					if (pos_input<(MAX_BLOCK-1))
						insert_input(0x02);
					if (code_cr==ENTER_KEY)
					{
						prepare_lcd();
						if (conv[0]==0x01)
						{
							sub_clear_input();
							if (check_user_password())
							{
								conv[0]=0x02;
								clear_input(0); // FIXED 07/17/2015 (Fri)
								lcd_clear_line(2);
								print_lcd2_P(NEW_PASSWORD_STR[(language&0x01)]);
								lcdData(':');
							}
							else
							{
								clear_input(0); // FIXED 07/17/2015 (Fri)
								SetCursor(4,1);
								print_lcd2_P(ACCESS_DENIED[(language&0x01)]);
							}
							SetCursor(3,1);
						}
						else if (conv[0]==0x02)
						{
							strcpy(conv, input);
							lcd_clear_line(2);
							print_lcd2_P(RETYPE_PASSWORD_STR[language&0x01]);
							lcdData(':');
							clear_input(1);
						}
						else
						{
							if (strcmp(input, conv)==0)
							{
							/*
								FIXED 07/12/2015 (Sun). You need digest it !!!!
								clear_passwd(passwd);
								strcpy(passwd, input);
								write_password_EEPROM(passwd);
								show_uart_menu_for_iot(1);
							*/
								clear_passwd(passwd);
								strcpy(passwd, input);
								write_password_EEPROM(digest(passwd));
								clear_passwd(passwd); // destroy password in memory (SECURITY)
								//strcpy(input, passwd); // destroy password in memoy (SECURITY)
								clear_input(0);
								show_other_menu(); // Fixed 07/18/2015
							}
							else
							{
								conv[0]=0x02;
								clear_input(0);
								lcd_clear_line(4);
								print_lcd2_P(PASSWD_MISMATCH_STR[(language&0x01)]);
								lcd_clear_line(2);
								print_lcd2_P(NEW_PASSWORD_STR[(language&0x01)]);
								lcd_clear_line(3);
							}
						}
						set_none();
					}
					//sei(); // Fixed 07/18/2015
				}
			}
			else if (menu==ABOUT_MENU)
			{
				if (code_cr==RETURN_KEY) show_initial_menu();
				dynamic();
			}
			else
			{
				dynamic();
				/*
				n=menu_kbd_interval('0','7');
				if (n)
				{
					switch (code_cr)
					{
						case '1': pseudo_latch_pinos^=PINO1; break;
						case '2': pseudo_latch_pinos^=PINO2; break;
						case '3': pseudo_latch_pinos^=PINO3; break;
						case '4': pseudo_latch_pinos^=PINO4; break;
						case '5': pseudo_latch_pinos^=PINO5; break;
						case '6': pseudo_latch_pinos^=PINO6;
	
					}
					//PORTD=pseudo_latch_pinos;
					set_latch_pinos();
					//beep();
				}
				n=hoursA&0x3F;
				if ((hours==n)&&(minutes==minutesA)) hoursA&=0xBF;
				*/
			}
			code_cr=0x01;
			beep();
		}
		if (code_cr==0x01)
		{
			cli();
			code_cr=0x00;
			cr_start=0x00;
			PCICR=(1<<PCIE0);
			sei();
		}
		//if (temporizador&TIMER1_ENABLE)
		if (read_write_temporizador(temporizador)&TIMER1_ENABLE)
		{
			//temporizador=0;
			//temporizador&=TIMER1_DISABLE;
			//read_write_temporizador(temporizador&TIMER1_DISABLE);
			cli();
			temporizador&=TIMER1_DISABLE;
			TCNT1=0xC2F7;//15625*1024=16000000 // 1 second pulse guaranted at 16 MHz
			sei();

			hours=read_from_rtc(RTC_HOURS_REG)&0x3F;
			minutes=read_from_rtc(RTC_MINUTES_REG)&0x7F;
			//seconds=read_from_rtc(RTC_SECONDS_REG);
			if (hoursA&0x80)
			{
				// 1111|1111
				// 0x80 alarm ON/OFF 1- On | 0 - Off
				// 0x40 ring ON/ OFF 1- On | 0 - Off

				if (hoursA&0x40)
				{
					n=hoursA&0x3F;
					if ((hours==n)&&(minutes==minutesA)) beepAlarm();
				}
				else if (!(minutes==minutesA)) hoursA|=0x40;
			}
			//read_sensor(TEMP1_SENSOR);
			//read_sensor(TEMP2_SENSOR);
			//read_sensor(B1_SENSOR);
			//read_sensor(B2_SENSOR);
			read_all_sensor();

			// for battery monitor:

			battery_monitor(CHARGER1_OFF, B1_LED_SIGNAL, b1);
			battery_monitor(CHARGER2_OFF, B2_LED_SIGNAL, b2);

			// end battery monitor

			// monitoring programmed pins

			monitor_pins(&pino1, PINO1);
			monitor_pins(&pino2, PINO2);
			monitor_pins(&pino3, PINO3);
			monitor_pins(&pino4, PINO4);
			monitor_pins(&pino5, PINO5);
			monitor_pins(&pino6, PINO6);

			// end monitoring programming pins

			if (!(menu & IS_MENU))
			{
// 02/jan/2014
// jan/02/2014
// 100.1°F  100.1°F
				prepare_lcd();
				lcd_clear_line(3);
				//print_lcd2("T1:");
				if (language&0x80)
				{
					print_lcd2(dtostrf(t1,1,1, conv));
					//print_lcd2(conv);
					lcdData(0x05); // °
					lcdData('C');
					SetCursor(3,9);
					//print_lcd2("C T2:");
					print_lcd2(dtostrf(t2,1,1,conv));
					//print_lcd2(conv);
					lcdData(0x05);
					lcdData('C');
				}
				else
				{
					print_lcd2(dtostrf(toFarenheit(&t1),1,1, conv));
					//print_lcd2(conv);
					lcdData(0x05);// °
					lcdData('F');
					SetCursor(3,9);
					print_lcd2(dtostrf(toFarenheit(&t2),1,1, conv));
					//print_lcd2(conv);
					lcdData(0x05);// °
					lcdData('F');
				}
				lcd_clear_line(2);
				get_week_day(read_from_rtc(RTC_WEEKDAYS_REG));
				print_lcd2(conv);
				lcdData(' ');
				////
				n=read_from_rtc(RTC_DAYS_REG);
				n&=0x3F;
				// Fatal This value are in BCD !!! 10/01/2015
				n2=read_from_rtc(RTC_MONTHS_CENTURY_REG); // Fatal fixed 10/18/2015 Date is not shown if is set to 19XX
				//get_month_day(n2-1);
				// FIXED
				//get_month_day(read_from_rtc(RTC_MONTHS_CENTURY_REG));
				get_month_day(n2);
				if (language&0x01)
				{
					//get_month_day(n2);
					print_lcd2(conv);
					lcdData('/');
					print_time_date();
					//lcdData('/');
				}
				else
				{
					//n=read_from_rtc(RTC_DAYS_REG);
					//n&=0x3F;
					print_time_date();
					lcdData('/');
					//n=n2;
					//n=read_from_rtc(RTC_MONTHS_CENTURY_REG);
					//get_month_day(n2);
					print_lcd2(conv);
					//lcdData('/');
				}
				lcdData('/');
				//n=n2;
				if (n2 & IS_19XX) print_lcd2_P(CENTURY_19XX); //print_lcd2("19");
				else print_lcd2_P(CENTURY_20XX);//print_lcd2("20");
				n=read_from_rtc(RTC_YEARS_REG);
				print_time_date();
				lcd_clear_line(1);
				/*
				if (language&0x01)
					print_lcd2_P(ERR_TIME);//print_lcd2("Time");
				else
					print_lcd2_P(ERR_TIME_BR);//print_lcd2("Hora");
				*/
				print_lcd2_P(ERR_TIME[language&0x01]);
				//print_lcd2("Hora: ");
				print_lcd2(": ");
				n=hours;
				print_time_date();
				lcdData(':');
				n=minutes;
				print_time_date();
				lcdData(':');
				n=read_from_rtc(RTC_SECONDS_REG);
				n&=0x7F;
				print_time_date();
				lcd_clear_line(4);
				print_lcd2("B1:");
				display_battery_status(CHARGER1_OFF,B1_LED_SIGNAL, b1);
				SetCursor(4,10);
				print_lcd2("B2:");
				display_battery_status(CHARGER2_OFF,B2_LED_SIGNAL, b2);

				set_none();
			}
			else if (menu==SENSOR_MENU) show_sensor_menu();
		}

		// prepares a new loop and clear all variables in input such as kbd and ir sensor.

		//cr_start=0;
		//if ((code_cr==0xFE)&&(cr_start==0x02)) cr_start=0x00;
		//{
			//cr_start=0x0;
			//for (n=0;n<MAX;n++) code[n]=0;
		//}

		//n=pseudo_latch_aux & IS_KBD_KEY_PRESSED;
		//if (n)
		if (pseudo_latch_aux & IS_KBD_KEY_PRESSED)
		{
			set_none();
			PORTD=(tmp_gpr<<2);
			if (!(check_pinc())) pseudo_latch_aux &= ~IS_KBD_KEY_PRESSED;
		}
		// beep event
		if (pseudo_latch_aux&SPK_ON)
		{
			//if ((temporizador&0x1F)==0)
			if ((read_write_temporizador(temporizador)&0x1F)==0)
			{
				pseudo_latch_aux&=~SPK_ON;
				set_latch_aux();
			}
		}
	}
	return 0; // it never reaches here !!!
}
