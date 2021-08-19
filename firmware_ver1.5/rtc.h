// 10/09/2014 20:20
// by Fábio Pereira da Silva
// RTC - Real Time Clock routines

//#define F_CPU 16000000
#include <avr/io.h>
#include <util/twi.h>

extern unsigned char temporizador;

#define SLA_R 0b10100011
#define SLA_W 0b10100010


#define RTC_SECONDS_REG 0x02 //<<1
#define RTC_MINUTES_REG 0x03 //<<1
#define RTC_HOURS_REG 0x04 //<<1
#define RTC_DAYS_REG 0x05 //<<1
#define RTC_WEEKDAYS_REG 0x06 //<<1
#define RTC_MONTHS_CENTURY_REG 0x07 //<<1
#define RTC_YEARS_REG 0x08 //<<1
#define IS_19XX 0x80
// Defining Owner's (Fábio) Error


static unsigned char rtc_disable=0;
static unsigned char old_date[7];

void check_twcr()
{

	asm volatile
	(
		"push r25" "\n\t"
		"lds r0, 0x00B1" "\n\t" // save state in TCCR2B
		"ldi r24, 0x60" "\n\t"
		"call asm_TURN_ON_TIMER_2" "\n\t" // start loop event 49.152 ms
	"VOLTAR:" "\n\t"
		"lds r25, 0x00BC" "\n\t"
		"sbrc r25, 7" "\n\t"
		"rjmp SAIR" "\n\t"
		"ld r24, X" "\n\t"
		"andi r24, 0x60" "\n\t"
		"brne VOLTAR" "\n\t"
	"SAIR:" "\n\t"
		"sts 0x00B1, r0" "\n\t"
		"pop r25"
	:
	:"x" (&temporizador)
	);
}

void send_cmd(unsigned char cmda)
{
	TWDR=cmda;
	TWCR=(1<<TWINT)|(1<<TWEN);
	check_twcr();
}

unsigned char receive_data()
{

	TWCR=(1<<TWINT)|(1<<TWEN);
	check_twcr();
	//send_stop();
	TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	return TWDR;
}

unsigned char read_from_rtc(unsigned char cmd)
{
	if (rtc_disable)
		return old_date[(cmd-2)];
	else
	{
		//send_start();
		TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
		check_twcr();
		send_cmd(SLA_W);
		send_cmd(cmd);
		//send_start();
		TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
		check_twcr();
		send_cmd(SLA_R);
		//return receive_data();
		old_date[(cmd-2)]=receive_data();
		return old_date[(cmd-2)];
	}
}

void write_to_rtc(unsigned char cmd, unsigned char rtc_reg)
{
	if (!(rtc_disable))
	{
		//send_start();
		TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
		check_twcr();
		send_cmd(SLA_W);
		send_cmd(rtc_reg);
		send_cmd(cmd);
		//send_stop();
		TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	}
}

