 /*
  * mhd.h
  * This file is part of se.c
  *
  * 2015 - Fábio Pereira da Silva
  * LICENSE: MIT
  */

//Ter Jun 16 20:16:28 BRT 2015
// My hash digest (mhd)

#include <avr/pgmspace.h>

#define MAX_BLOCK 7
#define HASH_BLOCK 4

extern void fill_null_char(const uint16_t address, unsigned char size);
static char passwd[MAX_BLOCK];
//static unsigned char hash[HASH_BLOCK];

void clear_passwd(char pwd_a[])
{
    //unsigned char i;
    //for (i=0;i<MAX_BLOCK; i++) pwd_a[i]=0x00;
	fill_null_char((uint16_t)pwd_a, MAX_BLOCK-1);
}
unsigned char get_index(char val)
{
    if ((val>='0')&&(val<='9')) return (unsigned char)(val-0x30);
    else return (unsigned char)0x0B;
}
unsigned char *digest(char pwd[])
{
	static const unsigned char qi[] PROGMEM={0xC4, 0x0C, 0x3A, 0xB5, 0x01, 0x17, 0xCB, 0x12, 0xC9, 0xD0, 0xA1};
	static const unsigned char qj[] PROGMEM={0x4B, 0x05, 0x89, 0xEE, 0x3C, 0x55, 0x5B, 0x6D, 0x9F, 0x14, 0x01};
	static const unsigned char qk[] PROGMEM={0xBD, 0x54, 0xD5, 0x61, 0x57, 0xD2, 0x62, 0x74, 0x7E, 0x8A, 0xFE};
	static const unsigned char ql[] PROGMEM={0x4B, 0xCE, 0x83, 0x9B, 0xB2, 0x76, 0xEA, 0x38, 0xE2, 0xC0, 0xB2};
	static const unsigned char qm[] PROGMEM={0x9D, 0xB8, 0xCC, 0xC0, 0xF1, 0x14, 0xDA, 0xF0, 0x20, 0x26, 0x7D};
	static const unsigned char qn[] PROGMEM={0x8E, 0x44, 0xB8, 0x73, 0xDC, 0x32, 0x6D, 0xC0, 0x21, 0xBB, 0x1B};
    static unsigned char a,b,c,d,e,f,g,h,i,j,k;
    //unsigned char h3, h2, h1, h0;
	static unsigned char tmp[HASH_BLOCK];
	a=((unsigned char)pwd[3]|pgm_read_byte_near(qk+get_index((unsigned char)pwd[1])));
    b=((unsigned char)pwd[1]^pgm_read_byte_near(qj+get_index((unsigned char)pwd[2])));
    c=((unsigned char)pwd[0]+pgm_read_byte_near(qi+get_index((unsigned char)pwd[3])));
    d=b&c;
    e=a+d;
    f=((unsigned char)pwd[2]&pgm_read_byte_near(ql+get_index((unsigned char)pwd[0])));
    g=c^f;
	h=(unsigned char)pwd[4];
    h=(~h)+(unsigned char)pwd[5]+25;
    i=(h^pgm_read_byte_near(qm+get_index((unsigned char)pwd[4])));
    j=(c+i)>>1;
    k=(b+pgm_read_byte_near(qn+get_index((unsigned char)pwd[5])));
    //h3=(j^k)+g;
    //h2=(e|j)^(k+(~g));
    //h1=(k&e)|(g^j);
    //h0=(e&j)+(e+g)^j;
	tmp[0]=(e&j)+((e+g)^j);
	tmp[1]=(k&e)|(g^j);
	tmp[2]=(e|j)^(k+(~g));
	tmp[3]=(j^k)+g;
    return tmp;
	//memcpy(pwd, tmp, HASH_BLOCK);
}
/////////////////////////
// PASSWORD routines  //
///////////////////////

unsigned char is_null_pass(unsigned char digst[])
{
	//static const unsigned char NULL_HASH[] PROGMEM={0xA6, 0xBB, 0x83, 0xFF}; // FIXED 07/18/2015
	static const unsigned char NULL_HASH[] PROGMEM={0xA6, 0x3B, 0xE5, 0x65}; // FIXED 07/18/2015
	// unlock menu if password matches with password given
	unsigned char val_b; // Fixed 07/18/2015
	asm volatile
	(
		"clr r25" "\n\t"
	"label_voltar_is_null_pass:" "\n\t"
		"ld r18, X+" "\n\t"
		//"add r30, %0" "\n\t"
		//"adc r31, r1" "\n\t"
		"lpm %0, Z+" "\n\t" // FIXED 07/11/2015
		"cp %0, r18" "\n\t"
		"brne label_erro_null_pass" "\n\t"
		"inc r25" "\n\t"
		"cpi r25, 0x04" "\n\t"
		"brne label_voltar_is_null_pass" "\n\t"
		"ldi %0, 0x01" "\n\t"
		"ret" "\n\t"
	"label_erro_null_pass:" "\n\t"
		"ldi %0, 0x00" "\n\t"
		:"=r"(val_b)
		:"z"(NULL_HASH), "x"(digst)
	);
	return val_b;

}

// End PASSWORD routines
