// TC0_fast_PWM.c
#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	unsigned char PWM_val1 = 0;		// 8-bit PWM value
	unsigned char PWM_val2 = 255;	// 8-bit PWM value
	unsigned char up_dn = 0;		// up down count flag
	
	DDRB   |= (1 << PINB2);                   // PWM output on PB2 - OC0A
	DDRB   |= (1 << PINB3);                   // PWM output on PB2 - OC0A
	DDRD   |= (1 << PIND5);                   // PWM output on PD5 - OC0B
	// fast PWM mode
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	TCCR1A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01);   // clock source = CLK/8, start PWM
	
	while(1)
	{
		if ((PWM_val1 == 255) || (PWM_val1 == 0)) {
			up_dn ^= 0x01;      // toggle count direction flag
		}
		_delay_ms(5);
		OCR0A  = PWM_val1;       // write new PWM value
		OCR1A = PWM_val2;
		OCR0B  = PWM_val2;
		if (up_dn) {            // increment or decrement PWM value
			PWM_val1++;
			PWM_val2--;
		}
		else {
			PWM_val1--;
			PWM_val2++;
		}
	}
}