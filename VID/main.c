// TC0_fast_PWM.c
#define F_CPU 1000000UL
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

unsigned char allTimePWM = 255;
unsigned char timeU = 200;
int maxRepeatCount = 500;
int repeatCount = 500;
char phaze = 0;
long circle = 0;

SIGNAL (TIMER0_COMPA_vect)
{	
	if((PINB & (1 << PINB0)) || (PINB & (1 << PINB1)) || (PINB & (1 << PINB2))){
		PORTB = 0;
		repeatCount--;
		OCR0A = allTimePWM-timeU;
		}else{
		if(repeatCount==0){
			repeatCount = maxRepeatCount;
			phaze++;
			if(phaze>2){
				phaze=0;
			}
		}
		switch (phaze){
			case 0:
			PORTB = (1 << PINB0);
			break;
			case 1:
			PORTB = (1 << PINB2);
			break;
			case 2:
			PORTB = (1 << PINB1);
			break;
		}
		OCR0A = timeU;
	}
}

SIGNAL (INT0_vect){
	if(PIND & (1 << PIND4)){
		maxRepeatCount+=10;
		}else{
		if(maxRepeatCount-10>0) {
			maxRepeatCount-=10;
		}
	}
}

double period;
double angleSpeed;
double endPeriod;

SIGNAL (INT1_vect){
	circle++;
	if (circle > 2){
		endPeriod = TCNT1;
		period = endPeriod / 10000;		// seconds
		angleSpeed = 2 * M_PI / period;
	}
	TCNT1=0;
}

void InitInterupt(){
	PCMSK |= (1<<PIND2);
	MCUCR |= (0<<ISC00) | (1<<ISC01) | (0<<ISC10)| (1<<ISC11);
	GIMSK |= (1<<INT0) | (1<<INT1);
}

int main(void)
{
	DDRB |= (1 << PINB0) | (1 << PINB1) | (1 << PINB2);
	DDRD |= (1 << PIND4) | (0 << PIND2) | (0 << PIND3);
	
	InitInterupt();
	
	TCCR0A = 0x02;      // Clear Timer on Compare Match (CTC) mode
	TIFR |= 0x01;       // clear interrupt flag
	TIMSK = 0x01;       // TC0 compare match A interrupt enable
	TCCR0B = (1 << CS00);      // clock source CLK, start timer
	
	TCCR1A = 0x00;             // CTC mode
	TCCR1B = (1 << CS00);   // start timer
	
	sei();
	
	while(1)
	{
	}
}