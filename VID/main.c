// TC0_fast_PWM.c
#define F_CPU 1000000UL
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

unsigned char pwmTime = 255;

long circle = 0;

double Lm = 0.000000056;
double Ls = 0.000002;
double Lr = 0.0000000053;
double Tr = 0.000000083;
double Rr = 0.064;
double Rs = 0.064;
double Sigma = 1.3;
double M = 0.0013;
double StepM = 0.0001;

SIGNAL (TIMER0_COMPA_vect)
{	
	//if((PINB & (1 << PINB0)) || (PINB & (1 << PINB1)) || (PINB & (1 << PINB2))){
		//PORTB = 0;
		//OCR0A = pwmTime-iterations[indexIterations].timeU;
	//}else{
		//indexIterations++;
		//if(indexIterations >= countPhaze){
			//indexIterations = 0;
		//}
		//switch (iterations[indexIterations].phaze){
			//case 0:
			//PORTB = (1 << PINB0);
			//break;
			//case 1:
			//PORTB = (1 << PINB2);
			//break;
			//case 2:
			//PORTB = (1 << PINB1);
			//break;
		//}
		//OCR0A = iterations[indexIterations].timeU;
	//}
}

SIGNAL (INT0_vect){
	if(PIND & (1 << PIND4)){
		M += StepM;
	}else{
		if(M-StepM>0) {
			M -= StepM;
		}
	}
}

double period;
double w;
double endPeriod;

double psi;
double psi1;
double psi2;
double psi3;

double id;
double id1;
double id2;
double id3;
double id4;
double id5;
double id6;
double id7;

double u;
double u1;
double u2;
double u3;
double u4;
double u5;
double u6;
double u7;
double u8;
double u9;
double u10;
double u11;
double u12;
double u13;

SIGNAL (INT1_vect){
	circle++;
	if (circle > 2){
		endPeriod = TCNT1;
		period = endPeriod / 10000;		// seconds
		w = 2 * M_PI / period;
		
		w = M_PI;
		
		GetPsi();
		GetId();
		GetU();
	}
}

void GetPsi(){
	psi1 = Sigma*Lr*Ls*Tr*w + Lm*Lm*Tr*w - Sigma*Lr*Ls - Lm*Lm;
	psi2 = sqrt(-6*Rr*psi1*M*Rs*Tr*Lr);
	psi3 = Rr*psi1;
	
	psi = 1/3 * psi2 / psi3;
}

void GetId(){
	id1 = - Lm*psi / (2*Lr*Ls*Sigma);
	id2 = Tr*w*psi/2;
	id3 = - Lr / (3*Lm*psi) * M / Rr;
	id4 = Lm*psi/(Lr*Ls*Sigma) - Tr*w*psi + 2/3 * Lr*M/(Lm*psi*psi*Rr*Tr);
	id5 = 0.25*id4*id4;
	id6 = 2*Lr*M/(3*Lm*Lm*Rr)*(Rs*Tr/(Ls*Sigma) + 1);
	id7 = Tr*w*psi*psi/(Lr*Ls*Sigma);
	
	id = id1 + id2 + id3 + sqrt(id5 + id6 + id7);
}

void GetU(){
	u1 = -Lm/(2*Lr*Ls*Sigma);
	u2 = Tr*w/2;
	u3 = Lr*M/(3*Lm*psi*psi*Rr);
	u4 = Lm*psi/(Lr*Ls*Sigma) - Tr*w*psi + 2*Lr*M/(3*Lm*psi*Rr);
	u5 = Lm/(Ls*Lr*Sigma)-Tr*w-2*Lr*M/(3*Lm*psi*psi*Rr);
	u6 = 4*Tr*w*psi/(Lr*Ls*Sigma);
	u7 = u4*u5+u6;
	u8 = 8*Lr*M/(3*Lm*Lm*Rr)*(Rs*Tr/(Ls*Sigma)+1);
	u9 = 4*Tr*w*psi*psi/(Lr*Ls*Sigma);
	u10 = 2*Tr*sqrt(u4*u4 + u8 + u9);
	u11 = Sigma*Ls*(u1 + u2 + u3 + u7/u10)*(-psi+Lm*id) + Rs*id;
	u12 = -2*Sigma*Lr*Ls*M/Rr*(w/(Lm*psi) - id/(Tr*psi*psi));
	u13 = -Lm/(Lr*Tr)*(psi - Lm*id);
	
	u = u11 + u12 + u13;
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