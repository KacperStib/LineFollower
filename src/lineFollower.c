/*
#ifndef F_CPU
#define F_CPU 1000000UL
#endif
*/
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>

#define leftMotorR PIND7
#define leftMotorF PIND6
#define leftMotorEn PIND5
#define rightMotorF PINB1
#define rightMotorR PINB0
#define rightotorEn PINB2
#define leftSensor PINC5
#define rightSensor PINC3
bool leftAlarm;
bool rightAlarm;
int8_t speed = 80;

void FORWARD();
void BACKWARD();
void RIGHT();
void LEFT();
void STOP();

int main(void)
{
    
	DDRB |= (1<<0)|(1<<1)|(1<<2); //PB0 PB1 PB2
	DDRC &= ~((1<<3)|(1<<6)); //PC3 PC5
	DDRD |= (1<<5)|(1<<6)|(1<<7);//11100000; //PD5 PD6 PD7
	//PORTD |= (1 << PIND5);
	//PORTB |= (1 << PINB2);
	//PWM D5
	TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
	TCCR0B=(0<<WGM02) | (1<<CS02) | (0<<CS01) | (0<<CS00);
	TCNT0=0x00;
	OCR0A=0x00;
	OCR0B=0x00;
	//PWM B2
	TCCR1A |= (0<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (1<<WGM10);
	TCCR1B |= (1<<WGM12) | (1<<CS12) | (0<<CS11) | (0<<CS10);
	TCNT1=0x00;
	OCR1A=0x00;
	OCR1B=0x00;
	
    while (1) 
    {
	  OCR0B = speed;
	  OCR1B = speed;
	  leftAlarm = bit_is_clear(PINC, leftSensor);
	  rightAlarm = bit_is_clear(PINC, rightSensor);
	  
	  if(leftAlarm == 1 && rightAlarm == 0){
		  RIGHT();
	  }
	  else if(leftAlarm == 0 && rightAlarm == 1){
		  LEFT();
	  }
	  else if(leftAlarm == 1 && rightAlarm == 1){
		  FORWARD();
	  }
	  else if(leftAlarm == 0 && rightAlarm == 0){
		  STOP();
	  }
	  _delay_ms(10);
    }
}

void FORWARD(){
	PORTD |= (1 << leftMotorF); //digitalWrite(leftMotorF,1);
	PORTD &= ~(1 << leftMotorR);//digitalWrite(leftMotorR,0);
	PORTB |= (1 << rightMotorF);//digitalWrite(rightMotorF,1);
	PORTB &= ~(1 << rightMotorR);//digitalWrite(rightotorR,0);
	_delay_ms(10);
}

void BACKWARD(){
	PORTD &= ~(1 << leftMotorF);//digitalWrite(leftMotorF,0);
	PORTD |= (1 << leftMotorR);//digitalWrite(leftMotorR,1);
	PORTB &= ~(1 << rightMotorF);//digitalWrite(rightMotorF,0);
	PORTB |= (1 << rightMotorR);//digitalWrite(rightotorR,1);
	_delay_ms(10);
}

void RIGHT(){
	PORTD |= (1 << leftMotorF);//digitalWrite(leftMotorF,1);
	PORTD &= ~(1 << leftMotorR);//digitalWrite(leftMotorR,0);
	PORTB &= ~(1 << rightMotorF);//digitalWrite(rightMotorF,0);
	PORTB |= (1 << rightMotorR);//digitalWrite(rightotorR,1);
	_delay_ms(10);
}

void LEFT(){
	PORTD &= ~(1 << leftMotorF);//digitalWrite(leftMotorF,0);
	PORTD |= (1 << leftMotorR);//digitalWrite(leftMotorR,1);
	PORTB |= (1 << rightMotorF);//digitalWrite(rightMotorF,1);
	PORTB &= ~(1 << rightMotorR);//digitalWrite(rightotorR,0);
	_delay_ms(10);
}

void STOP(){
	PORTD &= ~(1 << leftMotorF);//digitalWrite(leftMotorR,0);
	PORTD &= ~(1 << leftMotorR);//digitalWrite(leftMotorR,0);
	PORTB &= ~(1 << rightMotorF);//digitalWrite(rightMotorF,0);
	PORTB &= ~(1 << rightMotorR);//digitalWrite(rightotorR,0);
	_delay_ms(10);
}
