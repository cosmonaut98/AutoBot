//ESE519 Final Working Code for Butter Bot (Group 24)

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "uart.h"

int count = 0;

int rising_edge, falling_edge;
int time_diff, distance = 100;
#define THRESHOLD 10

char String[25];

void initialize()
{
	cli();
	botMotorSetup();
	
	//Set PIN D4 as input pin for RFID TRUE or FALSE
	DDRD &= ~(1 << DDD4);
	
	//Set PIN B1 as output for TRIG of Ultrasonic Sensor (PIN 9)
	DDRB |= (1<<DDB1);
	
	//Pin B2 for Servo Motor Signal PIN
	DDRB |= (1<<DDB2);
	
	//PIN B0 is input for input capture using timer 1 (ECHO)
	DDRB &= ~(1<<DDB0);
	
	//Set TIMER 1 in CTC MODE for Echo and Trigger
	TCCR1A &= ~(1 << WGM10);
	TCCR1A &= ~(1 << WGM11);
	TCCR1B |= (1 << WGM12);
	TCCR1B &= ~(1 << WGM13);

	//Prescale Timer 1 by 64
	TCCR1B |= (1<<CS11);
	TCCR1B |= (1<<CS10);
	
	OCR1A = 14999;
	OCR1B = 4999;
	
	TCCR1B |= (1<<ICES1);
	TIMSK1 |= (1<<ICIE1);
	
	TIMSK1 |= (1 << TOIE1);
	
	TIMSK1 |= (1<<OCIE1A) | (1<<OCIE1B);
	
	sei();
}

void botMotorSetup()
{
	DDRC |= (1 << DDC0);
	DDRC |= (1 << DDC1);
	DDRC |= (1 << DDC2);
	DDRC |= (1 << DDC3);
}

ISR(TIMER1_CAPT_vect)
{
	if ((TCCR1B & (1 << ICES1)) == (1 << ICES1))
	rising_edge = ICR1;
	else
	falling_edge = ICR1;
	
	if (rising_edge != 0 && falling_edge != 0)
	{
		time_diff = falling_edge - rising_edge;
		rising_edge = 0;
		falling_edge = 0;
	}
	
	TCCR1B ^= (1 << ICES1);
	TIFR1 = (1 << ICF1);	//Clear Input Capture Flag
}

ISR(TIMER1_COMPA_vect)
{
	//Sending a 10us Pulse to the Echo PIN of the Ultrasonic Sensor
	PORTB |= (1 << PINB1);
	_delay_us(10);
	PORTB &= ~(1 << PINB1);
}

ISR(TIMER1_COMPB_vect)
{
	botArmControl();
}

int grappleArmFlag = 0;

void botArmControl()
{
	if(grappleArmFlag == 1)
	{
		//Grapple ARM
		PORTB |= 1<<PORTB2;
		_delay_ms(2);
		PORTB &= ~(1<<PORTB2);
		grappleArmFlag = 0;
	}
	else if (grappleArmFlag == 0 && distance >= THRESHOLD)
	{
		//Release ARM
		PORTB |= 1<<PORTB2;
		_delay_ms(1);
		PORTB &= ~(1<<PORTB2);
	}
}

ISR(USART_RX_vect)
{
	char buffer;
	int i = 0;
	buffer = UDR0;

	while(buffer != "\0")
	{
		gridStatus[i] = (int) buffer;
		i++;
	}
}

#define NODE_TIME 250
#define XMIN -1
#define YMIN -1
#define XMAX 3
#define YMAX 3
#define NORTH 0
#define SOUTH 2
#define WEST 3
#define EAST 1
#define GRIDSIZE 9
#define GRIDDIM 3
#define OCCUPIED 1
#define EMPTY 0
#define TRUE 1
#define FALSE 0

int x_currBot = 0, y_currBot = 0;
int gridStatus[GRIDSIZE];
int objLoc[GRIDSIZE][2] = { -1 };
int botOrientation = NORTH;
int flag_x = 0, flag_y = 0, found = 0, RFID = FALSE;

void search()
{
	int num_of_objects = 0;
	
	//Storing grid data and location of objects
	for(int i = 0; i < GRIDDIM; i++)
	{
		for(int j = 0; j < GRIDDIM; j++)
		{
			if(gridStatus[i * GRIDDIM + j] == OCCUPIED)
			{
				objLoc[num_of_objects][0] = i;
				objLoc[num_of_objects][1] = j;
				num_of_objects++;
			}
		}
	}
	
// 	sprintf(String,"num_of_objects: %u\n", num_of_objects);
// 	UART_putstring(String);
	
	for(int i = 0; i < num_of_objects; i++)
	{
// 		sprintf(String,"Targeting X: %u\n", objLoc[0][0]);
// 		UART_putstring(String);
// 		sprintf(String,"Targeting Y: %u\n", objLoc[0][1]);
// 		UART_putstring(String);
		
		traverse(objLoc[i][0], objLoc[i][1]);
	}
	while(1);
}

void traverse(int x_target, int y_target)
{
	while((x_currBot != x_target) || (y_currBot != y_target))
	{
		sprintf(String,"The X CURR: %u\n", x_currBot);
		UART_putstring(String);
		sprintf(String,"The Y CURR: %u\n", y_currBot);
		UART_putstring(String);
		
		if(x_currBot > x_target && x_currBot < XMAX && flag_x == 0)
		{
			orientBot(SOUTH);
			while(x_currBot != x_target && flag_x == 0)
			{
				moveBotForward(&flag_x);
				//flag_y = 0;
			}
			if(flag_x == 1)
			moveBotReverse();
		}
		else if(x_currBot < x_target && x_currBot > XMIN && flag_x == 0)
		{
			orientBot(NORTH);
			while(x_currBot != x_target && flag_x == 0)
			{
				UART_putstring("i HAVE ENTERED LOOP");
				moveBotForward(&flag_x);
				//flag_y = 0;
			}
			if(flag_x == 1)
			moveBotReverse();
		}
		else if(y_currBot > y_target && y_currBot < YMAX && flag_y == 0)
		{
			orientBot(EAST);
			while(y_currBot != y_target && flag_y == 0)
			{
				moveBotForward(&flag_y);
				//flag_x = 0;
			}
			if(flag_y == 1)
			moveBotReverse();
		}
		else if(y_currBot < y_target && y_currBot > YMIN && flag_y == 0)
		{
			UART_putstring("Entering Y WEST Loop");
			orientBot(WEST);
			while(y_currBot != y_target && flag_y == 0)
			{
				moveBotForward(&flag_y);
				//flag_x = 0;
			}
			if(flag_y == 1)
			moveBotReverse();
		}
	}
}

void verify()
{
	RFID = PIND & (1 << PIND4);
	if(RFID == TRUE)
	{
		found = 1;
		grappleArmFlag = 1; //Grapple
		stopBot();
		traverse(0, 0);
	}
}

void orientBot(int targetOrientation)
{
	while(targetOrientation - botOrientation)
	{
		moveBotRight();
		botOrientation = (botOrientation + 1) % 4;
	}
}

void moveBotForward(int *flag)
{
	PORTC &= ~(1 << PORTC0);
	PORTC |= (1 << PORTC1);
	PORTC &= ~(1 << PORTC2);
	PORTC |= (1 << PORTC3);
	
	for(int i = 0; i < NODE_TIME; i++)
	{
		sprintf(String,"Printing Flag: %u\n", *flag);
		UART_putstring(String);
		
		distance = (int)(time_diff/ (58)* 2.54);	//Convert Distance from Inch to CM
		//distance = 100;
		
		sprintf(String,"The distance in centimeters is: %u\n", (int)(distance * 1.7));
		UART_putstring(String);
		
		_delay_ms(1);
		if(distance < THRESHOLD)
		{
			*flag = 1;
			break;
		}
	}
	stopBot();
	
	_delay_ms(1000);
	
	sprintf(String,"Printing Flag: %u\n", *flag);
	UART_putstring(String);

	if(botOrientation == NORTH)
	{
		x_currBot++;
	}
	else if(botOrientation == SOUTH)
	{
		x_currBot--;
	}
	else if(botOrientation == EAST)
	{
		y_currBot--;
	}
	else if(botOrientation == WEST)
	{
		y_currBot++;
	}
	
	if(*flag == 1)
	{
		UART_putstring("Entering Verify");
		verify();
	}
}

void moveBotReverse()
{
	PORTC |= (1 << PORTC0);
	PORTC &= ~(1 << PORTC1);
	PORTC |= (1 << PORTC2);
	PORTC &= ~(1 << PORTC3);
	_delay_ms(NODE_TIME - 75);
	stopBot();
	
	if(botOrientation == NORTH)
	{
		x_currBot--;
	}
	else if(botOrientation == SOUTH)
	{
		x_currBot++;
	}
	else if(botOrientation == EAST)
	{
		y_currBot++;
	}
	else if(botOrientation == WEST)
	{
		y_currBot--;
	}
	
	_delay_ms(1000);
}

void moveBotRight()
{
	PORTC &= ~(1 << PORTC0);
	PORTC |= (1 << PORTC1);
	PORTC |= (1 << PORTC2);
	PORTC &= ~(1 << PORTC3);
	_delay_ms(350);
	stopBot();
	_delay_ms(1000);
}

void moveBotLeft()
{
	PORTC |= (1 << PORTC0);
	PORTC &= ~(1 << PORTC1);
	PORTC &= ~(1 << PORTC2);
	PORTC |= (1 << PORTC3);
	_delay_ms(350);
	stopBot();
	_delay_ms(1000);
}

void stopBot()
{
	PORTC &= ~(1<<PORTC0); // LEFT MOTOR
	PORTC &= ~(1<<PORTC1);
	PORTC &= ~(1<<PORTC2); // RIGHT MOTOR
	PORTC &= ~(1<<PORTC3);
}

int main()
{
	UART_init(BAUD_PRESCALER);
	initialize();
	
	grappleArmFlag = 0;	//Release
	
	_delay_ms(5000);
	
	while(1)
	{
		//distance = (int)(time_diff/ (58)* 2.54);	//Convert Distance from Inch to CM
		
		// 		sprintf(String,"The distance in centimeters is: %u\n", (int)(distance * 1.7));
		// 		UART_putstring(String);
		
		search();
	}
}