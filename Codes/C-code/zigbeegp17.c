/**
Authors:

Group 17:
---------

ANUP NAIK 10305915 
VISHNU KANTH 10305020 
RAJ DEEPAK 10305081

AVR Studio Version 4.17, Build 666

Date: 8th November 2010

This project is about developing a robot which can collect balls randomly scattered around in an arena.

Concepts covered: Image Processing, Wireless Communication through Zigbee and Interrupt Handling.

Note:

1. Make sure that in the configuration options following settings are done for proper operation of the code

Microcontroller: atmega2560 Frequency: 11059200 Optimization: -O0 (For more information read section: Selecting proper optimization options below figure 4.22 in the hardware manual)

2. The matlab code uses a serial port for communicating data through zigbee module. This port number must be changed accordingly. 

*********************************************************************************/
/********************************************************************************

   Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in               -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/signal.h>
#include <math.h>
#include "lcd.c"
#define FCPU 11059200ul 	//defined here to make sure that program works properly

unsigned char data;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp,sharp1, distance, adc_reading;
unsigned int value;
float BATT_Voltage, BATT_V;
//unsigned char data;

int scr,scl;

void INIT_PORTS()
{
	DDRA=0x0F;
	PORTA=0x00;				//INITIALIZE
	DDRE=0xCF;
	PORTE=0xFF;	
	DDRL=0x18;
	PORTL=0x18;
	DDRC=0x00; 				// buzzer off
	PORTC =0x00;
	TCCR5B =0x00;
	TCCR5A = 0xA1;
	TCCR5B=0x0B;

}


void INIT_PORTS_ROTATE()
{
	DDRA=0x0F;
	PORTA=0x00;				//INITIALIZE
	DDRE=0xCF;
	PORTE=0xFF;	
	DDRL=0x18;
	PORTL=0x18;

}

/***** Function To Initialize UART0 *****/
// desired baud rate:9600
// actual baud rate:9600 (0.0%)
// char size: 8 bit
// parity: Disabled

void uart0_init(void)
{
 	UCSR0B = 0x00; 			//disable while setting baud rate
 	UCSR0A = 0x00;
 	UCSR0C = 0x06;
 	UBRR0L = 0x47; 			//set baud rate lo
 	UBRR0H = 0x00; 			//set baud rate hi
 	UCSR0B = 0x98;
}


SIGNAL(SIG_USART0_RECV)
{

	data = UDR0; 			//making copy of data from UDR0 in data variable
	
}
SIGNAL(SIG_USART0_TRANS)
{
}

void timer5_init()
{
	TCCR5B = 0x00;
	TCCR5A = 0xA1;
	TCCR5B = 0x0B;
}

void forward()
{

	PORTA=0x06;

}

void right() 				//function for moving right 
{
	PORTA=0x02;     				// Soft right
}
void left() 				//function for moving left
{
	PORTA=0x05;     				// Hard left
}
void back() 				//function for moving backward
{
	PORTA=0x09;     				// move back
}
void stop() 				//function for moving stop
{
	PORTA=0x00;						// Stop
}

void right_hard() 			//function for moving right
{
	PORTA=0x0A;						// Hard right
}

//Velocity control function of wheels
void velocity (unsigned char t1,unsigned char t2)
{
	OCR5AL = t1;                  
	OCR5BL = t2;
}


//initialization function of left wheel encoder

void left_position_encoder_interrupt_init(void)
{
	cli();
	EICRB=EICRB|0x02;
	EIMSK=EIMSK|0x10;
	sei();
}

//initialization function of right wheel encoder
void right_position_encoder_interrupt_init(void)
{
	cli();
	EICRB=EICRB|0x08;
	EIMSK=EIMSK|0x20;
	sei();
}

/*Angle rotation function 
When an angle is given the function calculates the number of steps needed for that angle.
Whenever the wheel cuts the shaft encoder, the corresponding ISR is invoked (based on the 
wheel which rotates) and scr or scl is incremented.This scl or scr value is compared with the
required count.If it is less then again rotation continues or the loop will break
*/

void angle_rotate(unsigned int Degrees)
{

    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees/ 2.045; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;    
	scr = 0; 
	scl = 0; 
	while (1)
	{
		if((scr>= ReqdShaftCountInt) | (scl >= ReqdShaftCountInt))
		{ 
					      
			break;
		}
		else
		{
						
			right();
		}
	}
   	stop(); 
}

/*Angle rotation function for hard right rotation 
Same as above but rotation function is hard right.
*/

void angle_rotate_right_hard(unsigned int Degrees)
{

    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees/ 2.045; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;    
	scr = 0; 
	scl = 0; 
	while (1)
	{
		if((scr>= ReqdShaftCountInt) | (scl >= ReqdShaftCountInt))
		{ 
					      
			break;
		}
		else
		{
						
			right_hard();
		}

	}
   	stop(); 

}

/*Angle rotation function for hard left rotation 
Same as above but rotation function is hard left.
*/
void angle_rotate_left(unsigned int Degrees)
{

    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees/ 2.045; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;    
	scr = 0; 
	scl = 0; 
	while (1)
	{
		if((scr>= ReqdShaftCountInt) | (scl >= ReqdShaftCountInt))
		{ 
					      
			break;
		}
		else
		{
						
			left();

		}

	}

   	stop(); 

}

//ISR for left wheel shaft encoder

ISR(INT4_vect)
{

	scl++;
}
//ISR for right wheel shaft encoder
ISR(INT5_vect)
{
	scr++;

}


/*Linear distance function
When a distance is given as input to this function, number of shaft count needed for that 
distance is calculated by the function.Then as the wheel rotates, the ISR is invoked 
and the scr or scl value increments in ISR and this value is compared with the required count
and if that value is reached, the loop breaks else the function for forward movement is 
executed 
*/

void linear_distance_mm(unsigned int DistanceInMM)
{ 	
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = DistanceInMM / 5.338; 	// division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	scr = 0; 	
	while(1) 	
	{
					  		
		if(scr > ReqdShaftCountInt)
		{
			break;
		}
		else
		{
			forward();
		}	

	} 

	stop(); //Stop action
}



/* code for distance calculation using IR sensor  */

void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; 		//all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; 		// all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00; 
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

void port_init()
{
	lcd_port_config();
	adc_pin_config();	
}
	
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;				//MUX5 = 0
	ADMUX = 0x20;				//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;				//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void init_devices (void)
{
	cli(); 						//Clears the global interrupts
	port_init();
	adc_init();
	sei(); 						//Enables the global interrupts
}
	
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;			// select the ch. > 7
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		//do not disturb the left adjustment
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; 		//clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}
	
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}



// Servo motor control codes

void servo1_pin_config (void)
{
 	DDRB  = DDRB | 0x20;  		//making PORTB 5 pin output
 	PORTB = PORTB | 0x20; 		//setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 	DDRB  = DDRB | 0x40;  		//making PORTB 6 pin output
 	PORTB = PORTB | 0x40; 		//setting PORTB 6 pin to logic 1
}



//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 	DDRB  = DDRB | 0x80;  		//making PORTB 7 pin output
 	PORTB = PORTB | 0x80; 		//setting PORTB 7 pin to logic 1
}

//Initialize the ports
void port_init_servo(void)
{ 
	servo1_pin_config(); 		//Configure PORTB 5 pin for servo motor 1 operation
 	servo2_pin_config(); 		//Configure PORTB 6 pin for servo motor 2 operation 
 	servo3_pin_config(); 		//Configure PORTB 7 pin for servo motor 3 operation  
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 42.187Hz 
void timer1_init(void)
{
 	TCCR1B = 0x00; 				//stop
 	TCNT1H = 0xFC; 				//Counter high value to which OCR1xH value is to be compared with
 	TCNT1L = 0x01;				//Counter low value to which OCR1xH value is to be compared with
 	OCR1AH = 0x03;				//Output compare eegister high value for servo 1
 	OCR1AL = 0xFF;				//Output Compare Register low Value For servo 1
 	OCR1BH = 0x03;				//Output compare eegister high value for servo 2
 	OCR1BL = 0xFF;				//Output Compare Register low Value For servo 2
 	OCR1CH = 0x03;				///Output compare eegister high value for servo 3
 	OCR1CL = 0xFF;				//Output Compare Register low Value For servo 3
 	ICR1H  = 0x03;	
 	ICR1L  = 0xFF;
 	TCCR1A = 0xAB; 				/*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 									For Overriding normal port functionalit to OCRnA outputs.
				  				{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 	TCCR1C = 0x00;
 	TCCR1B = 0x0C; 				//WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


//Function to initialize all the peripherals
void init_devices_servo(void)
{
 	cli(); 						//disable all interrupts
 	port_init_servo();
 	timer1_init();
 	sei(); 						//re-enable interrupts 
}


//Function to rotate Servo 1 by a specified angle in the multiples of 2.25 degrees
void servo_1(unsigned char degrees)  
{
 	float PositionPanServo = 0;
 	PositionPanServo = ((float)degrees / 2.25) + 21.0;
 	OCR1AH = 0x00;
 	OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 2.25 degrees
void servo_2(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 2.25) + 21.0;
 	OCR1BH = 0x00;
 	OCR1BL = (unsigned char) PositionTiltServo;
}


//Function to rotate Servo 3 by a specified angle in the multiples of 2.25 degrees
void servo_3(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 2.25) + 21.0;
 	OCR1CH = 0x00;
 	OCR1CL = (unsigned char) PositionTiltServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) 	//makes servo 1 free rotating
{
 	OCR1AH = 0x03; 
 	OCR1AL = 0xFF; 			//Servo 1 off
}

void servo_2_free (void) 	//makes servo 2 free rotating
{
 	OCR1BH = 0x03;
 	OCR1BL = 0xFF; 			//Servo 2 off
}

void servo_3_free (void) 	//makes servo 3 free rotating
{
 	OCR1CH = 0x03;
 	OCR1CL = 0xFF; 			//Servo 3 off
} 



//Main function - Performs the movement according to the data sent by matlab code.

void main()
{
	unsigned int value,value1;
	int a=0,b=0;
	cli();
	INIT_PORTS();										//Initialize ports
	uart0_init();										//Initialize UART0 for xbee communication
	timer5_init();
	sei();

	INIT_PORTS_ROTATE();								//Initialize ports 
	right_position_encoder_interrupt_init();			//Initialize control registers for wheel
	left_position_encoder_interrupt_init();				//           encoders.
			
	init_devices();
	lcd_set_4bit();										//LCD initialization functions.
	lcd_init();

	unsigned char angle = 0;
 	init_devices_servo();								//Initialize servo motors.

	data='0';
	sharp = ADC_Conversion(11);							//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
	lcd_print(1,1,value,3);

	sharp1 = ADC_Conversion(10);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	value1 = Sharp_GP2D12_estimation(sharp1);			//Stores Distance calsulated in a variable "value".
	lcd_print(1,5,value1,3);

	
	while(1)
	{

			while(data=='0')
			{

				velocity(150,150); 							//If no ball is detected the rotate and scan
				angle_rotate(3);							// for ball in the arena.
				stop();
				_delay_ms(500);
				sharp = ADC_Conversion(11);					//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
				value = Sharp_GP2D12_estimation(sharp);		//Stores Distance calsulated in a variable "value".
				lcd_print(1,1,value,3);
				sharp1 = ADC_Conversion(10);				//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
				value1 = Sharp_GP2D12_estimation(sharp1);	//Stores Distance calsulated in a variable "value".
				lcd_print(1,5,value1,3);


			}	
			/*If a ball(orange colour) is detected then matlab code sends a '5' signal through
			 zigbee.If a '5' is received then the robot stops rotating and moves towards the 
			 ball
			 */

			velocity(200,200);
			while(value>110 && value1>110)
			{
				sharp = ADC_Conversion(11);					//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
				value = Sharp_GP2D12_estimation(sharp);		//Stores Distance calsulated in a variable "value".
				lcd_print(1,1,value,3);
				sharp1 = ADC_Conversion(10);				//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
				value1 = Sharp_GP2D12_estimation(sharp1);	//Stores Distance calsulated in a variable "value".
				lcd_print(1,5,value1,3);

				forward();
			}

			stop();
			_delay_ms(2000);

			servo_3(0);								//code the open the gripper arm
			_delay_ms(1000);
			//servo1=90,servo2=95
			
			for (int angle1 = 0, angle2=80; angle1 <=77;angle1++,angle2--)
 			{
  				servo_1(angle1);					//code for downward movement of two servo motors 
  				_delay_ms(10);						// that holds the gripper
				servo_2(angle2);
  				_delay_ms(10);
 			}	
			

  			servo_3(120);							//Grab the ball and close the arm
  			_delay_ms(10);
			_delay_ms(1000);
		
			servo_1(0);								//code for upward movement of two servo motors 
			servo_2(185);							// that holds the gripper
			
			data='2';

			while(data=='2')				 		//Search for the basket. 
			{
				velocity(150,150);
				angle_rotate_left(3);
				stop();
				_delay_ms(500);
			}


			sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
			value = Sharp_GP2D12_estimation(sharp);			//Stores Distance calsulated in a variable "value".
			lcd_print(1,1,value,3);
			sharp1 = ADC_Conversion(10);					//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
			value1 = Sharp_GP2D12_estimation(sharp1);		//Stores Distance calsulated in a variable "value".
			lcd_print(1,5,value1,3);

			while(value>110 && value1>110)
			{
				sharp = ADC_Conversion(11);					//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
				value = Sharp_GP2D12_estimation(sharp);		//Stores Distance calsulated in a variable "value".
				lcd_print(1,1,value,3);
				sharp1 = ADC_Conversion(10);				//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
				value1 = Sharp_GP2D12_estimation(sharp1);	//Stores Distance calsulated in a variable "value".
				lcd_print(1,5,value1,3);

				forward();
			}
			stop();

			for (int angle1 = 0, angle2=52; angle1 <=50;angle1++,angle2--)
 			{
  				servo_1(angle1);			//code for downward movement of two servo motors
  				_delay_ms(10);				// that holds the gripper
				servo_2(angle2);
  				_delay_ms(10);
 			}
			servo_3(0);						//code the open the gripper arm to drop the ball
			servo_1(0);						//code for upward movement of two servo motors 
			servo_2(185);					// that holds the gripper

			for (angle = 0; angle <120; angle++)
 			{
  				servo_3(angle);					//code to close the gripper arm
  				_delay_ms(10);
 			}

			velocity(150,150);					//Code for rotation of the robot 
			angle_rotate_right_hard(25);		//to avoid the box
			stop();
			_delay_ms(500);
			
			data='0';
			
		
			sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
			value = Sharp_GP2D12_estimation(sharp);			//Stores Distance calsulated in a variable "value".
			lcd_print(1,1,value,3);
			sharp1 = ADC_Conversion(10);					//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
			value1 = Sharp_GP2D12_estimation(sharp1);		//Stores Distance calsulated in a variable "value".
			lcd_print(1,5,value1,3);

					
	}

}

