/*
 * main.c
 *
 * Created: 06/09/2018 6:29:10 PM
 * Author : Ritvik Sadana, Shashank D Prasad
 */

#include <stdio.h>
#include <avr/interrupt.h>
#include "uart.h"
#include <util/delay.h>

static int risingEdgeTimerVal;  // Record the time at rising edge.
static int fallingEdgeTimerVal; // Record the time at falling edge.
static int pulseWidth;			// pulseWidth = risingEdge - fallingEdge.
static int edgeCount = 0;		// Keep a count of how many edges are detected 
								// while calculating pulse width.
unsigned int edgeDetect = 0;	// For checking number of edges while 
								// pressing the switch
int contOrDisc = 1;

/*
 * Set Delay times of the 5us pulse that we send to the sensor.
 */
unsigned int DelayHi= 10;		// high time of the pulses to be created (5us).
unsigned int DelayLo= 38500;	// low time of the pulses to be created.

char HiorLo;					// Flag indicating the status of 5us pulse.

void setupADC(void);								// Function to setup ADC.
void adcStartConversion(void);						// Function to start ADC conversion.
void discrete_sampling(unsigned int pulsewidth);	// Function to enable discrete mode 
													// of setting value of OCR0A.
void continuos_sampling(unsigned int pulseWidth);	// Function to enable continuous mode 
													// of setting value of OCR0A.
void photoSensorSampling(unsigned int adcInput);	// Sample ADC values and control the volume using DAC.

int main(void)
{
	// Call uart_init for print functions.
	uart_init();
	
	//
	// Configuration for LED and Switch.
	// LED: PORTB-PIN5
	// Switch PORTB-PIN0 
	// Sensor PORTB-PIN1
	//
	DDRB |= 0x22;  // Set PINB5 (LED) as output mode.
	PORTB |= 0x00; 
	
	//
	// Configuration for Buzzer PORTD - PIN6;
	//
	DDRD  = 0x40;
	PORTD = 0x00;
	
	//Configuration for portD3 to be an external interrupt
	DDRD |= (0<<PORTD3);
	PORTD |= (1 << PORTD3);

	
	EIMSK |= (1<<INT1); //Enable external interupt for INT1 port 
	
	//
	// Timer 0 configuration for buzzer.
	//
	TCCR0A |= 0x42;				// Set to CTC mode with OCRA toggle.
	TIMSK0 |= (1 << OCIE0A);    // Enable the timer interrupt.
	TCCR0B |= (1 << CS02);		// Pre-scaler of 256.
	
	//
	// Timer 1 configuration for Sensor output and input.
	//
	 TCCR1B |=  (1<< ICES1) | (1 << ICNC1) | (1<<CS11); // Configure for rising edge initially.
	
	//Timer 1 configuration for Sensor interfacing
	TCCR1A |= 0x40;				//Toggle OC1A/OC1B on Compare Match.	
	
	OCR1A += DelayLo;			// Start the OC1 operation	
	HiorLo= 0;					// Next time use DelayLo as delay count of OC0 operation
	TIMSK1 |=  (1<<OCIE1A); 	// Output compare enable on OC1A.
	
    sei();						// Enable global interrupts.
	
	setupADC();					// Setup ADC.
	adcStartConversion();		// Start ADC conversion.
	
	DDRB |= 0B00011100;			// Set the DAC pins for volume control to output mode.
	
    while(1){
		
		// Poll if button is being pressed or not.
		if(contOrDisc){
			continuos_sampling(pulseWidth);
		} else{
			discrete_sampling(pulseWidth);
		}
		
		// Sample ADC values and control the volume using DAC.
		photoSensorSampling(ADC);
	}
}

void continuos_sampling(unsigned int pulseWidth){
	int offset = (((pulseWidth-300)*15)/4700) + 14; /* Range of vaules 300-5000 = 4700
							  Range of values of OCR0A 15-30 = 15 */
	OCR0A = offset;
}

void discrete_sampling(unsigned int pulsewidth){
	 
	if (pulseWidth >= 300 && pulseWidth <880){
		OCR0A = 15;
		} else if(pulseWidth >= 887 && pulseWidth <1470){
		OCR0A = 16;
		} else if(pulseWidth >= 1474  && pulseWidth <2055){
		OCR0A = 18;
		} else if(pulseWidth >= 2061  && pulseWidth <2643){
		OCR0A = 20;
		}  else if(pulseWidth >= 2648  && pulseWidth <3230){
		OCR0A = 22;
		}  else if(pulseWidth >= 3235  && pulseWidth <3818){
		OCR0A = 23;
		}  else if(pulseWidth >= 3822  && pulseWidth <4405){
		OCR0A = 26;
		} else if(pulseWidth >= 4409){
		OCR0A = 30;
		} else {
		//Do nothing;
	}
}

void photoSensorSampling(unsigned int adcInput)
{
	if(adcInput <= 964 && adcInput >= 901)
	{
							  // Set
		PORTB |= 0B00010000;  // PB4 1 0B00010000 1 << PORTB4
		PORTB |= 0B00001000;  // PB3 1 0B00001000 1 << PORTB3
		PORTB |= 0B00000100;  // PB2 1 0B00000100 1 << PORTB2
	}
	else if(adcInput < 901 && adcInput >= 838)
	{
							  // Set
		PORTB |= 0B00010000;  // PB4 1 0B00010000 1 << PORTB4
		PORTB |= 0B00001000;  // PB3 1 0B00001000 1 << PORTB3
		PORTB &= 0B11111011;  // PB2 0 0B11111011 !(1 << PORTB2)	
	}
	else if(adcInput < 838 && adcInput >= 775)
	{
							  // Set
		PORTB |= 0B00010000;  // PB4 1 0B00010000 1 << PORTB4
		PORTB &= 0B11110111;  // PB3 0 0B11110111 !(1 << PORTB3)
		PORTB |= 0B00000100;  // PB2 1 0B00000100 1 << PORTB2
	}
	else if(adcInput < 775 && adcInput >= 712)
	{ 
							  // Set 
		PORTB |= 0B00010000;  // PB4 1 0B00010000 1 << PORTB4
		PORTB &= 0B11110111;  // PB3 0 0B11110111 !(1 << PORTB3)
		PORTB &= 0B11111011;  // PB2 0 0B11111011 !(1 << PORTB2)
	}
	else if(adcInput < 712 && adcInput >= 649)
	{
							  // Set
		PORTB &= 0B11101111;  // PB4 0 0B11101111 !(1 << PORTB4)
		PORTB |= 0B00001000;  // PB3 1 0B00001000 1 << PORTB3
		PORTB |= 0B00000100;  // PB2 1 0B00000100 1 << PORTB2
	}
	else if(adcInput < 649 && adcInput >= 586)
	{
							  // Set
		PORTB &= 0B11101111;  // PB4 0 0B11101111 !(1 << PORTB4)
		PORTB |= 0B00001000;  // PB3 1 0B00001000 1 << PORTB3
		PORTB &= 0B11111011;  // PB2 0 0B11111011 !(1 << PORTB2)
	}
	else if(adcInput < 586 && adcInput >= 523)
	{
							  // Set
		PORTB &= 0B11101111;  // PB4 0 0B11101111 !(1 << PORTB4)
		PORTB &= 0B11110111;  // PB3 0 0B11110111 !(1 << PORTB3)
		PORTB |= 0B00000100;  // PB2 1 0B00000100 1 << PORTB2
	}
	else if(adcInput < 523 && adcInput >= 440)
	{
						      // Set
		PORTB &= 0B11101111;  // PB4 0 0B11101111 !(1 << PORTB4)
		PORTB &= 0B11110111;  // PB3 0 0B11110111 !(1 << PORTB3)
		PORTB &= 0B11111011;  // PB2 0 0B11111011 !(1 << PORTB2)
	}
	
}

void setupADC(void){

	ADMUX |= (1 << REFS0); 	// Setting ADC to use VCC as reference voltage 
	   					    //  Use A0 pin to read input

	ADCSRA |= (1 << ADSC) | (1 << ADEN) | (1 << ADPS2) | (1<< ADPS1) ; // Turn ADC on, 
																	   //  Use pre-scaler as 4
																	   //  Set auto-trigger. 

	DIDR0 |= (1<<ADC0D);   // Disable digital input buffer

}

void adcStartConversion(void){
	ADCSRA |= (1 << ADATE);
}


// ISR for 5us pulse.
ISR(TIMER1_COMPA_vect)
{
		if(HiorLo)
		{
			/*
			 * Enable the interrupts for pulse width detection
			 * upon reaching falling edge of the 5us pulse.
			 */
			
			OCR1A += DelayLo;
			HiorLo = 0;
			TIMSK1 |= (1 << ICIE1);		
		}
		else{
			OCR1A += DelayHi;
			HiorLo = 1;
			
		}
		
}

/* 
 * Empty ISR for timer 0 because it was interfering 
 * with other ISRs as the flag was not getting cleared.
 */
ISR(TIMER0_COMPA_vect){}

/*
 * ISR to handle button press and toggle between continuous and discrete mode
 */
ISR(INT1_vect){
	edgeDetect++;

	if(edgeDetect%2 == 0){ contOrDisc = 1; } 
	else{                  contOrDisc = 0; }
}


ISR(TIMER1_CAPT_vect)
{
	edgeCount++;
	
	/*
	 * if the edge count is 2, that means we have detected falling edge. 
	 * Hence, take the difference to find the pulse width.
	 */
	if(edgeCount % 2 == 0)
	{
		fallingEdgeTimerVal = ICR1;
		pulseWidth = (fallingEdgeTimerVal - risingEdgeTimerVal);
		
		unsigned int offset = ((pulseWidth - 300)/40) + 119;
		OCR0A = offset;
		edgeCount = 0;
		
		// Disable the interrupt for pulse calculation.
		TCCR1B |= (1 << ICES1);
		TIMSK1 &= 0xDF;

	}
	else
	{
		risingEdgeTimerVal = ICR1;
		TCCR1B &= 0xBF; 
	}
}
