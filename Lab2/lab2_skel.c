// lab2_skel.c 
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] = {
	0b11000000, 
	0b11111001,
	0b10100100,
	0b10110000, 
	0b10011001,
	0b10010010, 
	0b10000010, 
	0b11111000, 
        0b10000000,
	0b10011000,
	0b11111111,
	0b01111111
}; 


//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
uint8_t chk_buttons(uint8_t button) {
	static uint16_t state[8] = {0};
	state[button] = (state[button]<<1) | (! bit_is_clear(PINA, button)) | 0xE000;
	if (state[button] == 0xF000) return 1;
	return 0;
}
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
	//determine how many digits there are 
  	 //break up decimal sum into 4 digit-segments
	segment_data[0] = dec_to_7seg[(sum) % 10];
	segment_data[1] = dec_to_7seg[(sum/10) % 10];
	segment_data[3] = dec_to_7seg[(sum/100) % 10];
	segment_data[4] = dec_to_7seg[(sum/1000) % 10];

	segment_data[2] = 0xFF;

	//blank out leading zero digits 
	if(sum<1000) segment_data[4] = 0xFF;
	if(sum<100) segment_data[3] = 0xFF;
	if(sum<10) segment_data[1] = 0xFF;
	return;
}//segment_sum
//***********************************************************************************


//***********************************************************************************
uint8_t main()
{
	uint8_t i = 0;
	uint16_t count = 0x0000;
    DDRE = 0xFF;
    PORTE = 0x00;
	DDRB = 0xF0; //set port bits 4-7 B as outputs
	while(1){
  		_delay_ms(1); //insert loop delay for debounce
  		DDRA = 0x00; //make PORTA an input port with pullups
		PORTA = 0xFF; 
  		PORTB = 0xF0; //enable tristate buffer for pushbutton switches
			      //The logic for tristate buffer to be enabled is 
			      // 101 for bits 4-6. (01010000)
 		for (i=0; i<8; i++) { //now check each button and increment the count as needed
                	if (chk_buttons(i)) {
                       		count += (1<<i);	
			}
		}//for loop

  		PORTB = 0x00; //disable tristate buffer for pushbutton switches
  		
		if (count > 1023) {     //bound the count to 0 - 1023
                	count -= 1023;
		}
  		
		segsum(count);  //break up the disp_value to 4, BCD digits in the array: call (segsum)
  		
		//bound a counter (0-4) to keep track of digit to display 
		DDRA = 0xFF; //make PORTA an output
  		
		for(i=0; i<5; i++) {
  			PORTA = segment_data[i];  //send 7 segment code to LED segments
			PORTB = (i<<4);//send PORTB the digit to display
			_delay_ms(1);
		}
  	}//while
	return 0;
}//main
