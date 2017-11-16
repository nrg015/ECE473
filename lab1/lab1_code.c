// lab1_code.c 
// R. Traylor
// 7.21.08

//This program increments a binary display of the number of button pushes on switch 
//S0 on the mega128 board.

#include <avr/io.h>
#include <util/delay.h>

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}

//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounce_switch(), increment
// PORTB.  This will make an incrementing count on the port B LEDS. 
//*******************************************************************************
int main()
{
 DDRB = 0xFF;  //set port B to all outputs
 int i = 0;    //1's digit counter
 int j = 0;    //10's digit counter
 while(1){     //do forever
	if(debounce_switch()) {
		PORTB++; //increment value
		i++;     //increment 1's digit
 	}  //if switch true for 12 passes, increment port B
	
        if(i == 10) {                   //check if 10 has been reached 
		PORTB ^= 0x0A;          // if so, toggle back to 0, and add 1 to 10's digit
		i = 0;
		j++;
                PORTB += 0x10;
	}
	
	if(j == 10) {                   //check if 100 has been reached
		PORTB = 0x00;           // if so, reset back to 0
		i, j = 0;
	}
	_delay_ms(2);                    //keep in loop to debounce 24ms

 } //while 
} //main
