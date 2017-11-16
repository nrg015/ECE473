// lab3.c 
// N. Grubaugh
// 10.26.17

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTE bit 7 goes to the PWM transistor base.
//  PORTE bit 6 to SH_LD
//  
//  PORTB bit 0 to clk_inh
//  PORTB bit 1 to sck and srclk
//  PORTB bit 2 to SDIN
//  PORTB bit 3 to SOUT
//
//  PORTD bit 2 to regclk

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile int16_t count = 0x0000;

volatile uint8_t i = 0;

volatile uint8_t mode = 0x00;           //mode set by buttons
volatile uint8_t count_mode = 0x00;     //holds increment/decrement amount

volatile uint8_t enc_prevA[2];     //previous encoder values to determine
                                        //rotation
volatile uint8_t enc_prevB[2];
volatile uint8_t enc_prev2A = 0x00;
volatile uint8_t enc_prev2B = 0x00;

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
//                            spi_init
// Initializes the SPI port, nothing else.
//
void spi_init(void) {
    DDRB = 0x07;                //Turn on SS, MOSI, SCLK (SS is output)
    SPCR = (1<<SPE)|(1<<MSTR);  //SPI enabled, master, low polarity, MSB 1st
    SPSR = (1<<SPI2X);          //run at i/o clock/2
}
//******************************************************************************

//******************************************************************************
//                            spi_read
//Reads the SPI port.
//
uint8_t spi_read(void) {
    SPDR = 0x00;                        //"dummy" write
    while (bit_is_clear(SPSR,SPIF)){}   //wait till 8 clock cycles are done
    return(SPDR);                       //return incoming data from SPDR
}
//******************************************************************************

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
//                                  button_togg
void button_togg(uint8_t val) {
    if (val == 0) {
        mode ^= 0x01;
    }
    if (val == 1) {
        mode ^= 0x02;
    }
}
//***********************************************************************************

//***********************************************************************************
//                                  set_count_mode
void set_count_mode(uint8_t mode) {
    if (mode == 0x00) {
        count_mode = 0x01;
    }
    else if (mode == 0x01) {
        count_mode = 0x02;
    }
    else if (mode == 0x02) {
        count_mode = 0x04;
    }
    else if (mode == 0x03) {
        count_mode = 0x00;
    }
}
//***********************************************************************************

//***********************************************************************************
//                                  update_bar
void update_bar(uint8_t mode) {
    SPDR = mode;        //send data
    while(bit_is_clear(SPSR, SPIF)) {} //transfer
    
    PORTD |= 0x04;      //send rising edge
    PORTD &= ~0x04;     //send falling edge
}
//***********************************************************************************

//***********************************************************************************
//                                  read_enc

int8_t read_enc() {
    uint8_t enc_A[2], enc_B[2];
    uint8_t data;
    uint8_t delta = 0x00;

    PORTE |= 0x40;
    PORTE &= ~0x20;
    _delay_ms(1);

    PORTE |= 0x20;
    PORTE &= ~0x40;
    
    data = spi_read();

    enc_B[0] = data & 0x01;
    enc_A[0] = data & 0x02;
    enc_B[1] = data & 0x04;
    enc_A[1] = data & 0x08;
  
    for (i = 0; i<2; i++) {        
        if(enc_A[i] != enc_prevA[i]) {
            if(enc_A[i]) {
                if(!enc_B[i]) {
                    delta += count_mode;
                }
                else {
                    delta -= count_mode;
                }
            }
            else {
                if(enc_B[i]) {
                    delta += count_mode;
                }
                else {
                    delta -= count_mode;
                }
            }
        }
        enc_prevA[i] = enc_A[i];            
        if(enc_B[i] != enc_prevB[i]) {
            if(enc_B[i]) {
                if(enc_A[i]) {
                    delta += count_mode;;
                }
                else {
                    delta -= count_mode;
                }
            }
            else {
                if(!enc_A[i]) {
                    delta += count_mode;
                }
                else {
                    delta -= count_mode;
                }
            }
        }
        enc_prevB[i] = enc_B[i];            
    }
    return delta;
}
//***********************************************************************************

ISR(TIMER0_OVF_vect) {
	uint8_t portB_temp = PORTB;
    uint8_t portA_temp = PORTA;
    uint8_t temp = i;
    uint8_t data;           //data holder for spi
    int8_t enc_data_1, enc_data_2; //data holders foe encoder reads
  	DDRA = 0x00; //make PORTA an input port with pullups
	PORTA = 0xFF; 
  	PORTB = 0x70; //enable tristate buffer for pushbutton switches
			      //The logic for tristate buffer to be enabled is 
			      //111 for bits 4-6. (01110000)
 	
    for (i=0; i<2; i++) { //now check each button and increment the count as needed
        if (chk_buttons(i)) {
            button_togg(i);	
        }
	}//for loop

  	PORTB = 0x01; //disable tristate buffer for pushbutton switches
    DDRA = 0xFF;

    set_count_mode(mode);
    update_bar(mode);

    count += read_enc();
    
    if (count < 0) {
        count = 1023;
    }
    count %= 1024;

    i = temp;
    PORTB = portB_temp;
    DDRA = 0xFF;
    PORTA = portA_temp;
}//TIMER0_COMP_vect


uint8_t main()
{
    //timer counter 0 setup, running off i/o clock
    TIMSK |= (1<<TOIE0);            //enable interrupts
    TCCR0 |= (1<<CS02)|(1<<CS00);   //normal mode, prescale by 128
    spi_init(); //initialize SPI

	DDRB |= 0xF0; //set port bits 4-7 B as outputs
    DDRD |= 0xFF;
    DDRE |= 0xFF;
    PORTE |= 0x40;
    
    sei();

	while(1){
  		PORTB = 0x00;

		segsum(count);  //break up the disp_value to 4, BCD digits in the array: call (segsum)	
  		
        DDRA = 0xFF;

		for(i=0; i<5; i++) {
  			PORTA = segment_data[i];  //send 7 segment code to LED segments
			PORTB = (i<<4);//send PORTB the digit to display
			_delay_ms(1);
		}
  	}//while
	return 0;
}//main
