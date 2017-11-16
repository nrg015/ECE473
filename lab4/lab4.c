// lab4.c 
// N. Grubaugh
// 11.9.17

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  
//  PORTB bit 1 to sck and srclk
//  PORTB bit 2 to SDIN
//  PORTB bit 3 to SOUT
//  PORTB bit 4 to SEL0
//  PORTB bit 5 to SEL1
//  PORTB bit 6 to SEL2
//  PORTB bit 7 to PWM of LED
//
//  PORTC bit 0 to OPAMP R
//  PORTC bit 1 to OPAMP L
//
//  PORTD bit 2 to regclk
//
//  PORTE bit 3 to VOL (on audio amp)
//  PORTE bit 5 to SH_LD
//  PORTE bit 6 to CLK_INH
//
//  PORTF bit 7 to CDS cell
//

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define TEST_SNOOZE 1   //If 1, snooze will be 10 secs long instead of 10 min

#define SEC 2
#define MIN 1
#define HOUR 0

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"

volatile uint8_t i = 0;
uint16_t adc_result;
char lcd_str[32];
//char lcd_str_l[16];
div_t fp_adc_result, fp_low_result;

volatile uint8_t colon_toggle = 0x01;
volatile uint8_t pm = 0x00;
//Holds count for realtime clock
volatile int16_t count = 0x00;
volatile uint8_t display_counter = 0x00;

//Holds current time
volatile int8_t time[3] = {0x00,0x00,0x00};  //(0) hours, (1) min, (2)sec
int8_t new_time[3] = {0x00, 0x00, 0x00};

volatile float volume = 0.40;

volatile uint8_t set_clock[2] = {0x00, 0x00};      //set_clock mode
volatile uint8_t clock_format = 0x00;     //holds increment/decrement amount

volatile uint8_t set_alarm[2] = {0x00, 0x00}; //0 - set alarm toggle
                                              //1 - alarm set status
volatile uint8_t alarm[3] = {0x00, 0x00, 0x00}; //holds alarm

volatile uint8_t enc_prevA[2];     //previous encoder values to determine
volatile uint8_t enc_prevB[2];     //rotation

volatile uint8_t snooze = 0x00;
//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] = {
	0b11000000, //0
	0b11111001, //1
	0b10100100, //2
	0b10110000, //3
	0b10011001, //4
	0b10010010, //5
	0b10000010, //6
	0b11111000, //7
    0b10000000, //8
	0b10011000, //9
	0b11111100, //:
	0b11111011  //PM
}; 

//******************************************************************************
//                            spi_init
// Initializes the SPI port, nothing else.
//
void spi_init(void) {
    DDRF    |= 0x08; //enable LCD?
    PORTF   &= 0xF7;

    DDRB = 0x07;                //Turn on SS, MOSI, SCLK (SS is output)
    PORTB = (1<<PB1);

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
//                                   LED data update                                    
//takes 4 8-bit binary input values and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void led_data_update(uint8_t hours, uint8_t min, uint8_t colon, uint8_t pm) {
	uint8_t h = hours;
    
    if (clock_format) {
        if (h == 0) {
            h = 12;
        }
        
        if (h > 12) {
            h -= 12;
        }    
    }
 
    segment_data[0] = dec_to_7seg[(min) % 10];
	segment_data[1] = dec_to_7seg[(min/10) % 10];
	segment_data[3] = dec_to_7seg[(h) % 10];
	segment_data[4] = dec_to_7seg[(h/10) % 10];

	if (colon) {
        segment_data[2] = dec_to_7seg[10];
    }   
    
    else{
        segment_data[2] = 0xFF;
    }

    if (pm) {
        segment_data[2] &= dec_to_7seg[11];
    }

	return;
}//led_data_update
//***********************************************************************************

//***********************************************************************************
//                                  button_togg
void button_togg(uint8_t val) {
    if (val == 0) {
        snooze = 0x01;
    }
    if (val == 1) {
        if(volume < 0.6) {
            volume += 0.1;
        } 
    }
    if (val == 2) {
        if (volume > 0.2) {
            volume -= 0.1;
        }
    }
    if (val == 3) {
        if (set_alarm[0]) {
            alarm[HOUR] = new_time[HOUR];
            alarm[MIN] = new_time[MIN];
            set_alarm[0] = 0x00;
        }
        set_alarm[1] ^= 0x01;              //arms alarm
        
        if (set_alarm[1]) {
            string2lcd("Alarm Armed");
        }
        else {
            clear_display();
            cursor_home();
        }
    }
    if (val == 4) {
        set_alarm[0] ^= 0x01;              //allows user to input alarm
    }
    if (val == 5) {
        clock_format ^= 1;                 //24 or 12 hour format
    }
    if (val == 6) {
        if (set_clock[1]) {                 //pushes update to clock
            time[HOUR] = new_time[HOUR];
            time[MIN] = new_time[MIN];
            time[SEC] = 0x00;
            set_clock[1] = 0;
        }
    }
    if (val == 7) {
        set_clock[1] ^= 0x01;              //allows user to input clock
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

void read_enc() {
    uint8_t enc_A[2], enc_B[2];
    uint8_t data;
    uint8_t j = 2;

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
        j--;
        
        if(enc_A[i] != enc_prevA[i]) {
            if(enc_A[i]) {
                if(!enc_B[i]) {
                    new_time[j] += 1;
                }
                else {
                    new_time[j] -= 1;
                }
            }
            else {
                if(enc_B[i]) {
                    new_time[j] += 1;
                }
                else {
                    new_time[j] -= 1;
                }
            }
        }
        enc_prevA[i] = enc_A[i];            
        if(enc_B[i] != enc_prevB[i]) {
            if(enc_B[i]) {
                if(enc_A[i]) {
                    new_time[j] += 1;
                }
                else {
                    new_time[j] -= 1;
                }
            }
            else {
                if(!enc_A[i]) {
                    new_time[j] += 1;
                }
                else {
                    new_time[j] -= 1;
                }
            }
        }
        enc_prevB[i] = enc_B[i];            
    }
    return;
}
//***********************************************************************************

ISR(ADC_vect) {

}

ISR(TIMER1_COMPA_vect) {
    PORTC = PORTC ^ 0x3;
}

ISR(TIMER0_OVF_vect) {
	//Clock Handling
    count++;
    display_counter++;

    if (display_counter == 256) {
        display_counter = 0;
    }

    if ((count%32) == 0) {
        set_clock[0] ^= 0x01;    
    }
    
    if (count == 128) 
    {
        //1 Sec
        time[SEC] += 1;
        colon_toggle ^= 0x01;
        count = 0;
        if (time[SEC] == 60)
        {
            //1 Min
            time[SEC] = 0;
            time[MIN] += 1;
            if (time[MIN] == 60) 
            {
                //1 Hour
                time[MIN] = 0;
                time[HOUR] += 1;
                
                if (time[HOUR] == 24)
                {
                    //1 Day
                    time[HOUR] = 0;
                }
            }
        }    
    }

    //PM or AM
    if (time[HOUR] >= 12) 
    {
        pm = 1;
    }
    else 
    {
        pm = 0;
    }
    
    // Button/Encoder Handling
    uint8_t portB_temp = PORTB;
    uint8_t portA_temp = PORTA;
    uint8_t temp = i;
  	DDRA = 0x00; //make PORTA an input port with pullups
	PORTA = 0xFF; 
  	PORTB = 0x70; //enable tristate buffer for pushbutton switches
			      //The logic for tristate buffer to be enabled is 
			      //111 for bits 4-6. (01110000)
 	
    for (i=0; i<8; i++) { //now check each button and increment the count as needed
        if (chk_buttons(i)) {
            button_togg(i);	
        }
	}//for loop

  	PORTB = 0x01; //disable tristate buffer for pushbutton switches
    DDRA = 0xFF;

    if (set_alarm[1] == 0x07) {
        if(snooze) {
            DDRC &= 0b11111100;
            snooze = 0;
            set_alarm[1] &= 0x01;
            alarm[HOUR] = time[HOUR]; 
            alarm[MIN] = time[MIN];
            alarm[SEC] = time[SEC];
            #if TEST_SNOOZE==0
            alarm[MIN] += 10;
            if (alarm[MIN] > 60) {
                alarm[MIN] -= 60;
                alarm[HOUR] += 1;
                if (alarm[HOUR] > 24) {
                    alarm[HOUR] = 0;
                }
            
            #else
            alarm[SEC] += 10;
            if (alarm[SEC] > 60) {
                alarm[SEC] -= 60;
                alarm[MIN] += 1;
                if (alarm[MIN] > 60) {
                    alarm[MIN] -= 60;
                    alarm[HOUR] += 1;
                    if (alarm[HOUR] > 24) {
                        alarm[HOUR] = 0;
                    }
                }
            }
            #endif
        }    
    }

    
    //if(volume == 0.2) {
    //    set_alarm[1] ^= 0x80;    
    //}
    //else if(volume == 0.3) {
    //    set_alarm[1] ^= 0xC0;    
    //}
    //else if(volume == 0.4) {
    //    set_alarm[1] ^= 0xE0;    
    //}
    //else if(volume == 0.5) {
    //    set_alarm[1] ^= 0xF0;    
    //}
    //else if(volume == 0.6) {
    //   set_alarm[1] ^= 0xF8;    
    //}
  

    update_bar(set_alarm[1]);

    //if clock is being set, encoders take in user input
    if(set_clock[1] | set_alarm[0]) {
        read_enc();
    
        //modulate to a real time
        if(new_time[MIN] > 59) {
            new_time[MIN] = 0;
            new_time[HOUR] +=1;
        }
        else if(new_time[MIN] < 0) {
            new_time[MIN] = 59;
            new_time[HOUR] -=1;
        }

        if(new_time[HOUR] > 23) {
            new_time[HOUR] = 0;
        }
        else if(new_time[HOUR] < 0) {
            new_time[HOUR] = 23;
        }
        if(new_time[HOUR] > 12) {
            pm = 1;
        }
        else {
            pm = 0;
        }
    }    
    
    i = temp;
    PORTB = portB_temp;
    DDRA = 0xFF;
    PORTA = portA_temp;
}//TIMER0_COMP_vect


//***********************************************************************************
//                                  MAIN Program
int main()
{
    spi_init(); //initialize SPI
    
    //DDRF    |= 0x08; //enable LCD?
    //PORTF   &= 0xF7;

    lcd_init(); //initialize LCD
    clear_display();

    TIMSK |= (1<<TOIE0) | (1<<OCIE1A);            //enable interrupts
    
    //timer counter 0 setup, running off external 32khz clock
    TCCR0 |= (1<<CS00);   //normal mode, no prescale
    ASSR  |= (1<<AS0); //Asynchronous Clock

    //timer counter 1 setup
    TCCR1A = 0;
    TCCR1B = (1<<WGM12) | (1<<CS11); //CTC Mode
    OCR1A = 999;

    //timer counter 2 setup, fast PWM, 256 prescale, noninverting
    TCCR2 |= (1<<WGM20) | (1<<COM21) | (0<<COM20) | (1<<WGM21) | (1<<CS22) | (1<<CS21) | (1<<CS20);
    
    //timer counter 3 setup
    TCCR3A = (1<<COM3A1) | (0<<COM3A0) | (1<<WGM31);
    TCCR3B = (1<<WGM32) | (1<<WGM33) | (1<<CS31);
    OCR3A  = 61440 * volume;
    ICR3   = 0xF000;

	//ADC Initialization and associated ports
    DDRF &= ~(1<<DDF7);
    PORTF &= ~(1<<PF7);

    ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0);

    ADCSRA = (1<<ADEN)|(0<<ADFR)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
 
    //Setup Ports
    DDRB |= 0b01110000; //set port bits 4-7 B as outputs
    DDRC |= 0b00000000; //alarm is on pins 0 and 1
    DDRD |= 0b11111111;
    DDRE |= 0b11111111;
    
    PORTE |= 0x40;
    
    //sei();

	while(1){
        ADCSRA |= (1<<ADSC);
        
        while(bit_is_clear(ADCSRA, ADIF)){};
        
        ADCSRA |= (1<<ADIF);
    
        adc_result = ADC;
    
        
        //fp_adc_result = div(adc_result, 205);              //do division by 205 (204.8 to be exact)
        //itoa(fp_adc_result.quot, lcd_str_h, 10);           //convert non-fractional part to ascii strin
        itoa(adc_result, lcd_str, 10);           //convert non-fractional part to ascii strin
        //fp_low_result = div((fp_adc_result.rem*100), 205); //get the decimal fraction into non-fraction
        //itoa(fp_low_result.quot, lcd_str_l, 10);           //convert fractional part to ascii string
        
        //send string to LCD
        string2lcd(lcd_str);  //write upper half
        //char2lcd('.');          //write decimal point
        //string2lcd(lcd_str_l);  //write lower half
 
        for(i=0;i<=10;i++){ _delay_ms(50);}  //delay 0.5 sec
        clear_display();
        cursor_home();

		#if 0
        if (set_clock[1] | set_alarm[0]) {
            led_data_update(new_time[0], new_time[1], colon_toggle, pm);  	
        }
        
        else {
            new_time[HOUR] = 0;
            new_time[MIN] = 0;
            led_data_update(time[0], time[1], colon_toggle, pm);  	
  		}

        if (set_alarm[1] & 0x01) {
            if ((time[HOUR] == alarm[HOUR]) & (time[MIN] == alarm[MIN]) & (time[SEC] == alarm[SEC])) {
                DDRC |= 0x03;   //play alarm
                set_alarm[1] |= 0x06;   //signal alarm is going off
            }
        }
        
        DDRA = 0xFF;

		for(i=0; i<5; i++) {
  			PORTA = segment_data[i];  //send 7 segment code to LED segments
			
            if((set_clock[1] | set_alarm[0]) & set_clock[0] & (i!= 2)) {
                PORTA = 0xFF;
            }

            PORTB = (i<<4);//send PORTB the digit to display
			_delay_ms(1);
		}
        #endif
  	}//while
	return 0;
}//main
