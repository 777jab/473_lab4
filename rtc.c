#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* global vars */
//holds data to be sent to the segments. logic zero turns segment on
static uint8_t segment_data[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
//decimal to 7-segment LED display encodings, logic "0" turns on segment
const uint8_t dec_to_7seg[12] = { 
    0b11000000, //0 
    0b11111001, //1 
    0b10100100, //2 
    0b10110000, //3 
    0b10011001, //4 
    0b10010010, //5 
    0b10000010, //6 
    0b11111000, //7 
    0b10000000, //8 
    0b10010000, //9 
    0b11111111, //blank
    0b01111111  //D.P.
};

static uint8_t hr    = 17;
static uint8_t min   = 23;
static uint8_t sec   = 0;
static uint8_t col   = 0; //keeps track of whether the colon is lit.
static uint8_t which_digit = 0;//keeps track of which digit is lit.

ISR(TIMER0_COMP_vect){
    static uint16_t count = 0;
    count++;
    if(count >= 511){ 
        sec++; 
        count = 0;
        col ^= 1;
    }
    if(sec >= 59)   { min++; sec = 0;}
    if(min >= 59)   { hr++; min = 0;}
    if(hr >= 23)    { hr = 0; }
}
uint8_t seg_time(){
    /* colon */
    if(col){ segment_data[2] &= ~(0x03); }
    else{ segment_data[2] |= 0x03; }
    /* hours */
    segment_data[4] = dec_to_7seg[ hr / 10 ];
    segment_data[3] = dec_to_7seg[ hr % 10 ];
    /* minutes */
    segment_data[1] = dec_to_7seg[ min / 10 ];
    segment_data[0] = dec_to_7seg[ min % 10 ];
    return 0;
}
uint8_t main(){
    /* set up timer and interrupt */
    ASSR  |= (1<<AS0); //run off external 32khz osc (TOSC)
    //enable interrupts for output compare match 0
    TIMSK |= (1<<OCIE0);
    TCCR0 |=  (1<<WGM01) | (1<<CS00);  //CTC mode, no prescale
    OCR0  =   63;   //interrupt every 1/(2^9) sec
    sei();    
    /* set up display */
    DDRA  = 0xFF;   //start port A out as all outputs
    PORTA = 0xFF;   //start with the display off
    DDRB  = 0xFF;   //PB is outputs
    PORTB - 0x00;   //driven low
    for(;;){
        seg_time();
        which_digit++;
        if(which_digit == 5){ which_digit = 0; }
        /* send 7 segment code to LED segments */
        PORTA = segment_data[which_digit];
        /* send PORTB the digit to display */
        PORTB |= (which_digit<<4);
        PORTB &= ((which_digit<<4) | 0x8F);
        /* wait a bit before switching to the next digit */
        _delay_ms(2);
    }
}
