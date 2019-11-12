#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

ISR(TIMER1_COMPA_vect){
    static uint8_t beep = 0;
    static uint16_t count = 0;
    if(beep){ PORTE ^= (1<<5); }
    count++;
    if(count >= 2000) { count = 0; beep ^= 1; } //toggle beep every
                                                //half second
}

uint8_t main(){
    /* set up ports */
    DDRE  |= (1<<5);
    DDRC  |= (1<<0);
    PORTC |= (1<<0);
    /* set up timer 1 */
    TCCR1A = 0;     
    TCCR1B = (1<<WGM12) | (1<<CS12);//CTC mode, 256 prescale
    TIMSK = (1<<OCIE1A);//interrupt enable on output comapre 1A
    OCR1A = 0x000F; //interrupt 4000 times per second    
    TCNT1 = 0;//initialize the TC1 counter to 0
    sei(); //enable global interrupts

    while(1){}
    return 0;
}
