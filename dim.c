#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

int16_t adc_result;     //holds adc result 


uint8_t main() {
    //Initalize ADC and its ports
    DDRF  &= ~(_BV(DDF0)); //make port F bit 0 is ADC input  
    PORTF &= ~(_BV(PF0));  //port F bit 0 pullups must be off
    ADMUX = 0b01000000;//single-ended, input PORTF0, R-adjusted, 10 bits
    ADCSRA = 0b10000111;//ADC enabled, don't start yet, single shot mode 
    
    // light up the bar graph
    DDRB  = ~(1<<3); //MISO (PB3) is an input, everything else is output
    PORTB = (1<<3);  //set pullup resistor on MISO, drive all outputs low
    DDRD  = (1<<2);  //bit 2 is an output
    PORTD = 0;       //all outputs driven low
    DDRE  = (3<<6);  //bits 6 and 7 outputs
    PORTE = 0;       //all outputs driven low 
    SPCR  = 0b01010000; //master mode, clk low on idle, 
                        //leading edge sample
    SPSR  = 0b00000001; //choose double speed
    SPDR  = 0xFF; 
    while (bit_is_clear(SPSR, SPIF)){} //spin till SPI data is sent

    // fast pwm, no prescale, non-inverting
    TCCR2 |= (1<<WGM20) | (1<<WGM21) | (1<<CS20) | (1<<COM21); 

    //% duty cycle, LED is a bit dimmer
    OCR2 = 0x00; 
    
    for(;;){
        ADCSRA |= (1<<ADSC);//poke ADSC and start conversion
        while(bit_is_clear(ADCSRA,ADIF));//wait for conversion to finish
        ADCSRA |= (1<<ADIF);//its done, clear flag by writing a one 
        adc_result = ADC;//read the ADC output as 16 bits
        adc_result -= 700;//range is 700 (bright) to 960 (dark)
        if(adc_result < 0) {adc_result = 0;}
        if(adc_result > 0xF0) {adc_result = 0xF0;}
        OCR2 = adc_result;
    }
    return 0;
}
