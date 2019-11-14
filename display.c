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
volatile uint8_t mode = 0xAA;//stores which mode the counter is in
static int16_t sum = 1019;//stores the count to be displayed
static uint8_t which_digit = 0;//tracks which digit is being displayed

/* function prototypes */
uint8_t debounce();
void segsum(int16_t sum);
int8_t read_encoder();
uint8_t read_buttons();
void init();
void update_globals(uint8_t buttons, int8_t encoders);

/* function definitions */

/***********************************************************************
*                    timer 0 interrupt service routine
*
* This ISR pulls values from the buttons and encoders.
***********************************************************************/
ISR(TIMER0_COMP_vect){
    static uint8_t buttons = 0; //stores input from buttons
    static int8_t encoders = 0; //stores input from encoders
    /* read buttons */
    buttons  = read_buttons();
    /* read encoders */
    encoders = read_encoder();
    update_globals(buttons,encoders);
}

void update_globals(uint8_t buttons, int8_t encoders){
   if(buttons){ sum++; }
   if(encoders){ sum--; } 
}

/***********************************************************************
*                    timer 0 interrupt service routine
*
* This ISR pulls values from the buttons and encoders and uses those
* values to update the mode and sum global vars. 
***********************************************************************/
uint8_t read_buttons(){
    static uint8_t temp = 0;
    static uint8_t buttons = 0;
    temp = PORTA;//store the displayed digit for later
    //make PORTA an input port with pullups
    DDRA = 0x00; //input
    PORTA = 0xFF; //pullups
    //enable tristate buffer for pushbutton switches
    PORTB |= 0x70;
    //wait for those changes to take place
    asm("nop");
    asm("nop");
    buttons = debounce();//check buttons with debounce, store in temp
    /* put ports A and B back how they were */
    PORTB &= ((which_digit<<4) | 0x8F);//disable tristate buffer, put
                                       //correct digit on the display
    DDRA = 0xFF;  //set PA to output mode
    PORTA = temp; //restore the displayed digit
    return buttons;
}
/***********************************************************************
*                               init 
*
* This function performs all the setup required to run the program. It
* sets port modes and innitial values, enables interrupts, sets up
* timer/counter 0, and sets up serial communication.
***********************************************************************/
void init(){
    /* set up ports */
    DDRA  = 0xFF;    //start port A out as all outputs
    PORTA = 0xFF;    //start with the display off
    DDRB  = ~(1<<3); //MISO (PB3) is an input, everything else is output
    PORTB = (1<<3);  //set pullup resistor on MISO, drive all outputs low
    DDRD  = (1<<2);  //bit 2 is an output
    PORTD = 0;       //all outputs driven low
    DDRE  = (3<<6);  //bits 6 and 7 outputs
    PORTE = 0;       //all outputs driven low

    /* set up timer and interrupt */
    ASSR  |= (1<<AS0); //run off external 32khz osc (TOSC)
    //enable interrupts for output compare match 0
    TIMSK |= (1<<OCIE0);
    TCCR0 |=  (1<<WGM01) | (1<<CS00);  //CTC mode, no prescale
    OCR0  |=  0x041;   //compare at 66 (so the timer counts for 2 ms)
    
    /* set up serial communication */
    SPCR  = 0b01010000; //master mode, clk low on idle, 
                        //leading edge sample
    SPSR  = 0b00000001; //choose double speed
}
/***********************************************************************
*                           debounce
* Debounces 8 input buttons on port A simultaneously. Each bit in the
* return value corresponds to a button. Bit 0 -> far right button. Bit 7
* -> far left button. For a given button press, that button's bit will 
* cycle high only once. Debounce time is the external loop delay * 12. 
***********************************************************************/
uint8_t debounce() {
    //holds present state
    static uint16_t b_state[8] = { 0,0,0,0,0,0,0,0 };     
    uint8_t ret = 0;//bit 0 is button 0, bit 7 is button 7 
    for(uint8_t i=0; i<8; i++){ //for each button
        //advance the state if the buttin is depressed
        b_state[i] = (b_state[i] << 1) | (!bit_is_clear(PINA,i)) | 0xE000;
        // if state == 12, set the bit representing that button in the
        // return value
        if (b_state[i] == 0xF000) ret |= (1<<i);
    }
    return ret;
}//debounce

/************************************************************************
*                           segsum
* Takes a 16-bit binary input value and places the appropriate equivalent
* 4 digit BCD segment code in the array segment_data for display.
* Array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
************************************************************************/
void segsum(int16_t sum) {
    if(sum){ //if sum != 0
        //determine how many digits there are
        uint8_t digits = 0;
        if(sum / 1000) digits = 4;
        else if(sum / 100) digits = 3;
        else if(sum / 10) digits = 2;
        else if(sum) digits = 1;
        else digits = 0;
        //break up decimal sum into 4 digit-segments
        segment_data[0] = dec_to_7seg[sum % 10];
        segment_data[1] = dec_to_7seg[(sum % 100) / 10];
        segment_data[2] = dec_to_7seg[(sum % 1000) / 100];
        segment_data[3] = dec_to_7seg[sum / 1000];
        //blank out leading zero digits
        for(uint8_t i=0; i<(4 - digits);i++){
            segment_data[3 - i] = dec_to_7seg[10];
        }
        //now move data to right place for misplaced colon position
        segment_data[4] = segment_data[3];
        segment_data[3] = segment_data[2];
        segment_data[2] = dec_to_7seg[10];//clear the colon
    }
    else{ //display 0
        segment_data[0] = dec_to_7seg[0];
        segment_data[1] = dec_to_7seg[10]; //clear
        segment_data[3] = dec_to_7seg[10]; //the leading
        segment_data[4] = dec_to_7seg[10]; //digits
    } 
}//segsum

/**********************************************************************
 * read_encoder()
 * This function is based on the work of John Main which he published
 * on his site best-microcontroller-projects.com in the article:
 *
 * Rotary Encoder : How to use the Keys KY-040 Encoder on the Arduino
 *
 * It stores each new reading from the encoder in an 8-bit state
 * register, shifting out old readings as it goes. It looks for
 * 2-reading patterns that it recognizes and when it finds them,
 * it returns a number indicating ccw or cw rotation.
 *********************************************************************/
int8_t read_encoder() {
    /* about rot_enc_table: 
     * 0 means invalid input, 1 means valid input. You get the index
     * from the input by concatinating the previous and current states
     * of both switches in the encoder */
    static uint8_t rot_enc_table[16] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};
    static uint8_t input = 0; // stores the value read form SPDR
    static uint8_t state[2] = {0,0};//stores state of both encoders

    /* scan the register */
    PORTE &= ~(1<<6);//toggle shift/load low
    PORTE |= (1<<6);//toggle shift/load high
    SPDR = 0xFF;//start transmitting a dummy byte to get clock going
    while(bit_is_clear(SPSR, SPIF)){}//wait for the done receiving flag
    input = SPDR;//read value of SPDR

    /* update the state of encoder 1 */
    state[0] = (state[0] << 2);//make room for the latest data
    if (input & (1<<0)) state[0] |= 0x01;//if switch A open, set bit 0
    if (input & (1<<1)) state[0] |= 0x02;//if switch B open, set bit 1
    state[0] = (state[0] & 0x0F); //clear the 2 oldest states

    /* update the state of encoder 2 */
    state[1] = (state[1] << 2);//make room for the latest data
    if (input & (1<<2)) state[1] |= 0x01;//if switch A open, set bit 0
    if (input & (1<<3)) state[1] |= 0x02;//if switch B open, set bit 1
    state[1] = (state[1] & 0x0F); //clear the 2 oldest states

    /* check state 1 for ccw and cw rotation */
    if  (rot_enc_table[state[0]]) { //if  last 2 states indicate movement
        if (state[0] == 0x0B) return 1; //check for end of a ccw click
        if (state[0] == 0x07) return 2; //check for end of a cw click
    }

    /* check state 2 for ccw and cw rotation */
    if  (rot_enc_table[state[1]]) { //if last 2 states indicate movement
        if (state[1] == 0x0B) return 3; //check for end of a ccw click
        if (state[1] == 0x07) return 4; //check for end of a cw click
    }
    return 0;//report no turning
}//read_encoder

int main(){
    init(); //innitialize ports, timer/counter
    sei();  //enable global interrupts
    for(;;){
        /* bound the count to 0 - 1023 */
        /* compute segment values from sum */
        segsum(sum); //stores result in segment_data array
        /* update the digit to display using state machine */
        switch(which_digit){
            case 0:
                which_digit = 1;
                break;
            case 1:
                which_digit = 3;
                break;
            case 3:
                which_digit = 4;
                break;
            case 4:
                which_digit = 0;
                break;
        }
        /* send 7 segment code to LED segments */
        PORTA = segment_data[which_digit];
        /* send PORTB the digit to display */
        PORTB |= (which_digit<<4);
        PORTB &= ((which_digit<<4) | 0x8F);
        /* send current mode to the bar graph */
        cli();
        SPDR = mode; // send mode to the bar graph
        while (bit_is_clear(SPSR, SPIF)){} //spin till SPI data is sent
        PORTD |= (1<<2);  //send rising edge to regclk on HC595
        PORTD &= ~(1<<2);  //send falling edge to regclk on HC595;
        sei();
        /* wait a bit before switching to the next digit */
        _delay_ms(2);
    }
}
