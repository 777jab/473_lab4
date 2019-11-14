#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"

uint8_t  j;              //dummy variable
char     lcd_str_h[32];  //holds string to send to lcd  
/*******************************************************/
void spi_init(void){
 /* Run this code before attempting to write to the LCD.*/
 DDRF  |= 0x08;  //port F bit 3 is enable for LCD
 PORTF &= 0xF7;  //port F bit 3 is initially low

 DDRB  |= 0x07;  //Turn on SS, MOSI, SCLK
 PORTB |= _BV(PB1);  //port B initalization for SPI, SS_n off
//see: /$install_path/avr/include/avr/iom128.h for bit definitions   

 //Master mode, Clock=clk/4, Cycle half phase, Low polarity, MSB first
 SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
 SPSR=(1<<SPI2X);           //SPI at 2x speed (8 MHz)  
 }
/*******************************************************/

int main()
{
//initalize the SPI port then the LCD
spi_init();
lcd_init(); 
clear_display();
cursor_home();
strcpy(lcd_str_h, "hello                           ");

while(1){ 
  refresh_lcd(lcd_str_h);
  for(j=0;j<=10;j++){ _delay_ms(50);}  //delay 0.5 sec
  } //while
}//main
