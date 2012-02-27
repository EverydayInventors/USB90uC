/*

    USB90µC Minimial Development Board timer test program
    Programmed by David Dahl, EverydayInventors
    Copyright 2010, 2011 Free Software Foundation

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
*/

/*

  This program does a simple software PWM of the red and green LED briteness
  using timer 0 overflow and an ISR.
  
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include <util/delay.h>

#include <stdint.h>

#define RED_LED   4   /* red LED is on pin 4 of port C */
#define GREEN_LED 5   /* green LED is on pin 5 of port C */

#define CLOCKERROR  0   /* error flag bit for problem enabling external xtal */
#define PLLERROR    1   /* error flag bit for problem configuring USB PLL */

#define LED_DELAY 6   /* delay in ms between updating LED values */

/*[ Global Variables ]-------------------------------------------------------*/

/* intensities of red and green LEDs (off {0..255} fully on) read by ISR */
volatile uint8_t red_int    = 0;
volatile uint8_t green_int  = 0;

/*[ Constants ]--------------------------------------------------------------*/

/* table of sine values normalized to range {0..255} in set {0..255} */
const uint8_t sintab[256] PROGMEM =
  { 0x80, 0x83, 0x86, 0x89, 0x8C, 0x8F, 0x92, 0x95,
    0x98, 0x9C, 0x9F, 0xA2, 0xA5, 0xA8, 0xAB, 0xAE,
    0xB0, 0xB3, 0xB6, 0xB9, 0xBC, 0xBF, 0xC1, 0xC4,
    0xC7, 0xC9, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD8,
    0xDA, 0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8,
    0xEA, 0xEC, 0xED, 0xEF, 0xF0, 0xF2, 0xF3, 0xF5,
    0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFC,
    0xFD, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE,
    0xFD, 0xFC, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8, 0xF7,
    0xF6, 0xF5, 0xF3, 0xF2, 0xF0, 0xEF, 0xED, 0xEC,
    0xEA, 0xE8, 0xE6, 0xE4, 0xE2, 0xE0, 0xDE, 0xDC,
    0xDA, 0xD8, 0xD5, 0xD3, 0xD1, 0xCE, 0xCC, 0xC9,
    0xC7, 0xC4, 0xC1, 0xBF, 0xBC, 0xB9, 0xB6, 0xB3,
    0xB0, 0xAE, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9C,
    0x98, 0x95, 0x92, 0x8F, 0x8C, 0x89, 0x86, 0x83,
    0x7F, 0x7C, 0x79, 0x76, 0x73, 0x70, 0x6D, 0x6A,
    0x67, 0x63, 0x60, 0x5D, 0x5A, 0x57, 0x54, 0x51,
    0x4F, 0x4C, 0x49, 0x46, 0x43, 0x40, 0x3E, 0x3B,
    0x38, 0x36, 0x33, 0x31, 0x2E, 0x2C, 0x2A, 0x27,
    0x25, 0x23, 0x21, 0x1F, 0x1D, 0x1B, 0x19, 0x17,
    0x15, 0x13, 0x12, 0x10, 0x0F, 0x0D, 0x0C, 0x0A,
    0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x03,
    0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01,
    0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0C, 0x0D, 0x0F, 0x10, 0x12, 0x13,
    0x15, 0x17, 0x19, 0x1B, 0x1D, 0x1F, 0x21, 0x23,
    0x25, 0x27, 0x2A, 0x2C, 0x2E, 0x31, 0x33, 0x36,
    0x38, 0x3B, 0x3E, 0x40, 0x43, 0x46, 0x49, 0x4C,
    0x4F, 0x51, 0x54, 0x57, 0x5A, 0x5D, 0x60, 0x63,
    0x67, 0x6A, 0x6D, 0x70, 0x73, 0x76, 0x79, 0x7C };
    
/*[ prototypes ]-------------------------------------------------------------*/
int init_clock( void );
int init_status_leds( void );

/*[ Main Program ]===========================================================*/
int main()
{
  int errorcode; 
  uint8_t gi, ri, i;
  
  /* disable watchdog */
  MCUSR &= ~_BV(WDRF);
  wdt_disable();
  
  /* initialize red and green LEDs */
  init_status_leds();
  
  /*-- set processor speed to 16Mhz using external crystal --*/
  if( (errorcode = init_clock()) != 0 )
  {
    /* if here, there was an error so light red LED. */
    PORTC |= _BV(RED_LED); /* turn on red LED */
    
    if( errorcode & _BV(CLOCKERROR) )
      while(1); /* steady red LED for external clock error */
    else
      while(1){ /* quickly flashing red LED for PLL error */
        _delay_ms( 100 );
        PORTC &= ~_BV(RED_LED); /* turn off red LED */
        _delay_ms( 100 );
        PORTC |= _BV(RED_LED); /* turn on red LED */
      }
  }
  
  /*-- set timer 0 normal mode --*/
  TCCR0A &= ~( _BV(WGM01) | _BV(WGM00) );
  TCCR0B &= ~_BV(WGM02);
  
  /* no prescale */
  TCCR0B = (TCCR0B & ~(_BV(CS01) | _BV(CS02))) | _BV(CS00); /* p104 */
  
  /* enable timer 0 overflow interrupt */
  TIMSK0 |= _BV(TOIE0);
  
  /* enable interrupts */
  sei();
  
  /* set red and green indicies to return 0 values */
  gi = 192; /* sintab[192] == 0 */ 
  ri = 192;
  
  /* ramp up green until it is 180 degrees out of phase from red */
  for(i = 0; i<128; i++) {
    _delay_ms( LED_DELAY );
    
    /* disable interrupts before touching any ISR variables */
    cli();
    
    /* change green LED intensity based on sine table */
    green_int = pgm_read_byte( &sintab[ gi++ ] );
    
    /* enable interrupts */
    sei();
  }
  
  /* loop forever while changing red and green LED values */
  while(1){
    _delay_ms( LED_DELAY );

    /* disable interrupts before touching any ISR variables */
    cli();
    
    /* change red and green LED intensities based on sine table */
    green_int = pgm_read_byte( &sintab[ gi++ ] );
    red_int = pgm_read_byte( &sintab[ ri++ ] );
    
    /* enable interrupts */
    sei();
  };
}

/*[ Set clock to 16MHz external xtal ]---------------------------------------*/
int init_clock( void )
{
  int timeout;
  int errorcode;
  
  /********************************************************/
  /*                                                      */
  /* AT90USB162 factory-defaults to 8MHz internal clock.  */
  /*  [AT90USB162 data sheet, p29, p32]                   */
  /*                                                      */
  /* Following code sets clock to external 16MHz crystal. */
  /*                                                      */
  /********************************************************/

  /* assume no errors */
  errorcode = 0;
  
  /* set clock prescaler div factor to 1 */
  CLKPR = 0b10000000;   /* enable prescaler change */
  CLKPR = 0b00000000;   /* change prescale div factor to 1 */
  
  /* start external clock */
  CKSEL0 |= _BV(EXTE);
  
  /* wait for external clock to become ready */
  timeout = (500 / 10) / 2;  /* wait a maximum of 0.5 seconds total.          */
                             /*  (the /2 is because the 8mhz clock is active) */
  while( ( (CKSTA & _BV(EXTON)) == 0 ) && ( timeout > 0 ) )
  {
    _delay_ms( 10 / 2 ); /* wait aprox. 10ms before trying agin. */
    timeout--;
  }
  
  if( timeout <= 0 )
    errorcode |= _BV(CLOCKERROR);
  else
  {  
    /* select external clock as active clock */
    CKSEL0 |= _BV(CLKS);
  
    /* disable PLL */
    PLLCSR &= ~_BV(PLLE);
  
    /* set PLL prescale to divide by 2 for 16Mhz xtal to generate 8MHz clock */
    PLLCSR = ( PLLCSR & ~( _BV(PLLP1)|_BV(PLLP2) ) ) | _BV(PLLP0) | _BV(PLLE);
  
    /* disable RC clock */
    CKSEL0 &= ~_BV(RCE);
  
    /* wait for PLL to become ready */
    timeout = 500 / 5; /* wait a maximum of 1/2 a second total. */
    while( ( (PLLCSR & _BV(PLOCK)) == 0 ) && ( timeout > 0 ) )
    {
      _delay_ms( 5 ); /* wait 5ms before trying again. */
      timeout--;
    }
  
    if( timeout <= 0 )
      errorcode |= _BV(PLLERROR);
    
    /* unfreeze USB clock */
    USBCON &= ~_BV(FRZCLK);
  }
  
  return errorcode;
}

/*[ Initialize the red and greed LEDs ]--------------------------------------*/
int init_status_leds( void )
{
  /* Status LEDs are on port C, set pins as outputs */
  DDRC = DDRC | _BV(RED_LED) | _BV(GREEN_LED);

  /* make sure both LEDs are turned off */
  PORTC &= ~( _BV(RED_LED) | _BV(GREEN_LED) );
  
  return 1;
}

/*[ ISR for Timer ]==========================================================*/
ISR( TIMER0_OVF_vect )
{
  static uint8_t red_count    = 0;
  static uint8_t green_count  = 0;
  
  /* software PWM of red and green LEDs */
    
  if( red_count++ < red_int )
    PORTC |= _BV(RED_LED);
  else
    PORTC &= ~_BV(RED_LED);
  
  if( green_count++ < green_int )
    PORTC |= _BV(GREEN_LED);
  else
    PORTC &= ~_BV(GREEN_LED);
}
