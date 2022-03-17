#include "pins_arduino.h"

#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        6 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 16 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


byte buf [3];
volatile byte pos;
volatile boolean process_it;

void set_color(int r, int g, int b)
{
    for(int i=0; i<NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));    
  }
  pixels.show(); 
}


void setup (void)
{

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  // turn on interrupts
  SPCR |= _BV(SPIE);
  
  pos = 0;
  process_it = false;

  pixels.begin();
  pixels.clear();
  set_color(100,100,100);
  delay(100);
  pixels.clear();
  pixels.show();
  delay(100);
  set_color(100,100,100);
  delay(100);
  pixels.clear();
  pixels.show();
 
}  // end of setup


// SPI interrupt routine
ISR (SPI_STC_vect)
{
byte c = SPDR;
  
  // add to buffer if room

    buf [pos++] = c;
    
    // example: newline means time to process buffer
    if(pos == 3){
      process_it = true;
      pos = 0;
    }
}

// main loop - wait for flag set in interrupt routine
void loop (void)
{
  if (process_it)
    {  
    set_color(buf[0],buf[1],buf[2]);
    process_it = false;
    }  // end of flag set
    
}  // end of loop
