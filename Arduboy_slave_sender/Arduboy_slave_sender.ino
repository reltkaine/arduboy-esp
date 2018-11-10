// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this
#include <avr/sleep.h>

#define RGB_ON LOW   /**< For digitially setting an RGB LED on using digitalWriteRGB() */
#define RGB_OFF HIGH /**< For digitially setting an RGB LED off using digitalWriteRGB() */

#define RED_LED 5   /**< The pin number for the red color in the RGB LED. */
#define GREEN_LED 7 /**< The pin number for the greem color in the RGB LED. */
#define BLUE_LED 6   /**< The pin number for the blue color in the RGB LED. */


  // For frame funcions
  uint8_t eachFrameMillis;
  uint8_t thisFrameStart;
  bool justRendered;
uint8_t lastFrameDurationMs;

#define RED_LED_PORT PORTD
#define RED_LED_BIT PORTD5

#define GREEN_LED_PORT PORTD
#define GREEN_LED_BIT PORTD7

#define BLUE_LED_PORT PORTD
#define BLUE_LED_BIT PORTD6



#define PIN_LEFT_BUTTON 15
#define LEFT_BUTTON_PORT PORTC
#define LEFT_BUTTON_PORTIN PINC
#define LEFT_BUTTON_DDR DDRC
#define LEFT_BUTTON_BIT PORTC1

#define PIN_RIGHT_BUTTON 3
#define RIGHT_BUTTON_PORT PORTD
#define RIGHT_BUTTON_PORTIN PIND
#define RIGHT_BUTTON_DDR DDRD
#define RIGHT_BUTTON_BIT PORTD3

#define PIN_UP_BUTTON 17
#define UP_BUTTON_PORT PORTC
#define UP_BUTTON_PORTIN PINC
#define UP_BUTTON_DDR DDRC
#define UP_BUTTON_BIT PORTC3

#define PIN_DOWN_BUTTON 2
#define DOWN_BUTTON_PORT PORTD
#define DOWN_BUTTON_PORTIN PIND
#define DOWN_BUTTON_DDR DDRD
#define DOWN_BUTTON_BIT PORTD2

#define PIN_A_BUTTON 4
#define A_BUTTON_PORT PORTD
#define A_BUTTON_PORTIN PIND
#define A_BUTTON_DDR DDRD
#define A_BUTTON_BIT PORTD4

#define PIN_B_BUTTON 16
#define B_BUTTON_PORT PORTC
#define B_BUTTON_PORTIN PINC
#define B_BUTTON_DDR DDRC
#define B_BUTTON_BIT PORTC2

#define PIN_X_BUTTON 12
#define X_BUTTON_PORT PORTB
#define X_BUTTON_PORTIN PINB
#define X_BUTTON_DDR DDRB
#define X_BUTTON_BIT PORTB4

#define PIN_Y_BUTTON 10
#define Y_BUTTON_PORT PORTB
#define Y_BUTTON_PORTIN PINB
#define Y_BUTTON_DDR DDRB
#define Y_BUTTON_BIT PORTB2


#define LEFT_BUTTON _BV(5)  /**< The Left button value for functions requiring a bitmask */
#define RIGHT_BUTTON _BV(6) /**< The Right button value for functions requiring a bitmask */
#define UP_BUTTON _BV(7)    /**< The Up button value for functions requiring a bitmask */
#define DOWN_BUTTON _BV(4)  /**< The Down button value for functions requiring a bitmask */
#define A_BUTTON _BV(2)     /**< The A button value for functions requiring a bitmask */
#define B_BUTTON _BV(3)     /**< The B button value for functions requiring a bitmask */
#define X_BUTTON _BV(8)     /**< The X button value for functions requiring a bitmask */
#define Y_BUTTON _BV(1)     /**< The Y button value for functions requiring a bitmask */


 uint8_t buttons=0;


#define PIN_SPEAKER_1 9  /**< The pin number of the first lead of the speaker */
#define PIN_SPEAKER_2 11 /**< The pin number of the second lead of the speaker */

#define SPEAKER_1_PORT PORTB
#define SPEAKER_1_DDR DDRB
#define SPEAKER_1_BIT PORTB1

#define SPEAKER_2_PORT PORTB
#define SPEAKER_2_DDR DDRB
#define SPEAKER_2_BIT PORTB3
//BEEP TONE
uint8_t duration1 = 0;
uint8_t duration2 = 0;
uint16_t frameCount;
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  PORTB |= _BV(X_BUTTON_BIT) | _BV(Y_BUTTON_BIT);
  DDRB  |= _BV(X_BUTTON_BIT) | _BV(Y_BUTTON_BIT);
  
  PORTC |= _BV(LEFT_BUTTON_BIT) | _BV(UP_BUTTON_BIT) |
           _BV(B_BUTTON_BIT);
  DDRC  |= _BV(LEFT_BUTTON_BIT) | _BV(UP_BUTTON_BIT) |
           _BV(B_BUTTON_BIT);
  // Port D INPUT_PULLUP
  PORTD |= _BV(RIGHT_BUTTON_BIT) |
           _BV(DOWN_BUTTON_BIT) | _BV(A_BUTTON_BIT);
  DDRD  |= _BV(RIGHT_BUTTON_BIT) |
           _BV(DOWN_BUTTON_BIT) | _BV(A_BUTTON_BIT) |
           _BV(GREEN_LED_BIT)   | _BV(BLUE_LED_BIT) | _BV(RED_LED_BIT);
           

  // switch off LEDs by default
  PORTD |= _BV(GREEN_LED_BIT)   | _BV(BLUE_LED_BIT) | _BV(RED_LED_BIT);


///BeepPin1 begin
  TCCR1A = 0;
  TCCR1B = (bit(WGM12) | bit(CS11)); // CTC mode. Divide by 8 clock prescale

//==BEEP TONE
///beep2 begin
  TCCR2A = 0; // normal mode. Disable PWM
  TCCR2B = bit(CS22) | bit(CS20); // divide by 128 clock prescale
  OCR2A = 0; //  "
  
  Wire.begin(8);                // join i2c bus with address #8
//  Serial.begin(9600);
//  Wire.setClockStretchLimit(40000);
//Wire.beginTransmission(8);
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent1); // register event
  setFrameDuration(16);
  frameCount = 0;
justRendered = false;
}

void tone1(uint16_t count, uint8_t dur)
{
  duration1 = dur;
  TCCR1A = bit(COM1A0); // set toggle on compare mode (which connects the pin)
  OCR1A = count; // load the count (16 bits), which determines the frequency
}
void timer1()
{
  if (duration1 && (--duration1 == 0)) {
    TCCR1A = 0; // set normal mode (which disconnects the pin)
  }
}

void noTone1()
{
  duration1 = 0;
  TCCR1A = 0; // set normal mode (which disconnects the pin)
}

void tone2(uint16_t count, uint8_t dur)
{
  duration2 = dur;
  TCCR2A = bit(WGM21) | bit(COM2A0); // CTC mode, toggle on compare mode (which connects the pin)
  OCR2A = lowByte(count); //  which determines the frequency
}
void timer2()
{
  if (duration2 && (--duration2 == 0)) {
    TCCR2A = 0; // set normal mode (which disconnects the pin)
  }
}

void noTone2()
{
  duration2 = 0;
  TCCR2A = 0; // set normal mode (which disconnects the pin)
}

void loop() {
  if (!(nextFrame())) return;
  timer2();
  timer1();

}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent1() {
  int c[4];
 // int i=0;
 // while (1 < Wire.available()) { // loop through all but the last
    c[0] = Wire.read(); // receive byte as a character
    c[1] = Wire.read(); // receive byte as a character
    c[2] = Wire.read(); // receive byte as a character
    c[3] = Wire.read(); // receive byte as a character
      Serial.println(c[0]);
      Serial.println(c[1]);
      Serial.println(c[2]);
      Serial.println(c[3]);
      Serial.println(F_CPU);
 // }
  switch (c[0]) {
    case 1:    // beep1 tone
      tone(9, c[1], c[2]);
      break;
    case 2:    // beep1 timer
      timer1();
      break;
    case 3:    // beep1 notone
      noTone1();
      break;      
    case 4:    // beep2 tone
      tone2(c[1], c[2]);
      break;
    case 5:    // beep2 timer
      timer2();
      break;
    case 6:
      noTone2();
      break; 
    case 7:
      freeRGBled();
      break;        
    case 8:
      setRGBled(c[1], c[2]);
      break;        
     case 9:
      setRGBled(c[1], c[2], c[3]);
      break;        
     case 0:
      break;        
       
  }  
}
// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  buttons=0;
  if (bitRead(UP_BUTTON_PORTIN, UP_BUTTON_BIT) == 0) { buttons |= UP_BUTTON;}
  if (bitRead(DOWN_BUTTON_PORTIN, DOWN_BUTTON_BIT) == 0) { buttons |= DOWN_BUTTON;}
  if (bitRead(LEFT_BUTTON_PORTIN, LEFT_BUTTON_BIT) == 0) { buttons |= LEFT_BUTTON;}
  if (bitRead(RIGHT_BUTTON_PORTIN, RIGHT_BUTTON_BIT) == 0) { buttons |= RIGHT_BUTTON;}
  if (bitRead(A_BUTTON_PORTIN, A_BUTTON_BIT) == 0) { buttons |= A_BUTTON;}
  if (bitRead(B_BUTTON_PORTIN, B_BUTTON_BIT) == 0) { buttons |= B_BUTTON;}
  if (bitRead(X_BUTTON_PORTIN, X_BUTTON_BIT) == 0) { buttons |= X_BUTTON;}
  if (bitRead(Y_BUTTON_PORTIN, Y_BUTTON_BIT) == 0) { buttons |= Y_BUTTON;}
  
//  Serial.println("PRESSED");
//  Serial.println(buttons);
  //Wire.beginTransmission(8);
  Wire.write(buttons);
//  Wire.endTransmission();

}
void  freeRGBled()
{
  TCCR0A = _BV(WGM01) | _BV(WGM00);
}
void  setRGBled(uint8_t color, uint8_t val)
{
  if (color == RED_LED) {
    //    val = val?RGB_ON:RGB_OFF;
    //    bitWrite(RED_LED_PORT, RED_LED_BIT, val);
    OCR0B = 255 - val;
  } else if (color == GREEN_LED) {
    val = val?RGB_ON:RGB_OFF;
    bitWrite(GREEN_LED_PORT, GREEN_LED_BIT, val);
  } else if (color == BLUE_LED) {
    OCR0A = 255 - val;
  }
}

void  setRGBled(uint8_t red, uint8_t green, uint8_t blue)
{
  //  bitWrite(RED_LED_PORT, RED_LED_BIT, red ? RGB_ON : RGB_OFF);
  bitWrite(GREEN_LED_PORT, GREEN_LED_BIT, green ? RGB_ON : RGB_OFF);
  // timer 0: Fast PWM, OC0A clear on compare / set at top
  // We must stay in Fast PWM mode because timer 0 is used for system timing.
  // We can't use "inverted" mode because it won't allow full shut off.
  TCCR0A = _BV(COM0B1) | _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
  OCR0A = 255 - blue;
  OCR0B = 255 - red;
}
void setFrameRate(uint8_t rate)
{
  eachFrameMillis = 1000 / rate;
}

void setFrameDuration(uint8_t duration)
{
  eachFrameMillis = duration;
}

bool everyXFrames(uint8_t frames)
{
  return frameCount % frames == 0;
}

bool nextFrame()
{
  uint8_t now = (uint8_t) millis();
  uint8_t frameDurationMs = now - thisFrameStart;

  if (justRendered) {
    lastFrameDurationMs = frameDurationMs;
    justRendered = false;
    return false;
  }
  else if (frameDurationMs < eachFrameMillis) {
    // Only idle if at least a full millisecond remains, since idle() may
    // sleep the processor until the next millisecond timer interrupt.
    if (++frameDurationMs < eachFrameMillis) {
      idle();
    }

    return false;
  }

  // pre-render
  justRendered = true;
  thisFrameStart = now;
  frameCount++;

  return true;
}
void idle()
{
  SMCR = _BV(SE); // select idle mode and enable sleeping
  sleep_cpu();
  SMCR = 0; // disable sleeping
}
