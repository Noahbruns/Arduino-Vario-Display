#include <Adafruit_Sensor.h>
#include "Adafruit_BMP280_adapted.h"
#include "melody.h"
#include "U8glib.h"

#define debug false

//PINS
#define sensorPin A0
#define buzzer_pin 3
#define pace 450

//Vario Settings
#define RUNARRAY 180
#define MEASURE 60

int p = 0;
float A[RUNARRAY];
int last_measure;

//buzzer glob vars
int pitch = 0;
int pitch_before = 0;
bool state = true;
bool state_before = true;
int counter = 0;

U8GLIB_SH1106_128X64 u8g(12, 11, 8, 9, 10);
Adafruit_BMP280 bmp;

void setup() {
  #if debug
  Serial.begin(19200);
  #endif

  // declare the ledPin as an OUTPUT:
  pinMode(buzzer_pin, OUTPUT);
  pinMode(2, INPUT_PULLUP);  //internal pull-up

  //Set display Font
  u8g.setFont(u8g_font_fub17n);
  u8g.firstPage();
  u8g.nextPage();
  
  //connect to bmp
  if (!bmp.begin()) {
    tone(buzzer_pin, 880);
    while (1);
  }

  //Play Theme
  int size = sizeof(melody) / sizeof(int);
  for (int thisNote = 0; thisNote < size; thisNote++) {
    int noteDuration = 1400 / 12;

    tone(buzzer_pin, melody[thisNote] / 4, noteDuration);

    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
  }

  //Prefill Array
  last_measure = millis();
  
  for (int i = 0; i < RUNARRAY; i++) {
    A[i] = bmp.readAltitude();
  }

  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 2000;             // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_COMPA_vect) {
  //adapt timer
  if (counter > map(20 - pitch, 0, 20, 2, 12)) {
    state = !state;
    counter = 0;
  }

  //adapt tone
  if (state_before != state || pitch != pitch_before) {
    if (pitch == -1) {
      tone(buzzer_pin, 220);
    }
    else if (pitch == 0) {
      noTone(buzzer_pin);
    }
    else {
      if (state) {
        int t = 440.0 * pow(2, float(pitch + 4) / 12.0);
        tone(buzzer_pin, t);
      }
      else {
        noTone(buzzer_pin);
      }
    }
  }

  state_before = state;
  pitch_before = pitch;

  counter++;
}

void draw(float v, int a) {
  //Render Text
  {
    char s[8];
  
    //render Velocity
    dtostrf(v, 6, 2, s);
    u8g.drawStr( 128 - u8g.getStrWidth(s), 28, s);
  }

  //Render Altitude
  {
    char r[8];
    itoa(a, r, 10);
    u8g.drawStr( 128 - u8g.getStrWidth(r), 53, r);
  }

  //side Box
  int x = int(v * 10 + 21);
  if (x < 0){
    x = 0;
  }
  else if (x > 64) {
    x = 64;
  }

  //Lines
  u8g.drawBox(2, 64 - 60 ,3,1);
  u8g.drawBox(29, 64 - 60 ,3,1);
  
  u8g.drawBox(2, 64 - 50 ,3,1);
  u8g.drawBox(29, 64 - 50 ,3,1);
  
  u8g.drawBox(2, 64 - 40 ,3,1);
  u8g.drawBox(29, 64 - 40 ,3,1);

  u8g.drawBox(2, 64 - 30 ,3,1);
  u8g.drawBox(29, 64 - 30 ,3,1);
  
  u8g.drawBox(0, 64 - 21 ,5,3);
  u8g.drawBox(29, 64 - 21 ,5,3);
  
  u8g.drawBox(2, 64 - 10 ,3,1);
  u8g.drawBox(29, 64 - 10 ,3,1);

  u8g.drawBox(2, 64 - 1 ,3,1);
  u8g.drawBox(29, 64 - 1 ,3,1);
    
  u8g.drawBox(7,64 - x,20,x);
}

void loop() {
  A[p] = bmp.readAltitude(1013.25);
  //A[p] = 600.0 - (millis() / 1000.0);

  if (p % MEASURE == 0) {
    int t = millis() - last_measure;
    
    double a = 0;
    float sum = 0;

    int ip = p;

    //Least squares approximation
    for (int i = 0; i < RUNARRAY; i++) {
      ++ip %= RUNARRAY;
      a += A[ip] * (i - (RUNARRAY / 2) + 0.5) ;
      sum += A[ip];
    }

    //Adapting
    a /= 8.09975; //calculated by iphone * MEASURE * 1000 for adaptin to seconds
    a /= t;

    //Define Pitch
    if (a < -2.4)
      pitch = -1;
    else if (a > 0.5)
      pitch = min((int) a * 2, 20);
    else
      pitch = 0;

    //Display
    u8g.firstPage();
    do {  
      draw(a, (int) A[p]);
    } while( u8g.nextPage() );
    
    last_measure = millis();
  }

  #if debug
  Serial.print(A[p]);
  Serial.println();
  #endif

  //Inrcement
  ++p %= RUNARRAY;
}