/* Step/Dir Driver test firmware
 * Nickellick, October 2020
 * ver. 1
 */

#include <avr/io.h>
#include <avr/interrupt.h>


// Step/Dir driver connection PINS
#define STEP      3
#define DIR       2
#define DRIVER_EN 4

// IMPORTANT! DRIVER_EN is not in use right now

// Speed boarders (uint16_t)
// Magic numbers, recieved them in experiment
// For now it's just (UINT16_T - speed) wirttent in OCR1A register
#define MIN_SPEED     1
#define MAX_SPEED     65500
#define DEFAULT_SPEED 65500

// uint16_t max (needed for correct calculation of speed)
#define UINT16_T_MAX 65535

// Buttons pins
#define LEFT_BUTTON   8
#define RIGHT_BUTTON  9

// Button debounce timeout value (in mcs)
#define DEBOUNCE_DELAY_VALUE  300

// Serial Port baudrate
#define SERIAL_BAUDRATE 9600

// Global variable with current state of engine
typedef enum {
  STOP,
  LEFT,
  RIGHT
} states;

volatile states g_curr_state = LEFT;

// Raw string from serial
String raw_from_serial = "";

void setup() {
  io_init();
  timer_init();
  serial_init();
}

void io_init() {
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(LEFT_BUTTON, INPUT);
  pinMode(RIGHT_BUTTON, INPUT);
}

void timer_init() {
  cli();
  // Launch timer on CTC mode with OCR1A
  TCCR1A = 0;
  TCCR1B |= (1 << WGM12);
  // Set OCR1A value at DEFAULT_SPEED
  // Less OCR1A value => more speed
  uint16_t default_speed = UINT16_T_MAX - DEFAULT_SPEED;
  OCR1AH = default_speed >> 8;
  OCR1AL = (uint8_t)(default_speed & 0xFF);

  // Enable interrupt on CTC mode
  TIMSK1 |= (1 << OCIE1A);
  // No prescaler
  TCCR1B |= (1 << CS10);
  sei();
}

void serial_init() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.print("Set speed (" + String(MIN_SPEED) + " - " + String(MAX_SPEED) + "). Default is " + String(DEFAULT_SPEED) + "\n");
}

ISR(TIMER1_COMPA_vect)
{
  switch(g_curr_state) {
    case STOP:
      digitalWrite(STEP, STOP);
      break;
    case LEFT:
      digitalWrite(DIR, LOW);
      digitalWrite(STEP, !digitalRead(STEP));
      break;
    case RIGHT:
      digitalWrite(DIR, HIGH);
      digitalWrite(STEP, !digitalRead(STEP));
      break;
  }
}

void set_speed(uint16_t value) {
  cli();
  value = UINT16_T_MAX - value;
  OCR1AH = value >> 8;
  OCR1AL = (uint8_t)(value & 0xFF);
  sei();
}



void loop(){
  // Handle serial
  if (Serial.available()) {
    while (Serial.available()) {
      raw_from_serial = Serial.readString();
    }
    uint16_t data = atoi(raw_from_serial.c_str());
    data = data > MAX_SPEED ? MAX_SPEED : data;
    data = data < MIN_SPEED ? MIN_SPEED : data;
    set_speed(data);
    Serial.print("OK " + String(data) + "\n");
    String raw_from_serial = "";
  }

  // Handle button state
  if(!digitalRead(LEFT_BUTTON)) {
    delayMicroseconds(DEBOUNCE_DELAY_VALUE);
    if(!digitalRead(LEFT_BUTTON)) {
      g_curr_state = LEFT;
    }
  } else if (!digitalRead(RIGHT_BUTTON)) {
    delayMicroseconds(DEBOUNCE_DELAY_VALUE);
    if(!digitalRead(RIGHT_BUTTON)) {
      g_curr_state = RIGHT;
    }
  } else {
    g_curr_state = STOP;
  }
}
