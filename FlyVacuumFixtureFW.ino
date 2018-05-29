/* Revision History:
 *  
 *  
 *  
 */

#include "Arduino.h"
#include "pins.h"

#define BUTTON_DEBOUNCE_MS   200

volatile int switchA=LOW, switchB=LOW, dispenseButton=0;
volatile bool penTipTrigger = false;

enum FWState {
  FWSTATE_UNINITIALIZED = 0,
  FWSTATE_INITIALIZED = 1,
  FWSTATE_WAITING_FOR_DISPENSE = 2,
  FWSTATE_DISPENSED = 3,
  FWSTATE_WAITING_FOR_CAPTURE = 4,
  FWSTATE_CAPTURED = 5,
  FWSTATE_TEST_MODE = 6,
};



// ISR to debounce UI button A
void onUIButtonA() {
  static unsigned long last_interrupt = 0;
  unsigned long this_interrupt = millis();
  // If interrupts come faster than BUTTON_DEBOUNCE_MS, assume it's a bounce and ignore
  if (this_interrupt - last_interrupt > BUTTON_DEBOUNCE_MS)
  {
    switchA = HIGH;
    if ( digitalRead(BUTTONA) == LOW ) { 
      switchA = HIGH; 
    }
  }
  last_interrupt = this_interrupt;
}

// ISR to debounce UI button B
void onUIButtonB() {
  static unsigned long last_interrupt = 0;
  unsigned long this_interrupt = millis();
  // If interrupts come faster than BUTTON_DEBOUNCE_MS, assume it's a bounce and ignore
  if (this_interrupt - last_interrupt > BUTTON_DEBOUNCE_MS)
  {
    switchB = HIGH;
    if ( digitalRead(BUTTONB) == LOW ) { 
      switchB = HIGH; 
    }
  }
  last_interrupt = this_interrupt;  
}


void setup() {

  pinMode(BUTTON_IN, INPUT_PULLUP);
  pinMode(BUTTONA, INPUT);
  pinMode(BUTTONB, INPUT);

  pinMode(VACUUM_ENABLE, OUTPUT);
  pinMode(CAPTURE_ENABLE, OUTPUT);

  pinMode(SPEAKER, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  // apparently don't need to set pinMode for analog input pins...
  
  pinMode(PEN_TIP_PHOTOGATE, INPUT);
    
  attachInterrupt(BUTTONA_INT, onUIButtonA, FALLING);
  attachInterrupt(BUTTONB_INT, onUIButtonB, FALLING);

}


void loop() {
  char serialCmd = 0, testCmd = 0;
  static FWState state = FWSTATE_UNINITIALIZED;

  if (!serialCmd && Serial.available() > 0) {
    serialCmd = Serial.read();
  }

  if (serialCmd) {

    if (isspace(serialCmd) || serialCmd == 0) { // ignore spaces
      
      return;
      
    } else if (state == FWSTATE_TEST_MODE) {
      testCmd = serialCmd;
      
    } else if(serialCmd == 't') {
      Serial.println("Enter test mode");
      testCmd = '?';
      state = FWSTATE_TEST_MODE;
    } else {
      Serial.println("Unknown command.");
    }
  }

  if ( switchA && switchB ) {
    state = FWSTATE_UNINITIALIZED;
    switchA = switchB = 0;
  }

  switch( state ) {
    case FWSTATE_UNINITIALIZED:
      // Turn on vacuum, ensure capture vacuum is off
      digitalWrite(VACUUM_ENABLE, HIGH);
      digitalWrite(CAPTURE_ENABLE, LOW);
      switchA = switchB = LOW;
      state = FWSTATE_INITIALIZED;
      break;
    case FWSTATE_INITIALIZED:
      tone(SPEAKER, 440, 200);
      Serial.println("At any point, send 't' for test mode, A & B buttons together for reset");
      Serial.println("Initialized, waiting for dispense");
      Serial.println("Button A to indicate fly loaded");
      state = FWSTATE_WAITING_FOR_DISPENSE;
      break;
    case FWSTATE_WAITING_FOR_DISPENSE:
      if ( switchA ) {
        digitalWrite(VACUUM_ENABLE, LOW);
        state = FWSTATE_DISPENSED;
        switchA = LOW;
        penTipTrigger = LOW;
      }
      break;
    case FWSTATE_DISPENSED:
      tone(SPEAKER, 440, 200);
      Serial.println("Releasing fly, waiting for photogate to be interrupted");
      state = FWSTATE_WAITING_FOR_CAPTURE;
      break;
    case FWSTATE_WAITING_FOR_CAPTURE:
      if ( digitalRead(PEN_TIP_PHOTOGATE) ) {
        digitalWrite(CAPTURE_ENABLE, HIGH);
        Serial.println("Fly detected, capture enabled");
        Serial.println("Button B to disable capture and reset");
        state = FWSTATE_CAPTURED;
      }
      break;
    case FWSTATE_CAPTURED:
      if ( switchB ) {
        Serial.println("Resetting");
        state = FWSTATE_UNINITIALIZED;
      }
      break;
    case FWSTATE_TEST_MODE:
      switch (testCmd) {
        case 't':
          Serial.println("Exit test mode");
          state = FWSTATE_UNINITIALIZED;
          break;
        case 'v':
          if ( digitalRead(VACUUM_ENABLE) ) {
            digitalWrite(VACUUM_ENABLE, LOW);
            Serial.println("Vacuum disabled");
          } else {
            digitalWrite(VACUUM_ENABLE, HIGH);
            Serial.println("Vacuum enabled");
          }
          break;
        case 'c':
          if ( digitalRead(CAPTURE_ENABLE) ) {
            digitalWrite(CAPTURE_ENABLE, LOW);
            Serial.println("Capture disabled");
          } else {
            digitalWrite(CAPTURE_ENABLE, HIGH);
            Serial.println("Capture enabled");
          }
          break;
        case 'i':
          Serial.print("Vacuum: "); Serial.println(digitalRead(VACUUM_ENABLE));
          Serial.print("Capture: "); Serial.println(digitalRead(CAPTURE_ENABLE));
          Serial.print("Photogate: "); Serial.println(digitalRead(PEN_TIP_PHOTOGATE));
          Serial.print("Pressure: "); Serial.println(analogRead(PRESSURE_SENSOR_ADC));
          break;
        case '?':
          Serial.println("Test mode commands:");
          Serial.println(" t   - exit test mode");
          Serial.println(" v   - toggle vacuum");
          Serial.println(" c   - toggle capture");
          Serial.println(" i   - show info");
          Serial.println(" ?   - show help");
          
          break;
      }
      testCmd = 0;
      break;
    default:
      return;
  }


}
