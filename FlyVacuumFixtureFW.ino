/* Revision History:
 *  
 *  
 *  
 */

#include "Arduino.h"
#include "pins.h"

#define BUTTON_DEBOUNCE_MS   200
#define PAGER_PWM            255
#define FOAM_PWM             150
#define FOAM_ROLL_TIME_MS    250
#define CO2_TIME_MS          3000
#define CO2_DELAY_MS         500

volatile int switchA=LOW, switchB=LOW, dispenseButton=0;
volatile bool pgTrigger = false;
unsigned long startTime, endTime;
bool foamWheels = false;

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

void onPG() {
  pgTrigger = true;
}

void pagerMotor(bool onOff = LOW) {
  if (onOff) {
    digitalWrite(GEAR_MOTOR_IN1, LOW);
    digitalWrite(GEAR_MOTOR_IN2, HIGH);
    analogWrite(GEAR_MOTOR_PWM, PAGER_PWM);
  } else {
    digitalWrite(GEAR_MOTOR_IN1, HIGH);
    digitalWrite(GEAR_MOTOR_IN2, HIGH);
    analogWrite(GEAR_MOTOR_PWM, 0);
  }
}


void setup() {

  pinMode(BUTTON_IN, INPUT_PULLUP);
  pinMode(BUTTONA, INPUT);
  pinMode(BUTTONB, INPUT);

  // IF BUTTON A IS PUSHED, GO INTO SPECIAL MODE
  if ( !digitalRead(BUTTONA) ) {
    foamWheels = true;
    
  }

  pinMode(VACUUM_ENABLE, OUTPUT);
  pinMode(CAPTURE_ENABLE, OUTPUT);
  pinMode(EJECT_ENABLE, OUTPUT);
  
  pinMode(SPEAKER, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(GEAR_MOTOR_IN1, OUTPUT);
  pinMode(GEAR_MOTOR_IN2, OUTPUT);
  pinMode(GEAR_MOTOR_PWM, OUTPUT);
  
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  pinMode(FLASH_ENABLE, OUTPUT);
  digitalWrite(FLASH_ENABLE, HIGH);
  
  analogWrite(GEAR_MOTOR_PWM, 0);
  
  pinMode(PEN_TIP_PHOTOGATE, INPUT);
    
  attachInterrupt(BUTTONA_INT, onUIButtonA, FALLING);
  attachInterrupt(BUTTONB_INT, onUIButtonB, FALLING);
  attachInterrupt(PEN_TIP_PHOTOGATE_INT, onPG, FALLING);

}


void loop() {
  char serialCmd = 0, testCmd = 0;
  static FWState state = FWSTATE_UNINITIALIZED;

  if ( foamWheels ) {
    if ( switchB ) {
      digitalWrite(GEAR_MOTOR_IN1, HIGH); digitalWrite(GEAR_MOTOR_IN2, LOW);
      analogWrite(GEAR_MOTOR_PWM, FOAM_PWM);
      delay(FOAM_ROLL_TIME_MS);
      digitalWrite(GEAR_MOTOR_IN1, HIGH); digitalWrite(GEAR_MOTOR_IN2, HIGH);
      analogWrite(GEAR_MOTOR_PWM, 0);      
      switchB = 0;
    }

    return;
  }

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
      digitalWrite(EJECT_ENABLE, LOW);
      digitalWrite(FLASH_ENABLE, HIGH);
      pagerMotor(HIGH); delay(500); pagerMotor(LOW);
      switchA = switchB = LOW;
      startTime = endTime = 0;
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
        pgTrigger = false;
      }
      break;
    case FWSTATE_DISPENSED:
      Serial.println("Releasing fly, waiting for photogate to be interrupted");
      startTime = millis();
      pagerMotor(HIGH); delay(500); pagerMotor(LOW);
      tone(SPEAKER, 440, 200);
      state = FWSTATE_WAITING_FOR_CAPTURE;
      break;
    case FWSTATE_WAITING_FOR_CAPTURE:
      if ( pgTrigger ) {
        digitalWrite(FLASH_ENABLE, LOW);
        digitalWrite(CAPTURE_ENABLE, HIGH);
        endTime = millis();
        delay(CO2_DELAY_MS);
        digitalWrite(EJECT_ENABLE, HIGH); // CO2
        Serial.print("Fly detected, elapsed time (ms): "); Serial.println(endTime - startTime);
        Serial.println("Button B to disable capture and reset");
        delay(CO2_TIME_MS);
        digitalWrite(EJECT_ENABLE, LOW);
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
        case 'f':
          if ( digitalRead(FLASH_ENABLE) ) {
            digitalWrite(FLASH_ENABLE, LOW);
            Serial.println("Flash enabled");
          } else {
            digitalWrite(FLASH_ENABLE, HIGH);
            Serial.println("Flash disabled");
          }
          break;
        case '2':
          if ( digitalRead(EJECT_ENABLE) ) {
            digitalWrite(EJECT_ENABLE, LOW);
            Serial.println("CO2 disabled");
          } else {
            digitalWrite(EJECT_ENABLE, HIGH);
            Serial.println("CO2 enabled");
          }
          break;
        case 'P':
          Serial.println("Enable pager motor");
          pagerMotor(HIGH);
          break;
        case 'p':
          Serial.println("Disable pager motor");
          pagerMotor(LOW);
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
          Serial.println(" f   - toggle flash enable");
          Serial.println(" 2   - toggle CO2");
          Serial.println(" P/p - toggle pager motor");
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
