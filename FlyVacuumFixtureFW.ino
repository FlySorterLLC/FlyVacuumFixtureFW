/* Revision History:

   0.01 - 2018.09.24 - Initial version

*/

#include "Arduino.h"
#include "FlyVacuumFunctions.h"
#include "pins.h"

#define VERSION              "BN FW ver. 0.05"

/*
    Commands

    Each command sends a lowercase response (e.g. an 'f' for 'F') to acknowledge
    receipt, and then MAY send additional information. For example, the 'B' command
    will send a hexadecimal number (0-A) when the PC should capture images during
    fly rotation.

      V - send _V_ersion info
      I - (re)_I_nitialize
      T - _T_est mode
      F - a _F_ly has been sent (close gate, capture fly)
      B - _B_egin image capture process
      E - _E_ject fly
      P - _P_urge any fly that might be in the system

*/

enum FWState {
  FWSTATE_ERROR = -1,
  FWSTATE_UNINITIALIZED = 0,
  FWSTATE_INITIALIZED = 1,
  FWSTATE_DISPENSED = 2,
  FWSTATE_WAITING_FOR_CAPTURE = 3,
  FWSTATE_CAPTURED = 4,
  FWSTATE_IMAGING = 5,
  FWSTATE_WAITING_FOR_EJECT = 6,
  FWSTATE_EJECTING = 7,
  FWSTATE_TEST_MODE = 8,
};

void setup() {
  setupPins();
}


void loop() {
  char serialCmd = 0, testCmd = 0;
  static FWState state = FWSTATE_UNINITIALIZED;
  Status s;
  unsigned long int startTime;
  boolean timeOut;
  static int vacThreshold = CAPTURE_PRESSURE_THRESHOLD;

  if (!serialCmd && Serial.available() > 0) {
    serialCmd = Serial.read();
  }

  if (serialCmd) {

    if (isspace(serialCmd) || serialCmd == 0) { // ignore spaces
      return;
    } else if (state == FWSTATE_TEST_MODE) {
      testCmd = serialCmd;
    } else if (serialCmd == 'T') {
      Serial.println("Enter test mode");
      testCmd = '?';
      state = FWSTATE_TEST_MODE;
    } else if (serialCmd == 'F') {
      if ( state != FWSTATE_INITIALIZED ) {
        Serial.print("Error: not ready; STATE = "); printState(state);
      } else {
        Serial.println("f");
        state = FWSTATE_DISPENSED;
      }
    } else if (serialCmd == 'B') {
      if ( state != FWSTATE_CAPTURED ) {
        Serial.print("Error: not ready; STATE = "); printState(state);
      } else {
        Serial.println("b");
        state = FWSTATE_IMAGING;
      }
    } else if (serialCmd == 'E') {
      if ( ( state != FWSTATE_WAITING_FOR_EJECT ) && ( state != FWSTATE_CAPTURED ) ){
        Serial.print("Error: not ready; STATE = "); printState(state);
      } else {
        Serial.println("e");
        state = FWSTATE_EJECTING;
      }
    } else if (serialCmd == 'I') {
      state = FWSTATE_UNINITIALIZED;
    } else if (serialCmd == 'P') {
      state = FWSTATE_EJECTING;
    } else if (serialCmd == 'V') {
      Serial.println(VERSION);
    } else if (serialCmd == 'S') {
      printState(state);
    } else {
      Serial.println("Unknown command.");
    }
  }

  switch ( state ) {
    case FWSTATE_UNINITIALIZED:
      s = initialize();
      if ( s != SUCCESS ) {
        Serial.print("Initialization failure: "); printStatus(s);
        state = FWSTATE_ERROR;
        break;
      }
      s = openGate(FRONT_GATE);
      if ( s != SUCCESS ) {
        Serial.print("Failed to open front gate: "); printStatus(s);
        state = FWSTATE_ERROR;
        break;
      }      
      s = openGate(EJECT_GATE);
      if ( s != SUCCESS ) {
        Serial.print("Failed to open eject gate: "); printStatus(s);
        state = FWSTATE_ERROR;
        break;
      }      
      state = FWSTATE_INITIALIZED;
      break;
    case FWSTATE_INITIALIZED:
      // Ready state. Waiting for further input. 
      break;
    case FWSTATE_DISPENSED:
      // Fly has been dispensed into device
      s = closeGate(FRONT_GATE);
      if ( s != SUCCESS ) {
        Serial.print("Failed to close gate: "); printStatus(s);
        state = FWSTATE_ERROR;
        break;
      }
      digitalWrite(NEEDLE_NEG_EN, HIGH);
      delay(POST_DISPENSE_WAIT_MS);
      vacThreshold = getPressure() - 5;
      if ( vacThreshold < 0 ) { vacThreshold = 20; }
      // Serial.print("Vac threshold set to "); Serial.println(vacThreshold);
      digitalWrite(NEEDLE_NEG_EN, LOW);
      s = openGate(BACK_GATE);
      if ( s != SUCCESS ) {
        Serial.print("Failed to open back gate: "); printStatus(s);
        state = FWSTATE_ERROR;
        break;
      }
      digitalWrite(LURE_EN, HIGH);
      state = FWSTATE_WAITING_FOR_CAPTURE;
      break;
    case FWSTATE_WAITING_FOR_CAPTURE:
      // Back gate is open, waiting for fly to walk
      s = captureFly(vacThreshold);
      if ( s == SUCCESS ) {
        Serial.println("c");
        digitalWrite(LURE_EN, LOW);
        state = FWSTATE_CAPTURED;
      } else if ( s == PHOTOGATE_FAILURE ) {
        Serial.println("t");
        state = FWSTATE_EJECTING;
      } else if ( s == CAPTURE_FAILURE ) {
        Serial.println("t");
        state = FWSTATE_ERROR;
      } else {
        Serial.print("Unknown error: "); printStatus(s);
        state = FWSTATE_ERROR;
      }
      break;
    case FWSTATE_CAPTURED:
      // Ready state. Waiting for further input.
      break;
    case FWSTATE_IMAGING:
      s = imageFly();
      if ( s == SUCCESS ) {
        Serial.println("b");
        state = FWSTATE_WAITING_FOR_EJECT;
      } else if ( s == NEEDLE_TIMEOUT ) {
        Serial.println("t");
        state = FWSTATE_ERROR;
      } else {
        Serial.print("Unknown error: "); printStatus(s);
        state = FWSTATE_ERROR;
      }
      break;
      break;
    case FWSTATE_WAITING_FOR_EJECT:
      // Ready state. Waiting for further input.
      break;
    case FWSTATE_EJECTING:
      s = ejectFly();
      if ( s == SUCCESS ) {
        Serial.println("e");
        state = FWSTATE_UNINITIALIZED;
      } else if ( s == EJECT_GATE_FAILURE ) {
        Serial.println("g");
        state = FWSTATE_ERROR;        
      } else {
        Serial.print("Unknown error: "); printStatus(s);
        state = FWSTATE_ERROR;
      }
      break;
    case FWSTATE_TEST_MODE:
      switch (testCmd) {
        case 'T':
          Serial.println("Exit test mode");
          state = FWSTATE_UNINITIALIZED;
          break;
        case 'i':
          Serial.println("Switches:");
          Serial.print("  Gates Eject: "); Serial.println(digitalRead(GATE_SW_EJECT));
          Serial.print("  Gates Inlet: "); Serial.println(digitalRead(GATE_SW_INPUT));
          Serial.print("  Gates Outlet: "); Serial.println(digitalRead(GATE_SW_OUTPUT));
          Serial.print("  Gates Right Closed: "); Serial.println(digitalRead(GATE_SW_RCLOSED));
          Serial.print("  Gates Left Closed: "); Serial.println(digitalRead(GATE_SW_LCLOSED));
          Serial.print("  Needle index: "); Serial.println(digitalRead(NEEDLE_SW));
          Serial.println("Solenoids:");
          Serial.print("  Push: "); Serial.println(digitalRead(PUSH_EN));
          Serial.print("  Eject: "); Serial.println(digitalRead(EJECT_EN));
          Serial.print("  Fly vac: "); Serial.println(digitalRead(NEEDLE_NEG_EN));
          Serial.print("  Spare: "); Serial.println(digitalRead(SPARE_EN));
          Serial.print("Photogate: "); Serial.println(digitalRead(PHOTOGATE));
          Serial.print("Pressure: "); Serial.println(getPressure());
          break;
        case 'v':
          togglePin(NEEDLE_NEG_EN, "Needle vacuum");
          break;
        case 'p':
          togglePin(PUSH_EN, "Push");
          break;
        case 'e':
          togglePin(EJECT_EN, "Eject");
          break;
        case 's':
          togglePin(SPARE_EN, "Spare switch");
          break;
        case 'd':
          togglePin(LURE_EN, "LED lure");
          break;
        case 'l':
          togglePin(ILLUM_EN1, "Illumination 1");
          break;
        case 'L':
          togglePin(ILLUM_EN2, "Illumination 2");
          break;
        case 'f':
          s = openGate(FRONT_GATE);
          printStatus(s);
          break;
        case 'b':
          s = openGate(BACK_GATE);
          printStatus(s);
          break;
        case 'g':
          s = openGate(EJECT_GATE);
          printStatus(s);
          break;
        case 'c':
          s = closeGate(FRONT_GATE);
          printStatus(s);
          break;
        case 'C':
          s = closeGate(EJECT_GATE);
          printStatus(s);
          break;
        case 'r':
          s = rotateNeedle(1);
          printStatus(s);
          break;
        case '?':
          Serial.println("Test mode commands:");
          Serial.println("   T   - exit test mode");
          Serial.println("   v   - toggle needle vacuum");
          Serial.println("   p   - toggle positive 'push' solenoid");
          Serial.println("   e   - toggle eject solenoid");
          Serial.println("   s   - toggle spare solenoid");
          Serial.println("   d   - toggle LED lure");
          Serial.println("   l/L - toggle camera illumination (1/2)");
          Serial.println("  c/C  - Close left/right gates");
          Serial.println(" f/b/g - Open front/back/eject gate");
          Serial.println("   r   - rotate needle one notch");
          Serial.println("   i   - show status info");
          Serial.println("   ?   - show help");
          break;
      }
      testCmd = 0;
      break;
    default:
      return;
  }


}

void printState (FWState s) {
  if ( s == FWSTATE_ERROR ) { Serial.println("FWSTATE_ERROR"); }
  if ( s == FWSTATE_UNINITIALIZED ) { Serial.println("FWSTATE_UNINITIALIZED"); }
  if ( s == FWSTATE_INITIALIZED ) { Serial.println("FWSTATE_INITIALIZED"); }
  if ( s == FWSTATE_DISPENSED ) { Serial.println("FWSTATE_DISPENSED"); }
  if ( s == FWSTATE_WAITING_FOR_CAPTURE ) { Serial.println("FWSTATE_WAITING_FOR_CAPTURE"); }
  if ( s == FWSTATE_CAPTURED ) { Serial.println("FWSTATE_CAPTURED"); }
  if ( s == FWSTATE_IMAGING ) { Serial.println("FWSTATE_IMAGING"); }
  if ( s == FWSTATE_WAITING_FOR_EJECT ) { Serial.println("FWSTATE_WAITING_FOR_EJECT"); }
  if ( s == FWSTATE_EJECTING ) { Serial.println("FWSTATE_EJECTING"); }
}

