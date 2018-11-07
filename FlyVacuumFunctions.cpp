#include "arduino.h"
#include "pins.h"
#include "FlyVacuumFunctions.h"

void setupPins() {
  pinMode(PHOTOGATE, INPUT_PULLUP);

  pinMode(GATE_SW_LCLOSED, INPUT);
  pinMode(GATE_SW_RCLOSED, INPUT);
  pinMode(GATE_SW_EJECT, INPUT);
  pinMode(GATE_SW_INPUT, INPUT);
  pinMode(GATE_SW_OUTPUT, INPUT);
  pinMode(NEEDLE_SW, INPUT);

  pinMode(ILLUM_EN1, OUTPUT);
  pinMode(ILLUM_EN2, OUTPUT);
  pinMode(LURE_EN, OUTPUT);

  pinMode(PUSH_EN, OUTPUT);
  pinMode(EJECT_EN, OUTPUT);
  pinMode(NEEDLE_NEG_EN, OUTPUT);
  pinMode(SPARE_EN, OUTPUT);

  pinMode(NEEDLE_PWM, OUTPUT);
  pinMode(NEEDLE_FWD, OUTPUT);
  pinMode(NEEDLE_REV, OUTPUT);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_REV, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_REV, OUTPUT);
  
}

Status initialize() {
  Status s;
  motorsOff();
  digitalWrite(PUSH_EN, LOW);
  digitalWrite(EJECT_EN, LOW);
  digitalWrite(NEEDLE_NEG_EN, LOW);
  digitalWrite(LURE_EN, LOW);
  digitalWrite(ILLUM_EN1, LOW);
  digitalWrite(ILLUM_EN2, LOW);

  if ( digitalRead(PHOTOGATE) != HIGH ) { return PHOTOGATE_FAILURE; }

  s = homeGates();
  if ( s != SUCCESS ) { return s; }
  
  
  return SUCCESS;
}

void motorsOff() {
  driveMotor(MOTOR_NEEDLE, MOTOR_FWD, 0);
  driveMotor(MOTOR_LEFT, MOTOR_FWD, 0);
  driveMotor(MOTOR_RIGHT, MOTOR_FWD, 0);
}

void driveMotor(Motor m, MotorDirection d, int pwm) {

  int fwdPin, revPin, pwmPin;

  switch ( m ) {
    case MOTOR_NEEDLE:
      fwdPin = NEEDLE_FWD;
      revPin = NEEDLE_REV;
      pwmPin = NEEDLE_PWM;
      break;
    case MOTOR_LEFT:
      fwdPin = LEFT_FWD;
      revPin = LEFT_REV;
      pwmPin = LEFT_PWM;
      break;
    case MOTOR_RIGHT:
      fwdPin = RIGHT_FWD;
      revPin = RIGHT_REV;
      pwmPin = RIGHT_PWM;
      break;
    default:
      return;
  }
  
  if (pwm == 0) {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(revPin, HIGH);   
  } else if (d == MOTOR_FWD) {
    digitalWrite(fwdPin, LOW);
    digitalWrite(revPin, HIGH);
  } else {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(revPin, LOW);
  }

  analogWrite(pwmPin, pwm);
  
}

Status driveMotorUntil(Motor m, MotorDirection d, int pwm, int switchPin, boolean desiredState, int t) {
  unsigned long int startTime = millis();
  int switchReads = 0;
  boolean timeOut = false;
  
  driveMotor(m, d, pwm);
  while ( switchReads < SWITCH_DEBOUNCE_READS ) {
    delay(1);
    if ( digitalRead(switchPin) == desiredState ) { switchReads++; }
    if ( millis() - startTime > t ) { timeOut = true; break; }
  }
  driveMotor(m, d, 0);
  if ( timeOut == true ) { return GATE_FAILURE; }

  return SUCCESS;
}

Status rotateNeedle(int c, MotorDirection d ) {
  Status s;

  for (int n = 0; n < c; n++ ) {
    s = driveMotorUntil(MOTOR_NEEDLE, d, NEEDLE_MOTOR_SPEED, NEEDLE_SW, HIGH, NEEDLE_MOTOR_TIMEOUT_MS);
    if ( s == TIMEOUT ) { return NEEDLE_TIMEOUT; }
    s = driveMotorUntil(MOTOR_NEEDLE, d, NEEDLE_MOTOR_SPEED, NEEDLE_SW, LOW, NEEDLE_MOTOR_TIMEOUT_MS);
    if ( s == TIMEOUT ) { return NEEDLE_TIMEOUT; }
  }
  
  return SUCCESS;

}

Status homeGates() {
  Status s;

  // First, drive MOTOR_RIGHT until GATE_SW_OUPUT is triggered
  s = driveMotorUntil(MOTOR_RIGHT, MOTOR_REV, GATE_MOTOR_SPEED, GATE_SW_OUTPUT, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s != SUCCESS ) { return GATE_FAILURE; }

  // Now reverse until GATE_SW_RCLOSED is triggered
  s = driveMotorUntil(MOTOR_RIGHT, MOTOR_FWD, GATE_MOTOR_SPEED, GATE_SW_RCLOSED, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s != SUCCESS ) { return GATE_FAILURE; }

  // Drive MOTOR_LEFT until GATE_SW_LCLOSED is triggered
  s = driveMotorUntil(MOTOR_LEFT, MOTOR_FWD, GATE_MOTOR_SPEED, GATE_SW_LCLOSED, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s != SUCCESS ) { return GATE_FAILURE; }

  return SUCCESS;
}

Status openGate(Gate g) {
  Motor m;
  MotorDirection d;
  int switchPin;
  Status s;

  if (g == FRONT_GATE ) {
    if ( digitalRead(GATE_SW_INPUT) == LOW ) { return SUCCESS; }
    // Serial.println("ML R INPUT");
    m = MOTOR_LEFT;
    d = MOTOR_REV;
    switchPin = GATE_SW_INPUT;
  } else if ( g == BACK_GATE ) {
    if ( digitalRead(GATE_SW_OUTPUT) == LOW ) { return SUCCESS; }
    // Serial.println("MR R OUTPUT");
    m = MOTOR_RIGHT;
    d = MOTOR_REV;
    switchPin = GATE_SW_OUTPUT;
  } else if ( g == EJECT_GATE ) {
    if ( digitalRead(GATE_SW_EJECT) == LOW ) { return SUCCESS; }
    // Serial.println("MR F EJECT");
    m = MOTOR_RIGHT;
    d = MOTOR_FWD; 
    switchPin = GATE_SW_EJECT;
  } else {
    return GATE_FAILURE;
  }
  
  s = driveMotorUntil(m, d, GATE_MOTOR_SPEED, switchPin, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s != SUCCESS ) { return GATE_FAILURE; }
  
  return SUCCESS;
}

Status closeGate(Gate g) {
  Motor m;
  MotorDirection d;
  int switchPin;
  Status s;

  if ((g == BACK_GATE ) || ( g == EJECT_GATE)) {
    if ( digitalRead(GATE_SW_RCLOSED) == LOW ) { return SUCCESS; }
    m = MOTOR_RIGHT;
    if ( digitalRead(GATE_SW_OUTPUT) == LOW ) {
      // Serial.println("MR F RCLOSED");
      d = MOTOR_FWD;
    } else {
      // Serial.println("MR R RCLOSED");
      d = MOTOR_REV;
    }
    switchPin = GATE_SW_RCLOSED;
  } else if ( g == FRONT_GATE ) {
    if ( digitalRead(GATE_SW_LCLOSED) == LOW ) { return SUCCESS; }
    // Serial.println("ML F LCLOSED");
    m = MOTOR_LEFT;
    d = MOTOR_FWD;
    switchPin = GATE_SW_LCLOSED;
  } else {
    return GATE_FAILURE;
  }

  s = driveMotorUntil(m, d, GATE_MOTOR_SPEED, switchPin, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s != SUCCESS ) { return GATE_FAILURE; }

  return SUCCESS;
}

Status captureFly(int vacThresh) {
  unsigned long int startTime;
  boolean timeOut;

  // Serial.println("captureFly");
  
  for (int n=0; n < CAPTURE_ATTEMPTS; n++) {
    // Serial.print(" attempt #"); Serial.println(n+1);
    startTime = millis();
    timeOut = false;
    // Wait for photogate trigger (up to PHOTOGATE_TIMEOUT_MS)
    while ( digitalRead(PHOTOGATE) != LOW ) {
      if ( millis() - startTime > PHOTOGATE_TIMEOUT_MS ) { timeOut = true; break; }
    }
    
    if ( timeOut ) {
      Serial.println("p");
      if ( n == (CAPTURE_ATTEMPTS-1) ) {
        return PHOTOGATE_FAILURE;
      }
    } else {

      Serial.println("d");
      digitalWrite(NEEDLE_NEG_EN, HIGH);
      digitalWrite(PUSH_EN, HIGH);
      startTime = millis();
      while ( millis() - startTime < MAX_NEEDLE_CAPTURE_DURATION_MS ) {
        int pressure = getPressure();
        // Serial.print("Pressure is "); Serial.println(pressure);
        if ( pressure < vacThresh ) {
          digitalWrite(PUSH_EN, LOW);
          return SUCCESS;
        }
        delay(1);
      }
      // Serial.println("  Missed");
    }

    digitalWrite(NEEDLE_NEG_EN, LOW);
    digitalWrite(PUSH_EN, LOW);
    digitalWrite(EJECT_EN, HIGH); delay(FLY_RETRY_PUFF_DURATION_MS); digitalWrite(EJECT_EN, LOW);

  }

  return CAPTURE_FAILURE;
  
}

Status imageFly() {
  Status s;
  // Illumination!

  // Go 180 from beginning, then reverse back to start, then 180 in other direction
  for (int n = 0; n < NEEDLE_ROTATION_COUNT/2; n++) {
    Serial.println(n, HEX); // Send signal that fly is ready for imaging
    delay(FLY_IMAGING_DELAY_MS);
    s = rotateNeedle(2, MOTOR_FWD);
    if ( s != SUCCESS ) { return s; }
  }

  s = rotateNeedle(NEEDLE_ROTATION_COUNT, MOTOR_REV);
  if ( s != SUCCESS ) { return s; }

  for (int n = NEEDLE_ROTATION_COUNT/2; n < NEEDLE_ROTATION_COUNT; n++) {
    Serial.println(n, HEX); // Send signal that fly is ready for imaging
    delay(FLY_IMAGING_DELAY_MS);
    s = rotateNeedle(2, MOTOR_REV);
    if ( s != SUCCESS ) { return s; }
  }

  return SUCCESS;
}

Status ejectFly() {
  Status s;

  s = openGate(BACK_GATE);
  if ( s != SUCCESS ) { return GATE_FAILURE; }
  s = openGate(FRONT_GATE);
  if ( s != SUCCESS ) { return GATE_FAILURE; }

  digitalWrite(NEEDLE_NEG_EN, LOW);
  digitalWrite(EJECT_EN, HIGH); delay(FLY_EJECT_DURATION_MS); digitalWrite(EJECT_EN, LOW);

  return SUCCESS;
}

void printStatus(Status s) {
  if ( s == SUCCESS ) { Serial.println("Success"); }
  if ( s == GATE_FAILURE ) { Serial.println("Inlet/outlet gate failure"); }
  if ( s == EJECT_GATE_FAILURE ) { Serial.println("Eject gate failure"); }
  if ( s == PRESSURE_SENSOR_FAILURE ) { Serial.println("Pressure sensor failure"); }
  if ( s == PHOTOGATE_FAILURE ) { Serial.println("Photogate failure"); }
  if ( s == NEEDLE_TIMEOUT ) { Serial.println("Needle rotation timeout"); }
  if ( s == CAPTURE_FAILURE ) { Serial.println("Capture failure"); }
  
}

int getPressure() {
  long int readings = 0;
  for (int n = 0; n < PRESSURE_READING_COUNT; n++ ) {
    readings += analogRead(PRESSURE_SENSOR_ADC);
  }
  return (int)(readings/PRESSURE_READING_COUNT);
}

void togglePin(int p, String s) {
  if ( digitalRead(p) ) {
    digitalWrite(p, LOW);
    if ( s != "" ) {
      Serial.print(s);
      Serial.println(" disabled");
    }
  } else {
    digitalWrite(p, HIGH);
    if ( s != "" ) {
      Serial.print(s);
      Serial.println(" enabled");
    }
  }
}


