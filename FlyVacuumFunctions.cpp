#include "arduino.h"
#include "pins.h"
#include "FlyVacuumFunctions.h"

void setupPins() {
  pinMode(PHOTOGATE, INPUT_PULLUP);

  pinMode(EJECT_SW_OPEN, INPUT);
  pinMode(EJECT_SW_CLOSED, INPUT);
  pinMode(GATE_SW_INPUT, INPUT);
  pinMode(GATE_SW_CLOSED, INPUT);
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

  pinMode(GATE_PWM, OUTPUT);
  pinMode(GATE_FWD, OUTPUT);
  pinMode(GATE_REV, OUTPUT);

  pinMode(EJECT_PWM, OUTPUT);
  pinMode(EJECT_FWD, OUTPUT);
  pinMode(EJECT_REV, OUTPUT);
  
}

Status initialize() {
  Status s;
  motorsOff();
  if ( digitalRead(PHOTOGATE) != HIGH ) { return PHOTOGATE_FAILURE; }

  s = homeGates();
  if ( s != SUCCESS ) { return s; }
  
  
  return SUCCESS;
}

void motorsOff() {
  driveMotor(MOTOR_NEEDLE, MOTOR_FWD, 0);
  driveMotor(MOTOR_EJECT, MOTOR_FWD, 0);
  driveMotor(MOTOR_GATE, MOTOR_FWD, 0);
}

void driveMotor(Motor m, MotorDirection d, int pwm) {

  int fwdPin, revPin, pwmPin;

  switch ( m ) {
    case MOTOR_NEEDLE:
      fwdPin = NEEDLE_FWD;
      revPin = NEEDLE_REV;
      pwmPin = NEEDLE_PWM;
      break;
    case MOTOR_EJECT:
      fwdPin = EJECT_FWD;
      revPin = EJECT_REV;
      pwmPin = EJECT_PWM;
      break;
    case MOTOR_GATE:
      fwdPin = GATE_FWD;
      revPin = GATE_REV;
      pwmPin = GATE_PWM;
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
  if ( timeOut ) { return GATE_FAILURE; }

  return SUCCESS;
}

Status rotateNeedle(int c) {
  Status s;

  for (int n = 0; n < c; n++ ) {
    s = driveMotorUntil(MOTOR_NEEDLE, MOTOR_FWD, NEEDLE_MOTOR_SPEED, NEEDLE_SW, HIGH, NEEDLE_MOTOR_TIMEOUT_MS);
    if ( s == TIMEOUT ) { return NEEDLE_TIMEOUT; }
    s = driveMotorUntil(MOTOR_NEEDLE, MOTOR_FWD, NEEDLE_MOTOR_SPEED, NEEDLE_SW, LOW, NEEDLE_MOTOR_TIMEOUT_MS);
    if ( s == TIMEOUT ) { return NEEDLE_TIMEOUT; }
  }
  
  return SUCCESS;

}

Status homeGates() {
  Status s;

  // First, drive MOTOR_GATE until GATE_SW_INPUT is triggered
  s = driveMotorUntil(MOTOR_GATE, MOTOR_FWD, GATE_MOTOR_SPEED, GATE_SW_INPUT, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s == TIMEOUT ) { return GATE_FAILURE; }

  // Now reverse until GATE_SW_CLOSED is triggered
  s = driveMotorUntil(MOTOR_GATE, MOTOR_REV, GATE_MOTOR_SPEED, GATE_SW_CLOSED, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s == TIMEOUT ) { return GATE_FAILURE; }

  // Drive MOTOR_EJECT until EJECT_SW_CLOSED is triggered
  s = driveMotorUntil(MOTOR_EJECT, MOTOR_FWD, GATE_MOTOR_SPEED, EJECT_SW_CLOSED, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s == TIMEOUT ) { return GATE_FAILURE; }

  return SUCCESS;
}

Status openGate(Gate g) {
  Motor m;
  MotorDirection d;
  int switchPin;
  Status s;

  if (g == FRONT_GATE ) {
    if ( digitalRead(GATE_SW_INPUT) == LOW ) { return SUCCESS; }
    m = MOTOR_GATE;
    d = MOTOR_FWD;
    switchPin = GATE_SW_INPUT;
  } else if ( g == BACK_GATE ) {
    if ( digitalRead(GATE_SW_OUTPUT) == LOW ) { return SUCCESS; }
    m = MOTOR_GATE;
    d = MOTOR_REV;
    switchPin = GATE_SW_OUTPUT;
  } else if ( g == EJECT_GATE ) {
    if ( digitalRead(EJECT_SW_OPEN) == LOW ) { return SUCCESS; }
    m = MOTOR_EJECT;
    d = MOTOR_REV;
    switchPin = EJECT_SW_OPEN;
  } else {
    return GATE_FAILURE;
  }
  
  s = driveMotorUntil(m, d, GATE_MOTOR_SPEED, switchPin, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s == TIMEOUT ) { return GATE_FAILURE; }
  
  return SUCCESS;
}

Status closeGate(Gate g) {
  Motor m;
  MotorDirection d;
  int switchPin;
  Status s;

  if ((g == FRONT_GATE ) || ( g == BACK_GATE)) {
    if ( digitalRead(GATE_SW_CLOSED) == LOW ) { return SUCCESS; }
    m = MOTOR_GATE;
    if ( digitalRead(GATE_SW_OUTPUT) == LOW ) {
      d = MOTOR_FWD;
    } else {
      d = MOTOR_REV;
    }
    switchPin = GATE_SW_CLOSED;
  } else if ( g == EJECT_GATE ) {
    if ( digitalRead(EJECT_SW_CLOSED) == LOW ) { return SUCCESS; }
    m = MOTOR_EJECT;
    d = MOTOR_FWD;
    switchPin = EJECT_SW_CLOSED;
  } else {
    return GATE_FAILURE;
  }

  s = driveMotorUntil(m, d, GATE_MOTOR_SPEED, switchPin, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s == TIMEOUT ) { return GATE_FAILURE; }

  return SUCCESS;
}

Status captureFly() {
  unsigned long int startTime;
  boolean timeOut;

  for (int n=0; n < CAPTURE_ATTEMPTS; n++) {
    startTime = millis();
    timeOut = false;
    // Wait for photogate trigger (up to PHOTOGATE_TIMEOUT_MS)
    while ( digitalRead(PHOTOGATE) != LOW ) {
      delay(1);
      if ( millis() - startTime > PHOTOGATE_TIMEOUT_MS ) { timeOut = true; break; }
    }
    if ( timeOut ) { return PHOTOGATE_FAILURE; }

    digitalWrite(NEEDLE_NEG_EN, HIGH);
    digitalWrite(PUSH_EN, HIGH);
    delay( NEEDLE_CAPTURE_DURATION_MS );
    if ( getPressure() < CAPTURE_PRESSURE_THRESHOLD ) {
      digitalWrite(PUSH_EN, LOW);
      return SUCCESS;
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
  
  for (int n = 0; n < NEEDLE_ROTATION_COUNT; n++) {
    Serial.println("i"); // Send signal that fly is ready for imaging
    delay(FLY_IMAGING_DELAY_MS);
    s = rotateNeedle(2);
    if ( s != SUCCESS ) { return s; }
  }

  return SUCCESS;
}

Status ejectFly() {
  Status s;

  s = openGate(EJECT_GATE);
  if ( s != SUCCESS ) { return EJECT_GATE_FAILURE; }
  digitalWrite(NEEDLE_NEG_EN, LOW);
  digitalWrite(EJECT_EN, HIGH); delay(FLY_EJECT_DURATION_MS); digitalWrite(EJECT_EN, LOW);

  s = closeGate(EJECT_GATE);
  if ( s != SUCCESS ) { return EJECT_GATE_FAILURE; }

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


