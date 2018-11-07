
enum {
  GATE_MOTOR_SPEED = 150,
  NEEDLE_MOTOR_SPEED = 200,
  SWITCH_DEBOUNCE_READS = 5,
  NEEDLE_MOTOR_TIMEOUT_MS = 500,
  GATE_MOTOR_TIMEOUT_MS = 250,
  POST_DISPENSE_WAIT_MS = 1000,
  PHOTOGATE_TIMEOUT_MS = 10000,
  MAX_NEEDLE_CAPTURE_DURATION_MS = 2500,
  CAPTURE_ATTEMPTS = 3,
  NOMINAL_VACUUM_THRESHOLD = 450,
  CAPTURE_PRESSURE_THRESHOLD = 40,
  PRESSURE_READING_COUNT = 10,
  FLY_RETRY_PUFF_DURATION_MS = 250,
  NEEDLE_ROTATION_COUNT = 16,
  FLY_IMAGING_DELAY_MS = 250,
  FLY_EJECT_DURATION_MS = 2000,
};

enum Gate {
  FRONT_GATE = 0,
  BACK_GATE = 1,
  EJECT_GATE = 2
};

enum Status {
  SUCCESS = 0,
  TIMEOUT = 1,
  GATE_FAILURE = 2,
  EJECT_GATE_FAILURE = 3,
  PRESSURE_SENSOR_FAILURE = 4,
  PHOTOGATE_FAILURE = 5,
  NEEDLE_TIMEOUT = 6,
  CAPTURE_FAILURE = 7,
};

enum Motor {
  MOTOR_NEEDLE = 0,
  MOTOR_LEFT = 1,
  MOTOR_RIGHT = 2
};

enum MotorDirection {
  MOTOR_FWD = 0,
  MOTOR_REV = 1
};

void setupPins();
Status initialize();
void motorsOff();
void driveMotor(Motor m, MotorDirection d, int pwm);
Status driveMotorUntil(Motor m, MotorDirection d, int pwm, int switchPin, boolean desiredState, int t);
Status rotateNeedle(int c, MotorDirection d = MOTOR_FWD);
Status homeGates();
Status openGate(Gate g);
Status closeGate(Gate g);
Status captureFly(int vacThresh);
Status imageFly();
Status ejectFly();
int getPressure();
void printStatus(Status s);

void togglePin(int p, String s = "");


