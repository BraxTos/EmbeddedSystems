/***************************************************
 * Robot with MPU6050 + Start/Stop buttons
 * - START: A0 to GND (INPUT_PULLUP)  -> starts / resumes program
 * - STOP : D3 to GND (INPUT_PULLUP)  -> emergency stop via interrupt
 *
 * IMPORTANT:
 * - All delays in motion logic use smartDelay() so STOP reacts quickly.
 * - EN pin logic can be configured via EN_ACTIVE_LOW.
 ***************************************************/

// PWM control pin
#define PWM1_PIN            5
#define PWM2_PIN            6

// 74HCT595N Chip pins
#define SHCP_PIN            2     // shift register clock
#define EN_PIN              7     // enable pin (often active-low on many boards)
#define DATA_PIN            8     // serial data
#define STCP_PIN            4     // storage register clock (latch)

// Motion commands (to shift register)
const int Forward       = 106;
const int Backward      = 163;
const int Turn_Left     = 149;
const int Turn_Right    = 106;
const int Top_Left      = 20;
const int Bottom_Left   = 129;
const int Top_Right     = 72;
const int Bottom_Right  = 34;
const int Stop          = 0;
const int Contrarotate  = 139;
const int Clockwise     = 83;
const int RotateRight   = 101;
const int RotateLeft    = 154;

float K_TURN_R = 1.02;
float K_TURN_L = 1.02;

// Rotation PWM tuning
const int ROT_FAST_PWM = 200;
const int ROT_MID_PWM  = 185;
const int ROT_FINE_PWM = 180;
const int MIN_ROT_PWM  = 175;

const float ANGLE_TOL  = 2.0;
const int STABLE_CNT   = 10;

// If your board uses EN active-low (LOW enables motors), keep true.
// If HIGH enables motors, set false.
const bool EN_ACTIVE_LOW = true;

// --- Buttons ---
const int BTN_START = A0;   // START button: A0 -> GND
const int BTN_STOP  = 3;    // STOP button:  D3 -> GND (interrupt)

// Control flags
volatile bool stopReq = false;
bool runEnabled = false;

#include <Wire.h>
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

// --------- Prototypes ----------
void isrStop();

inline void enableMotors(bool en);
inline bool shouldStopNow();

bool smartDelay(unsigned long ms);

void pollStartButton();
void waitForStartAfterStop();

float norm180(float a);
void debugRotate(float z, float target, float err, int sp, int cmd);
void debugForw(float z);

void Motor(int Dir, int Speed);

bool Rotate(float phi);
void Forw(float time_ms);

// Optional
void detectYawSign();

// -------------------------------

void setup()
{
  pinMode(SHCP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(STCP_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);

  // Make motors disabled at boot (safer)
  enableMotors(false);
  Motor(Stop, 0);

  Serial.begin(9600);
  Serial.println("Boot...");
  delay(200);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);

  if (status != 0) {
    Serial.println("MPU6050 initialization failed!");
    while (1) { /* halt */ }
  }

  delay(500);
  Serial.println("Calibrating MPU6050... DO NOT MOVE!");
  mpu.calcOffsets(true, true);
  Serial.println("Calibration complete!");

  // Buttons
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BTN_STOP), isrStop, FALLING);

  Serial.println("Press START to begin...");
}

void loop()
{
  // wait for START
  if (!runEnabled) {
    pollStartButton();
    delay(10);
    return;
  }

  // if STOP requested -> halt until START
  if (stopReq) {
    waitForStartAfterStop();
    return;
  }

  // --- route ---
  Forw(2000);  if (stopReq) { waitForStartAfterStop(); return; }
  Rotate(90);  if (stopReq) { waitForStartAfterStop(); return; }

  Forw(2000);  if (stopReq) { waitForStartAfterStop(); return; }
  Rotate(90);  if (stopReq) { waitForStartAfterStop(); return; }

  Forw(2000);  if (stopReq) { waitForStartAfterStop(); return; }
  Rotate(90);  if (stopReq) { waitForStartAfterStop(); return; }

  Forw(2000);  if (stopReq) { waitForStartAfterStop(); return; }
  Rotate(90);  if (stopReq) { waitForStartAfterStop(); return; }

  // pause
  if (!smartDelay(5000)) { waitForStartAfterStop(); return; }
}

// ---------------- Buttons / Stop logic ----------------

void isrStop() {
  // Basic debounce in ISR (works on Uno in practice)
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last > 80) {      // 80ms debounce
    stopReq = true;
    last = now;
  }
}

inline void enableMotors(bool en) {
  // EN_ACTIVE_LOW: LOW enables; HIGH disables
  if (EN_ACTIVE_LOW) digitalWrite(EN_PIN, en ? LOW : HIGH);
  else               digitalWrite(EN_PIN, en ? HIGH : LOW);
}

inline bool shouldStopNow() {
  if (stopReq) {
    Motor(Stop, 0);
    enableMotors(false);
    return true;
  }
  return false;
}

bool smartDelay(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (shouldStopNow()) return false;
    delay(2);
  }
  return true;
}

void pollStartButton() {
  // Debounced "press event" (triggers once per press)
  static bool stable = HIGH;
  static bool lastRead = HIGH;
  static unsigned long tLast = 0;

  bool r = digitalRead(BTN_START);
  if (r != lastRead) { tLast = millis(); lastRead = r; }

  if (millis() - tLast > 40) {
    if (r != stable) {
      stable = r;
      if (stable == LOW) {   // pressed
        runEnabled = true;
      }
    }
  }
}

void waitForStartAfterStop() {
  Motor(Stop, 0);
  enableMotors(false);
  Serial.println("STOP: halted. Press START to run.");

  // reset flags
  stopReq = false;
  runEnabled = false;

  while (!runEnabled) {
    pollStartButton();
    delay(10);
  }

  Serial.println("START: running.");
}

// ---------------- Utilities / Debug ----------------

float norm180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void debugRotate(float z, float target, float err, int sp, int cmd) {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 50) {
    last = now;
    Serial.print("Z=");
    Serial.print(z, 2);
    Serial.print(" target=");
    Serial.print(target, 2);
    Serial.print(" err=");
    Serial.print(err, 2);
    Serial.print(" sp=");
    Serial.print(sp);
    Serial.print(" cmd=");
    Serial.println(cmd);
  }
}

void debugForw(float z) {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 100) {
    last = now;
    Serial.print("Forw Z=");
    Serial.println(z, 2);
  }
}

// ---------------- Motor / Motion ----------------

void Motor(int Dir, int Speed)
{
  // If Speed is 0 or Dir is Stop, we can disable motors for harder stop
  if (Speed <= 0 || Dir == Stop) {
    analogWrite(PWM1_PIN, 0);
    analogWrite(PWM2_PIN, 0);

    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, Stop);
    digitalWrite(STCP_PIN, HIGH);

    // keep disabled (optional but good for emergency stop)
    enableMotors(false);
    return;
  }

  enableMotors(true);

  analogWrite(PWM1_PIN, Speed);
  analogWrite(PWM2_PIN, Speed);

  digitalWrite(STCP_PIN, LOW);
  shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, Dir);
  digitalWrite(STCP_PIN, HIGH);
}

void Forw(float time_ms)
{
  if (shouldStopNow()) return;
  if (!smartDelay(200)) return;

  Serial.println("Forward");
  unsigned long start = millis();

  // Warm-up pulses
  for (int i = 0; i < 3; i++) {
    if (shouldStopNow()) return;

    Motor(Forward, 210);
    if (!smartDelay(30)) return;

    Motor(Stop, 0);
    if (!smartDelay(30)) return;
  }

  if (shouldStopNow()) return;
  Motor(Forward, 250);

  while (millis() - start < (unsigned long)time_ms) {
    if (shouldStopNow()) return;

    mpu.update();
    debugForw(mpu.getAngleZ());

    if (!smartDelay(10)) return;
  }

  Serial.println("Forward end");
  Motor(Stop, 0);
}

bool Rotate(float phi)
{
  if (shouldStopNow()) return false;

  mpu.update();
  float startAngle = mpu.getAngleZ();

  float k = (phi > 0) ? K_TURN_R : K_TURN_L;
  float targetAngle = startAngle - (phi * k);

  const int MIN_PWM  = max(MIN_ROT_PWM, 160);
  const int FAST_PWM = ROT_FAST_PWM;
  const int MID_PWM  = ROT_MID_PWM;
  const int FINE_PWM = max(ROT_FINE_PWM, MIN_PWM);

  const float TOL = ANGLE_TOL;
  const int STABLE = STABLE_CNT;
  const unsigned long TIMEOUT_MS = 9000;

  bool overshot = false;

  unsigned long t0 = millis();
  int stableCnt = 0;

  int cmdMain = (phi > 0) ? RotateRight : RotateLeft;
  int cmdOpp  = (phi > 0) ? RotateLeft  : RotateRight;

  // startup kick
  Motor(cmdMain, FAST_PWM);
  if (!smartDelay(60)) { Motor(Stop, 0); return false; }
  Motor(Stop, 0);
  if (!smartDelay(50)) return false;

  int cmd = cmdMain;

  const float OVERSHOOT_DEG = 6.0;

  mpu.update();
  float err0 = norm180(targetAngle - mpu.getAngleZ());
  int sign0 = (err0 >= 0) ? +1 : -1;

  while (true) {
    if (shouldStopNow()) return false;

    mpu.update();
    float z = mpu.getAngleZ();
    float err = norm180(targetAngle - z);
    float ae = abs(err);
    int sign = (err >= 0) ? +1 : -1;

    if (millis() - t0 > TIMEOUT_MS) {
      Serial.println("Rotate timeout");
      Motor(Stop, 0);
      return false;
    }

    // within tolerance
    if (ae <= TOL) {
      stableCnt++;
      Motor(Stop, 0);
      debugRotate(z, targetAngle, err, 0, Stop);

      if (stableCnt >= STABLE) {
        return true;
      }

      if (!smartDelay(25)) return false;
      continue;
    } else {
      stableCnt = 0;
    }

    if (!overshot && (sign != sign0) && (ae > OVERSHOOT_DEG)) {
      overshot = true;
    }

    cmd = overshot ? cmdOpp : cmdMain;

    // --- Zones ---
    if (ae < 10.0) {

      // MICRO zone: <6°
      if (ae < 6.0) {
        const int MICRO_STEPS    = 3;
        const int MICRO_KICK_MS  = 8;
        const int MICRO_HOLD_MS  = 20;
        const int MICRO_PAUSE_MS = 25;

        for (int i = 0; i < MICRO_STEPS; i++) {
          if (shouldStopNow()) return false;

          mpu.update();
          float err2 = norm180(targetAngle - mpu.getAngleZ());
          float ae2  = abs(err2);
          if (ae2 <= TOL) break;

          int cmd2 = (err2 >= 0) ? cmdMain : cmdOpp;
          if (overshot) cmd2 = cmdOpp;

          Motor(cmd2, FINE_PWM);
          if (!smartDelay(MICRO_KICK_MS)) return false;

          Motor(cmd2, MIN_PWM);
          if (!smartDelay(MICRO_HOLD_MS)) return false;

          Motor(Stop, 0);
          if (!smartDelay(MICRO_PAUSE_MS)) return false;

          mpu.update();
          float errAfter = norm180(targetAngle - mpu.getAngleZ());
          if ((errAfter >= 0) != (err2 >= 0)) break;
        }

        debugRotate(mpu.getAngleZ(), targetAngle,
                    norm180(targetAngle - mpu.getAngleZ()),
                    MIN_PWM, cmd);
        continue;
      }

      // NEAR zone: 6..10°
      const int NEAR_KICK_MS  = 12;
      const int NEAR_HOLD_MS  = 40;
      const int NEAR_PAUSE_MS = 35;

      Motor(cmd, FINE_PWM);
      if (!smartDelay(NEAR_KICK_MS)) return false;

      Motor(cmd, MIN_PWM);
      if (!smartDelay(NEAR_HOLD_MS)) return false;

      Motor(Stop, 0);
      if (!smartDelay(NEAR_PAUSE_MS)) return false;

      debugRotate(z, targetAngle, err, MIN_PWM, cmd);
      continue;
    }

    // MID zone: 10..25°
    if (ae < 25.0) {
      Motor(cmd, MID_PWM);
      if (!smartDelay(12)) return false;
      debugRotate(z, targetAngle, err, MID_PWM, cmd);
      continue;
    }

    // FAR zone: >=25°
    Motor(cmd, FAST_PWM);
    if (!smartDelay(10)) return false;
    debugRotate(z, targetAngle, err, FAST_PWM, cmd);
    continue;
  }
}

// ---------------- Optional ----------------

void detectYawSign() {
  mpu.update();
  float z0 = mpu.getAngleZ();

  Motor(RotateRight, max(MIN_ROT_PWM, 160));
  delay(350);
  Motor(Stop, 0);
  delay(150);

  mpu.update();
  float z1 = mpu.getAngleZ();
  float dz = norm180(z1 - z0);

  bool RIGHT_INCREASES_ANGLEZ = (dz > 0);
  Serial.print("Yaw sign test: dz=");
  Serial.print(dz, 2);
  Serial.print(" => RIGHT_INCREASES_ANGLEZ=");
  Serial.println(RIGHT_INCREASES_ANGLEZ ? "true" : "false");
}