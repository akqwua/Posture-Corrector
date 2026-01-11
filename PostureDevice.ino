#include <Wire.h>
#include <math.h>

struct Vec3 { float x, y, z; };
struct Cal  { Vec3 g0; float baseFlex; };

// ------------------ IMU Addresses ------------------
static const uint8_t ADDR_UPPER  = 0x68; // AD0 -> GND
static const uint8_t ADDR_PELVIS = 0x69; // AD0 -> VCC

// MPU registers
static const uint8_t REG_PWR_MGMT_1    = 0x6B;
static const uint8_t REG_WHO_AM_I      = 0x75;
static const uint8_t REG_ACCEL_XOUT_H  = 0x3B;

// ------------------ TUNABLES ------------------
static const float    DEFAULT_SLOUCH_CURV_THRESH_DEG = 12.0f; // fallback if bad-cal fails
static const uint32_t SLOUCH_HOLD_MS                 = 8000;  // 8 seconds hold

static const float    BEND_PELVIS_THRESH_DEG         = 20.0f; // suppress slouch while bending forward
static const uint32_t CAL_MS                         = 2000; // averaging time for GOOD
static const uint32_t BAD_CAL_MS                     = 2000; // averaging time for BAD

// Poor Posture Percentage
static const float    BAD_TO_THRESH_FRACTION          = 0.75f;

// ------------------ LED (2-color, common GND) ------------------
static const int LED_R = A3;
static const int LED_G = A2;

void ledOff()    { digitalWrite(LED_R, LOW);  digitalWrite(LED_G, LOW); }
void ledGreen()  { digitalWrite(LED_R, LOW);  digitalWrite(LED_G, HIGH); }
void ledRed()    { digitalWrite(LED_R, HIGH); digitalWrite(LED_G, LOW); }
void ledYellow() { digitalWrite(LED_R, HIGH); digitalWrite(LED_G, HIGH); } // R+G

// ------------------ Button ------------------
static const int BUTTON_PIN = A0;              // LOW when pressed (INPUT_PULLUP)
static const uint32_t BUTTON_DEBOUNCE_MS = 30; // debounce time
static const uint32_t RECAL_DELAY_MS = 1000;   // 1s after release (short-press)
static const uint32_t LONG_PRESS_MS  = 3000;   // 3s hold toggles ON/OFF

// Device state
bool deviceActive = false; // starts OFF; long-press turns it ON (and calibrates)

// Debounce state
bool btnStable = HIGH;
bool btnLastReading = HIGH;
uint32_t btnLastChangeMs = 0;

bool btnSawPress = false;
uint32_t pressStartMs = 0;
bool longPressFired = false;

// Recal scheduling (short press)
bool recalPending = false;
uint32_t recalDueMs = 0;

// ------------------ Personalized slouch threshold ------------------
float slouchCurvThreshDeg = DEFAULT_SLOUCH_CURV_THRESH_DEG; // gets set during calibration

// ------------------ I2C helpers ------------------
void i2cRecoverBus(); // forward declare

bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);

  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    i2cRecoverBus();
    return false;
  }

  if (got != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void i2cRecoverBus() {
  pinMode(SCL, OUTPUT);
  pinMode(SDA, INPUT_PULLUP);

  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
  }

  Wire.end();
  delay(5);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(25000, true);
}

int16_t be16(const uint8_t* b) { return (int16_t)((b[0] << 8) | b[1]); }

bool readAccelRaw(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t b[6];
  if (!i2cReadBytes(addr, REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
  ax = be16(&b[0]);
  ay = be16(&b[2]);
  az = be16(&b[4]);
  return true;
}

Vec3 normalizeAccel(int16_t ax, int16_t ay, int16_t az) {
  float x = ax / 16384.0f;
  float y = ay / 16384.0f;
  float z = az / 16384.0f;
  float n = sqrtf(x*x + y*y + z*z);
  if (n < 1e-6f) return {0,0,1};
  return {x/n, y/n, z/n};
}

float flexionAroundXDeg(const Vec3 &g0, const Vec3 &g) {
  float v0y = g0.y, v0z = g0.z;
  float vy  = g.y,  vz  = g.z;

  float n0 = sqrtf(v0y*v0y + v0z*v0z);
  float n  = sqrtf(vy*vy + vz*vz);
  if (n0 < 1e-6f || n < 1e-6f) return 0.0f;

  v0y /= n0; v0z /= n0;
  vy  /= n;  vz  /= n;

  float cross_x = v0y * vz - v0z * vy;
  float dot     = v0y * vy + v0z * vz;

  return atan2f(cross_x, dot) * 180.0f / PI;
}

// ------------------ Calibration ------------------
Cal calUpper, calPelvis;

void blinkGreenService(uint32_t &lastToggleMs, bool &on) {
  uint32_t now = millis();
  if (now - lastToggleMs >= 350) {
    lastToggleMs = now;
    on = !on;
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, on ? HIGH : LOW);
  }
}

void blinkRedService(uint32_t &lastToggleMs, bool &on) {
  uint32_t now = millis();
  if (now - lastToggleMs >= 350) {
    lastToggleMs = now;
    on = !on;
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_R, on ? HIGH : LOW);
  }
}

Cal calibrateOne(uint8_t addr, uint32_t ms) {
  Serial.print("Calibrating GOOD posture 0x");
  Serial.print(addr, HEX);
  Serial.println(" ... hold still.");

  uint32_t lastToggleMs = millis();
  bool ledOn = true;

  double sx=0, sy=0, sz=0;
  int count=0;
  uint32_t start = millis();

  while (millis() - start < ms) {
    int16_t ax, ay, az;
    if (readAccelRaw(addr, ax, ay, az)) {
      Vec3 g = normalizeAccel(ax, ay, az);
      sx += g.x; sy += g.y; sz += g.z;
      count++;
    }
    blinkGreenService(lastToggleMs, ledOn);
    delay(10);
  }

  Cal out;
  if (count == 0) {
    out.g0 = {0,0,1};
    out.baseFlex = 0;
    Serial.println("  Calibration failed (no reads).");
    return out;
  }

  out.g0 = { (float)(sx/count), (float)(sy/count), (float)(sz/count) };
  float n0 = sqrtf(out.g0.x*out.g0.x + out.g0.y*out.g0.y + out.g0.z*out.g0.z);
  out.g0 = { out.g0.x/n0, out.g0.y/n0, out.g0.z/n0 };

  int16_t ax, ay, az;
  readAccelRaw(addr, ax, ay, az);
  Vec3 g = normalizeAccel(ax, ay, az);
  out.baseFlex = flexionAroundXDeg(out.g0, g);

  Serial.print("  baseFlex=");
  Serial.println(out.baseFlex, 2);
  return out;
}

float dFlexFor(uint8_t addr, const Cal &cal) {
  int16_t ax, ay, az;
  if (!readAccelRaw(addr, ax, ay, az)) return 0.0f;
  Vec3 g = normalizeAccel(ax, ay, az);
  float flex = flexionAroundXDeg(cal.g0, g);
  return flex - cal.baseFlex;
}

// Measure "bad posture" curvature using the GOOD calibration as reference.
bool measureBadCurvature(uint32_t ms, float &outBadCurvDeg) {
  Serial.println("\nNow assume BAD posture (slouch) and hold still...");
  Serial.println("(Blinking RED while sampling bad posture)\n");

  uint32_t lastToggleMs = millis();
  bool ledOn = true;

  double sumCurv = 0.0;
  int count = 0;
  uint32_t start = millis();

  while (millis() - start < ms) {
    float upper  = dFlexFor(ADDR_UPPER,  calUpper);
    float pelvis = dFlexFor(ADDR_PELVIS, calPelvis);
    float curvature = upper - pelvis;

    // Only accept samples if we successfully read (dFlexFor returns 0 on a failed read;
    // if that's a concern, you can add a validity flag. For now, we just count all samples.)
    sumCurv += curvature;
    count++;

    blinkRedService(lastToggleMs, ledOn);
    delay(10);
  }

  if (count <= 0) return false;
  outBadCurvDeg = (float)(sumCurv / count);
  return true;
}

// ------------------ Slouch timing state ------------------
uint32_t slouchStartMs = 0;
bool slouching = false;

void resetSlouchTimer() { slouchStartMs = 0; slouching = false; }

void recalibrateAll() {
  Serial.println("\n=== Calibration: GOOD then BAD ===");
  Serial.println("Step 1) Hold GOOD posture still...");

  calUpper  = calibrateOne(ADDR_UPPER,  CAL_MS);
  calPelvis = calibrateOne(ADDR_PELVIS, CAL_MS);

  ledYellow();
  delay(2000);

  // Step 2: BAD posture measurement (curvature reference)
  float badCurv = 0.0f;
  bool ok = measureBadCurvature(BAD_CAL_MS, badCurv);

  if (!ok) {
    slouchCurvThreshDeg = DEFAULT_SLOUCH_CURV_THRESH_DEG;
    Serial.println("Bad-posture measurement failed. Using default slouch threshold.");
  } else {
    // If badCurv is negative (weird orientation), use magnitude
    float badMag = fabsf(badCurv);

    // Compute personalized threshold from bad posture curvature
    float t = badMag * BAD_TO_THRESH_FRACTION;

    // Clamp to sane bounds so one weird sample doesn't break it
    if (t < 6.0f)  t = 6.0f;
    if (t > 35.0f) t = 35.0f;

    slouchCurvThreshDeg = t;

    Serial.print("Measured BAD curvature avg = ");
    Serial.println(badCurv, 2);
    Serial.print("Set personalized slouch threshold = ");
    Serial.print(slouchCurvThreshDeg, 2);
    Serial.println(" deg");
  }

  resetSlouchTimer();
  ledGreen();
  Serial.println("=== Done. Monitoring... ===\n");
}

void shutdownDevice() {
  recalPending = false;
  resetSlouchTimer();
  ledOff();
  Serial.println("\n=== Device OFF ===\n");
}

void toggleDeviceOnOff() {
  deviceActive = !deviceActive;
  if (deviceActive) {
    Serial.println("\n=== Device ON: starting calibration ===");
    recalibrateAll();
  } else {
    shutdownDevice();
  }
}

// ------------------ Button service ------------------
void buttonService() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != btnLastReading) {
    btnLastReading = reading;
    btnLastChangeMs = millis();
  }

  if ((millis() - btnLastChangeMs) > BUTTON_DEBOUNCE_MS) {
    if (btnStable != reading) {
      btnStable = reading;

      if (btnStable == LOW) {
        btnSawPress = true;
        pressStartMs = millis();
        longPressFired = false;
      } else {
        if (btnSawPress) {
          btnSawPress = false;

          if (longPressFired) {
            toggleDeviceOnOff();
          } else {
            if (deviceActive) {
              recalPending = true;
              recalDueMs = millis() + RECAL_DELAY_MS;
              ledYellow();
              Serial.println("Button released: recalibration scheduled in 1s...");
            }
          }
        }
      }
    }
  }

  if (btnStable == LOW && btnSawPress && !longPressFired) {
    if (millis() - pressStartMs >= LONG_PRESS_MS) {
      longPressFired = true;
      ledYellow();
      Serial.println("Long press detected: release to toggle ON/OFF...");
    }
  }
}

void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  ledOff();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(500);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(25000, true);

  uint8_t who = 0;
  if (i2cReadBytes(ADDR_UPPER, REG_WHO_AM_I, &who, 1)) {
    Serial.print("Upper WHO_AM_I=0x"); Serial.println(who, HEX);
  } else Serial.println("Upper IMU not responding at 0x68");

  if (i2cReadBytes(ADDR_PELVIS, REG_WHO_AM_I, &who, 1)) {
    Serial.print("Pelvis WHO_AM_I=0x"); Serial.println(who, HEX);
  } else Serial.println("Pelvis IMU not responding at 0x69");

  i2cWriteByte(ADDR_UPPER,  REG_PWR_MGMT_1, 0x00);
  i2cWriteByte(ADDR_PELVIS, REG_PWR_MGMT_1, 0x00);

  delay(200);

  Serial.println("Device starts OFF.");
  Serial.println("Hold button 3s, then release to turn ON (GOOD then BAD calibration).");
  Serial.println("Hold button 3s again to turn OFF.");
  Serial.println("Short press (when ON) recalibrates after 1s (GOOD then BAD).");
  Serial.println("Commands: type 'c' to recalibrate (forces ON).");
}

void loop() {
  buttonService();

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') {
      deviceActive = true;
      recalPending = false;
      recalibrateAll();
    }
  }

  if (!deviceActive) {
    delay(20);
    return;
  }

  if (recalPending) {
    if ((int32_t)(millis() - recalDueMs) >= 0) {
      recalPending = false;
      recalibrateAll();
    } else {
      delay(10);
    }
    return;
  }

  float upper  = dFlexFor(ADDR_UPPER,  calUpper);
  float pelvis = dFlexFor(ADDR_PELVIS, calPelvis);
  float curvature = upper - pelvis;

  uint32_t now = millis();
  bool bending = (pelvis > BEND_PELVIS_THRESH_DEG);

  if (bending) {
    resetSlouchTimer();
    ledGreen();
  } else {
    if (curvature > slouchCurvThreshDeg) {
      if (slouchStartMs == 0) slouchStartMs = now;
      if ((now - slouchStartMs) >= SLOUCH_HOLD_MS) slouching = true;
    } else {
      resetSlouchTimer();
    }

    if (curvature <= slouchCurvThreshDeg) {
      ledGreen();
    } else if (!slouching) {
      ledYellow();
    } else {
      ledRed();
    }
  }

  Serial.print("upper=");
  Serial.print(upper, 1);
  Serial.print(" pelvis=");
  Serial.print(pelvis, 1);
  Serial.print(" curvature=");
  Serial.print(curvature, 1);
  Serial.print(" thresh=");
  Serial.print(slouchCurvThreshDeg, 1);
  Serial.print(" | LED=");
  Serial.println((curvature <= slouchCurvThreshDeg) ? "GREEN" : (slouching ? "RED" : "YELLOW"));

  delay(50);
}
