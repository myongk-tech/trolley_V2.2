/**
 * DD Trolley Motor Control
 * version 12.18
 *
 * ESC changed from HobbyWing FPV G2 (one-directional, LEDC 50 Hz, 1000–2000 µs)
 * to MAXYNOS 70A AM32 (bidirectional, ESP32Servo 80 Hz, 1250–1750 µs, neutral 1500 µs).
 *   Arm signal : 1500 µs (neutral) for 3 s
 *   Forward    : 1550 µs (just past deadband) → 1750 µs (100 %)
 *   Reverse    : 1450 µs (just past deadband) → 1250 µs (100 %)
 *   Deadband   : ±50 µs around neutral (1450–1550 µs); skipped instantly by ramp engine
 * All other functions (encoder, line sensor, servos, IMU, stations, web UI) unchanged.
 *
 * Encoder counts POSITIVELY as the motor runs forward (encInvert=false by default).
 * Station locations are user-defined positive encoder counts set through the web UI.
 * No line sensor or pattern detection.  Approach is purely encoder-count based:
 *   - Motor runs at throttle speed toward the target station.
 *   - When (targetCount - encCount) < approachZoneCount, ramp to approachSlowPct.
 *   - When encCount >= targetCount, hard stop.
 *
 * Encoder reading runs in a dedicated FreeRTOS task (encoderTask) pinned to core 0.
 * This guarantees a hard 5 ms sample period regardless of WiFi / web-server activity
 * on core 1, preventing the wrap-detection aliasing that causes count errors at speed.
 * Shared output variables (encCount, positionM, velocityRPM, velocityMs) are protected
 * by a portMUX_TYPE spinlock (encMux).  All other encoder internals are private to the
 * task (static locals) so no other synchronisation is needed.
 *
 * Line sensor (TCRT5000 active-LOW on D5) runs in its own FreeRTOS task (lineSensorTask)
 * also pinned to core 0 at 5 ms period.  The ONLY stop condition is a 1→0 transition
 * (HIGH→LOW, i.e. sensor newly detects a line) while the trolley is inside the approach
 * zone.  When that edge is detected the task sets lineStopPending = true; the main loop
 * reads this flag in updateGotoApproach() and calls onStationArrival() immediately.
 *
 * Board   : Arduino Nano ESP32  (ESP32-S3 core)
 * WiFi    : AP  SSID=dd_trolley  PW=yay!y@y!   URL: http://192.168.4.1
 *
 * Pin map:
 *   D3  — Line sensor input (TCRT5000, active-LOW)
 *   D6  — NeoPixel LED strip data
 *   D9  — ESC PWM (MAXYNOS 70A AM32, 80 Hz, bidirectional)
 *   D2  — DS3245 servo PWM
 *   D0  — Serial1 RX  (STS3215 via URT-1)
 *   D1  — Serial1 TX
 *   A0  — Current sense  I = V_A0 / 0.01175
 *   D10 — Encoder CS  (AS5047P SPI)
 *   D11 — MOSI / D12 — MISO / D13 — SCK
 *   SDA / SCL — BNO055 I2C
 *   SDA - A4 / SCL - A5
 *
 * ESC MAXYNOS 70A AM32 (bidirectional):
 *   1500 µs = neutral / arm    1250 µs = full reverse    1750 µs = full forward
 *   Deadband ±50 µs (1450–1550 µs); AM32 firmware ignores pulses in this zone
 *
 * Servo behaviour:
 *   STS3215 (URT-1 half-duplex):
 *     Drive Thru arrival  → rotates to 180° and re-zeros the encoder after stop
 *     Any Goto command    → returns to 0° first (SERVO_RETURN_WAIT_MS delay), then motor starts
 *   DS3245 (PWM):
 *     Independent auxiliary servo, optionally stabilized from a selectable BNO055 Euler axis
 *
 * Libraries: ESP32Servo, SMS_STS, Adafruit BNO055, Adafruit Unified Sensor,
 *            Adafruit NeoPixel, Preferences (built-in)
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <ESP32Servo.h>
#define DS_SERVO_LIB_NAME "ESP32Servo.h"  // Think of it like a sticky note. The #include actually brings the tools into the room; the #define just writes the name of the toolset on a piece of paper so other parts of the code can refer to it later.
#define DS_SERVO_USING_ARDUINO_SERVO 0
#include <SMS_STS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>

// ═══════════════════════════════════════════════════════════════════════════
//  KALMAN FILTER
//  Unified interface so both legacy trolley code and the DS3245/BNO055 code
//  can share the same implementation.
// ═══════════════════════════════════════════════════════════════════════════
class KalmanFilter {
private:
  float Q;
  float R;
  float P;
  float K;
  float X;
  bool  ready;

public:
  KalmanFilter(float processNoise=0.01f,
               float measurementNoise=0.5f,
               float estimationError=1.0f,
               float initialValue=0.0f)
  : Q(processNoise), R(measurementNoise), P(estimationError),
    K(0.0f), X(initialValue), ready(false) {}

  void configure(float q, float r, float p=1.0f) {
    Q = q;
    R = r;
    P = p;
  }

  // Legacy trolley-style init(q, r, x0)
  void init(float q, float r, float x0) {
    Q = q;
    R = r;
    P = 1.0f;
    K = 0.0f;
    X = x0;
    ready = true;
  }

  // Servo/IMU-style init(initialValue)
  void init(float initialValue) {
    X = initialValue;
    P = 1.0f;
    K = 0.0f;
    ready = true;
  }

  float update(float measurement) {
    if (!ready) {
      X = measurement;
      ready = true;
      return X;
    }
    P += Q;
    K = P / (P + R);
    X += K * (measurement - X);
    P *= (1.0f - K);
    return X;
  }

  void reset(float x0=0.0f) {
    X = x0;
    P = 1.0f;
    K = 0.0f;
    ready = false;
  }

  float getValue() const { return X; }
  void setProcessNoise(float q) { Q = q; }
  void setMeasurementNoise(float r) { R = r; }
};

// ── Enums ─────────────────────────────────────────────────────────────────
enum StationType { ST_NONE=0, ST_HOME=1, ST_DRIVE_THRU=2, ST_DRONE_PICKUP=3 };

// ── Station record ────────────────────────────────────────────────────────
constexpr uint8_t MAX_STATIONS = 10;
struct Station {
  char        name[24];
  StationType type;
  int32_t     encStopCount;
  bool        active;
};

// ── WiFi ──────────────────────────────────────────────────────────────────
static const char* AP_SSID     = "dd_trolley";
static const char* AP_PASSWORD = "yay!y@y!";

// ── Edge IO block (kept physically adjacent on the header) ───────────────
// constexpr int      LINE_SENSOR_PIN      = D3;
// constexpr int      LED_PIN              = D4;
// constexpr int      ESC_PIN              = D9;
// constexpr int      SERVO_PWM_PIN        = D2;

constexpr int      LINE_SENSOR_PIN      = 6;
constexpr int      LED_PIN              = 7;
constexpr int      ESC_PIN              = D9;
constexpr int      SERVO_PWM_PIN        = 5;

// ── NeoPixel LED strip ────────────────────────────────────────────────────
constexpr uint8_t  LED_NUMPIXELS        = 20;
constexpr uint32_t LED_STEP_INTERVAL_MS = 40;
constexpr uint32_t LED_FLASH_HALF_MS    = 250;  // 2 Hz full flash cycle

// ── ESC — MAXYNOS 70A AM32 (raw LEDC, bidirectional) ─────────────────────
// Raw LEDC is used deliberately instead of ESP32Servo so the ESC does NOT
// consume a second hardware timer from the ESP32Servo pool.  The DS3245
// servo already uses one timer from that pool; a second Servo object at a
// different frequency (80 Hz vs ~50 Hz) must claim its own dedicated timer,
// and with all four ESP32-S3 LEDC timers pre-allocated to the pool via
// allocateTimer(0-3) the conflict corrupts the LEDC peripheral and breaks
// the WiFi coexistence hardware that depends on it.
constexpr uint32_t ESC_PWM_FREQ_HZ       = 50;    // 50 Hz: proven stable, AM32 accepts it
constexpr uint8_t  ESC_PWM_RESOLUTION    = 14;
// LEDC channel 7 is pinned to hardware timer 3 on the ESP32-S3
// (channels 6 & 7 share timer 3; channels 0-5 use timers 0-2).
// Timer 3 is NOT added to the ESP32Servo allocateTimer pool, so no
// servo attach() can ever reconfigure it.  The DS3245 servo uses
// timers 0-2 via the pool — completely separate silicon.
// Channel 0 is never used by raw LEDC (ESP32Servo claims it for DS3245).
constexpr uint8_t  ESC_LEDC_CHANNEL      = 7;
constexpr uint16_t THROTTLE_NEUTRAL_US   = 1500;   // arm / stop signal
constexpr uint16_t THROTTLE_FULL_FWD_US  = 1750;   // 100 % forward
constexpr uint16_t THROTTLE_FULL_REV_US  = 1250;   // 100 % reverse
constexpr uint16_t THROTTLE_DEADBAND_US  = 50;     // ±deadband around neutral
constexpr uint16_t THROTTLE_START_FWD_US = THROTTLE_NEUTRAL_US + THROTTLE_DEADBAND_US;  // 1550
constexpr uint16_t THROTTLE_START_REV_US = THROTTLE_NEUTRAL_US - THROTTLE_DEADBAND_US;  // 1450
constexpr uint32_t ARM_TIME_MS           = 3000;

// ── Ramp ──────────────────────────────────────────────────────────────────
constexpr uint32_t RAMP_INTERVAL_MS = 10;
constexpr uint32_t RAMP_TIME_DEF_MS = 5000;

// ── Current sense ─────────────────────────────────────────────────────────
constexpr int      CURRENT_PIN         = A0;
constexpr float    ADC_RESOLUTION      = 4095.0f;
constexpr float    ADC_VREF            = 3.3f;
constexpr float    CURRENT_SENSE_OHMS  = 0.01175f;
constexpr uint32_t CURRENT_INTERVAL_MS = 50;
constexpr uint8_t  ADC_SAMPLES         = 8;

// ── AS5047P ───────────────────────────────────────────────────────────────
constexpr int      ENC_CS_PIN         = D10;
constexpr uint32_t ENC_SPI_HZ         = 5000000UL;
constexpr uint16_t AS5047_ANGLECOM    = 0x3FFF;
constexpr uint16_t AS5047_NOP         = 0x0000;
constexpr uint16_t AS5047_CPR_DEFAULT = 16384;
// Period for the encoder FreeRTOS task (core 0).  Must be short enough that
// the motor never travels more than CPR/2 counts between reads at full speed.
constexpr uint32_t ENC_TASK_PERIOD_MS = 5;

// ── Line sensor (TCRT5000, active-LOW) ───────────────────────────────────
// Same period as encoder task — matched so the sensor is never the slow path.
constexpr uint32_t LINE_TASK_PERIOD_MS   = 5;

// ── STS3215 servo (URT-1 half-duplex over Serial1) ───────────────────────
constexpr uint8_t  STS_SERVO_ID             = 1;
constexpr uint32_t STS_SERVO_BAUD           = 1000000UL;
constexpr int      STS_SERVO_UART_RX        = D0;
constexpr int      STS_SERVO_UART_TX        = D1;
constexpr uint16_t STS_SERVO_POS_0DEG_DEF   = 75;
constexpr uint16_t STS_SERVO_POS_180DEG_DEF = 2150;
constexpr uint16_t STS_SERVO_SPEED          = 1500;
constexpr uint8_t  STS_SERVO_ACC            = 50;
// STS3215 at STS_SERVO_SPEED=1500 steps/s travels 2048 steps in ~1.37 s.
constexpr uint32_t SERVO_RETURN_WAIT_MS     = 1500;

// ── DS3245 servo (PWM) ────────────────────────────────────────────────────
constexpr uint16_t DS_SERVO_MIN_PULSE_US     = 500;
constexpr uint16_t DS_SERVO_MAX_PULSE_US     = 2500;
constexpr uint16_t DS_SERVO_CENTER_PULSE_US  = 1500;
constexpr uint16_t DS_SERVO_PULSE_0DEG_DEF   = 680;
constexpr uint16_t DS_SERVO_PULSE_180DEG_DEF = 1960;

// ── BNO055 IMU ────────────────────────────────────────────────────────────
constexpr uint8_t  IMU_I2C_ADDR          = 0x28;
constexpr uint32_t IMU_SAMPLE_RATE_MS    = 10;
constexpr uint8_t  IMU_CAL_SAMPLES       = 60;
constexpr uint16_t IMU_CAL_DELAY_MS      = 15;
constexpr float    IMU_CAL_STABILITY_DEG = 2.0f;
constexpr float    IMU_ROLL_MIN_DEG      = -90.0f;
constexpr float    IMU_ROLL_MAX_DEG      =  90.0f;
constexpr int      IMU_SERVO_ANGLE_MIN   = 0;
constexpr int      IMU_SERVO_ANGLE_MAX   = 180;
constexpr int      DS_SERVO_NEUTRAL_DEG  = 90;

// ── User-adjustable defaults ──────────────────────────────────────────────
constexpr float    MAX_SPEED_PCT_DEF       = 12.0f;
constexpr float    APPROACH_SLOW_PCT_DEF   = 6.0f;   // speed inside approach zone
constexpr int32_t  APPROACH_ZONE_DEF_COUNTS = 3000000; // enc counts from target to start slowing
constexpr float    DRIVE_RATIO_DEF         = 17.07f;
constexpr bool     ENC_INVERT_DEF          = true;
constexpr float    DEMO_THROTTLE_PCT_DEF   = 12.0f;  // demo cruise speed
constexpr uint32_t DEMO_BOARDING_WAIT_MS   = 4000;   // STS3215 at 180 before returning to 0
constexpr uint32_t DEMO_DRONE_WAIT_MS      = 10000;  // wait at drone-pickup before next leg

// ═══════════════════════════════════════════════════════════════════════════
//  GLOBAL STATE
// ═══════════════════════════════════════════════════════════════════════════
bool     escReady=false, arming=false, userArmed=false, motorOn=false;
bool     motorReverse=false;

uint8_t  throttlePct  = 0;
float    currentUsF   = (float)THROTTLE_NEUTRAL_US;
uint32_t rampTimeMs   = RAMP_TIME_DEF_MS;
float    maxSpeedPct  = MAX_SPEED_PCT_DEF;
float    approachSlowPct    = APPROACH_SLOW_PCT_DEF;
int32_t  approachZoneCount  = APPROACH_ZONE_DEF_COUNTS;

// Current sense
float rawCurrentA=0.f, filtCurrentA=0.f;

// Encoder — outputs shared with main loop (protected by encMux)
// Internal state (encLastRaw, encInitDone, encLastCount) lives inside encoderTask as
// static locals so no synchronisation is needed for them.
portMUX_TYPE encMux        = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t encTaskHandle = nullptr;

bool     encInvert   = false;  // false = no inversion; forward motor rotation = positive counts
                               // true  = invert raw diff (use if encoder counts backwards)
int32_t  encCount    = 0;
float    positionM   = 0.f;
float    velocityRPM = 0.f, velocityMs = 0.f;
volatile bool encoderZeroPending = false;

// Line sensor — shared between lineSensorTask (core 0) and main loop (core 1).
// Both variables are plain bool (byte-wide); the ESP32-S3 guarantees atomic
// byte reads/writes so no spinlock is needed.  volatile prevents the compiler
// from caching them in registers across the core boundary.
TaskHandle_t     lineSensorTaskHandle = nullptr;
volatile bool    lineStopPending      = false; // task sets; main loop clears + acts
volatile bool    lineCurrentState     = true;  // true = HIGH = "1" (no line)

// Machine params
uint16_t countsPerRev = AS5047_CPR_DEFAULT;
float    driveRatio   = 1.0f;
float    pitchDiamM   = 0.10f;

// Goto / approach state
int      gotoStationIdx    = -1;   // -1 = not navigating
bool     approachSlowing   = false; // true once inside approach zone
int32_t  gotoTargetCount   = 0;    // absolute encoder count to stop at
int32_t  gotoMinTravelCount = 0;   // must pass this count before approach zone activates

// Stations
Station stations[MAX_STATIONS];
uint8_t stationCount = 0;

// Servo / IMU / return-to-zero before motion
bool     servoReturnPending = false;
uint32_t servoReturnStartMs = 0;
int      pendingGotoIdx     = -1;

// ── Demo automation state machine ────────────────────────────────────────
// The demo button on the home page chains a fixed routine:
//   1. Re-zero IMU + zero encoder
//   2. STS3215 to 180° (boarding pose), hold for 4 s
//   3. STS3215 back to 0°, wait for travel time
//   4. Goto Drone Pickup station (uses normal approach-zone slow-down)
//   5. Hold at Drone Pickup for 10 s (countdown reported to UI)
//   6. Goto Drive Thru station — onStationArrival() spins STS to 180° and
//      re-zeros the encoder, then the demo returns to idle awaiting another
//      press of the demo button.
enum DemoState : uint8_t {
  DEMO_IDLE = 0,
  DEMO_IMU_REZERO,         // waiting for IMU calibration to finish
  DEMO_BOARDING_180,       // STS at 180°, holding 4 s
  DEMO_BOARDING_RETURN,    // STS commanded back to 0°, waiting for travel
  DEMO_GOTO_DRONE,         // motion in progress to drone-pickup station
  DEMO_AT_DRONE_WAIT,      // stopped at drone pickup, 10 s countdown
  DEMO_GOTO_DRIVETHRU,     // motion in progress to drive-thru station
};

DemoState demoState           = DEMO_IDLE;
uint32_t  demoStateStartMs    = 0;
uint32_t  demoStateDurationMs = 0;   // for timed waits — reported to UI
int       demoDroneStationIdx = -1;
int       demoDriveThruIdx    = -1;

// Timers
uint32_t bootArmStartMs=0, userArmStartMs=0;
uint32_t lastRampMs=0, lastCurrentMs=0;

WebServer server(80);

// STS3215: station-management servo via URT-1 half-duplex
SMS_STS   stsServo;
uint16_t  stsServoCmdPos   = STS_SERVO_POS_0DEG_DEF;
uint16_t  stsServoPos0     = STS_SERVO_POS_0DEG_DEF;
uint16_t  stsServoPos180   = STS_SERVO_POS_180DEG_DEF;
bool      stsServoEnabled  = false;

// DS3245: auxiliary PWM servo for IMU-linked function
Servo     ds3245Servo;
int       dsServoCmdDeg    = 0;
uint16_t  dsServoPulse0    = DS_SERVO_PULSE_0DEG_DEF;
uint16_t  dsServoPulse180  = DS_SERVO_PULSE_180DEG_DEF;
uint16_t  dsServoPulseUs   = DS_SERVO_CENTER_PULSE_US;
bool      dsServoEnabled   = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55, IMU_I2C_ADDR, &Wire);
enum ImuAxis : uint8_t { IMU_AXIS_X = 0, IMU_AXIS_Y = 1, IMU_AXIS_Z = 2 };
bool      imuReady             = false;
bool      imuCalibrated        = false;
bool      imuCalibrating       = false;
bool      imuStabilizeEnabled  = true;
ImuAxis   imuAxisSelect        = IMU_AXIS_Y;
float     imuAxisOffsetDeg     = 0.0f;
float     imuRollOffsetDeg     = 0.0f; // alias retained for older code / prefs compatibility
float     imuRollRawDeg        = 0.0f;
float     imuRollFiltDeg       = 0.0f;
float     imuEulerXDeg         = 0.0f;
float     imuEulerYDeg         = 0.0f;
float     imuEulerZDeg         = 0.0f;
uint32_t  lastImuMs            = 0;
uint8_t   imuCalSampleIndex    = 0;
float     imuCalSumDeg         = 0.0f;
float     imuCalSumSqDeg       = 0.0f;
uint32_t  imuCalNextSampleMs   = 0;

KalmanFilter currentKF, velocityKF, imuRollKF;
Preferences  prefs;

// ── LED state ─────────────────────────────────────────────────────────────
enum LedMode : uint8_t {
  LED_MODE_OFF = 0,
  LED_MODE_RUN_GREEN,
  LED_MODE_APPROACH_YELLOW,
  LED_MODE_STOP_RED,
};

Adafruit_NeoPixel ledPixels(LED_NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
LedMode   ledMode   = LED_MODE_OFF;
uint32_t  lastLedMs = 0;
int       ledStep   = 0;
bool      ledFlashOn = false;
bool      ledStopLatched = false;

// ═══════════════════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════
static void requestEncoderZero();
void onStationArrival(int idx);
void updateGotoApproach();
void startGotoMotion(int idx);
void savePrefs();
void loadPrefs();
static void stsServoMove(uint16_t pos, const char* reason);
static uint16_t dsServoDegToPulseUs(int deg);
static void dsServoWriteAngleInternal(int deg);
static void dsServoMove(int deg, const char* reason);
static bool initIMU();
static void readImuEulerDeg(float& xDeg, float& yDeg, float& zDeg);
static void readImuGravityTiltDeg(float& xTiltDeg, float& yTiltDeg);
static const char* imuAxisName(ImuAxis axis);
static float readImuControlAxisDeg();
static void beginIMUCalibration();
static void updateIMUCalibration();
static void updateIMU();
static bool dsServoAttachOrReattach();
void updateLEDs();
static int findFirstStationOfType(StationType t);
static bool startDemo();
static void cancelDemo(const char* reason);
static void updateDemo();
static const char* demoStateName(DemoState s);

// ═══════════════════════════════════════════════════════════════════════════
//  ESC HELPERS  (MAXYNOS 70A AM32 — bidirectional, raw LEDC)
// ═══════════════════════════════════════════════════════════════════════════
static uint32_t pulseUsToDuty(uint16_t us) {
  const uint32_t mx = (1UL << ESC_PWM_RESOLUTION) - 1UL;
  return ((uint32_t)us * mx + 10000UL) / 20000UL;
}
static void writeEscPulseUs(uint16_t us) {
  const uint32_t d = pulseUsToDuty(us);
  // v3 API: ledcWrite(pin, duty)  — '7' would be GPIO-7, not channel 7!
  // v2 API: ledcWrite(channel, duty) — channel 7 is correct.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(ESC_PIN, d);
#else
  ledcWrite(ESC_LEDC_CHANNEL, d);
#endif
}

// Map 0–maxSpeedPct % to µs, skipping the ±50 µs deadband around neutral.
//   pct==0           → 1500 µs (neutral / stop)
//   pct>0, rev=false → 1550–1750 µs (forward)
//   pct>0, rev=true  → 1450–1250 µs (reverse)
static uint16_t pctToUs(float pct, bool rev=false) {
  pct = constrain(pct, 0.0f, maxSpeedPct);
  if (pct == 0.0f) return THROTTLE_NEUTRAL_US;
  if (!rev) {
    return (uint16_t)(THROTTLE_START_FWD_US +
      (pct / 100.0f) * (float)(THROTTLE_FULL_FWD_US - THROTTLE_START_FWD_US));
  } else {
    return (uint16_t)(THROTTLE_START_REV_US -
      (pct / 100.0f) * (float)(THROTTLE_START_REV_US - THROTTLE_FULL_REV_US));
  }
}

// Returns signed speed % corresponding to currentUsF (−100..+100).
static int rampedPct() {
  const int pwm = (int)currentUsF;
  if (pwm == THROTTLE_NEUTRAL_US) return 0;
  if (pwm > THROTTLE_NEUTRAL_US)
    return (int)map(pwm, THROTTLE_NEUTRAL_US, THROTTLE_FULL_FWD_US,  0,  100);
  else
    return (int)map(pwm, THROTTLE_NEUTRAL_US, THROTTLE_FULL_REV_US,  0, -100);
}

// ═══════════════════════════════════════════════════════════════════════════
//  RAMP ENGINE
//  Priority (highest first):
//    1. approachSlowing  — inside approach zone: ramp to approachSlowPct
//    2. normal throttle  — userArmed && motorOn
//    3. stopped
// ═══════════════════════════════════════════════════════════════════════════
void updateRamp() {
  const uint32_t now = millis();
  if (now - lastRampMs < RAMP_INTERVAL_MS) return;
  const float dt = (float)(now - lastRampMs);
  lastRampMs = now;

  float targetUs;
  if (approachSlowing && userArmed && motorOn) {
    targetUs = (float)pctToUs(approachSlowPct, false);
  } else if (userArmed && motorOn) {
    targetUs = (float)pctToUs(throttlePct, motorReverse);
  } else {
    targetUs = (float)THROTTLE_NEUTRAL_US;   // MAXYNOS stop = neutral (1500 µs)
  }

  if (currentUsF == targetUs) return;
  const float step = (1000.0f / (float)rampTimeMs) * dt;
  const float diff = targetUs - currentUsF;
  currentUsF = (fabsf(diff) <= step) ? targetUs : currentUsF + (diff>0?step:-step);
  currentUsF = constrain(currentUsF, (float)THROTTLE_FULL_REV_US, (float)THROTTLE_FULL_FWD_US);

  // Skip through the AM32 deadband instantly so the motor responds without
  // hesitation at the start of a move and stops cleanly without hunting.
  if (targetUs > (float)THROTTLE_NEUTRAL_US &&
      currentUsF > (float)THROTTLE_NEUTRAL_US &&
      currentUsF < (float)THROTTLE_START_FWD_US)
    currentUsF = (float)THROTTLE_START_FWD_US;
  if (targetUs < (float)THROTTLE_NEUTRAL_US &&
      currentUsF < (float)THROTTLE_NEUTRAL_US &&
      currentUsF > (float)THROTTLE_START_REV_US)
    currentUsF = (float)THROTTLE_START_REV_US;
  writeEscPulseUs((uint16_t)currentUsF);
}

// ═══════════════════════════════════════════════════════════════════════════
//  CURRENT SENSE
// ═══════════════════════════════════════════════════════════════════════════
void updateCurrent() {
  const uint32_t now = millis();
  if (now - lastCurrentMs < CURRENT_INTERVAL_MS) return;
  lastCurrentMs = now;
  uint32_t sum=0;
  for (uint8_t i=0;i<ADC_SAMPLES;i++) sum+=analogRead(CURRENT_PIN);
  const float v = ((float)sum/(float)ADC_SAMPLES/ADC_RESOLUTION)*ADC_VREF;
  rawCurrentA  = v / CURRENT_SENSE_OHMS;
  filtCurrentA = currentKF.update(rawCurrentA);
}

static void requestEncoderZero() {
  taskENTER_CRITICAL(&encMux);
  encCount = 0;
  taskEXIT_CRITICAL(&encMux);
  positionM = 0.0f;
  velocityRPM = 0.0f;
  velocityMs = 0.0f;
  velocityKF.reset(0.0f);
  encoderZeroPending = true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  AS5047P SPI DRIVER
// ═══════════════════════════════════════════════════════════════════════════
static uint16_t spiAddParity(uint16_t w) {
  uint16_t p=w&0x7FFF; p^=p>>8; p^=p>>4; p^=p>>2; p^=p>>1;
  return (p&1)?(w|0x8000):(w&0x7FFF);
}
static uint16_t spiFrame(uint16_t tx) {
  uint8_t h=(uint8_t)(tx>>8), l=(uint8_t)(tx&0xFF);
  digitalWrite(ENC_CS_PIN,LOW); h=SPI.transfer(h); l=SPI.transfer(l); digitalWrite(ENC_CS_PIN,HIGH);
  return ((uint16_t)h<<8)|l;
}
// lastGood = the caller's most recent valid angle, returned unchanged if the
// AS5047P sets its error flag (bit 14).  Passed by the caller so this function
// has no dependency on the global/task-local encLastRaw.
static uint16_t readAS5047Angle(uint16_t lastGood) {
  const SPISettings cfg(ENC_SPI_HZ,MSBFIRST,SPI_MODE1);
  SPI.beginTransaction(cfg);
  spiFrame(spiAddParity(AS5047_ANGLECOM));
  uint16_t resp=spiFrame(spiAddParity(AS5047_NOP));
  SPI.endTransaction();
  if (resp&0x4000) return lastGood;   // error bit set — hold last good reading
  return resp&0x3FFF;
}

// ═══════════════════════════════════════════════════════════════════════════
//  ENCODER TASK — runs on core 0, period = ENC_TASK_PERIOD_MS
//
//  Pinning to core 0 isolates the sample loop from WiFi/TCP activity which
//  lives on core 1 alongside the Arduino loop().  vTaskDelayUntil() gives a
//  hard periodic wakeup: if the previous iteration ran long (e.g. SPI bus
//  contention) the scheduler compensates so the *average* period stays exact.
//
//  Why this fixes the count errors:
//    The AS5047P returns a 14-bit angle (0–16383).  Consecutive reads are
//    unwrapped by comparing the signed difference to ±CPR/2.  If the gap
//    between reads is long enough for the motor to travel ≥8192 counts the
//    unwrap logic folds in the wrong direction, producing a ±16384-count
//    spike.  At high speed a web-server stall of just a few ms is enough to
//    cross that threshold.  A dedicated task on its own core eliminates the
//    stall entirely.
//
//  Shared outputs: encCount, positionM, velocityRPM, velocityMs
//    encCount is written under encMux (spinlock).  The three floats are each
//    naturally 32-bit aligned; the ESP32-S3 (Xtensa LX7) guarantees atomic
//    32-bit loads/stores to aligned addresses, so the main loop can read them
//    without locking.  The zeroenc command takes encMux before writing encCount
//    so it never races the task mid-accumulation.
//
//  Private state (static locals — no locking needed):
//    encLastRaw, encInitDone, encLastCount
// ═══════════════════════════════════════════════════════════════════════════
void encoderTask(void* /*pvParameters*/) {
  static int16_t  encLastRaw   = 0;
  static bool     encInitDone  = false;
  static int32_t  encLastCount = 0;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(ENC_TASK_PERIOD_MS);

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    const float dt_s = (float)ENC_TASK_PERIOD_MS / 1000.f;

    // Pass encLastRaw as the fallback so readAS5047Angle() needs no globals.
    const uint16_t raw = readAS5047Angle((uint16_t)encLastRaw);

    if (encoderZeroPending) {
      taskENTER_CRITICAL(&encMux);
      encCount = 0;
      taskEXIT_CRITICAL(&encMux);
      encLastRaw = (int16_t)raw;
      encLastCount = 0;
      positionM = 0.0f;
      velocityRPM = 0.0f;
      velocityMs = 0.0f;
      velocityKF.reset(0.0f);
      encoderZeroPending = false;
      encInitDone = true;
      continue;
    }

    if (!encInitDone) {
      encLastRaw   = (int16_t)raw;
      encLastCount = encCount;   // atomic 32-bit read
      encInitDone  = true;
      continue;
    }

    // Unwrap 14-bit angle — valid as long as the task period is short enough
    // that the motor never travels ≥CPR/2 counts between reads.
    int16_t diff = (int16_t)raw - encLastRaw;
    const int16_t half = (int16_t)(countsPerRev / 2);
    if (diff >  half) diff -= (int16_t)countsPerRev;
    if (diff < -half) diff += (int16_t)countsPerRev;
    if (encInvert) diff = -diff;

    encLastRaw = (int16_t)raw;

    // Critical section: only encCount += diff needs protection.
    taskENTER_CRITICAL(&encMux);
    encCount += diff;
    const int32_t localCount = encCount;
    taskEXIT_CRITICAL(&encMux);

    // Derived values computed outside the critical section.
    const float motorRevs = (float)localCount / (float)countsPerRev;
    const float pos       = (float)M_PI * pitchDiamM * motorRevs / driveRatio;

    const float cps     = (float)(localCount - encLastCount) / dt_s;
    encLastCount        = localCount;
    const float rawRPM  = (cps / (float)countsPerRev) * 60.f;
    const float filtRPM = velocityKF.update(rawRPM);
    const float filtMs  = (filtRPM / 60.f) * (float)M_PI * pitchDiamM / driveRatio;

    // Three aligned 32-bit stores — atomic on LX7, no lock needed.
    positionM   = pos;
    velocityRPM = filtRPM;
    velocityMs  = filtMs;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  LINE SENSOR TASK — runs on core 0, period = LINE_TASK_PERIOD_MS
//
//  Reads the TCRT5000 (active-LOW, internal pull-up) every 5 ms.
//  Stop condition:
//    Any LOW level (sensor over a line) while the trolley is inside the
//    approach zone of an active goto (approachSlowing && gotoStationIdx>=0
//    && motorOn).  This is level-triggered, NOT edge-triggered, so it is
//    robust against:
//      - pre-zone glitches that would otherwise leave the previous-state
//        variable in a bad place,
//      - the trolley entering the approach zone already on top of a line.
//
//  Once the gate fires, lineStopPending=true is published.  The main loop
//  reads it in updateGotoApproach() and calls onStationArrival() which
//  clears motorOn / approachSlowing — so a sustained LOW only triggers a
//  single stop, and subsequent ticks while the line is still under the
//  sensor are no-ops because the gate condition is no longer satisfied.
//
//  Outside the approach zone the sensor is still sampled every tick so the
//  /status JSON's lineSensed field stays live for UI feedback, but the
//  stop flag is never raised.
//
//  lineCurrentState is published every tick for UI telemetry:
//    true  = pin HIGH = sensor output "1" = no line
//    false = pin LOW  = sensor output "0" = line detected
// ═══════════════════════════════════════════════════════════════════════════
void lineSensorTask(void* /*pvParameters*/) {
  // Latched once-per-stop log so the serial monitor reports the trigger
  // exactly once per zone entry, instead of spamming every 5 ms while the
  // trolley is rolling over the line.
  static bool stopLatched = false;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(LINE_TASK_PERIOD_MS);

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    const bool curHigh = (digitalRead(LINE_SENSOR_PIN) == HIGH); // true = "1"

    // Publish current state for UI telemetry (volatile byte write — atomic).
    lineCurrentState = curHigh;

    // ── Level-triggered stop while inside the approach zone ───────────────
    // Reading approachSlowing / gotoStationIdx / motorOn from core 1 is safe:
    // they are bool/int32 aligned globals updated only on core 1 while we
    // only read them here — no torn read is possible.
    const bool armed = (approachSlowing && gotoStationIdx >= 0 && motorOn);

    if (armed && !curHigh) {
      lineStopPending = true;   // volatile byte write — seen immediately by core 1
      if (!stopLatched) {
        stopLatched = true;
        Serial.println("Line sensor: LOW inside approach zone — stop pending.");
      }
    } else if (!armed) {
      // Re-arm the once-per-zone log latch as soon as we leave the zone
      // (motor stops, goto cancels, or a new goto begins).
      stopLatched = false;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  STATION MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════
static const char* stationTypeName(StationType t) {
  switch(t) {
    case ST_HOME:         return "Home";
    case ST_DRIVE_THRU:   return "Drive Thru";
    case ST_DRONE_PICKUP: return "Drone";
    default:              return "Unknown";
  }
}

static void clearStations() {
  for (uint8_t i=0;i<MAX_STATIONS;i++) stations[i].active=false;
  stationCount=0;
}

static bool addStation(const char* nm, StationType t, int32_t cnt) {
  if (stationCount>=MAX_STATIONS) return false;
  for (uint8_t i=0;i<MAX_STATIONS;i++) {
    if (!stations[i].active) {
      strncpy(stations[i].name,nm,23); stations[i].name[23]='\0';
      stations[i].type=t; stations[i].encStopCount=cnt; stations[i].active=true;
      stationCount++; return true;
    }
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════
//  STATION ARRIVAL — called when encCount reaches gotoTargetCount
// ═══════════════════════════════════════════════════════════════════════════
void onStationArrival(int idx) {
  // Hard stop
  throttlePct      = 0;
  motorOn          = false;
  approachSlowing  = false;
  currentUsF       = (float)THROTTLE_NEUTRAL_US;
  writeEscPulseUs(THROTTLE_NEUTRAL_US);
  ledStopLatched   = true;

  if (idx >= 0 && idx < MAX_STATIONS && stations[idx].active) {
    Serial.printf("Arrived at [%s] enc=%d\n", stations[idx].name, (int)encCount);

    if (stations[idx].type == ST_DRIVE_THRU) {
      stsServoMove(stsServoPos180, "drive-thru");
      Serial.println("STS3215: rotating to 180° for Drive Thru.");
      requestEncoderZero();
      Serial.println("Drive Thru reached and stopped — encoder auto-rezeroed.");
    }
  }

  gotoStationIdx    = -1;
  gotoTargetCount   = 0;

  // ── Demo state-machine arrival hooks ────────────────────────────────────
  // These run after the regular per-type behaviour (e.g. Drive Thru already
  // rotated the STS3215 to 180° and zeroed the encoder above), so the demo
  // just needs to schedule the next step.
  if (demoState == DEMO_GOTO_DRONE && idx == demoDroneStationIdx) {
    demoState           = DEMO_AT_DRONE_WAIT;
    demoStateStartMs    = millis();
    demoStateDurationMs = DEMO_DRONE_WAIT_MS;
    Serial.println("Demo: arrived at Drone Pickup; waiting 10 s.");
  } else if (demoState == DEMO_GOTO_DRIVETHRU && idx == demoDriveThruIdx) {
    Serial.println("Demo: arrived at Drive Thru; routine complete. Press Demo again to repeat.");
    demoState           = DEMO_IDLE;
    demoStateStartMs    = 0;
    demoStateDurationMs = 0;
    demoDroneStationIdx = -1;
    demoDriveThruIdx    = -1;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  GOTO APPROACH — encoder-count proximity check, called every loop tick
// ═══════════════════════════════════════════════════════════════════════════
void updateGotoApproach() {
  if (!motorOn || gotoStationIdx < 0) return;
  if (!userArmed || !escReady) return;

  // ── Line sensor stop — highest priority inside approach zone ─────────────
  // lineStopPending is set by lineSensorTask on core 0 on a 1→0 edge.
  // Clear it here before acting so a second edge during the stop sequence
  // cannot fire again.
  if (lineStopPending) {
    lineStopPending = false;
    Serial.println("Line sensor stop: 1->0 transition detected.");
    onStationArrival(gotoStationIdx);
    return;
  }

  // Minimum travel guard — don't trigger approach zone until past this point
  if (encCount < gotoMinTravelCount) return;

  const int32_t remaining = gotoTargetCount - encCount;

  // ── Stop condition ────────────────────────────────────────────────────────
  if (remaining <= 0) {
    onStationArrival(gotoStationIdx);
    return;
  }

  // ── Approach zone: remaining < approachZoneCount ──────────────────────────
  if (remaining < approachZoneCount && !approachSlowing) {
    approachSlowing = true;
    Serial.printf("Approach zone: %d counts to [%s] — slowing to %.1f%%\n",
                  (int)remaining, stations[gotoStationIdx].name, approachSlowPct);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  GOTO MOTION — shared by gotostation handler and servo-return completion
// ═══════════════════════════════════════════════════════════════════════════
void startGotoMotion(int idx) {
  if (idx<0||idx>=MAX_STATIONS||!stations[idx].active) return;

  gotoTargetCount    = stations[idx].encStopCount;
  // Minimum travel = 10% of approach zone so we don't stop immediately
  // if the trolley is already near the station.
  gotoMinTravelCount = encCount + (approachZoneCount / 10);
  approachSlowing    = false;
  ledStopLatched     = false;
  gotoStationIdx     = idx;
  motorOn            = true;
  motorReverse       = false;
  // Auto-throttle to the configured max-speed limit on every goto.  This
  // gives the operator a predictable cruise speed for station-to-station
  // moves and avoids the previous behaviour where throttlePct was left at
  // whatever the slider was set to (including 0 % after a station arrival,
  // which would silently leave the trolley stationary).
  throttlePct = (uint8_t)constrain((int)lroundf(maxSpeedPct), 1, (int)maxSpeedPct);

  // Clear any stale line sensor flag from the previous run.
  lineStopPending = false;

  Serial.printf("Goto [%s]: target enc=%d  approach zone starts at enc=%d\n",
                stations[idx].name,
                (int)gotoTargetCount,
                (int)(gotoTargetCount - approachZoneCount));
}

// ═══════════════════════════════════════════════════════════════════════════
//  PREFERENCES
// ═══════════════════════════════════════════════════════════════════════════
void savePrefs() {
  prefs.begin("trolley",false);
  prefs.putULong("rampMs",  rampTimeMs);
  prefs.putFloat("maxSpd",  maxSpeedPct);
  prefs.putFloat("apSlw",   approachSlowPct);
  prefs.putInt  ("apZone",  (int)approachZoneCount);
  prefs.putUInt ("cpr",     countsPerRev);
  prefs.putFloat("drive",   driveRatio);
  prefs.putFloat("pitch",   pitchDiamM*1000.f);
  prefs.putBool ("encInv",  encInvert);
  prefs.putUInt ("sv0",     stsServoPos0);
  prefs.putUInt ("sv180",   stsServoPos180);
  prefs.putUInt ("dsv0",    dsServoPulse0);
  prefs.putUInt ("dsv180",  dsServoPulse180);
  prefs.putFloat("imuOff",  imuAxisOffsetDeg);
  prefs.putUInt ("imuAxis", (uint32_t)imuAxisSelect);
  // Save stations
  prefs.putUInt("stCnt", stationCount);
  uint8_t si=0;
  for (uint8_t i=0;i<MAX_STATIONS;i++) {
    if (!stations[i].active) continue;
    char k[12];
    snprintf(k,12,"sn%d",si); prefs.putString(k, stations[i].name);
    snprintf(k,12,"sc%d",si); prefs.putInt(k, (int)stations[i].encStopCount);
    snprintf(k,12,"st%d",si); prefs.putUInt(k, (uint32_t)stations[i].type);
    si++;
  }
  prefs.end();
  Serial.printf("Prefs saved (%d stations).\n", stationCount);
}

void loadPrefs() {
  prefs.begin("trolley",true);
  rampTimeMs        = prefs.getULong("rampMs",  RAMP_TIME_DEF_MS);
  maxSpeedPct       = prefs.getFloat("maxSpd",  MAX_SPEED_PCT_DEF);
  approachSlowPct   = prefs.getFloat("apSlw",   APPROACH_SLOW_PCT_DEF);
  approachZoneCount = (int32_t)prefs.getInt("apZone", (int)APPROACH_ZONE_DEF_COUNTS);
  countsPerRev      = (uint16_t)prefs.getUInt("cpr",  AS5047_CPR_DEFAULT);
  driveRatio        = prefs.getFloat("drive",   DRIVE_RATIO_DEF);
  pitchDiamM        = prefs.getFloat("pitch",   100.f)/1000.f;
  encInvert         = prefs.getBool ("encInv",  ENC_INVERT_DEF);
  stsServoPos0      = (uint16_t)constrain(prefs.getUInt("sv0",   STS_SERVO_POS_0DEG_DEF),   0U, 4095U);
  stsServoPos180    = (uint16_t)constrain(prefs.getUInt("sv180", STS_SERVO_POS_180DEG_DEF), 0U, 4095U);
  dsServoPulse0     = (uint16_t)constrain(prefs.getUInt("dsv0",   DS_SERVO_PULSE_0DEG_DEF),
                                          DS_SERVO_MIN_PULSE_US, DS_SERVO_MAX_PULSE_US);
  dsServoPulse180   = (uint16_t)constrain(prefs.getUInt("dsv180", DS_SERVO_PULSE_180DEG_DEF),
                                          DS_SERVO_MIN_PULSE_US, DS_SERVO_MAX_PULSE_US);
  if (dsServoPulse180 < dsServoPulse0) {
    uint16_t tmp = dsServoPulse0;
    dsServoPulse0 = dsServoPulse180;
    dsServoPulse180 = tmp;
  }
  imuAxisOffsetDeg  = prefs.getFloat("imuOff", 0.0f);
  imuRollOffsetDeg  = imuAxisOffsetDeg;
  imuAxisSelect     = (ImuAxis)constrain((int)prefs.getUInt("imuAxis", (uint32_t)IMU_AXIS_Y), 0, 2);
  clearStations();
  uint8_t savedCnt = (uint8_t)prefs.getUInt("stCnt", 0);
  for (uint8_t i=0;i<savedCnt&&i<MAX_STATIONS;i++) {
    char k[12];
    snprintf(k,12,"sn%d",i); String nm = prefs.getString(k,"");
    snprintf(k,12,"sc%d",i); int32_t cnt = (int32_t)prefs.getInt(k,0);
    snprintf(k,12,"st%d",i); StationType t = (StationType)prefs.getUInt(k,0);
    if (nm.length()>0) addStation(nm.c_str(), t, cnt);
  }
  prefs.end();
  Serial.printf("Prefs loaded (%d stations).\n", stationCount);
}

// ═══════════════════════════════════════════════════════════════════════════
//  SERVO + IMU HELPERS
// ═══════════════════════════════════════════════════════════════════════════
static float mapFloatLinear(float x, float inMin, float inMax, float outMin, float outMax) {
  const float denom = inMax - inMin;
  if (fabsf(denom) < 1e-6f) return outMin;
  return (x - inMin) * (outMax - outMin) / denom + outMin;
}

static float normalizeAngle180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

static void stsServoMove(uint16_t pos, const char* reason) {
  if (!stsServoEnabled) return;
  stsServoCmdPos = pos;
  stsServo.WritePosEx(STS_SERVO_ID, pos, STS_SERVO_SPEED, STS_SERVO_ACC);
  Serial.printf("STS3215: %u steps (%s)\n", pos, reason);
}

static bool dsServoAttachOrReattach() {
  if (ds3245Servo.attached()) ds3245Servo.detach();
  pinMode(SERVO_PWM_PIN, OUTPUT);
#if !DS_SERVO_USING_ARDUINO_SERVO
  ds3245Servo.setPeriodHertz(50);
#endif
  const int attachResult = ds3245Servo.attach(SERVO_PWM_PIN, dsServoPulse0, dsServoPulse180);
  dsServoEnabled = ds3245Servo.attached();
  Serial.printf("DS3245 attach: lib=%s pin=%d result=%d attached=%s range=%u-%u us\n",
                DS_SERVO_LIB_NAME, SERVO_PWM_PIN, attachResult,
                dsServoEnabled ? "yes" : "no", dsServoPulse0, dsServoPulse180);
  if (!dsServoEnabled) Serial.println("ERROR: DS3245 servo attach failed.");
  return dsServoEnabled;
}

static uint16_t dsServoDegToPulseUs(int deg) {
  deg = constrain(deg, 0, 180);
  return (uint16_t)constrain(
    lroundf(mapFloatLinear((float)deg, 0.0f, 180.0f,
                           (float)dsServoPulse0, (float)dsServoPulse180)),
    (long)DS_SERVO_MIN_PULSE_US,
    (long)DS_SERVO_MAX_PULSE_US
  );
}

static void dsServoWriteAngleInternal(int deg) {
  if (!dsServoEnabled || !ds3245Servo.attached()) return;
  dsServoCmdDeg = constrain(deg, 0, 180);
  dsServoPulseUs = dsServoDegToPulseUs(dsServoCmdDeg);
  ds3245Servo.write(dsServoCmdDeg);
}

static void dsServoMove(int deg, const char* reason) {
  dsServoWriteAngleInternal(deg);
  Serial.printf("DS3245: %d deg (%u us, %s)\n", dsServoCmdDeg, dsServoPulseUs, reason);
}

static bool initIMU() {
  if (!bno.begin()) {
    Serial.println("[IMU] BNO055 not detected; servo stabilization disabled.");
    imuReady = false;
    imuCalibrated = false;
    imuCalibrating = false;
    imuStabilizeEnabled = false;
    return false;
  }
  delay(100);
  bno.setExtCrystalUse(true);
  imuReady = true;
  Serial.println("[IMU] BNO055 ready.");
  return true;
}

static void readImuEulerDeg(float& xDeg, float& yDeg, float& zDeg) {
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
  xDeg = normalizeAngle180(event.orientation.x);
  yDeg = normalizeAngle180(event.orientation.y);
  zDeg = normalizeAngle180(event.orientation.z);
  imuEulerXDeg = xDeg;
  imuEulerYDeg = yDeg;
  imuEulerZDeg = zDeg;
}

static void readImuGravityTiltDeg(float& xTiltDeg, float& yTiltDeg) {
  sensors_event_t gravEvent;
  bno.getEvent(&gravEvent, Adafruit_BNO055::VECTOR_GRAVITY);

  const float gx = gravEvent.acceleration.x;
  const float gy = gravEvent.acceleration.y;
  const float gz = gravEvent.acceleration.z;
  constexpr float RAD_TO_DEG_F = 57.2957795131f;

  // Use gravity-derived tilt for X/Y stabilization so the control signal stays
  // signed across the full useful range and does not fold back the way Euler can.
  xTiltDeg = atan2f(gy, gz) * RAD_TO_DEG_F;
  yTiltDeg = atan2f(-gx, gz) * RAD_TO_DEG_F;

  if (!isfinite(xTiltDeg)) xTiltDeg = 0.0f;
  if (!isfinite(yTiltDeg)) yTiltDeg = 0.0f;
}

static const char* imuAxisName(ImuAxis axis) {
  switch (axis) {
    case IMU_AXIS_X: return "X-axis";
    case IMU_AXIS_Y: return "Y-axis";
    case IMU_AXIS_Z: return "Z-axis";
    default:         return "Y-axis";
  }
}

static float readImuControlAxisDeg() {
  float xEulerDeg, yEulerDeg, zEulerDeg;
  readImuEulerDeg(xEulerDeg, yEulerDeg, zEulerDeg);

  float xTiltDeg, yTiltDeg;
  readImuGravityTiltDeg(xTiltDeg, yTiltDeg);

  switch (imuAxisSelect) {
    case IMU_AXIS_X: return xTiltDeg;
    case IMU_AXIS_Z: return zEulerDeg;
    case IMU_AXIS_Y:
    default:         return yTiltDeg;
  }
}

static void beginIMUCalibration() {
  if (!imuReady) return;

  imuCalibrating     = true;
  imuCalibrated      = false;
  imuCalSampleIndex  = 0;
  imuCalSumDeg       = 0.0f;
  imuCalSumSqDeg     = 0.0f;
  imuCalNextSampleMs = millis();
  imuRollRawDeg      = 0.0f;
  imuRollFiltDeg     = 0.0f;

  if (dsServoEnabled && ds3245Servo.attached()) {
    dsServoWriteAngleInternal(DS_SERVO_NEUTRAL_DEG);
  }

  Serial.printf("[IMU] %s calibration started. Keep the platform in the desired neutral pose and still. DS3245 neutral set to 90 deg.",
  imuAxisName(imuAxisSelect));
}

static void updateIMUCalibration() {
  if (!imuReady || !imuCalibrating) return;

  const uint32_t now = millis();
  if ((int32_t)(now - imuCalNextSampleMs) < 0) return;

  const float sample = readImuControlAxisDeg();
  imuCalSumDeg   += sample;
  imuCalSumSqDeg += sample * sample;
  imuCalSampleIndex++;
  imuCalNextSampleMs = now + IMU_CAL_DELAY_MS;

  if (imuCalSampleIndex < IMU_CAL_SAMPLES) return;

  const float mean = imuCalSumDeg / (float)IMU_CAL_SAMPLES;
  const float variance = max(0.0f, imuCalSumSqDeg / (float)IMU_CAL_SAMPLES - mean * mean);
  const float stdDev = sqrtf(variance);

  imuAxisOffsetDeg = mean;
  imuRollOffsetDeg = imuAxisOffsetDeg;
  imuRollRawDeg    = 0.0f;
  imuRollFiltDeg   = 0.0f;
  imuCalibrated    = true;
  imuCalibrating   = false;
  imuRollKF.init(0.02f, 0.25f, 0.0f);
  lastImuMs        = now;

  if (dsServoEnabled && ds3245Servo.attached()) {
    dsServoWriteAngleInternal(DS_SERVO_NEUTRAL_DEG);
  }

  Serial.printf("[IMU] %s calibration offset=%.2f deg, stdDev=%.2f deg",
  imuAxisName(imuAxisSelect), mean, stdDev);
  if (stdDev > IMU_CAL_STABILITY_DEG)
    Serial.println("[IMU] Warning: calibration variance high; continuing with measured offset.");
}

static void updateIMU() {
  if (!imuReady || !imuCalibrated || imuCalibrating) return;

  const uint32_t now = millis();
  if (now - lastImuMs < IMU_SAMPLE_RATE_MS) return;
  lastImuMs = now;

  imuRollRawDeg  = normalizeAngle180(readImuControlAxisDeg() - imuAxisOffsetDeg);
  imuRollFiltDeg = imuRollKF.update(imuRollRawDeg);

  if (imuStabilizeEnabled && dsServoEnabled && ds3245Servo.attached()) {
    const float constrainedRoll = constrain(imuRollFiltDeg, IMU_ROLL_MIN_DEG, IMU_ROLL_MAX_DEG);
    const int servoAngle = (int)lroundf(
      mapFloatLinear(constrainedRoll,
                     IMU_ROLL_MIN_DEG, IMU_ROLL_MAX_DEG,
                     (float)IMU_SERVO_ANGLE_MIN, (float)IMU_SERVO_ANGLE_MAX));
    dsServoWriteAngleInternal(servoAngle);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  LED UPDATE — non-blocking state-driven patterns
// ═══════════════════════════════════════════════════════════════════════════
void updateLEDs() {
  const uint32_t now = millis();

  LedMode nextMode = LED_MODE_OFF;
  if (motorOn && approachSlowing)      nextMode = LED_MODE_APPROACH_YELLOW;
  else if (motorOn)                    nextMode = LED_MODE_RUN_GREEN;
  else if (ledStopLatched)             nextMode = LED_MODE_STOP_RED;

  if (nextMode != ledMode) {
    ledMode = nextMode;
    ledStep = 0;
    ledFlashOn = false;
    lastLedMs = 0;
  }

  uint32_t intervalMs = LED_STEP_INTERVAL_MS;
  if (ledMode == LED_MODE_STOP_RED) intervalMs = LED_FLASH_HALF_MS;

  if ((now - lastLedMs) < intervalMs) return;
  lastLedMs = now;

  ledPixels.clear();

  switch (ledMode) {
    case LED_MODE_RUN_GREEN:
      for (int i = 0; i <= ledStep && i < LED_NUMPIXELS; ++i) {
        ledPixels.setPixelColor(i, ledPixels.Color(0, 140, 0));
      }
      ledStep = (ledStep + 1) % (LED_NUMPIXELS + 1);
      break;

    case LED_MODE_APPROACH_YELLOW:
      for (int i = 0; i <= ledStep && i < LED_NUMPIXELS; ++i) {
        ledPixels.setPixelColor(i, ledPixels.Color(180, 120, 0));
      }
      ledStep = (ledStep + 1) % (LED_NUMPIXELS + 1);
      break;

    case LED_MODE_STOP_RED:
      ledFlashOn = !ledFlashOn;
      if (ledFlashOn) {
        for (int i = 0; i < LED_NUMPIXELS; ++i) {
          ledPixels.setPixelColor(i, ledPixels.Color(180, 0, 0));
        }
      }
      ledStep = 0;
      break;

    case LED_MODE_OFF:
    default:
      ledStep = 0;
      ledFlashOn = false;
      break;
  }

  ledPixels.show();
}

// ═══════════════════════════════════════════════════════════════════════════
//  DEMO STATE MACHINE
//  Drives the home-page "Start Demo" button through a fixed routine.
//  All transitions are non-blocking: each step sets demoStateStartMs +
//  demoStateDurationMs (when applicable) and updateDemo() polls them on the
//  main loop.  onStationArrival() advances DEMO_GOTO_* states to the
//  corresponding wait/finish state.  cancelDemo() is invoked from the Stop
//  command and from any aborted goto so the demo never drives the trolley
//  unattended after the user intervenes.
// ═══════════════════════════════════════════════════════════════════════════
static const char* demoStateName(DemoState s) {
  switch (s) {
    case DEMO_IDLE:             return "idle";
    case DEMO_IMU_REZERO:       return "imu-rezero";
    case DEMO_BOARDING_180:     return "boarding-180";
    case DEMO_BOARDING_RETURN:  return "boarding-return";
    case DEMO_GOTO_DRONE:       return "goto-drone";
    case DEMO_AT_DRONE_WAIT:    return "at-drone-wait";
    case DEMO_GOTO_DRIVETHRU:   return "goto-drivethru";
    default:                    return "?";
  }
}

static int findFirstStationOfType(StationType t) {
  for (uint8_t i = 0; i < MAX_STATIONS; i++) {
    if (stations[i].active && stations[i].type == t) return i;
  }
  return -1;
}

// Returns true if the demo was kicked off.  Reasons it may decline:
//   - ESC not ready / motor not user-armed
//   - missing Drone Pickup or Drive Thru station
//   - already running (re-press is ignored, not restarted, until DEMO_IDLE)
static bool startDemo() {
  if (!escReady || !userArmed) {
    Serial.println("Demo: declined — ESC not ready or motor not armed.");
    return false;
  }
  if (demoState != DEMO_IDLE) {
    Serial.printf("Demo: already running (%s) — ignoring start.\n", demoStateName(demoState));
    return false;
  }
  demoDroneStationIdx = findFirstStationOfType(ST_DRONE_PICKUP);
  demoDriveThruIdx    = findFirstStationOfType(ST_DRIVE_THRU);
  if (demoDroneStationIdx < 0 || demoDriveThruIdx < 0) {
    Serial.println("Demo: declined — need at least one Drone Pickup and one Drive Thru station.");
    return false;
  }

  Serial.println("Demo: starting routine.");

  // Force the trolley to drive at the demo cruise speed (clamped by the user's
  // configured max speed limit).  Without this the demo would inherit whatever
  // value the Speed Control slider was last left at — including 0 %, which
  // would silently leave the motor stationary.
  const float demoPct = (DEMO_THROTTLE_PCT_DEF < maxSpeedPct) ? DEMO_THROTTLE_PCT_DEF : maxSpeedPct;
  throttlePct  = (uint8_t)constrain((int)demoPct, 1, (int)maxSpeedPct);
  motorReverse = false;

  // Step 1: IMU re-zero + encoder zero.  If the IMU isn't available we just
  // skip the calibration wait and proceed; encoder zero still happens.
  requestEncoderZero();
  if (imuReady) {
    beginIMUCalibration();
    dsServoMove(DS_SERVO_NEUTRAL_DEG, "demo-imu-rezero");
    demoState = DEMO_IMU_REZERO;
  } else {
    // Skip straight to the boarding pose if no IMU is available.
    stsServoMove(stsServoPos180, "demo-boarding");
    demoState = DEMO_BOARDING_180;
    demoStateStartMs    = millis();
    demoStateDurationMs = DEMO_BOARDING_WAIT_MS;
  }
  return true;
}

static void cancelDemo(const char* reason) {
  if (demoState == DEMO_IDLE) return;
  Serial.printf("Demo: cancelled in %s (%s).\n", demoStateName(demoState), reason);
  demoState           = DEMO_IDLE;
  demoStateStartMs    = 0;
  demoStateDurationMs = 0;
  demoDroneStationIdx = -1;
  demoDriveThruIdx    = -1;
}

static void updateDemo() {
  if (demoState == DEMO_IDLE) return;

  // Safety: any time the user's arming status drops out, abort.
  if (!escReady || !userArmed) {
    cancelDemo("disarmed");
    return;
  }

  const uint32_t now = millis();

  switch (demoState) {

    case DEMO_IMU_REZERO: {
      // Wait for the IMU calibration routine (or timeout fallback) to finish,
      // then move the STS3215 to its 180° boarding pose.
      if (imuCalibrating) return;
      stsServoMove(stsServoPos180, "demo-boarding");
      demoState           = DEMO_BOARDING_180;
      demoStateStartMs    = now;
      demoStateDurationMs = DEMO_BOARDING_WAIT_MS;
      Serial.println("Demo: IMU re-zeroed; STS3215 -> 180° (boarding) for 4 s.");
      break;
    }

    case DEMO_BOARDING_180: {
      if (now - demoStateStartMs < demoStateDurationMs) return;
      // Return STS3215 to 0° and wait for the rotation to finish before driving.
      stsServoMove(stsServoPos0, "demo-boarding-return");
      demoState           = DEMO_BOARDING_RETURN;
      demoStateStartMs    = now;
      demoStateDurationMs = SERVO_RETURN_WAIT_MS;
      Serial.println("Demo: boarding hold complete; STS3215 -> 0°, then to Drone Pickup.");
      break;
    }

    case DEMO_BOARDING_RETURN: {
      if (now - demoStateStartMs < demoStateDurationMs) return;
      if (demoDroneStationIdx < 0 || !stations[demoDroneStationIdx].active) {
        cancelDemo("drone station missing");
        return;
      }
      startGotoMotion(demoDroneStationIdx);
      demoState           = DEMO_GOTO_DRONE;
      demoStateStartMs    = now;
      demoStateDurationMs = 0;
      break;
    }

    case DEMO_AT_DRONE_WAIT: {
      // 10 s countdown reported to the UI via /status (demoTimeRemainingMs).
      if (now - demoStateStartMs < demoStateDurationMs) return;
      if (demoDriveThruIdx < 0 || !stations[demoDriveThruIdx].active) {
        cancelDemo("drive-thru station missing");
        return;
      }
      startGotoMotion(demoDriveThruIdx);
      demoState           = DEMO_GOTO_DRIVETHRU;
      demoStateStartMs    = now;
      demoStateDurationMs = 0;
      Serial.println("Demo: drone-pickup wait complete; heading to Drive Thru.");
      break;
    }

    case DEMO_GOTO_DRONE:
    case DEMO_GOTO_DRIVETHRU:
      // Advanced from onStationArrival() — nothing to do here.
      break;

    default:
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  WEB PAGE
// ═══════════════════════════════════════════════════════════════════════════
static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>DD Trolley</title>
<style>
:root{--bg:#0f0f13;--card:#1a1a24;--acc:#00c8ff;--dan:#ff4444;--ok:#00e676;
      --warn:#ffc800;--pur:#c084fc;--txt:#e8e8f0;--mut:#6b6b80;--r:14px;}
*{box-sizing:border-box;margin:0;padding:0;}
body{background:var(--bg);color:var(--txt);font-family:'Segoe UI',system-ui,sans-serif;
     min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:12px 14px 18px;}
h1{font-size:1.3rem;letter-spacing:.08em;color:var(--acc);margin-bottom:3px;}
.sub{color:var(--mut);font-size:.82rem;margin-bottom:18px;}
.card{background:var(--card);border-radius:var(--r);padding:20px 18px;width:100%;
      max-width:420px;margin-bottom:12px;box-shadow:0 4px 20px rgba(0,0,0,.4);}
.ct{font-size:.66rem;letter-spacing:.13em;text-transform:uppercase;color:var(--mut);margin-bottom:12px;}
.badge{display:inline-flex;align-items:center;gap:7px;font-size:.86rem;font-weight:700;
       padding:5px 13px;border-radius:999px;transition:all .3s;}
.badge::before{content:'';width:7px;height:7px;border-radius:50%;background:currentColor;}
.badge.stopped{background:rgba(255,68,68,.15);color:var(--dan);}
.badge.arming{background:rgba(255,200,0,.15);color:var(--warn);}
.badge.armed{background:rgba(0,200,255,.15);color:var(--acc);}
.badge.running{background:rgba(0,230,118,.15);color:var(--ok);}
.badge.seeking{background:rgba(192,132,252,.15);color:var(--pur);}
.pu{font-size:.76rem;color:var(--mut);}
.brs{margin-bottom:4px;}
.br{display:grid;grid-template-columns:56px 1fr 44px;align-items:center;gap:10px;margin-bottom:9px;}
.bl{font-size:.76rem;color:var(--mut);}
.bt{height:12px;background:#101018;border-radius:999px;overflow:hidden;}
.bf{height:100%;width:0%;border-radius:999px;transition:width .18s linear;}
.bf.tgt{background:linear-gradient(90deg,var(--acc),#6ee7ff);}
.bf.act{background:linear-gradient(90deg,var(--ok),#7dffbd);}
.bp{font-size:.78rem;text-align:right;color:var(--txt);font-variant-numeric:tabular-nums;}
.sh{display:flex;justify-content:space-between;align-items:center;font-size:.9rem;margin-bottom:10px;}
.v{font-size:1.05rem;font-weight:800;color:var(--acc);}
input[type=range]{width:100%;height:10px;border-radius:999px;background:#0e0e15;outline:none;
  -webkit-appearance:none;margin:10px 0 14px;}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:24px;height:24px;
  border-radius:50%;background:var(--acc);box-shadow:0 0 0 5px rgba(0,200,255,.12);cursor:pointer;}
input[type=range]:disabled{opacity:.3;cursor:not-allowed;}
.rr,.sr,.add-row{display:flex;align-items:center;gap:10px;margin-bottom:10px;}
.rr label,.sr label,.add-row label{font-size:.8rem;color:var(--mut);flex:1;}
.ni{background:#11111a;border:1px solid #2b2b3a;color:var(--txt);border-radius:10px;
    padding:8px 10px;font-size:.85rem;text-align:right;}
.ni.sm{width:74px;}.ni.med{width:110px;}
.u{font-size:.75rem;color:var(--mut);min-width:28px;}
.br2{display:flex;gap:10px;}
button{border:none;border-radius:12px;padding:12px 14px;font-size:.88rem;font-weight:700;
       cursor:pointer;transition:.18s;flex:1;}
button:active:not(:disabled){transform:scale(.96);}
button:disabled{opacity:.3;cursor:not-allowed;}
.barm{background:linear-gradient(135deg,#264653,#2a9d8f);color:white;}
.bfr{background:#1f2937;color:#d1d5db;}
.bfr.active{background:#7c2d12;color:#ffedd5;}
.bst{background:linear-gradient(135deg,#16a34a,#22c55e);color:white;}
.bsp{background:linear-gradient(135deg,#991b1b,#ef4444);color:white;}
.bsv{background:linear-gradient(135deg,#1d4ed8,#38bdf8);color:white;}
.bgo{background:linear-gradient(135deg,#7c3aed,#a855f7);color:white;}
.badd{background:linear-gradient(135deg,#0f766e,#14b8a6);color:white;}
.bdel{background:#2a1114;color:#ff7b7b;flex:none;padding:8px 10px;}
.bcap{background:#1f2937;color:#d1d5db;flex:none;padding:7px 10px;}
.bzero{background:#3f3f46;color:#fafafa;}
.bsv0{background:#1f2937;color:#d1d5db;}
.bsv180{background:#7c2d12;color:#ffedd5;}
.bn{background:rgba(255,200,0,.15);color:var(--warn);
    border:1px solid rgba(255,200,0,.3);padding:10px 14px;border-radius:12px;max-width:420px;
    width:100%;margin-bottom:12px;text-align:center;font-size:.84rem;}
.bn.h{display:none;}
.apw{display:none;margin-top:12px;height:8px;background:#0f1018;border-radius:999px;overflow:hidden;}
.apw.vis{display:block;}.apb{height:100%;width:0;background:linear-gradient(90deg,var(--warn),#ffe082);}
.apzone{display:none;margin-top:8px;}
.apzone.vis{display:block;}
.apzone-lbl{display:flex;justify-content:space-between;font-size:.72rem;color:var(--mut);margin-bottom:5px;}
.apzone-bar{height:10px;background:#11111a;border-radius:999px;overflow:hidden;}
.apzone-fill{height:100%;background:linear-gradient(90deg,var(--warn),var(--pur));}
.am{display:flex;align-items:end;justify-content:space-between;gap:14px;margin-bottom:10px;}
.amv{font-size:2.2rem;font-weight:800;color:var(--acc);line-height:1;}
.amv.warn{color:var(--warn);} .amv.high{color:var(--dan);} .amu{font-size:.9rem;font-weight:700;margin-left:3px;}
.amr{font-size:.8rem;color:var(--mut);white-space:nowrap;}
.cw{height:14px;background:#0f1018;border-radius:999px;overflow:hidden;}.cf{height:100%;width:0;background:linear-gradient(90deg,#22c55e,#84cc16,#facc15);}
.cf.warn{background:linear-gradient(90deg,#eab308,#f97316);} .cf.high{background:linear-gradient(90deg,#f97316,#ef4444);}
.cs{display:flex;justify-content:space-between;font-size:.68rem;color:var(--mut);margin-top:5px;}
.eg{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;}
.ec{background:#11111a;border:1px solid #282838;border-radius:12px;padding:12px;}
.ecl{font-size:.7rem;color:var(--mut);margin-bottom:7px;}
.ecv{font-size:1.1rem;font-weight:800;} .ecv.ac{color:var(--acc);} .ecv.pu{color:var(--pur);} .ecu{font-size:.7rem;margin-left:3px;}
.st-item{display:flex;align-items:center;gap:8px;background:#11111a;border:1px solid #282838;
         border-radius:12px;padding:10px;margin-bottom:8px;}
.st-active{border-color:var(--pur);box-shadow:0 0 0 1px rgba(192,132,252,.35) inset;}
.st-nm{font-size:.9rem;font-weight:700;display:flex;align-items:center;gap:8px;flex-wrap:wrap;}
.st-badge{font-size:.64rem;padding:3px 8px;border-radius:999px;text-transform:uppercase;letter-spacing:.08em;}
.st-badge.home{background:rgba(34,197,94,.15);color:#4ade80;}
.st-badge.drivethru{background:rgba(250,204,21,.15);color:#facc15;}
.st-badge.drone{background:rgba(56,189,248,.15);color:#38bdf8;}
.st-inf{font-size:.75rem;color:var(--mut);}
.svr{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;}
.svb{display:inline-flex;align-items:center;justify-content:center;padding:6px 12px;border-radius:999px;
     background:rgba(0,200,255,.15);color:var(--acc);font-weight:800;min-width:90px;}
.svb.p180{background:rgba(250,204,21,.15);color:#facc15;}
.svb.off{background:rgba(255,68,68,.15);color:var(--dan);}
/* ── Home / Advanced layout ─────────────────────────────────────────────── */
.appbar{position:sticky;top:0;z-index:5;display:flex;align-items:center;justify-content:space-between;
  width:100%;max-width:420px;padding:6px 4px 14px;background:var(--bg);}
.appbar .ttl{font-size:1.3rem;letter-spacing:.08em;color:var(--acc);font-weight:700;}
.hburg{background:#1f2937;color:#d1d5db;border:none;border-radius:10px;
  width:42px;height:42px;font-size:1.1rem;cursor:pointer;flex:none;padding:0;
  display:flex;align-items:center;justify-content:center;transition:.18s;}
.hburg:active{transform:scale(.94);} .hburg.act{background:#7c2d12;color:#ffedd5;}
#homeView,#advView{display:flex;flex-direction:column;align-items:center;width:100%;}
#advView{display:none;}
body.adv #homeView{display:none;} body.adv #advView{display:flex;}
.bdemo{background:linear-gradient(135deg,#16a34a,#22c55e);color:white;
  font-size:1.05rem;padding:18px 14px;}
.bdemo:disabled{background:linear-gradient(135deg,#374151,#4b5563);}
.bspbig{background:linear-gradient(135deg,#991b1b,#ef4444);color:white;
  font-size:1.05rem;padding:18px 14px;}
.demo-state{font-size:.86rem;color:var(--mut);text-align:center;margin-top:10px;
  min-height:1.2em;line-height:1.35;}
.demo-state .v{color:var(--acc);}
.demo-state.warn{color:var(--warn);} .demo-state.warn .v{color:var(--warn);}
.demo-state.run{color:var(--ok);} .demo-state.run .v{color:var(--ok);}
</style>
</head>
<body>
<div class="appbar">
  <span class="ttl">DD Trolley</span>
  <button id="hburg" class="hburg" onclick="toggleView()" title="Advanced settings"
          aria-label="Toggle advanced settings">&#9776;</button>
</div>
<div id="bn" class="bn">&#9203; ESC initialising...</div>

<!-- ─────────────────────────  HOME VIEW  ────────────────────────── -->
<div id="homeView">

  <!-- ARM (home) -->
  <div class="card">
    <div class="ct">Arm Sequence</div>
    <button class="barm" id="btnArm" onclick="doArm()">&#128274; Arm Motor</button>
    <div id="apw" class="apw"><div id="apb" class="apb"></div></div>
  </div>

  <!-- DEMO -->
  <div class="card">
    <div class="ct">Demo</div>
    <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:12px;">
      <span id="stBadge" class="badge stopped">Stopped</span>
      <span id="puUs" class="pu">1500 us</span>
    </div>
    <button class="bdemo" id="btnDemo" onclick="cmd('demostart')" disabled>
      &#9654; Start Demo</button>
    <div style="height:10px;"></div>
    <button class="bspbig" id="btnStopHome" onclick="cmd('stop')" disabled>
      &#9646;&#9646; Stop</button>
    <div id="demoState" class="demo-state">Arm the motor to enable the demo.</div>
  </div>

</div>

<!-- ───────────────────────  ADVANCED VIEW  ──────────────────────── -->
<div id="advView">

<!-- SPEED CONTROL -->
<div class="card">
  <div class="ct">Speed Control</div>
  <div class="sh"><span>Throttle</span><span class="v"><span id="pctD">0</span>%</span></div>
  <input type="range" id="slider" min="0" max="100" value="0" disabled
    oninput="onSli(this.value)" onchange="onSlc(this.value)">
  <div class="rr">
    <label>Ramp time</label>
    <input type="number" class="ni" id="rampI" value="5.0" min="0.3" step="0.1" style="width:76px"
           onchange="sendRamp(this.value)">
    <span class="u">sec</span>
  </div>
  <div class="br2" style="margin-bottom:9px;">
    <button class="bfr" id="btnDir" onclick="toggleDir()">&#8594; Forward</button>
  </div>
  <div class="br2">
    <button class="bst" id="btnStart" onclick="cmd('start')" disabled>&#9654; Start</button>
    <button class="bsp" id="btnStop"  onclick="cmd('stop')"  disabled>&#9646;&#9646; Stop</button>
  </div>
</div>

<!-- STATION MANAGER -->
<div class="card">
  <div class="ct">Station Manager</div>
  <div class="add-form">
    <div class="ct">Add Station</div>
    <div class="add-row">
      <label>Name</label>
      <input type="text" class="ni" id="newName" placeholder="Station name" maxlength="23"
             style="flex:1;text-align:left;min-width:100px;">
    </div>
    <div class="add-row">
      <label>Type</label>
      <select class="ni" id="newType">
        <option value="1">Home</option>
        <option value="2">Drive Thru</option>
        <option value="3">Drone Pickup</option>
      </select>
    </div>
    <div class="add-row">
      <label>Encoder count</label>
      <input type="number" class="ni med" id="newCount" value="0" step="1000">
      <button class="bcap" onclick="captureCount('newCount')" title="Set to current encoder position">&#9654; Capture</button>
    </div>
    <button class="badd" onclick="addStation()">&#43; Add Station</button>
  </div>
  <div id="stationList"><p style="font-size:.8rem;color:var(--mut)">No stations defined yet.</p></div>
  <div style="margin-top:12px;">
    <button class="bsv" style="width:100%;" onclick="cmd('save')">&#128190; Save to EEPROM</button>
  </div>
</div>

<!-- STATUS -->
<div class="card">
  <div class="ct">Status</div>
  <div class="brs">
    <div class="br"><span class="bl">Target</span>
      <div class="bt"><div id="bTgt" class="bf tgt"></div></div><span id="pTgt" class="bp">0%</span></div>
    <div class="br"><span class="bl">Actual</span>
      <div class="bt"><div id="bAct" class="bf act"></div></div><span id="pAct" class="bp">0%</span></div>
  </div>
  <div id="apZoneBar" class="apzone">
    <div class="apzone-lbl">
      <span>Approach zone</span>
      <span id="apZoneRem">—</span>
    </div>
    <div class="apzone-bar"><div id="apZoneFill" class="apzone-fill" style="width:0%"></div></div>
  </div>
  <div style="display:flex;align-items:center;justify-content:space-between;margin-top:10px;">
    <span style="font-size:.72rem;color:var(--mut);">&#9135; Line sensor D5</span>
    <span id="lsBadge" class="badge stopped" style="font-size:.72rem;padding:3px 10px;">&#9632; Inactive</span>
  </div>
</div>

<!-- CURRENT -->
<div class="card">
  <div class="ct">Motor Current</div>
  <div class="am">
    <div>
      <div style="font-size:.7rem;color:var(--mut);margin-bottom:4px;">Filtered (Kalman)</div>
      <div id="amv" class="amv">0.00<span class="amu">A</span></div>
    </div>
    <div class="amr">Raw&nbsp;<span id="rawA">0.00</span> A</div>
  </div>
  <div class="cw"><div id="cbar" class="cf"></div></div>
  <div class="cs"><span>0A</span><span>25A</span><span>50A</span><span>75A</span><span>100A</span></div>
</div>

<!-- ENCODER / POSITION -->
<div class="card">
  <div class="ct">Encoder &amp; Position</div>
  <div class="eg">
    <div class="ec"><div class="ecl">Position</div>
      <div id="ePos" class="ecv ac">0.000<span class="ecu">m</span></div></div>
    <div class="ec"><div class="ecl">Velocity</div>
      <div id="eRPM" class="ecv pu">0.0<span class="ecu">RPM</span></div></div>
    <div class="ec"><div class="ecl">Speed</div>
      <div id="eMs" class="ecv">0.000<span class="ecu">m/s</span></div></div>
    <div class="ec"><div class="ecl">Counts</div>
      <div id="eCnt" class="ecv">0</div></div>
  </div>
  <div style="display:flex;gap:9px;margin-top:12px;">
    <button class="bzero" onclick="cmd('zeroenc')">&#8635; Zero Encoder</button>
  </div>
</div>

<!-- APPROACH SETTINGS -->
<div class="card">
  <div class="ct">Approach Settings</div>
  <div class="sr"><label>Approach speed (in zone)</label>
    <input type="number" class="ni sm" id="setSlw" value="2" min="0.5" step="0.5" onchange="sendApproachSettings()">
    <span class="u">%</span></div>
  <div class="sr"><label>Approach zone distance</label>
    <input type="number" class="ni med" id="setZone" value="870000" min="10000" step="10000" onchange="sendApproachSettings()">
    <span class="u">counts</span></div>
  <div class="sr"><label>Max speed limit</label>
    <input type="number" class="ni sm" id="setMax" value="15" min="1" max="100" step="1" onchange="sendApproachSettings()">
    <span class="u">%</span></div>
</div>

<!-- MACHINE SETTINGS -->
<div class="card">
  <div class="ct">Machine Settings</div>
  <div class="sr"><label>Counts / Rev</label>
    <input type="number" class="ni med" id="setCPR" value="16384" min="1" onchange="sendMs()">
    <span class="u">cts</span></div>
  <div class="sr"><label>Drive Ratio</label>
    <input type="number" class="ni sm" id="setDrv" value="1.0" min="0.01" step="0.01" onchange="sendMs()">
    <span class="u">:1</span></div>
  <div class="sr"><label>Pitch Diameter</label>
    <input type="number" class="ni sm" id="setPit" value="100" min="0.1" step="0.1" onchange="sendMs()">
    <span class="u">mm</span></div>
  <div class="sr"><label>Invert Encoder</label>
    <input type="checkbox" id="setInv" onchange="sendMs()" style="width:22px;height:22px;accent-color:var(--acc);">
  </div>
  <button class="bsv" style="width:100%;margin-top:6px;" onclick="cmd('save')">&#128190; Save to EEPROM</button>
</div>

<!-- STS3215 SERVO -->
<div class="card">
  <div class="ct">STS3215 Station Servo</div>
  <div class="svr">
    <span style="font-size:.8rem;color:var(--mut);">Commanded position</span>
    <span id="stsBadge" class="svb">0&deg;</span>
  </div>
  <div class="sr">
    <label>0&deg; position</label>
    <input type="number" class="ni med" id="sv0" value="0" min="0" max="4095" step="1"
           onchange="sendServoTrims()">
    <span class="u">steps</span>
    <button class="bsv0" id="bs0" onclick="servoGo(0)" style="flex:none;padding:8px 14px;">Test</button>
  </div>
  <div class="sr">
    <label>180&deg; position</label>
    <input type="number" class="ni med" id="sv180" value="2048" min="0" max="4095" step="1"
           onchange="sendServoTrims()">
    <span class="u">steps</span>
    <button class="bsv180" id="bs180" onclick="servoGo(180)" style="flex:none;padding:8px 14px;">Test</button>
  </div>
  <button class="bsv" style="width:100%;margin-top:8px;" onclick="cmd('save')">&#128190; Save to EEPROM</button>
</div>

<!-- SERVO + IMU -->
<div class="card">
  <div class="ct">DS3245 Servo + BNO055</div>
  <div class="svr">
    <span style="font-size:.8rem;color:var(--mut);">Commanded angle</span>
    <span id="dsBadge" class="svb">0&deg;</span>
  </div>
  <div class="sr">
    <label>IMU status</label>
    <span id="imuBadge" class="svb off">Offline</span>
  </div>
  <div class="sr">
    <label>Stabilize from selected axis</label>
    <input type="checkbox" id="imuEnable" onchange="sendImuToggle()" style="width:22px;height:22px;accent-color:var(--acc);">
  </div>
  <div class="sr">
    <label>Control axis</label>
    <select id="imuAxis" class="ni med" onchange="sendImuAxis()">
      <option value="0">X-axis</option>
      <option value="1" selected>Y-axis</option>
      <option value="2">Z-axis</option>
    </select>
    <span class="u">Euler</span>
  </div>
  <div class="sr">
    <label>Re-zero IMU / set neutral</label>
    <button class="bsv" id="imuZeroBtn" onclick="sendImuZero()" style="flex:none;padding:8px 14px;">Re-zero</button>
  </div>
  <div class="eg" style="margin-bottom:12px;">
    <div class="ec"><div class="ecl" id="imuRawLbl">Y-axis raw</div><div id="imuRaw" class="ecv">0.0<span class="ecu">&deg;</span></div></div>
    <div class="ec"><div class="ecl" id="imuFiltLbl">Y-axis filt</div><div id="imuFilt" class="ecv ac">0.0<span class="ecu">&deg;</span></div></div>
  </div>
  <div class="eg" style="margin-bottom:12px;">
    <div class="ec"><div class="ecl">X Euler</div><div id="imuX" class="ecv">0.0<span class="ecu">&deg;</span></div></div>
    <div class="ec"><div class="ecl">Y Euler</div><div id="imuY" class="ecv">0.0<span class="ecu">&deg;</span></div></div>
    <div class="ec"><div class="ecl">Z Euler</div><div id="imuZ" class="ecv">0.0<span class="ecu">&deg;</span></div></div>
  </div>
  <div class="sr">
    <label>0&deg; pulse</label>
    <input type="number" class="ni med" id="dsv0" value="1000" min="500" max="2500" step="1"
           onchange="sendDsServoTrims()">
    <span class="u">us</span>
    <button class="bsv0" id="dbs0" onclick="dsServoGo(0)" style="flex:none;padding:8px 14px;">Test</button>
  </div>
  <div class="sr">
    <label>180&deg; pulse</label>
    <input type="number" class="ni med" id="dsv180" value="2000" min="500" max="2500" step="1"
           onchange="sendDsServoTrims()">
    <span class="u">us</span>
    <button class="bsv180" id="dbs180" onclick="dsServoGo(180)" style="flex:none;padding:8px 14px;">Test</button>
  </div>
  <button class="bsv" style="width:100%;margin-top:8px;" onclick="cmd('save')">&#128190; Save to EEPROM</button>
</div>

</div><!-- /advView -->

<script>
const CMAX=100;
let srv={escReady:false,arming:false,userArmed:false,motorOn:false,
  pct:0,curPct:0,pulseUs:1500,apPct:0,rampSec:5.0,
  rawA:0,filtA:0,
  approachSlowing:false,gotoIdx:-1,apRemaining:0,
  posM:0,velRPM:0,velMs:0,encCounts:0,
  cpr:16384,driveRatio:17.07,pitchMm:100,encInvert:true,
  apSlw:6,apZone:3000000,maxSpd:12,
  stations:[],servoDeg:0,motorReverse:false,servoRetPend:false,
  dsServoDeg:0,dsServoPulse:1500,dsSv0:680,dsSv180:1960,
  demoState:'idle',demoActive:false,demoWaitMs:0,demoRemMs:0,
  imuReady:false,imuCal:false,imuCalibrating:false,imuCalPct:0,imuStab:true,imuAxis:1,imuAxisName:'Y-axis',imuRollRaw:0,imuRollFilt:0,imuX:0,imuY:0,imuZ:0};
let drag=false,spdT=null,revFlag=false;

function toggleView(){
  document.body.classList.toggle('adv');
  document.getElementById('hburg').classList.toggle('act',document.body.classList.contains('adv'));
}

function onSli(v){drag=true;document.getElementById('pctD').textContent=v;
  clearTimeout(spdT);spdT=setTimeout(()=>setSpd(v),100)}
function onSlc(v){clearTimeout(spdT);drag=false;setSpd(v)}
async function setSpd(v){await f('/cmd?action=speed&pct='+v)}

async function sendRamp(v){
  let s=parseFloat(v); if(isNaN(s)||s<0.3){s=0.3;document.getElementById('rampI').value='0.3'}
  await f('/cmd?action=ramp&sec='+s.toFixed(2))
}

async function toggleDir(){
  revFlag=!revFlag;
  const b=document.getElementById('btnDir');
  b.textContent=revFlag?'\u2190 Reverse':'\u2192 Forward';
  b.className=revFlag?'bfr active':'bfr';
  await f('/cmd?action=dir&rev='+(revFlag?1:0))
}

async function sendApproachSettings(){
  const sl=parseFloat(document.getElementById('setSlw').value)||2;
  const zn=parseInt(document.getElementById('setZone').value)||870000;
  const mx=parseFloat(document.getElementById('setMax').value)||15;
  await f(`/cmd?action=approachsettings&sl=${sl.toFixed(1)}&zone=${zn}&mx=${mx.toFixed(1)}`)
}

async function sendMs(){
  const cpr=parseInt(document.getElementById('setCPR').value)||16384;
  const drv=parseFloat(document.getElementById('setDrv').value)||1;
  const pit=parseFloat(document.getElementById('setPit').value)||100;
  const inv=document.getElementById('setInv').checked?1:0;
  await f(`/cmd?action=settings&cpr=${cpr}&drive=${drv.toFixed(3)}&pitch=${pit.toFixed(2)}&inv=${inv}`)
}

function captureCount(inputId){ document.getElementById(inputId).value=srv.encCounts }

async function addStation(){
  const nm=document.getElementById('newName').value.trim();
  const t=parseInt(document.getElementById('newType').value);
  const c=parseInt(document.getElementById('newCount').value)||0;
  if(!nm){alert('Enter a station name.');return;}
  await f(`/cmd?action=addstation&name=${encodeURIComponent(nm)}&type=${t}&count=${c}`);
  document.getElementById('newName').value='';
  document.getElementById('newCount').value='0';
  await refresh();
}

async function captureStationCount(idx){
  await f(`/cmd?action=setstationcount&idx=${idx}&count=${srv.encCounts}`);
  await refresh();
}
async function setStationCount(idx,val){ const c=parseInt(val)||0; await f(`/cmd?action=setstationcount&idx=${idx}&count=${c}`) }
async function gotoStation(i){ await f('/cmd?action=gotostation&idx='+i); await refresh(); }
async function deleteStation(i){ if(!confirm('Delete this station?')) return; await f('/cmd?action=deletestation&idx='+i); await refresh(); }
async function cmd(c){ await f('/cmd?action='+c); await refresh(); }
async function doArm(){ document.getElementById('btnArm').disabled=true; await f('/cmd?action=arm') }
async function servoGo(d){ await f('/cmd?action=servo&deg='+d); await refresh(); }
async function sendServoTrims(){
  const p0=parseInt(document.getElementById('sv0').value)||0;
  const p180=parseInt(document.getElementById('sv180').value)||2048;
  await f('/cmd?action=servotrims&p0='+p0+'&p180='+p180)
}
async function dsServoGo(d){ await f('/cmd?action=dsservo&deg='+d); await refresh(); }
async function sendDsServoTrims(){
  const p0=parseInt(document.getElementById('dsv0').value)||500;
  const p180=parseInt(document.getElementById('dsv180').value)||2500;
  await f('/cmd?action=dstrims&p0='+p0+'&p180='+p180)
}
async function sendImuToggle(){
  const en=document.getElementById('imuEnable').checked?1:0;
  await f('/cmd?action=imu&stab='+en);
  await refresh();
}
async function sendImuZero(){
  await f('/cmd?action=imuzero');
  await refresh();
}
async function sendImuAxis(){
  const axis=parseInt(document.getElementById('imuAxis').value)||0;
  await f('/cmd?action=imuaxis&axis='+axis);
  await refresh();
}
async function f(u){ try{ await fetch(u) }catch(e){} }

function updAm(filt,raw){
  const el=document.getElementById('amv'),b=document.getElementById('cbar');
  el.innerHTML=filt.toFixed(2)+'<span class="amu">A</span>';
  const lv=filt>75?'high':filt>40?'warn':'';
  el.className='amv'+(lv?' '+lv:'');
  b.style.width=Math.min(100,filt/CMAX*100)+'%';
  b.className='cf'+(lv?' '+lv:'');
  document.getElementById('rawA').textContent=raw.toFixed(2);
}

function stTypeCls(t){return ['','home','drivethru','drone'][t]||''}
function stTypeLbl(t){return ['','Home','Drive Thru','Drone Pickup'][t]||'?'}

function buildStationList(sts, gotoIdx){
  const d=document.getElementById('stationList');
  if(!sts||!sts.length){
    d.innerHTML='<p style="font-size:.8rem;color:var(--mut)">No stations defined yet.</p>';
    return;
  }
  const armed=srv.userArmed&&srv.escReady&&!srv.motorOn;
  d.innerHTML=sts.map(s=>{
    const active=(gotoIdx===s.i);
    const cls=stTypeCls(s.t);
    return `<div class="st-item${active?' st-active':''}">
      <div style="flex:1;min-width:0;">
        <div class="st-nm">${active?'&#9654;&nbsp;':''}${s.n}
          <span class="st-badge ${cls}">${stTypeLbl(s.t)}</span>
        </div>
        <div class="st-inf" style="display:flex;align-items:center;gap:6px;margin-top:4px;">
          <input type="number" class="ni" value="${s.c}" step="1000"
            style="width:110px;font-size:.78rem;padding:3px 6px;"
            onchange="setStationCount(${s.i},this.value)"
            title="Encoder count for this station">
          <button class="bcap" style="padding:4px 8px;font-size:.72rem;"
            onclick="captureStationCount(${s.i})" title="Set to current encoder position">Capture</button>
        </div>
      </div>
      <button class="bgo" onclick="gotoStation(${s.i})"
        ${!armed||active?'disabled':''} style="flex:none;padding:8px 14px;">Go</button>
      <button class="bdel" onclick="deleteStation(${s.i})" title="Delete">&#128465;</button>
    </div>`;
  }).join('');
}

function updateUI(d){
  srv=d;
  document.getElementById('bn').classList.toggle('h',d.escReady);
  const sb=document.getElementById('stBadge');
  if(d.approachSlowing){sb.className='badge seeking';sb.textContent='\u25b6 Approach'}
  else if(d.gotoIdx>=0&&d.motorOn){sb.className='badge running';sb.textContent='\u2192 Goto'}
  else if(d.motorOn){sb.className='badge running';sb.textContent='\u26a1 Running'}
  else if(d.arming){sb.className='badge arming';sb.textContent='\u23f3 Arming...'}
  else if(d.userArmed){sb.className='badge armed';sb.textContent='\u2713 Armed'}
  else{sb.className='badge stopped';sb.textContent='\u25a0 Stopped'}

  document.getElementById('puUs').textContent=d.pulseUs+' us';
  document.getElementById('bTgt').style.width=d.pct+'%';
  document.getElementById('pTgt').textContent=d.pct+'%';
  document.getElementById('bAct').style.width=d.curPct+'%';
  document.getElementById('pAct').textContent=d.curPct+'%';

  const azb=document.getElementById('apZoneBar');
  if(d.gotoIdx>=0&&d.motorOn){
    azb.classList.add('vis');
    const pct=d.apZone>0?Math.max(0,Math.min(100,(1-(d.apRemaining/d.apZone))*100)):0;
    document.getElementById('apZoneFill').style.width=pct+'%';
    document.getElementById('apZoneRem').textContent=(d.approachSlowing?'Slowing ':'')+Math.max(0,d.apRemaining)+' cts';
  } else {
    azb.classList.remove('vis');
  }

  updAm(d.filtA,d.rawA);
  document.getElementById('ePos').innerHTML=d.posM.toFixed(3)+'<span class="ecu">m</span>';
  document.getElementById('eRPM').innerHTML=d.velRPM.toFixed(1)+'<span class="ecu">RPM</span>';
  document.getElementById('eMs').innerHTML=d.velMs.toFixed(3)+'<span class="ecu">m/s</span>';
  document.getElementById('eCnt').textContent=d.encCounts;

  const ba=document.getElementById('btnArm');
  if(d.userArmed){ba.disabled=true;ba.textContent='\u2713 Armed'}
  else if(d.arming||!d.escReady){ba.disabled=true;ba.textContent=d.arming?'\u23f3 Arming...':'\ud83d\udd12 Arm Motor'}
  else{ba.disabled=false;ba.textContent='\ud83d\udd12 Arm Motor'}
  document.getElementById('apw').classList.toggle('vis',d.arming);
  document.getElementById('apb').style.width=d.apPct+'%';

  document.getElementById('btnStart').disabled=!d.userArmed||d.motorOn||!d.escReady;
  document.getElementById('btnStop').disabled=!d.motorOn&&!d.approachSlowing;

  // ── Home page: demo button + stop button + demo status line ──
  const bd=document.getElementById('btnDemo');
  const bsh=document.getElementById('btnStopHome');
  const ds=document.getElementById('demoState');
  const ready=d.userArmed&&d.escReady&&!d.arming;
  bd.disabled=!ready||d.demoActive;
  bsh.disabled=!d.motorOn&&!d.approachSlowing&&!d.demoActive;
  if(d.demoActive){
    bd.textContent='\u23f3 Demo Running';
  } else {
    bd.textContent='\u25b6 Start Demo';
  }
  let msg='', cls='demo-state';
  if(!d.escReady){ msg='ESC initialising\u2026'; cls+=' warn'; }
  else if(d.arming){ msg='Arming motor\u2026'; cls+=' warn'; }
  else if(!d.userArmed){ msg='Arm the motor to enable the demo.'; }
  else {
    switch(d.demoState){
      case 'imu-rezero':
        msg='Re-zeroing IMU\u2026'; cls+=' warn'; break;
      case 'boarding-180':{
        const s=Math.ceil(d.demoRemMs/1000);
        msg='Boarding pose: <span class="v">'+s+' s</span> remaining'; cls+=' run'; break;}
      case 'boarding-return':
        msg='Returning STS3215 to 0\u00b0\u2026'; cls+=' run'; break;
      case 'goto-drone':
        msg='En route to <span class="v">Drone Pickup</span>'+(d.approachSlowing?' (approach)':''); cls+=' run'; break;
      case 'at-drone-wait':{
        const s=Math.ceil(d.demoRemMs/1000);
        msg='At Drone Pickup: <span class="v">'+s+' s</span> until Drive Thru'; cls+=' run'; break;}
      case 'goto-drivethru':
        msg='En route to <span class="v">Drive Thru</span>'+(d.approachSlowing?' (approach)':''); cls+=' run'; break;
      case 'idle':
      default:
        msg='Press Start Demo to run the routine.';
        break;
    }
  }
  ds.className=cls;
  ds.innerHTML=msg;
  const sl=document.getElementById('slider');
  sl.disabled=!d.userArmed||!d.escReady;
  if(!drag){sl.value=d.pct;document.getElementById('pctD').textContent=d.pct}

  const aid=document.activeElement?document.activeElement.id:'';
  if(aid!='rampI') document.getElementById('rampI').value=d.rampSec.toFixed(1);
  if(d.motorReverse!==revFlag){revFlag=d.motorReverse;
    const b=document.getElementById('btnDir');
    b.textContent=revFlag?'\u2190 Reverse':'\u2192 Forward';
    b.className=revFlag?'bfr active':'bfr'}

  if(aid!='setSlw')  document.getElementById('setSlw').value=d.apSlw;
  if(aid!='setZone') document.getElementById('setZone').value=d.apZone;
  if(aid!='setMax')  document.getElementById('setMax').value=d.maxSpd;
  if(aid!='setCPR') document.getElementById('setCPR').value=d.cpr;
  if(aid!='setDrv') document.getElementById('setDrv').value=d.driveRatio.toFixed(2);
  if(aid!='setPit') document.getElementById('setPit').value=d.pitchMm.toFixed(1);
  document.getElementById('setInv').checked=d.encInvert;

  const stsb=document.getElementById('stsBadge');
  stsb.textContent=d.servoRetPend?'Returning…':(d.servoDeg===180?'180°':'0°');
  stsb.className='svb'+(d.servoDeg===180&&!d.servoRetPend?' p180':'');
  document.getElementById('bs0').disabled=d.servoRetPend||(d.servoDeg===0&&!d.servoRetPend);
  document.getElementById('bs180').disabled=d.servoRetPend||(d.servoDeg===180&&!d.servoRetPend);
  if(aid!='sv0') document.getElementById('sv0').value=d.sv0;
  if(aid!='sv180') document.getElementById('sv180').value=d.sv180;

  const dsvb=document.getElementById('dsBadge');
  dsvb.textContent=`${d.dsServoDeg}° / ${d.dsServoPulse} us`;
  dsvb.className='svb'+(d.dsServoDeg>=150?' p180':'');
  document.getElementById('dbs0').disabled=d.imuStab;
  document.getElementById('dbs180').disabled=d.imuStab;
  if(aid!='dsv0') document.getElementById('dsv0').value=d.dsSv0;
  if(aid!='dsv180') document.getElementById('dsv180').value=d.dsSv180;

  const ib=document.getElementById('imuBadge');
  if(!d.imuReady){ib.textContent='Offline'; ib.className='svb off'}
  else if(d.imuCalibrating){ib.textContent=`Calibrating ${d.imuCalPct}%`; ib.className='svb'}
  else if(d.imuStab){ib.textContent='Stabilizing'; ib.className='svb'}
  else if(d.imuCal){ib.textContent='Ready'; ib.className='svb'}
  else{ib.textContent='Uncal'; ib.className='svb off'}
  document.getElementById('imuEnable').checked=d.imuStab;
  document.getElementById('imuEnable').disabled=!d.imuReady||!d.imuCal||d.imuCalibrating;
  document.getElementById('imuZeroBtn').disabled=!d.imuReady||d.imuCalibrating;
  document.getElementById('imuAxis').value=String(d.imuAxis);
  document.getElementById('imuAxis').disabled=!d.imuReady||d.imuCalibrating;
  document.getElementById('imuRawLbl').textContent=d.imuAxisName+' raw';
  document.getElementById('imuFiltLbl').textContent=d.imuAxisName+' filt';
  document.getElementById('imuRaw').innerHTML=d.imuRollRaw.toFixed(1)+'<span class="ecu">&deg;</span>';
  document.getElementById('imuFilt').innerHTML=d.imuRollFilt.toFixed(1)+'<span class="ecu">&deg;</span>';
  document.getElementById('imuX').innerHTML=d.imuX.toFixed(1)+'<span class="ecu">&deg;</span>';
  document.getElementById('imuY').innerHTML=d.imuY.toFixed(1)+'<span class="ecu">&deg;</span>';
  document.getElementById('imuZ').innerHTML=d.imuZ.toFixed(1)+'<span class="ecu">&deg;</span>';

  const lsb=document.getElementById('lsBadge');
  if(d.lineArmed){
    if(d.lineSensed){lsb.className='badge running';lsb.textContent='\u25cf Line'}
    else{lsb.className='badge seeking';lsb.textContent='\u25b6 Armed'}
  } else {
    lsb.className='badge stopped';lsb.textContent='\u25a0 Inactive'
  }

  buildStationList(d.stations, d.gotoIdx);
}

async function refresh(){ try{const r=await fetch('/status');const d=await r.json();updateUI(d)}catch(e){} }
function poll(){
  const fast=srv.arming||!srv.escReady||srv.approachSlowing||srv.motorOn||srv.servoRetPend||srv.imuStab||srv.demoActive;
  setTimeout(async()=>{await refresh();poll()},fast?200:600)
}
poll();refresh();
</script>
</body></html>
)rawhtml";

// ═══════════════════════════════════════════════════════════════════════════
//  HTTP HANDLERS
// ═══════════════════════════════════════════════════════════════════════════
void handleRoot() { server.send_P(200,"text/html",INDEX_HTML); }

void handleStatus() {
  uint8_t apPct=0;
  if (arming) apPct=(uint8_t)min(100UL,(millis()-userArmStartMs)*100UL/ARM_TIME_MS);
  uint16_t actUs=(uint16_t)constrain(currentUsF,(float)THROTTLE_FULL_REV_US,(float)THROTTLE_FULL_FWD_US);

  int32_t apRemaining = (gotoStationIdx>=0)
                        ? max((int32_t)0, gotoTargetCount - encCount)
                        : (int32_t)0;

  String stArr="[";
  for (uint8_t i=0;i<MAX_STATIONS;i++) {
    if (!stations[i].active) continue;
    if (stArr.length()>1) stArr+=",";
    stArr+="{\"i\":";   stArr+=i;
    stArr+=",\"n\":\""; stArr+=stations[i].name;
    stArr+="\",\"t\":"; stArr+=(int)stations[i].type;
    stArr+=",\"c\":";   stArr+=stations[i].encStopCount;
    stArr+="}";
  }
  stArr+="]";

  String j="{";
  j+="\"escReady\":"      +String(escReady    ?"true":"false")+",";
  j+="\"arming\":"        +String(arming      ?"true":"false")+",";
  j+="\"userArmed\":"     +String(userArmed   ?"true":"false")+",";
  j+="\"motorOn\":"       +String(motorOn     ?"true":"false")+",";
  j+="\"motorReverse\":"  +String(motorReverse?"true":"false")+",";
  j+="\"pct\":"           +String(throttlePct)+",";
  j+="\"curPct\":"        +String(rampedPct())+",";
  j+="\"pulseUs\":"       +String(actUs)+",";
  j+="\"apPct\":"         +String(apPct)+",";
  j+="\"rampSec\":"       +String((float)rampTimeMs/1000.f,2)+",";
  j+="\"rawA\":"          +String(rawCurrentA,2)+",";
  j+="\"filtA\":"         +String(filtCurrentA,2)+",";
  j+="\"approachSlowing\":"+String(approachSlowing?"true":"false")+",";
  j+="\"apRemaining\":"   +String(apRemaining)+",";
  j+="\"gotoIdx\":"       +String(gotoStationIdx)+",";
  j+="\"posM\":"          +String(positionM,4)+",";
  j+="\"velRPM\":"        +String(velocityRPM,2)+",";
  j+="\"velMs\":"         +String(velocityMs,4)+",";
  j+="\"encCounts\":"     +String(encCount)+",";
  j+="\"servoDeg\":"      +String(stsServoCmdPos==stsServoPos180?180:0)+",";
  j+="\"sv0\":"           +String(stsServoPos0)+",";
  j+="\"sv180\":"         +String(stsServoPos180)+",";
  j+="\"dsServoDeg\":"    +String(dsServoCmdDeg)+",";
  j+="\"dsServoPulse\":"  +String(dsServoPulseUs)+",";
  j+="\"dsSv0\":"         +String(dsServoPulse0)+",";
  j+="\"dsSv180\":"       +String(dsServoPulse180)+",";
  j+="\"servoRetPend\":"  +String(servoReturnPending?"true":"false")+",";
  j+="\"imuReady\":"      +String(imuReady?"true":"false")+",";
  j+="\"imuCal\":"        +String(imuCalibrated?"true":"false")+",";
  j+="\"imuCalibrating\":"+String(imuCalibrating?"true":"false")+",";
  j+="\"imuCalPct\":"     +String((int)((imuCalSampleIndex*100UL)/IMU_CAL_SAMPLES))+",";
  j+="\"imuStab\":"       +String(imuStabilizeEnabled?"true":"false")+",";
  j+="\"imuAxis\":"       +String((int)imuAxisSelect)+",";
  j+="\"imuAxisName\":\"" + String(imuAxisName(imuAxisSelect)) + "\",";
  j+="\"imuRollRaw\":"    +String(imuRollRawDeg,2)+",";
  j+="\"imuRollFilt\":"   +String(imuRollFiltDeg,2)+",";
  j+="\"imuX\":"          +String(imuEulerXDeg,2)+",";
  j+="\"imuY\":"          +String(imuEulerYDeg,2)+",";
  j+="\"imuZ\":"          +String(imuEulerZDeg,2)+",";
  j+="\"cpr\":"           +String(countsPerRev)+",";
  j+="\"driveRatio\":"    +String(driveRatio,3)+",";
  j+="\"pitchMm\":"       +String(pitchDiamM*1000.f,2)+",";
  j+="\"encInvert\":"     +String(encInvert?"true":"false")+",";
  j+="\"apSlw\":"         +String(approachSlowPct,1)+",";
  j+="\"apZone\":"        +String(approachZoneCount)+",";
  j+="\"maxSpd\":"        +String(maxSpeedPct,1)+",";
  // Demo state — UI uses these to show the demo button label and the
  // 10 s drone-pickup countdown.  demoTimeRemainingMs is only nonzero in
  // timed-wait states (boarding hold, drone-pickup wait).
  uint32_t demoRemMs = 0;
  if (demoStateDurationMs > 0) {
    const uint32_t elapsed = millis() - demoStateStartMs;
    demoRemMs = (elapsed >= demoStateDurationMs) ? 0 : (demoStateDurationMs - elapsed);
  }
  j+="\"demoState\":\""   +String(demoStateName(demoState))+"\",";
  j+="\"demoActive\":"    +String(demoState!=DEMO_IDLE?"true":"false")+",";
  j+="\"demoWaitMs\":"    +String(demoStateDurationMs)+",";
  j+="\"demoRemMs\":"     +String(demoRemMs)+",";
  j+="\"lineArmed\":"     +String((approachSlowing&&gotoStationIdx>=0&&motorOn)?"true":"false")+",";
  j+="\"lineSensed\":"    +String(lineCurrentState?"false":"true")+",";
  j+="\"stations\":"      +stArr;
  j+="}";
  server.send(200,"application/json",j);
}

void handleCmd() {
  if (!server.hasArg("action")){server.send(400,"text/plain","Missing");return;}
  const String a=server.arg("action");

  if (a=="arm") {
    if (escReady&&!arming&&!userArmed){arming=true;motorOn=false;ledStopLatched=false;userArmStartMs=millis();}
  }
  else if (a=="start") {
    if (userArmed&&escReady&&!approachSlowing){
      motorOn=true;
      ledStopLatched=false;
      gotoStationIdx=-1;   // manual run — no station target
      Serial.printf("START %d%% %s\n",throttlePct,motorReverse?"REV":"FWD");
    }
  }
  else if (a=="stop") {
    motorOn=false;
    approachSlowing=false;
    ledStopLatched=false;
    servoReturnPending=false; pendingGotoIdx=-1;
    gotoStationIdx=-1;
    gotoTargetCount=0;
    cancelDemo("user stop");
    Serial.println("STOP");
  }
  else if (a=="demostart") {
    startDemo();
  }
  else if (a=="demostop") {
    cancelDemo("user demo-stop");
  }
  else if (a=="speed") {
    if (server.hasArg("pct"))
      throttlePct=(uint8_t)constrain(server.arg("pct").toInt(),0,(int)maxSpeedPct);
  }
  else if (a=="ramp") {
    if (server.hasArg("sec")){float s=server.arg("sec").toFloat();
      if(s<0.3f)s=0.3f; rampTimeMs=(uint32_t)(s*1000.f);}
  }
  else if (a=="dir") {
    if (server.hasArg("rev")) motorReverse=(server.arg("rev").toInt()!=0);
  }
  else if (a=="approachsettings") {
    if (server.hasArg("sl"))   approachSlowPct=constrain(server.arg("sl").toFloat(),0.5f,50.f);
    if (server.hasArg("zone")) approachZoneCount=(int32_t)constrain(server.arg("zone").toInt(),10000,100000000);
    if (server.hasArg("mx"))   maxSpeedPct=constrain(server.arg("mx").toFloat(),1.f,100.f);
  }
  else if (a=="settings") {
    if (server.hasArg("cpr"))   countsPerRev=(uint16_t)constrain(server.arg("cpr").toInt(),1,65535);
    if (server.hasArg("drive")) driveRatio=max(0.01f,server.arg("drive").toFloat());
    if (server.hasArg("pitch")) pitchDiamM=max(0.0001f,server.arg("pitch").toFloat()/1000.f);
    if (server.hasArg("inv"))   encInvert=(server.arg("inv").toInt()!=0);
    velocityKF.reset(0.f);
  }
  else if (a=="addstation") {
    if (server.hasArg("name") && server.hasArg("type") && server.hasArg("count")) {
      String nm = server.arg("name");
      StationType t = (StationType)constrain(server.arg("type").toInt(),0,3);
      int32_t cnt = (int32_t)server.arg("count").toInt();
      if (nm.length()>0 && nm.length()<24) {
        if (addStation(nm.c_str(), t, cnt))
          Serial.printf("Station added: [%s] type=%d count=%d\n", nm.c_str(),(int)t,(int)cnt);
        else
          Serial.println("Station add failed: max stations reached.");
      }
    }
  }
  else if (a=="setstationcount") {
    if (server.hasArg("idx") && server.hasArg("count")) {
      int idx = server.arg("idx").toInt();
      if (idx>=0 && idx<MAX_STATIONS && stations[idx].active) {
        stations[idx].encStopCount = (int32_t)server.arg("count").toInt();
        Serial.printf("Station [%s] count updated to %d\n",
                      stations[idx].name, (int)stations[idx].encStopCount);
      }
    }
  }
  else if (a=="gotostation") {
    if (server.hasArg("idx")&&userArmed&&escReady) {
      int idx=server.arg("idx").toInt();
      if (idx>=0&&idx<MAX_STATIONS&&stations[idx].active) {
        if (stsServoCmdPos != stsServoPos0) {
          ledStopLatched = false;
          stsServoMove(stsServoPos0, "goto-return");
          servoReturnPending = true;
          servoReturnStartMs = millis();
          pendingGotoIdx     = idx;
          Serial.printf("Goto [%s]: STS3215 returning — motor deferred %dms\n",
                        stations[idx].name,(int)SERVO_RETURN_WAIT_MS);
        } else {
          startGotoMotion(idx);
        }
      }
    }
  }
  else if (a=="servo") {
    if (server.hasArg("deg")) {
      stsServoMove(server.arg("deg").toInt()==180 ? stsServoPos180 : stsServoPos0, "manual");
    }
  }
  else if (a=="servotrims") {
    if (server.hasArg("p0"))
      stsServoPos0   = (uint16_t)constrain(server.arg("p0").toInt(),   0, 4095);
    if (server.hasArg("p180"))
      stsServoPos180 = (uint16_t)constrain(server.arg("p180").toInt(), 0, 4095);
    Serial.printf("STS3215 trims updated: 0deg=%u  180deg=%u\n", stsServoPos0, stsServoPos180);
  }
  else if (a=="dsservo") {
    if (server.hasArg("deg")) {
      if (imuStabilizeEnabled) imuStabilizeEnabled = false;
      dsServoMove(server.arg("deg").toInt(), "manual");
    }
  }
  else if (a=="dstrims") {
    if (server.hasArg("p0"))
      dsServoPulse0   = (uint16_t)constrain(server.arg("p0").toInt(),   DS_SERVO_MIN_PULSE_US, DS_SERVO_MAX_PULSE_US);
    if (server.hasArg("p180"))
      dsServoPulse180 = (uint16_t)constrain(server.arg("p180").toInt(), DS_SERVO_MIN_PULSE_US, DS_SERVO_MAX_PULSE_US);
    if (dsServoPulse180 < dsServoPulse0) {
      const uint16_t tmp = dsServoPulse0;
      dsServoPulse0 = dsServoPulse180;
      dsServoPulse180 = tmp;
    }
    dsServoAttachOrReattach();
    dsServoWriteAngleInternal(dsServoCmdDeg);
    Serial.printf("DS3245 pulse trims updated: 0deg=%u us  180deg=%u us\n", dsServoPulse0, dsServoPulse180);
  }
  else if (a=="imu") {
    if (server.hasArg("stab")) {
      const bool want = (server.arg("stab").toInt() != 0);
      if (!want) {
        imuStabilizeEnabled = false;
      } else if (imuReady && imuCalibrated) {
        imuStabilizeEnabled = true;
      } else {
        Serial.println("IMU stabilization request ignored: IMU not ready/calibrated.");
      }
    }
  }
  else if (a=="imuzero") {
    if (imuReady) {
      beginIMUCalibration();
      dsServoMove(DS_SERVO_NEUTRAL_DEG, "imu-rezero");
      Serial.printf("IMU %s re-zero requested from web UI.\n", imuAxisName(imuAxisSelect));
    } else {
      Serial.println("IMU re-zero request ignored: IMU not ready.");
    }
  }
  else if (a=="imuaxis") {
    if (server.hasArg("axis")) {
      imuAxisSelect = (ImuAxis)constrain(server.arg("axis").toInt(), 0, 2);
      imuCalibrated = false;
      imuRollRawDeg = 0.0f;
      imuRollFiltDeg = 0.0f;
      if (imuReady) beginIMUCalibration();
      Serial.printf("IMU control axis set to %s.\n", imuAxisName(imuAxisSelect));
    }
  }
  else if (a=="zeroenc") {
    requestEncoderZero();
    Serial.println("Encoder zeroed.");
  }
  else if (a=="deletestation") {
    if (server.hasArg("idx")) {
      int idx = server.arg("idx").toInt();
      if (idx>=0 && idx<MAX_STATIONS && stations[idx].active) {
        Serial.printf("Deleted station: %s\n", stations[idx].name);
        stations[idx].active = false;
        stations[idx].name[0] = '\0';
        if (stationCount > 0) stationCount--;
      }
    }
  }
  else if (a=="save") { savePrefs(); }

  server.send(200,"text/plain","OK");
}

void handleNotFound(){server.send(404,"text/plain","Not found");}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200); delay(250);
  Serial.println("\n=== DD Trolley Motor Control v12.18 (MAXYNOS 70A AM32) ===");

  loadPrefs();

  // NeoPixel LED strip
  ledPixels.begin();
  ledPixels.clear();
  ledPixels.show();
  Serial.println("NeoPixel ready on D6.");

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(CURRENT_PIN, ADC_11db);
  currentKF.init(0.01f,0.8f,0.f);
  velocityKF.init(1.0f,5.0f,0.f);
  imuRollKF.init(0.02f,0.25f,0.f);

  // STS3215 servo — Serial1 on D0/D1 via URT-1 half-duplex adapter
  delay(200);
  Serial1.begin(STS_SERVO_BAUD, SERIAL_8N1, STS_SERVO_UART_RX, STS_SERVO_UART_TX);
  delay(50);
  stsServo.pSerial = &Serial1;
  stsServoEnabled = true;
  delay(200);
  stsServoMove(stsServoPos0, "boot");
  Serial.println("STS3215 ready on Serial1 via URT-1.");

  // DS3245 servo + BNO055 IMU
  // Timers 0–2 go to the ESP32Servo pool (three timers, one servo = ample).
  // Timer 3 is intentionally NOT allocated here — it is reserved exclusively
  // for the ESC raw-LEDC channel (channel 7 → timer 3 on ESP32-S3).
  // Keeping timer 3 out of the ESP32Servo pool prevents any future servo
  // attach() call from reconfiguring it and disrupting the ESC PWM.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  dsServoCmdDeg = DS_SERVO_NEUTRAL_DEG;
  dsServoPulseUs = dsServoDegToPulseUs(dsServoCmdDeg);
  dsServoAttachOrReattach();
  dsServoMove(90, "boot-center");
  if (dsServoEnabled) Serial.println("DS3245 servo ready on D8.");

  Wire.begin();
  initIMU();

  // SPI for AS5047P
  pinMode(ENC_CS_PIN,OUTPUT); digitalWrite(ENC_CS_PIN,HIGH);
  SPI.begin();
  Serial.println("AS5047P ready on SPI.");

  // ESC — MAXYNOS 70A AM32 (raw LEDC — keeps ESC off the ESP32Servo timer pool)
  // ledcAttachChannel() used for both API versions so the channel is always
  // pinned to ESC_LEDC_CHANNEL=7, never auto-assigned to 0 (DS3245 channel).
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  if (!ledcAttachChannel(ESC_PIN, ESC_PWM_FREQ_HZ, ESC_PWM_RESOLUTION, ESC_LEDC_CHANNEL)) {
    Serial.println("ERROR: ledcAttachChannel failed."); while(true) delay(1000); }
#else
  ledcSetup(ESC_LEDC_CHANNEL, ESC_PWM_FREQ_HZ, ESC_PWM_RESOLUTION);
  ledcAttachPin(ESC_PIN, ESC_LEDC_CHANNEL);
#endif
  const uint32_t now=millis();
  bootArmStartMs=now; lastRampMs=now; lastCurrentMs=now;
  writeEscPulseUs(THROTTLE_NEUTRAL_US);
  currentUsF=(float)THROTTLE_NEUTRAL_US;
  Serial.println("Boot: ESC arming at 1500us (neutral) for 3s...");

  WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID,AP_PASSWORD);
  Serial.print("AP: "); Serial.print(AP_SSID);
  Serial.print("  http://"); Serial.println(WiFi.softAPIP());

  server.on("/",       handleRoot);
  server.on("/status", handleStatus);
  server.on("/cmd",    handleCmd);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Web server ready.");

  if (imuReady) beginIMUCalibration();

  // Line sensor
  pinMode(LINE_SENSOR_PIN, INPUT_PULLUP);
  Serial.println("Line sensor ready on D5 (TCRT5000, active-LOW).");

  // Encoder task — pinned to core 0 so WiFi/web-server activity on core 1
  // cannot delay the sample loop and cause wrap-detection aliasing.
  xTaskCreatePinnedToCore(
    encoderTask,      // task function
    "encoderTask",    // name (for debug)
    2048,             // stack in bytes (SPI + math fits easily)
    nullptr,          // parameter
    5,                // priority — above Arduino loop (1), below WiFi driver
    &encTaskHandle,   // handle
    0                 // core 0
  );
  Serial.println("Encoder task started on core 0.");

  // Line sensor task — also pinned to core 0, same priority as encoder task.
  xTaskCreatePinnedToCore(
    lineSensorTask,       // task function
    "lineSensorTask",     // name (for debug)
    1024,                 // stack — digitalRead + edge logic needs very little
    nullptr,              // parameter
    5,                    // same priority as encoderTask; both on core 0
    &lineSensorTaskHandle,// handle
    0                     // core 0
  );
  Serial.println("Line sensor task started on core 0.");
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
  const uint32_t now=millis();

  // Boot arm
  if (!escReady) {
    writeEscPulseUs(THROTTLE_NEUTRAL_US);   // AM32 arms on neutral, not min
    if (now-bootArmStartMs>=ARM_TIME_MS) {
      escReady=true;
      currentUsF=(float)THROTTLE_NEUTRAL_US;
      Serial.println("Boot: ESC ready.");
    }
  }

  // User arm
  if (arming) {
    writeEscPulseUs(THROTTLE_NEUTRAL_US);
    if (now-userArmStartMs>=ARM_TIME_MS) {
      arming=false; userArmed=true;
      currentUsF=(float)THROTTLE_NEUTRAL_US;
      lastRampMs=now;
      Serial.println("User arm: complete.");
    }
  }

  // Servo return-to-zero before goto motion
  if (servoReturnPending && (now-servoReturnStartMs >= SERVO_RETURN_WAIT_MS)) {
    servoReturnPending=false;
    Serial.println("STS3215 return wait complete — starting goto motion.");
    startGotoMotion(pendingGotoIdx);
    pendingGotoIdx=-1;
  }

  // Encoder — handled by encoderTask on core 0

  // Goto approach check (always)
  updateGotoApproach();

  // Demo automation state machine
  updateDemo();

  // Ramp engine
  if (escReady&&!arming) updateRamp();

  // Current sense
  updateCurrent();

  // IMU calibration + optional servo stabilization
  updateIMUCalibration();
  updateIMU();

  // LED strip (non-blocking)
  updateLEDs();

  server.handleClient();
}
