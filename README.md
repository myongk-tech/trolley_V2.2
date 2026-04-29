# trolley_V2.2
This code is to control trolley V2 used for demo.  It is the black and white trolleys.
This uploaed version is v_12_18_3.  It removes the hobbywing ESC and adds the maxynos ESC.

ESC changed from HobbyWing FPV G2 (one-directional, LEDC 50 Hz, 1000–2000 µs)
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
