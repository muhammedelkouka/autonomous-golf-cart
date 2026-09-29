/*****************************************************
    Name:    Mohammed Alaa Elkouka
    Date:    18_August_2025  |  Revised: May 2026 (v28 - Smooth Human-Like Steering)
    Subject: Master Thesis - Golf Cart Development
    Topic:   Unified Low-level Control - Autonomous + Manual
    Platform: ESP32 DevKit V1 + PCB + CS-D508 + MCP4725

    ---------------------------------------------------------------
    v28 CHANGES (from v27):
      - Steering speed tuned: flat 8 deg/s → dual-speed (cruise 8 / creep 5 deg/s)
      - STEER_SEND_MIN_DELTA increased to 2.0 deg (PC side) - prevents
        micro-commands like 0.20°, 0.31° that cause vibration
      - ESP32 deadbands adjusted to 0.20-0.30 deg to match 2° minimum
      - Key principle: PC only sends commands the stepper can execute
        smoothly. No more infinite loops of tiny corrections.
      - Result: Smooth human-like steering without vibration
    
    v27/v26 CHANGES: (see git history)
    ---------------------------------------------------------------
    Copyrights belong to Mohammed Elkouka
******************************************************/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>

// =====================================================
// SIMULATION MODE — set true for desktop testing,
//                   false for real hardware
// =====================================================
#define SIMULATION_MODE false

// =====================================================
// Wi-Fi credentials
// =====================================================
const char* ssid     = "Muhammed's Galaxy A54 5G";
const char* password = "123456789";

// =====================================================
// MODE SWITCH PASSWORD (web UI)
// =====================================================
#define MODE_SWITCH_PASSWORD "1234"

// =====================================================
// PINS
// =====================================================
const int STEP_PIN   = 2;
const int DIR_PIN    = 4;

#define DAC_ADDRESS   0x60
#define FWD_PIN       18
#define REV_PIN       5
#define BRAKE_PIN_1   19
#define BRAKE_PIN_2   23
#define I2C_SDA       21
#define I2C_SCL       22

// =====================================================
// ██████████  CALIBRATION SECTION  ██████████
//
// HOW TO CALIBRATE (do this once on the real cart):
//
//  Step 1 — Set wheel to mechanical centre, send RESETZERO.
//
//  Step 2 — Drive to one full lock with RIGHT_START (HTTP)
//            until the hard stop is reached.  Serial will
//            print the step count when the limit fires.
//            Confirm it reads ±9000.  If not, update
//            PULSES_TO_LOCK below.
//
//  Step 3 — With the wheel still at lock, place a digital
//            angle gauge (or a phone with an inclinometer
//            app) flat on the wheel hub face.  Read the
//            angle from straight ahead.  Enter it as
//            WHEEL_LOCK_DEG below.
//
//  Step 4 — Flash the firmware.  STEPS_PER_DEGREE is
//            computed automatically.  Verify via the web
//            UI: at full lock it should read ±WHEEL_LOCK_DEG.
//
// ── KNOWN HARDWARE CONSTANTS ─────────────────────────
//  Driver:        CS-D508, default DIP = SW1-SW4 all ON
//  Microsteps:    1 600 steps/rev  (confirmed from datasheet)
//  Belt ratio:    1 : 4  (small stepper sprocket → large column sprocket)
//  Column travel: ~1.4 column revolutions centre → one lock
//  Pulses check:  1.4 col-rev × 4 belt × 1600 steps/rev = 8 960 ≈ 9 000 ✓
// ─────────────────────────────────────────────────────
//
//  ↓↓ ONLY THESE TWO LINES NEED TO CHANGE AFTER MEASURING ↓↓
// =====================================================

// Total pulses from centre to one steering lock.
// Confirmed by driving to hard stop: serial prints the step count.
// Current value: 9 000 (matches kinematic estimate above).
const long  PULSES_TO_LOCK  = 9000L;

// Actual wheel angle (degrees) at full lock, measured with a
// protractor or angle gauge placed on the wheel hub.
// PLACEHOLDER — replace 22.5f with your measured value.
// Typical golf cart range: 20°–28°.  22.5° is the estimate
// that makes STEPS_PER_DEGREE exactly 400; adjust after measuring.
const float WHEEL_LOCK_DEG  = 23.0f;

// ── DERIVED — do not edit these directly ─────────────
// Steps per 1° of WHEEL angle (what STEER commands refer to).
// Formula: PULSES_TO_LOCK / WHEEL_LOCK_DEG
// Example: 9000 / 22.5 = 400.0   (current estimate)
// Example: 9000 / 25.0 = 360.0   (if you measure 25°)
const float STEPS_PER_DEGREE = (float)PULSES_TO_LOCK / WHEEL_LOCK_DEG;

// Software travel limits in pulses (= ± one lock).
// These match the physical hard stops so the motor never
// grind against the mechanical end.
const long  MAX_STEPS_LEFT   = -PULSES_TO_LOCK;
const long  MAX_STEPS_RIGHT  =  PULSES_TO_LOCK;

// =====================================================
// ██████████  STEERING SPEED TUNING  ██████████
//
// Dual-speed profile: cruise at bulk speed, creep for final approach.
// All values are in wheel-degrees/s (STEPS_PER_DEGREE absorbs the
// belt ratio, so these map directly to wheel angle rate).
//
// When remaining distance to target > STEER_CREEP_ZONE → STEER_SPEED_CRUISE.
// When remaining distance ≤ STEER_CREEP_ZONE         → STEER_SPEED_CREEP.
// In real autonomous driving the wheel is rarely more than 2° from the
// last command, so the motor runs at creep speed almost exclusively —
// giving smooth, low-vibration motion without sacrificing range speed.
// =====================================================
const float STEER_SPEED_CRUISE = 8.0f;  // deg/s — bulk travel
const float STEER_SPEED_CREEP  = 5.0f;  // deg/s — final approach
const float STEER_CREEP_ZONE   = 2.0f;  // deg  — switch threshold

// Stop the motor when within this many wheel-degrees of target.
const float STEER_DEADBAND_DEG  = 0.20f;  // Balanced precision

// Ignore a new STEER command if the new target is within this
// many degrees of the current active target (prevents jitter from
// rapid repeated commands with tiny differences).
const float STEER_CMD_DEADBAND  = 0.30f;  // Filter out micro-updates

// Don't restart the motor if the wheel is already this close to
// the commanded angle (prevents enable-signal toggling).
const float STEER_HOLD_DEADBAND_DEG = 0.20f;

// CS-D508 minimum pulse width is 2.5 µs; use 3 µs safety margin.
const unsigned long MIN_PULSE_US = 3UL;

// Physical direction of the motor vs the steering convention.
// +1 = right (positive STEER angle) drives the DIR pin HIGH.
// If the wheel steers the wrong way, flip this to false.
// Using a single constant here avoids the dual-polarity branches
// that existed in v21 (which could flip direction if autonomousMode
// toggled mid-move).
const bool STEER_DIR_HIGH_IS_RIGHT = true;

// =====================================================
// DRIVE PARAMETERS
// =====================================================
const float MAX_VOLTAGE       = 5.0f;
const float MIN_VOLTAGE       = 0.0f;
const float RAMP_UP_V_PER_S   = 3.0f;
const float RAMP_DOWN_V_PER_S = 8.0f;
const float INTERLOCK_ZERO_V  = 0.05f;

// =====================================================
// WHEEL SPEED SENSOR
// =====================================================
// Hall effect sensor on GPIO 13 — same pin as PIDerrorUI students used.
// One magnet glued to the inner rear wheel rim, sensor fixed to axle bracket.
// Confirmed from raw edge log test: 1 pulse per full revolution.
// (Previous students assumed 2 — incorrect. Raw log showed only 1 real pulse
//  per revolution with bounce pulses < 0.1 ms which are rejected by debounce.)
//
// Tyre: 205/50-10  →  diameter = 10" rim + 2×(205×0.50 mm sidewall)
//                   = 254 + 205 = 459 mm = 0.459 m
//   Circumference = 0.459 × π = 1.442 m
//   Distance per pulse = 1.442 / 1 = 1.442 m  (one full revolution)
//
// At autonomous speed ~1 m/s:
//   Real pulse interval = 1.442 / 1.0 = 1.442 s
//   Debounce 5 ms is safe — confirmed bounce < 0.1 ms from raw edge log
//   Timeout set to 4 s — safe margin above 1.442 s real interval
//
// Speed is computed as:  v = DIST_PER_PULSE / pulse_interval_s
// A 4-second timeout with no pulse sets speed to 0 (cart stopped).
// Speed is sent in every GET_STATE and STEER_DONE telemetry packet
// as "WHEEL_MPS,<value>" so the PC uses it directly.

const int   SPEED_SENSOR_PIN     = 13;
const float WHEEL_CIRCUMFERENCE  = 1.442f;   // metres — 205/50-10 tyre
const int   PULSES_PER_REV       = 1;        // confirmed: one magnet, one pulse per full revolution
const float DIST_PER_PULSE       = WHEEL_CIRCUMFERENCE / (float)PULSES_PER_REV; // 1.442 m
const unsigned long DEBOUNCE_US         = 5000UL;    // 5 ms — bounce confirmed < 0.1 ms
const unsigned long MIN_PULSE_US_SPEED  = 480000UL;  // 480 ms = 1.442 m / 3 m/s max cart speed
                                                      // Pulses faster than this are stepper EMI/vibration, not real wheel revolutions
const unsigned long SPEED_TIMEOUT_US    = 4000000UL; // 4 s — safe margin above 1.442 s pulse interval at 1 m/s

// ISR state — all volatile
volatile unsigned long spd_lastPulseUs = 0;
volatile unsigned long spd_gapUs       = 0;   // gap between last two accepted pulses
volatile bool          spd_newPulse    = false;

void IRAM_ATTR onWheelPulse() {
    unsigned long now = micros();
    unsigned long gap = now - spd_lastPulseUs;
    if (gap < DEBOUNCE_US) return;          // reject bounce < 5 ms
    if (gap < MIN_PULSE_US_SPEED) return;   // reject stepper EMI (< 480 ms = faster than 3 m/s)
    spd_gapUs       = gap;
    spd_lastPulseUs = now;
    spd_newPulse    = true;
}

// Returns current wheel speed in m/s.
// Returns 0 if no pulse received in the last SPEED_TIMEOUT_US (4 s).
// Thread-safe: disables interrupts only for the read snapshot.
float getWheelSpeedMps() {
    noInterrupts();
    unsigned long gap     = spd_gapUs;
    unsigned long lastPls = spd_lastPulseUs;
    interrupts();

    // If no recent pulse, cart is stopped
    if (gap == 0 || (micros() - lastPls) > SPEED_TIMEOUT_US) return 0.0f;

    float gap_s = (float)gap / 1000000.0f;
    return DIST_PER_PULSE / gap_s;
}

// =====================================================
// FWD/REV INTERLOCK STATE MACHINE
// =====================================================
enum InterlockState { INTERLOCK_IDLE, INTERLOCK_RAMPING_DOWN, INTERLOCK_SWITCH_RELAY };

// =====================================================
// STATE VARIABLES
// =====================================================

// --- Steering ---
volatile long current_step_position = 0;
volatile int  steerDirection        = 0;   // -1 | 0 | +1
bool          centerRequested       = false;

long  autoTargetSteps    = 0;
bool  autoTargetActive   = false;
bool  steerDoneSent      = false;
unsigned long lastStepMicros = 0;
bool          stepPinState   = false;

// --- Drive ---
float current_voltage = 0.0f;
float target_voltage  = 0.0f;
bool  brake_active    = false;
bool  reverse_mode    = false;
bool  system_ok       = false;

InterlockState interlockState   = INTERLOCK_IDLE;
bool           interlockReverse = false;

float user_max_voltage = MAX_VOLTAGE;

// --- Timing ---
const unsigned long SAFETY_CHECK_MS = 3000UL;
unsigned long lastSafetyCheck  = 0;
unsigned long lastRampUpdateMs = 0;

// =====================================================
// AUTONOMOUS / SERIAL MODE
// =====================================================
bool autonomousMode = false;

const unsigned long SERIAL_TIMEOUT_MS = 2000UL;
unsigned long lastSerialCommand  = 0;
bool          autoStopMsgPrinted = false;

// =====================================================
// WEB SERVER
// =====================================================
WiFiServer server(80);
unsigned long lastClientConnect = 0;
const unsigned long SERVER_TIMEOUT_MS = 30000UL;
bool serverRestarting = false;

// =====================================================
// UDP — AUTONOMOUS COMMAND CHANNEL
// =====================================================
WiFiUDP udpCmd;
WiFiUDP udpTelem;
IPAddress g_pc_ip(0,0,0,0);  // cached PC IP — remoteIP() resets after every parsePacket()
uint16_t  g_pc_port = 4211; // cached source port of last PC command — reply here (= UDP_TELEM_PORT default)

#define UDP_CMD_PORT   4210
#define UDP_TELEM_PORT 4211

char udpBuf[256];

const unsigned long UDP_TIMEOUT_MS = 2000UL;
unsigned long lastUdpCommand = 0;
unsigned long udpCmdCount    = 0;

// =====================================================
// SIMULATION STATE
// =====================================================
float  sim_vehicle_speed = 0.0f;
String sim_last_cmd      = "-";
String sim_steer_status  = "IDLE";

// =====================================================
// FORWARD DECLARATIONS
// =====================================================
void  setDAC(float v);
void  setBrake(bool engaged);
void  setDriveModeImmediate(bool reverse);
void  requestDriveMode(bool reverse);
void  emergencyStop();
bool  verifyDAC();
void  handleHTTP(WiFiClient &client);
void  updateDriveRamp();
void  updateStepper();
void  _checkSteeringStop();
float computeInstantSteeringSpeed();
void  handleSerialCommands();
void  handleUdpCommands();
void  sendUdpTelem(const String& msg);
void  processCommand(String cmd);
void  handleMATLABCommands(String cmd);
void  simulateHardware();
void  restartWebServer();
void  printSystemStatus();

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  if (!SIMULATION_MODE) {
    pinMode(STEP_PIN,    OUTPUT); digitalWrite(STEP_PIN,    LOW);
    pinMode(DIR_PIN,     OUTPUT); digitalWrite(DIR_PIN,     LOW);
    pinMode(FWD_PIN,     OUTPUT); digitalWrite(FWD_PIN,     HIGH);
    pinMode(REV_PIN,     OUTPUT); digitalWrite(REV_PIN,     LOW);
    pinMode(BRAKE_PIN_1, OUTPUT); digitalWrite(BRAKE_PIN_1, LOW);
    pinMode(BRAKE_PIN_2, OUTPUT); digitalWrite(BRAKE_PIN_2, LOW);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);   // 100 kHz — 400 kHz caused occasional DAC stalls
    Wire.setTimeOut(50);

    // Wheel speed sensor — Hall effect on GPIO 13
    // INPUT_PULLUP: sensor pulls line LOW when magnet passes (open-collector output)
    // FALLING edge: trigger on the leading edge of each magnet pass
    pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), onWheelPulse, FALLING);
    Serial.println("[SYSTEM] Wheel speed sensor ISR attached on GPIO 13");
  }

  if (!SIMULATION_MODE) {
    system_ok = verifyDAC();
    if (system_ok) { setDAC(0.0f); Serial.println("[SYSTEM] DAC OK"); }
    else            { Serial.println("[SYSTEM] DAC NOT FOUND - check wiring"); }
  } else {
    system_ok = true;
    Serial.println("[SYSTEM] SIMULATION MODE active");
  }

  Serial.print("Connecting to WiFi: "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  int wt = 0;
  while (WiFi.status() != WL_CONNECTED && wt < 40) { delay(500); Serial.print("."); wt++; }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\n[WIFI] Connected - IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WIFI] Fallback - AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("GolfCart_AP", "12345678");
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  }

  server.begin();
  udpCmd.begin(UDP_CMD_PORT);
  udpTelem.begin(UDP_TELEM_PORT);
  Serial.printf("[UDP] Listening on port %d\n", UDP_CMD_PORT);
  lastClientConnect = millis();
  lastRampUpdateMs  = millis();

  Serial.println("\n==========================================");
  Serial.println("  GOLF CART CONTROL SYSTEM READY  v26");
  Serial.println("==========================================");
  Serial.printf("  PULSES_TO_LOCK  : %ld\n",   PULSES_TO_LOCK);
  Serial.printf("  WHEEL_LOCK_DEG  : %.2f\n",  WHEEL_LOCK_DEG);
  Serial.printf("  STEPS_PER_DEGREE: %.2f\n",  STEPS_PER_DEGREE);
  Serial.printf("  STEER_CRUISE    : %.1f deg/s  STEER_CREEP: %.1f deg/s  zone: %.1f deg\n",
               STEER_SPEED_CRUISE, STEER_SPEED_CREEP, STEER_CREEP_ZONE);
  printSystemStatus();
  Serial.println("Commands: AUTO MANUAL STATUS REBOOT RECOVER HELP ESTOP");
  Serial.println("  STEER,<angle>  SPEED,<V>  BRAKE,<0|1>  MODE,<0|1>  FWD_MODE");
  Serial.println("  CENTER  STOP  GET_STATE  RESETZERO  PING");
  Serial.println("==========================================");
  Serial.println("ESP32_READY_FOR_COMMANDS");
  Serial.print("HTTP:  http://"); Serial.println(WiFi.localIP());
  Serial.printf("UDP:   %s:%d\n", WiFi.localIP().toString().c_str(), UDP_CMD_PORT);
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  // 1. Serial commands
  handleSerialCommands();

  // 1b. UDP commands — drains ALL pending packets
  handleUdpCommands();

  // 2. Autonomous timeout: zero speed, hold steering angle
  if (autonomousMode) {
    unsigned long lastCmd = max(lastSerialCommand, lastUdpCommand);
    if (millis() - lastCmd > SERIAL_TIMEOUT_MS) {
      if (target_voltage > 0.1f) {
        target_voltage = 0.0f;
        if (!autoStopMsgPrinted) {
          Serial.println("[AUTO-STOP] No commands 2s - speed zeroed, steering held");
          autoStopMsgPrinted = true;
        }
      }
    } else {
      autoStopMsgPrinted = false;
    }
  }

  // 3. HTTP server
  if (!serverRestarting) {
    WiFiClient client = server.available();
    if (client) {
      lastClientConnect = millis();
      unsigned long clientDeadline = millis() + 5000UL;
      while (!client.available() && millis() < clientDeadline) {
        updateStepper();
        updateDriveRamp();
        delayMicroseconds(50);
      }
      if (client.available()) handleHTTP(client);
      // Pump stepper immediately after handleHTTP so the HTML build
      // time does not create a step gap / jerk.
      updateStepper();
      client.stop();
    }
    if (millis() - lastClientConnect > SERVER_TIMEOUT_MS) {
      serverRestarting = true;
    }
  } else {
    restartWebServer();
    serverRestarting  = false;
    lastClientConnect = millis();
  }

  // 4. Core motion tasks
  updateStepper();
  updateDriveRamp();

  // 5. Simulation
  if (SIMULATION_MODE) simulateHardware();

  // 6. Periodic DAC heartbeat
  unsigned long now = millis();
  if (now - lastSafetyCheck >= SAFETY_CHECK_MS) {
    lastSafetyCheck = now;
    if (!SIMULATION_MODE) {
      bool ok = verifyDAC();
      if (!ok) {
        Serial.println("[SAFETY] DAC not responding - ESTOP!");
        emergencyStop();
        system_ok = false;
      } else if (!system_ok) {
        system_ok = true;
        Serial.println("[SAFETY] DAC recovered");
      }
    }
  }
}

// =====================================================
// STEERING SPEED — dual-speed profile
//   > STEER_CREEP_ZONE remaining → cruise
//   ≤ STEER_CREEP_ZONE remaining → creep
// =====================================================
float computeInstantSteeringSpeed() {
  float remaining = fabsf((float)(autoTargetSteps - current_step_position) / STEPS_PER_DEGREE);
  if (remaining <= STEER_CREEP_ZONE) {
    return STEER_SPEED_CREEP;
  }
  return STEER_SPEED_CRUISE;
}

// =====================================================
// CHECK STEERING STOP — called after every step pulse
// Catches overshoot in both directions.
// =====================================================
void _checkSteeringStop() {
  if (centerRequested) {
    if ((steerDirection < 0 && current_step_position <= 0) ||
        (steerDirection > 0 && current_step_position >= 0)) {
      steerDirection        = 0;
      centerRequested       = false;
      current_step_position = 0;
      autoTargetActive      = false;
      Serial.println("[STEERING] Centered at 0 deg");
      if (autonomousMode) {
        String done = "STEER_DONE,0.00,0,WHEEL_MPS," + String(getWheelSpeedMps(), 4);
        Serial.println(done);
        sendUdpTelem(done);
      }
    }
    return;
  }

  if (autoTargetActive) {
    long  diff     = autoTargetSteps - current_step_position;
    float degError = fabsf((float)diff / STEPS_PER_DEGREE);

    // Overshoot: passed target — stop and snap
    bool overshoot = (steerDirection > 0 && current_step_position > autoTargetSteps) ||
                     (steerDirection < 0 && current_step_position < autoTargetSteps);

    if (degError <= STEER_DEADBAND_DEG || overshoot) {
      steerDirection   = 0;
      autoTargetActive = false;
      if (overshoot) current_step_position = autoTargetSteps;
      if (!steerDoneSent) {
        steerDoneSent    = true;
        float finalAngle = (float)current_step_position / STEPS_PER_DEGREE;
        // Include WHEEL_MPS so PC updates speed estimate immediately on steer completion
        String done = "STEER_DONE," + String(finalAngle, 2) + "," +
                      String(current_step_position) + ",WHEEL_MPS," +
                      String(getWheelSpeedMps(), 4);
        Serial.println(done);
        sendUdpTelem(done);
      }
    }
  }
}

// =====================================================
// UPDATE STEPPER — non-blocking, called every loop tick
//
// DIR_PIN convention (v23 unified):
//   STEER_DIR_HIGH_IS_RIGHT = true  → HIGH = right (+), LOW = left (-)
//   STEER_DIR_HIGH_IS_RIGHT = false → inverted (flip if wired backwards)
// Single constant replaces the dual autonomous/manual polarity
// branches that were fragile across mode transitions.
// =====================================================
void updateStepper() {
  if (SIMULATION_MODE) return;

  // Hard limit guard — never grind into mechanical stops
  if (steerDirection != 0) {
    if ((steerDirection < 0 && current_step_position <= MAX_STEPS_LEFT) ||
        (steerDirection > 0 && current_step_position >= MAX_STEPS_RIGHT)) {
      steerDirection   = 0;
      autoTargetActive = false;
      Serial.println("[STEERING] Hard limit reached");
      // Send STEER_DONE so PC does not hang waiting for an ack that would never arrive
      float limitAngle = (float)current_step_position / STEPS_PER_DEGREE;
      String done = "STEER_DONE," + String(limitAngle, 2) + "," +
                    String(current_step_position) + ",WHEEL_MPS," +
                    String(getWheelSpeedMps(), 4);
      sendUdpTelem(done);
    }
  }

  if (steerDirection == 0) {
    // Hold position: keep STEP low, leave DIR and enable as-is.
    // CS-D508 holds motor current when enable stays HIGH → no drift.
    if (stepPinState) {
      digitalWrite(STEP_PIN, LOW);
      stepPinState = false;
    }
    return;
  }

  // Set DIR before pulse (CS-D508 requires DIR stable before STEP rising edge)
  bool wantHigh = (steerDirection > 0) ? STEER_DIR_HIGH_IS_RIGHT : !STEER_DIR_HIGH_IS_RIGHT;
  digitalWrite(DIR_PIN, wantHigh ? HIGH : LOW);

  // Latch halfPeriod at the rising edge and reuse it for the falling edge
  // of the SAME pulse.  Recomputing per-tick caused asymmetric pulse widths
  // (audible jitter) when speed changes inside the slowdown zone.
  static unsigned long latchedHalfPeriod = 0;

  float instSpeed = computeInstantSteeringSpeed();

  // Speed dropped to zero inside slowdown zone — treat as arrival
  if (instSpeed < 0.1f) {
    steerDirection = 0;
    if (stepPinState) { digitalWrite(STEP_PIN, LOW); stepPinState = false; }
    _checkSteeringStop();
    return;
  }

  unsigned long now = micros();

  if (!stepPinState) {
    // Rising edge — compute period from current speed
    float stepsPerSec   = instSpeed * STEPS_PER_DEGREE;
    latchedHalfPeriod   = (unsigned long)(500000.0f / stepsPerSec);
    if (latchedHalfPeriod < MIN_PULSE_US) latchedHalfPeriod = MIN_PULSE_US;

    if (now - lastStepMicros >= latchedHalfPeriod) {
      digitalWrite(STEP_PIN, HIGH);
      stepPinState   = true;
      lastStepMicros = now;
    }
  } else {
    // Falling edge — use same period as rising edge
    if (now - lastStepMicros >= latchedHalfPeriod) {
      digitalWrite(STEP_PIN, LOW);
      stepPinState   = false;
      lastStepMicros = now;
      current_step_position += (steerDirection > 0) ? 1 : -1;
      _checkSteeringStop();
    }
  }
}

// =====================================================
// SIMULATION — advances step counter and display vars
// =====================================================
void simulateHardware() {
  static unsigned long lastSimStep = 0;
  unsigned long now = micros();

  if (steerDirection != 0) {
    float instSpeed = computeInstantSteeringSpeed();
    if (instSpeed < 0.1f) {
      steerDirection = 0;
      _checkSteeringStop();
    } else {
      unsigned long interval = (unsigned long)(1000000.0f / (instSpeed * STEPS_PER_DEGREE));
      if (now - lastSimStep >= interval) {
        lastSimStep = now;
        if      (steerDirection > 0 && current_step_position < MAX_STEPS_RIGHT) { current_step_position++; }
        else if (steerDirection < 0 && current_step_position > MAX_STEPS_LEFT)  { current_step_position--; }
        else steerDirection = 0;
        _checkSteeringStop();
      }
    }
  }

  sim_vehicle_speed = (current_voltage / MAX_VOLTAGE) * 8.0f;

  if      (steerDirection > 0) sim_steer_status = "MOVING RIGHT";
  else if (steerDirection < 0) sim_steer_status = "MOVING LEFT";
  else if (autoTargetActive)   sim_steer_status = "APPROACHING";
  else                         sim_steer_status = "IDLE";
}

// =====================================================
// DRIVE RAMP + FWD/REV INTERLOCK STATE MACHINE
// =====================================================
void updateDriveRamp() {
  if (!system_ok) {
    if (current_voltage != 0.0f) setDAC(0.0f);
    return;
  }

  unsigned long now = millis();
  float dt = (float)(now - lastRampUpdateMs) / 1000.0f;
  if (dt <= 0.0f) return;
  lastRampUpdateMs = now;

  if (brake_active) {
    if (current_voltage != 0.0f) setDAC(0.0f);
    return;
  }

  switch (interlockState) {
    case INTERLOCK_RAMPING_DOWN:
      target_voltage = 0.0f;
      if (current_voltage > INTERLOCK_ZERO_V) {
        float step = RAMP_DOWN_V_PER_S * dt;
        float next = current_voltage - step;
        setDAC(next > 0.0f ? next : 0.0f);
      } else {
        setDAC(0.0f);
        interlockState = INTERLOCK_SWITCH_RELAY;
      }
      return;

    case INTERLOCK_SWITCH_RELAY:
      setDriveModeImmediate(interlockReverse);
      interlockState = INTERLOCK_IDLE;
      Serial.println("[INTERLOCK] Relay switched - direction change complete");
      return;

    case INTERLOCK_IDLE:
    default:
      break;
  }

  float diff = target_voltage - current_voltage;
  if (fabsf(diff) < 0.005f) return;

  float rate = (diff > 0.0f) ? RAMP_UP_V_PER_S : RAMP_DOWN_V_PER_S;
  float step = rate * dt;
  if (fabsf(diff) <= step) setDAC(target_voltage);
  else                     setDAC(current_voltage + (diff > 0.0f ? step : -step));
}

// =====================================================
// SERIAL COMMAND DISPATCHER
// =====================================================
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  processCommand(cmd);
}

// =====================================================
// UDP COMMAND HANDLER — drains ALL pending packets
// =====================================================
void handleUdpCommands() {
  for (int pkt = 0; pkt < 8; pkt++) {
    int pktSize = udpCmd.parsePacket();
    if (pktSize <= 0) break;

    // Read payload first — some ESP32 WiFiUDP builds return 0.0.0.0 from
    // remoteIP() until the packet data has been read out of the buffer.
    int len = udpCmd.read(udpBuf, sizeof(udpBuf) - 1);
    if (len <= 0) break;
    udpBuf[len] = '\0';

    // Cache PC IP and source port after read().
    // Using source port instead of hardcoded 4211 handles the case where the
    // PC's bind() failed and it is sending from a random OS-assigned port.
    IPAddress candidate = udpCmd.remoteIP();
    uint16_t  srcPort   = udpCmd.remotePort();
    if (candidate != IPAddress(0,0,0,0)) {
      g_pc_ip   = candidate;
      if (srcPort != 0) g_pc_port = srcPort;
      Serial.printf("[UDP] PC addr cached: %s:%u\n", g_pc_ip.toString().c_str(), g_pc_port);
    } else {
      Serial.println("[UDP] WARNING: remoteIP()=0.0.0.0 after read — g_pc_ip not updated");
    }

    String cmd(udpBuf);
    cmd.trim();
    if (cmd.length() == 0) continue;

    lastUdpCommand = millis();
    udpCmdCount++;
    // Suppress per-packet Serial print while steering is active —
    // at 115200 baud each print blocks ~1.5 ms; 8 packets/loop = ~12 ms
    // stall that starves updateStepper() and causes audible jitter.
    if (steerDirection == 0) {
      Serial.print("[UDP] "); Serial.println(cmd);
    }

    processCommand(cmd);
  }
}

// =====================================================
// SEND TELEMETRY OVER UDP
// =====================================================
void sendUdpTelem(const String& msg) {
  if (g_pc_ip == IPAddress(0,0,0,0)) {
    Serial.println("[TELEM] DROPPED: g_pc_ip=0.0.0.0 — no command received yet");
    return;
  }
  // Send FROM udpCmd (port 4210) not udpTelem (port 4211).
  // The mobile hotspot creates a NAT/routing entry only for port 4210 when the
  // PC sends commands to esp32:4210. Replies from esp32:4211 are silently
  // dropped because the hotspot has no entry for that port.
  // Using udpCmd to reply keeps the traffic on the same port the hotspot knows about.
  udpCmd.beginPacket(g_pc_ip, g_pc_port);
  udpCmd.print(msg);
  uint8_t ok = udpCmd.endPacket();
  if (!ok) Serial.printf("[TELEM] endPacket FAILED -> %s:%u\n",
                         g_pc_ip.toString().c_str(), g_pc_port);
  else     Serial.printf("[TELEM] sent to %s:%u — %s\n",
                         g_pc_ip.toString().c_str(), g_pc_port, msg.c_str());
}

// =====================================================
// COMMAND PROCESSOR
// =====================================================
void processCommand(String cmd) {
  int    comma1  = cmd.indexOf(',');
  String cmdType = (comma1 == -1) ? cmd : cmd.substring(0, comma1);
  String param   = (comma1 == -1) ? "" : cmd.substring(comma1 + 1);
  param.trim();
  cmdType.trim();
  cmdType.toUpperCase();

  lastSerialCommand = millis();
  if (steerDirection == 0) {
    Serial.print("[CMD] "); Serial.println(cmd);
  }

  // ── Always-available commands ──────────────────────
  if (cmdType == "AUTONOMOUS" || cmdType == "AUTO") {
    autonomousMode = true;
    Serial.println("[MODE] AUTONOMOUS");
    return;
  }

  if (cmdType == "MANUAL") {
    autonomousMode = false;
    // Auto-center on manual entry: after autonomous the wheel may be
    // at a non-zero angle and the step counter may have drifted.
    // Centering returns to a known physical reference.
    if (current_step_position != 0 || steerDirection != 0) {
      autoTargetSteps  = 0;
      autoTargetActive = true;
      steerDoneSent    = false;
      centerRequested  = true;
      int newDir = (current_step_position < 0) ? 1 : -1;
      if (newDir != steerDirection) steerDirection = newDir;
      Serial.println("[STEERING] Auto-centering on MANUAL entry");
    }
    Serial.println("[MODE] MANUAL");
    Serial.print("  http://"); Serial.println(WiFi.localIP());
    return;
  }

  if (cmdType == "STATUS")      { printSystemStatus(); return; }
  if (cmdType == "REBOOT")      { Serial.println("[SYSTEM] Rebooting..."); delay(500); ESP.restart(); }
  if (cmdType == "RESETSERVER") { restartWebServer(); return; }

  if (cmdType == "RECOVER" || cmdType == "RESET") {
    setBrake(false);
    steerDirection   = 0;
    target_voltage   = 0.0f;
    centerRequested  = false;
    autoTargetActive = false;
    autonomousMode   = false;
    interlockState   = INTERLOCK_IDLE;
    system_ok        = SIMULATION_MODE ? true : verifyDAC();
    Serial.println("[SYSTEM] Recovered");
    printSystemStatus();
    return;
  }

  if (cmdType == "HELP") {
    Serial.println("AUTO MANUAL STATUS REBOOT RECOVER HELP");
    Serial.println("STEER,<angle>  SPEED,<V>  BRAKE,<0|1>  MODE,<0|1>");
    Serial.println("CENTER  STOP  ESTOP  GET_STATE  RESETZERO  PING");
    return;
  }

  if (cmdType == "STATE" || cmdType == "GET_STATE") { handleMATLABCommands("GET_STATE"); return; }
  if (cmdType == "PING")  { Serial.println("MATLAB_PONG"); sendUdpTelem("MATLAB_PONG"); return; }

  if (cmdType == "ESTOP") {
    emergencyStop();
    autonomousMode = false;
    Serial.println("[ESTOP] Emergency stop - MANUAL mode");
    return;
  }

  // ── Mode-gated commands ────────────────────────────
  if (autonomousMode) {
    handleMATLABCommands(cmd);
  } else {
    if (cmdType == "STEER" || cmdType == "SPEED" || cmdType == "BRAKE" ||
        cmdType == "MODE"  || cmdType == "CENTER"|| cmdType == "STOP"  ||
        cmdType == "RESETZERO") {
      Serial.println("[MANUAL] Ignored - send AUTO first");
    } else {
      Serial.print("[CMD] Unknown: "); Serial.println(cmd);
    }
  }
}

// =====================================================
// MATLAB / AUTONOMOUS COMMAND HANDLER
// =====================================================
void handleMATLABCommands(String cmd) {
  int    commaPos = cmd.indexOf(',');
  String cmdType  = (commaPos == -1) ? cmd : cmd.substring(0, commaPos);
  String param    = (commaPos == -1) ? "" : cmd.substring(commaPos + 1);
  param.trim();
  cmdType.trim();
  cmdType.toUpperCase();

  sim_last_cmd = cmd;

  // --------------------------------------------------
  // STEER,<angle>   angle in wheel-degrees, -WHEEL_LOCK_DEG to +WHEEL_LOCK_DEG
  //   + = right,  - = left
  //   The ESP converts wheel-degrees → steps using STEPS_PER_DEGREE.
  // --------------------------------------------------
  if (cmdType == "STEER") {
    if (commaPos == -1) { Serial.println("MATLAB_ERROR:STEER_MISSING_PARAM"); return; }

    // Negate angle here to reconcile autonomous sender convention with
    // the hardware DIR_PIN polarity (which is correct for HTTP / manual).
    // HTTP sets steerDirection = +1 for RIGHT directly; autonomous sends
    // a signed angle whose positive direction was physically opposite.
    // Flipping only here keeps HTTP untouched.
    float targetAngle = constrain(-param.toFloat(), -WHEEL_LOCK_DEG, WHEEL_LOCK_DEG);
    long  targetSteps = (long)roundf(targetAngle * STEPS_PER_DEGREE);
    targetSteps = constrain(targetSteps, MAX_STEPS_LEFT, MAX_STEPS_RIGHT);

    float currentAngle = (float)current_step_position / STEPS_PER_DEGREE;

    // FIX 1: if new target is within deadband of current wheel position,
    // send STEER_DONE immediately and do not move the motor.
    // This prevents the PC from waiting 3000 ms for a reply that would
    // never arrive when the wheel is already at the commanded angle.
    // Also clears autoTargetActive so no subsequent CMD_DEADBAND silent-return.
    if (fabsf((float)(targetSteps - current_step_position)) <= STEER_DEADBAND_DEG * STEPS_PER_DEGREE) {
      autoTargetActive = false;
      String done = "STEER_DONE," + String(currentAngle, 2) + "," +
                    String(current_step_position) + ",WHEEL_MPS," +
                    String(getWheelSpeedMps(), 4);
      Serial.println(done);
      sendUdpTelem(done);
      return;
    }

    // Ignore if new target barely differs from current active target
    if (autoTargetActive) {
      float existingTargetAngle = (float)autoTargetSteps / STEPS_PER_DEGREE;
      if (fabsf(targetAngle - existingTargetAngle) < STEER_CMD_DEADBAND) return;
    }

    // Already close enough — acknowledge without restarting motor
    if (fabsf(targetAngle - currentAngle) <= STEER_HOLD_DEADBAND_DEG) {
      autoTargetSteps  = targetSteps;
      autoTargetActive = true;
      steerDoneSent    = false;
      String done = "STEER_DONE," + String(currentAngle, 2) + "," + String(current_step_position);
      Serial.println(done);
      sendUdpTelem(done);
      return;
    }

    // Accept new target and start/continue motor
    autoTargetSteps  = targetSteps;
    autoTargetActive = true;
    steerDoneSent    = false;
    centerRequested  = false;

    int newDir = (targetSteps > current_step_position) ? 1 : -1;
    // Only reset direction when it actually reverses — avoids
    // continuous soft-starts on each UDP packet.
    if (newDir != steerDirection) steerDirection = newDir;

    // Suppress Serial ACK while steering is moving — at 115200 baud each
    // Serial.print blocks ~1.5 ms.  At 50 Hz that is 75 ms/s of blocking
    // time which directly starves updateStepper() and causes audible jitter.
    // Print only when motor is idle (diagnostic value only anyway).
    if (steerDirection == 0) {
      Serial.print("STEER_ACK,"); Serial.print(targetAngle, 2);
      Serial.print(",FROM,");      Serial.print(currentAngle, 2);
      Serial.print(",STEPS,");     Serial.println(targetSteps);
    }
  }

  // --------------------------------------------------
  // SPEED,<voltage>   0.00 – 5.00 V
  // --------------------------------------------------
  else if (cmdType == "SPEED") {
    if (commaPos == -1) { Serial.println("MATLAB_ERROR:SPEED_MISSING_PARAM"); return; }
    if (interlockState != INTERLOCK_IDLE) {
      Serial.println("MATLAB_ERROR:SPEED_BLOCKED_DIRECTION_SWITCH_IN_PROGRESS");
      return;
    }
    float v = constrain(param.toFloat(), 0.0f, user_max_voltage);
    target_voltage = v;
    if (steerDirection == 0) {
      Serial.print("MATLAB_ACK:SPEED,"); Serial.println(v, 3);
    }
  }

  // --------------------------------------------------
  // BRAKE,<0|1>
  // --------------------------------------------------
  else if (cmdType == "BRAKE") {
    if (commaPos == -1) { Serial.println("MATLAB_ERROR:BRAKE_MISSING_PARAM"); return; }
    setBrake(param.toInt() == 1);
    Serial.print("MATLAB_ACK:BRAKE,"); Serial.println(param.toInt());
  }

  // --------------------------------------------------
  // MODE,<0|1>    0 = FORWARD   1 = REVERSE
  // --------------------------------------------------
  else if (cmdType == "MODE") {
    if (commaPos == -1) { Serial.println("MATLAB_ERROR:MODE_MISSING_PARAM"); return; }
    bool wantReverse = (param.toInt() == 1);
    requestDriveMode(wantReverse);
    Serial.print("MATLAB_ACK:MODE,"); Serial.println(param.toInt());
  }

  // --------------------------------------------------
  // CENTER — return wheel to straight ahead (step count = 0)
  // --------------------------------------------------
  else if (cmdType == "CENTER") {
    if (current_step_position == 0) {
      steerDirection   = 0;
      autoTargetActive = false;
      String done = "STEER_DONE,0.00,0";
      Serial.println(done);
      sendUdpTelem(done);
      return;
    }
    autoTargetSteps  = 0;
    autoTargetActive = true;
    steerDoneSent    = false;
    centerRequested  = true;
    int newDir = (current_step_position < 0) ? 1 : -1;
    if (newDir != steerDirection) steerDirection = newDir;
    Serial.println("MATLAB_ACK:CENTER");
  }

  // --------------------------------------------------
  // STOP — halt all motion immediately
  // --------------------------------------------------
  else if (cmdType == "STOP") {
    steerDirection   = 0;
    autoTargetActive = false;
    centerRequested  = false;
    target_voltage   = 0.0f;
    if (interlockState == INTERLOCK_RAMPING_DOWN) interlockState = INTERLOCK_IDLE;
    Serial.println("MATLAB_ACK:STOP");
  }

  // --------------------------------------------------
  // GET_STATE — full telemetry reply
  // --------------------------------------------------
  else if (cmdType == "GET_STATE") {
    float currentAngle = (float)current_step_position / STEPS_PER_DEGREE;
    float targetAngle  = (float)autoTargetSteps        / STEPS_PER_DEGREE;
    float instSpd      = computeInstantSteeringSpeed();
    float stepFreqHz   = instSpd * STEPS_PER_DEGREE;
    uint16_t dacRaw    = (uint16_t)((current_voltage / MAX_VOLTAGE) * 4095.0f);

    String telem = "";
    telem += "MATLAB_STATE:";
    telem += "STEER_DEG,";    telem += String(currentAngle,    2);
    telem += ",TARGET_DEG,";  telem += String(autoTargetActive ? targetAngle : currentAngle, 2);
    telem += ",STEPS,";       telem += String(current_step_position);
    telem += ",MOVING,";      telem += String((steerDirection != 0) ? "1" : "0");
    telem += ",SPEED_V,";     telem += String(current_voltage,  3);
    telem += ",TARGET_V,";    telem += String(target_voltage,   3);
    telem += ",DAC_RAW,";     telem += String(dacRaw);
    telem += ",STEP_HZ,";     telem += String(stepFreqHz,       1);
    telem += ",REV,";         telem += String(reverse_mode    ? "1" : "0");
    telem += ",BRAKE,";       telem += String(brake_active    ? "1" : "0");
    telem += ",STEER_SPD,";   telem += String(instSpd,          1);
    telem += ",SYS_OK,";      telem += String(system_ok       ? "1" : "0");
    telem += ",INTERLOCK,";   telem += String((interlockState != INTERLOCK_IDLE) ? "1" : "0");
    telem += ",UDP_RX,";      telem += String(udpCmdCount);
    telem += ",UPTIME_MS,";   telem += String(millis());
    telem += ",WHEEL_MPS,";   telem += String(getWheelSpeedMps(), 4);  // real wheel speed m/s
    Serial.println(telem);
    sendUdpTelem(telem);
  }

  // --------------------------------------------------
  // RESETZERO — declare current wheel position as centre
  //             Only allowed when motor and vehicle are stopped.
  // --------------------------------------------------
  else if (cmdType == "RESETZERO") {
    if (steerDirection != 0 || current_voltage > 0.05f) {
      Serial.println("MATLAB_ERROR:RESETZERO_NOT_ALLOWED_WHILE_MOVING");
      return;
    }
    current_step_position = 0;
    autoTargetSteps       = 0;
    autoTargetActive      = false;
    steerDoneSent         = false;
    Serial.println("MATLAB_ACK:RESETZERO");
  }

  // --------------------------------------------------
  // FWD_MODE,<optional_voltage> — latch forward relay + set cruise speed
  // --------------------------------------------------
  else if (cmdType == "FWD_MODE") {
    if (brake_active) {
      Serial.println("MATLAB_ERROR:FWD_MODE_BRAKE_ENGAGED");
      return;
    }
    if (interlockState != INTERLOCK_IDLE) {
      Serial.println("MATLAB_ERROR:FWD_MODE_INTERLOCK_BUSY");
      return;
    }
    float cruiseV = 0.0f;  // default 0 — PC always follows with SPEED,x immediately
    if (commaPos != -1) cruiseV = constrain(param.toFloat(), 0.0f, user_max_voltage);
    requestDriveMode(false);
    if (interlockState == INTERLOCK_IDLE) target_voltage = cruiseV;
    Serial.printf("MATLAB_ACK:FWD_MODE,%.2f\n", cruiseV);
  }

  else {
    Serial.print("MATLAB_ERROR:UNKNOWN_CMD,"); Serial.println(cmd);
  }
}

// =====================================================
// HTTP HANDLER
// =====================================================
void handleHTTP(WiFiClient &client) {
  client.setTimeout(1000);
  String req = client.readStringUntil('\r');
  client.flush();
  if (req.length() < 10) { client.stop(); return; }

  // --- JSON STATE ---
  if (req.indexOf("GET /STATE") != -1) {
    float ang = (float)current_step_position / STEPS_PER_DEGREE;
    float tgt = (float)autoTargetSteps        / STEPS_PER_DEGREE;
    String j = "{";
    j += "\"autonomous_mode\":"    + String(autonomousMode  ? "true" : "false") + ",";
    j += "\"system_ok\":"          + String(system_ok       ? "true" : "false") + ",";
    j += "\"reverse_mode\":"       + String(reverse_mode    ? "true" : "false") + ",";
    j += "\"brake_active\":"       + String(brake_active    ? "true" : "false") + ",";
    j += "\"interlock_active\":"   + String((interlockState != INTERLOCK_IDLE) ? "true" : "false") + ",";
    j += "\"steering_angle\":"     + String(ang, 2) + ",";
    j += "\"target_angle\":"       + String(tgt, 2) + ",";
    j += "\"current_step_position\":" + String(current_step_position) + ",";
    j += "\"steering_moving\":"    + String((steerDirection != 0) ? "true" : "false") + ",";
    j += "\"current_voltage\":"    + String(current_voltage, 3) + ",";
    j += "\"target_voltage\":"     + String(target_voltage,  3) + ",";
    j += "\"sim_mode\":"           + String(SIMULATION_MODE ? "true" : "false") + ",";
    j += "\"sim_vehicle_speed\":"  + String(sim_vehicle_speed, 2) + ",";
    j += "\"sim_steer_status\":\"" + sim_steer_status + "\",";
    j += "\"sim_last_cmd\":\""     + sim_last_cmd + "\"";
    j += "}";
    client.println("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n");
    client.println(j);
    client.stop();
    return;
  }

  // --- SWITCH MODE ---
  if (req.indexOf("GET /SWITCH_MODE") != -1) {
    int pi = req.indexOf("pass=");
    String pw = "";
    if (pi != -1) {
      int end = req.indexOf(' ', pi + 5);
      pw = req.substring(pi + 5, end);
    }
    if (pw == MODE_SWITCH_PASSWORD) {
      autonomousMode = !autonomousMode;
      client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n");
      client.println(autonomousMode ? "Switched to AUTONOMOUS" : "Switched to MANUAL");
    } else {
      client.println("HTTP/1.1 403 Forbidden\r\nContent-Type: text/plain\r\nConnection: close\r\n");
      client.println("Wrong password");
    }
    client.stop(); return;
  }

  // --- SETSPEED ---
  if (req.indexOf("GET /SETSPEED") != -1 && !autonomousMode) {
    int vi = req.indexOf("val=");
    if (vi != -1) {
      int end = req.indexOf(' ', vi + 4);
      float v = req.substring(vi + 4, end).toFloat() / 100.0f;
      user_max_voltage = constrain(v, 0.0f, MAX_VOLTAGE);
      if (target_voltage > user_max_voltage) target_voltage = user_max_voltage;
    }
    client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n");
    client.println("OK"); client.stop(); return;
  }

  // --- MANUAL CONTROLS ---
  if (!autonomousMode) {
    bool handled = true;
    if      (req.indexOf("GET /LEFT_START")  != -1) {
      if (current_step_position > MAX_STEPS_LEFT)  { steerDirection = -1; centerRequested = false; autoTargetActive = false; }
    }
    else if (req.indexOf("GET /LEFT_STOP")   != -1) { steerDirection = 0; }
    else if (req.indexOf("GET /RIGHT_START") != -1) {
      if (current_step_position < MAX_STEPS_RIGHT) { steerDirection =  1; centerRequested = false; autoTargetActive = false; }
    }
    else if (req.indexOf("GET /RIGHT_STOP")  != -1) { steerDirection = 0; }
    else if (req.indexOf("GET /CENTER")      != -1) {
      if (current_step_position == 0) {
        steerDirection = 0;
      } else {
        autoTargetSteps  = 0; autoTargetActive = true; steerDoneSent = false;
        centerRequested  = true;
        int newDir = (current_step_position < 0) ? 1 : -1;
        if (newDir != steerDirection) steerDirection = newDir;
      }
    }
    else if (req.indexOf("GET /FWD_START") != -1) {
      if (!brake_active) {
        requestDriveMode(false);
        if (interlockState == INTERLOCK_IDLE) target_voltage = user_max_voltage;
      }
    }
    else if (req.indexOf("GET /FWD_STOP")  != -1) { target_voltage = 0.0f; }
    else if (req.indexOf("GET /REV_START") != -1) {
      if (!brake_active) {
        requestDriveMode(true);
        if (interlockState == INTERLOCK_IDLE) target_voltage = user_max_voltage;
      }
    }
    else if (req.indexOf("GET /REV_STOP")   != -1) { target_voltage = 0.0f; }
    else if (req.indexOf("GET /BRAKE_DOWN") != -1) { setBrake(true); }
    else if (req.indexOf("GET /BRAKE_UP")   != -1) { setBrake(false); }
    else { handled = false; }

    if (handled) {
      client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n");
      client.println("OK"); client.stop(); return;
    }
  }

  // --- ESTOP ---
  if (req.indexOf("GET /ESTOP") != -1) {
    emergencyStop();
    autonomousMode = false;
    client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n");
    client.println("ESTOP_OK"); client.stop(); return;
  }

  // =====================================================
  // HTML PAGE
  // =====================================================
  client.println("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n");

  String h = "<!DOCTYPE html><html><head>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1'/>";
  h += "<title>Golf Cart Control v26</title>";
  h += "<style>";
  h += "*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;}";
  h += "body{font-family:Arial,sans-serif;margin:0;background:#eceff1;min-height:100vh;}";
  h += ".wrap{max-width:480px;margin:0 auto;padding:12px;}";
  h += "h1{text-align:center;color:#1a237e;font-size:19px;margin:6px 0 12px;letter-spacing:.3px;}";
  h += "h2{color:#37474f;font-size:14px;font-weight:bold;text-transform:uppercase;letter-spacing:.8px;border-bottom:1.5px solid #e0e0e0;padding-bottom:5px;margin:0 0 12px;}";
  h += ".card{background:#fff;border-radius:14px;padding:14px 16px;margin:10px 0;box-shadow:0 2px 8px rgba(0,0,0,.08);}";
  h += ".badge{display:inline-block;padding:3px 11px;border-radius:20px;font-size:12px;font-weight:bold;}";
  h += ".bg{background:#e8f5e9;color:#2e7d32;}.br{background:#ffebee;color:#b71c1c;}";
  h += ".bo{background:#fff3e0;color:#bf360c;}.bp{background:#f3e5f5;color:#4a148c;}";
  h += ".lbl{font-size:12px;color:#78909c;margin-bottom:2px;}";
  h += ".val{font-size:18px;font-weight:bold;color:#1a237e;line-height:1.2;}";
  h += ".g2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:12px;}";
  h += ".gauge{width:100%;height:10px;background:#e0e0e0;border-radius:5px;overflow:hidden;margin:5px 0 10px;}";
  h += ".gf{height:100%;border-radius:5px;transition:width .35s;}";
  h += ".gs{background:#1565c0;}.gv{background:#2e7d32;}";
  h += ".dpad{display:grid;grid-template-columns:90px 90px 90px;grid-template-rows:90px 90px 90px;gap:8px;width:290px;margin:14px auto 10px;}";
  h += ".btn{display:flex;flex-direction:column;align-items:center;justify-content:center;border:none;border-radius:12px;cursor:pointer;font-size:12px;font-weight:bold;gap:4px;transition:transform .1s,filter .1s;user-select:none;-webkit-user-select:none;}";
  h += ".btn:active{transform:scale(.91);filter:brightness(.85);}";
  h += ".btn svg{pointer-events:none;}";
  h += ".bfwd{background:#2e7d32;color:#fff;grid-column:2;grid-row:1;}";
  h += ".blft{background:#1565c0;color:#fff;grid-column:1;grid-row:2;}";
  h += ".bctr{background:#e65100;color:#fff;grid-column:2;grid-row:2;border-radius:50%;}";
  h += ".brgt{background:#1565c0;color:#fff;grid-column:3;grid-row:2;}";
  h += ".brev{background:#4e342e;color:#fff;grid-column:2;grid-row:3;}";
  h += ".bempty{background:transparent;pointer-events:none;}";
  h += ".bbrk{flex:1;padding:13px;background:#c62828;color:#fff;border:none;border-radius:10px;cursor:pointer;font-size:13px;font-weight:bold;transition:transform .1s;}";
  h += ".bbrk:active{transform:scale(.94);}";
  h += ".estop{width:100%;padding:15px;background:#b71c1c;color:#fff;border:none;border-radius:12px;cursor:pointer;font-size:15px;font-weight:bold;letter-spacing:.5px;margin-top:10px;transition:transform .1s;}";
  h += ".estop:active{transform:scale(.97);}";
  h += ".warn-box{background:#fff8e1;border-left:5px solid #f9a825;padding:11px 14px;border-radius:8px;margin:8px 0;font-size:13px;}";
  h += ".ilock-box{background:#fce4ec;border-left:5px solid #c62828;padding:8px 14px;border-radius:8px;margin:6px 0;font-size:12px;display:none;}";
  h += ".sim-box{background:#e8eaf6;border-left:5px solid #3949ab;padding:11px 14px;border-radius:8px;margin:8px 0;}";
  h += ".mono{font-family:monospace;background:#263238;color:#80cbc4;padding:2px 7px;border-radius:4px;font-size:12px;}";
  h += ".spd-box{background:#f5f5f5;border-radius:8px;padding:11px 13px;}";
  h += "input[type=range]{width:100%;margin:6px 0;}";
  h += ".disabled{opacity:.35;pointer-events:none;}";
  h += "</style>";

  h += "<script>";
  h += "function hold(u1,u2,el){";
  h += "var a=function(){if(!isAuto()&&!isLk())fetch(u1).catch(function(){})};";
  h += "var b=function(){fetch(u2).catch(function(){})};";
  h += "el.addEventListener('mousedown',a);";
  h += "el.addEventListener('mouseup',b);";
  h += "el.addEventListener('mouseleave',b);";
  h += "el.addEventListener('touchstart',function(e){e.preventDefault();a();},{passive:false});";
  h += "el.addEventListener('touchend',function(e){e.preventDefault();b();},{passive:false});}";
  h += "function isAuto(){return document.getElementById('aWarn').style.display==='block';}";
  h += "function isLk(){return document.getElementById('ilkBox').style.display==='block';}";
  h += "function setSpd(v){if(!isAuto()){fetch('/SETSPEED?val='+v).catch(function(){});document.getElementById('spdVal').innerText=(v/100).toFixed(2)+' V';}}";
  h += "function swMode(){var p=prompt('Password:','');if(p){fetch('/SWITCH_MODE?pass='+encodeURIComponent(p)).then(function(r){if(r.ok)return r.text();throw new Error('Wrong password');}).then(function(d){alert(d);upd();}).catch(function(e){alert(e.message);});}}";
  h += "function upd(){fetch('/STATE').then(function(r){return r.json();}).then(function(d){";
  h += "var mb=document.getElementById('mBadge');mb.className='badge '+(d.autonomous_mode?'bp':'bg');mb.innerText=d.autonomous_mode?'AUTONOMOUS':'MANUAL';";
  h += "var sb=document.getElementById('sBadge');sb.className='badge '+(d.system_ok?'bg':'br');sb.innerText=d.system_ok?'OK':'FAULT';";
  h += "var dm=document.getElementById('dMode');dm.className='badge '+(d.reverse_mode?'bo':'bg');dm.innerText=d.reverse_mode?'REVERSE':'FORWARD';";
  h += "var bk=document.getElementById('bState');bk.className='badge '+(d.brake_active?'br':'bg');bk.innerText=d.brake_active?'ENGAGED':'RELEASED';";
  h += "document.getElementById('ilkBox').style.display=d.interlock_active?'block':'none';";
  h += "document.getElementById('aWarn').style.display=d.autonomous_mode?'block':'none';";
  h += "['fwd','rev','left','right','brk','ctr','spdSlider'].forEach(function(id){var el=document.getElementById(id);if(!el)return;if(d.autonomous_mode){el.classList.add('disabled');if(id==='spdSlider')el.disabled=true;}else{el.classList.remove('disabled');if(id==='spdSlider')el.disabled=false;}});";
  h += "document.getElementById('sGauge').style.width=((parseFloat(d.steering_angle)+45)/90*100).toFixed(1)+'%';";
  h += "document.getElementById('sAngle').innerText=parseFloat(d.steering_angle).toFixed(2)+' deg';";
  h += "document.getElementById('sSteps').innerText=d.current_step_position+' steps';";
  h += "document.getElementById('sTarget').innerText=parseFloat(d.target_angle).toFixed(2)+' deg';";
  h += "var sm=document.getElementById('sMoving');sm.className='badge '+(d.steering_moving?'bo':'bg');sm.innerText=d.steering_moving?'Moving':'Idle';";
  h += "document.getElementById('vGauge').style.width=(parseFloat(d.current_voltage)/5*100).toFixed(1)+'%';";
  h += "document.getElementById('vVolt').innerText=parseFloat(d.current_voltage).toFixed(3)+' V';";
  h += "if(d.sim_mode){document.getElementById('simPanel').style.display='block';";
  h += "document.getElementById('simAngle').innerText=parseFloat(d.steering_angle).toFixed(2)+' deg';";
  h += "document.getElementById('simSteps').innerText=d.current_step_position;";
  h += "document.getElementById('simStatus').innerText=d.sim_steer_status;";
  h += "document.getElementById('simSpd').innerText=parseFloat(d.sim_vehicle_speed).toFixed(2)+' km/h';";
  h += "document.getElementById('simCmd').innerText=d.sim_last_cmd;}";
  h += "}).catch(function(){});}";
  h += "window.addEventListener('load',function(){";
  h += "hold('/LEFT_START','/LEFT_STOP',document.getElementById('left'));";
  h += "hold('/RIGHT_START','/RIGHT_STOP',document.getElementById('right'));";
  h += "hold('/FWD_START','/FWD_STOP',document.getElementById('fwd'));";
  h += "hold('/REV_START','/REV_STOP',document.getElementById('rev'));";
  h += "hold('/BRAKE_DOWN','/BRAKE_UP',document.getElementById('brk'));";
  h += "upd();setInterval(upd,1000);});";
  h += "</script></head><body><div class='wrap'>";

  h += "<h1>Golf Cart Control v26</h1>";

  h += "<div class='card'>";
  h += "<div style='display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px'>";
  h += "<div><div class='lbl'>Mode</div><span id='mBadge' class='badge bg'>MANUAL</span></div>";
  h += "<div><div class='lbl'>Health</div><span id='sBadge' class='badge bg'>OK</span></div>";
  h += "<div><div class='lbl'>Drive</div><span id='dMode' class='badge bg'>FORWARD</span></div>";
  h += "<div><div class='lbl'>Brake</div><span id='bState' class='badge bg'>RELEASED</span></div>";
  h += "<button onclick='swMode()' style='background:#6a1b9a;color:#fff;border:none;border-radius:8px;padding:9px 14px;font-weight:bold;cursor:pointer;font-size:13px'>Mode</button>";
  h += "</div></div>";

  h += "<div id='aWarn' class='warn-box' style='display:none'><b>AUTONOMOUS MODE</b> - HTTP controls locked.</div>";
  h += "<div id='ilkBox' class='ilock-box'><b>INTERLOCK ACTIVE</b> - Ramping to zero before switching direction.</div>";

  if (SIMULATION_MODE) {
    h += "<div id='simPanel' class='sim-box'>";
    h += "<div style='font-size:13px;font-weight:bold;margin-bottom:8px'>SIMULATION - live state</div>";
    h += "<div class='g2'>";
    h += "<div><div class='lbl'>Angle</div><div class='val' id='simAngle'>0.00 deg</div></div>";
    h += "<div><div class='lbl'>Steps</div><div class='val' id='simSteps'>0</div></div>";
    h += "<div><div class='lbl'>Status</div><div class='val' style='font-size:14px' id='simStatus'>IDLE</div></div>";
    h += "<div><div class='lbl'>Speed</div><div class='val' id='simSpd'>0.00 km/h</div></div>";
    h += "</div>";
    h += "<div class='lbl' style='margin-top:6px'>Last command</div><span class='mono' id='simCmd'>-</span>";
    h += "</div>";
  }

  h += "<div class='card'>";
  h += "<div class='g2'>";
  h += "<div><div class='lbl'>Steering angle</div><div class='val' id='sAngle'>-</div></div>";
  h += "<div><div class='lbl'>Target angle</div><div class='val' id='sTarget'>-</div></div>";
  h += "<div><div class='lbl'>Steps</div><div class='val' style='font-size:14px' id='sSteps'>-</div></div>";
  h += "<div><div class='lbl'>State</div><span id='sMoving' class='badge bg'>Idle</span></div>";
  h += "</div>";
  h += "<div class='lbl'>Wheel: left — centre — right</div>";
  h += "<div class='gauge'><div class='gf gs' id='sGauge' style='width:50%'></div></div>";
  h += "<div class='lbl'>Speed: 0 – 5 V</div>";
  h += "<div class='gauge'><div class='gf gv' id='vGauge' style='width:0%'></div></div>";
  h += "<div style='margin-top:2px'><div class='lbl'>Voltage</div><div class='val' id='vVolt'>-</div></div>";
  h += "</div>";

  h += "<div class='card'>";
  h += "<h2>Controls (hold to move)</h2>";
  h += "<div class='dpad'>";
  h += "<div class='bempty'></div>";
  h += "<button id='fwd' class='btn bfwd'><svg width='26' height='26' viewBox='0 0 26 26'><polygon points='13,3 24,22 2,22' fill='white'/></svg>FWD</button>";
  h += "<div class='bempty'></div>";
  h += "<button id='left' class='btn blft'><svg width='26' height='26' viewBox='0 0 26 26'><polygon points='4,13 22,3 22,23' fill='white'/></svg>LEFT</button>";
  h += "<button id='ctr' class='btn bctr' onclick=\"if(!isAuto())fetch('/CENTER')\"><svg width='22' height='22' viewBox='0 0 22 22'><circle cx='11' cy='11' r='7' fill='white'/></svg>CTR</button>";
  h += "<button id='right' class='btn brgt'><svg width='26' height='26' viewBox='0 0 26 26'><polygon points='22,13 4,3 4,23' fill='white'/></svg>RIGHT</button>";
  h += "<div class='bempty'></div>";
  h += "<button id='rev' class='btn brev'><svg width='26' height='26' viewBox='0 0 26 26'><polygon points='13,23 24,4 2,4' fill='white'/></svg>REV</button>";
  h += "<div class='bempty'></div>";
  h += "</div>";
  h += "<div style='display:flex;gap:8px;margin-top:4px'><button id='brk' class='bbrk'>BRAKE (hold)</button></div>";
  h += "<button class='estop' onclick=\"fetch('/ESTOP')\">&#9888; EMERGENCY STOP</button>";
  h += "</div>";

  h += "<div class='card'><h2>Max Speed Limit</h2>";
  h += "<div class='spd-box'>";
  h += "<div class='lbl'>Maximum voltage: <b><span id='spdVal'>5.00 V</span></b></div>";
  h += "<input id='spdSlider' type='range' min='0' max='500' value='500' step='1' oninput='setSpd(this.value)'/>";
  h += "</div></div>";

  h += "<div class='card'><h2>Connection</h2>";
  h += "<p style='margin:4px 0;font-size:13px'>IP: <span class='mono'>" + WiFi.localIP().toString() + "</span></p>";
  h += "<p style='margin:4px 0;font-size:13px'>URL: <span class='mono'>http://" + WiFi.localIP().toString() + "</span></p>";
  h += "<p style='margin:4px 0;font-size:13px'>Firmware: <span class='mono'>v28 — smooth human-like steering</span></p>";
  if (SIMULATION_MODE) h += "<p style='color:#3949ab;font-size:13px;margin:6px 0 0'><b>Simulation mode — no hardware outputs</b></p>";
  h += "</div>";
  h += "</div></body></html>";
  client.println(h);
  client.stop();
}

// =====================================================
// DRIVE HELPERS
// =====================================================
void setDAC(float v) {
  float c = constrain(v, MIN_VOLTAGE, MAX_VOLTAGE);
  if (SIMULATION_MODE) { current_voltage = c; return; }
  uint16_t val = (uint16_t)((c / MAX_VOLTAGE) * 4095.0f);
  Wire.beginTransmission(DAC_ADDRESS);
  Wire.write(0x40);
  Wire.write((uint8_t)(val >> 4));
  Wire.write((uint8_t)((val & 0x0F) << 4));
  uint8_t err = Wire.endTransmission();
  if (err == 0) current_voltage = c;
  else { Serial.printf("[ERROR] DAC I2C err %u\n", err); system_ok = false; }
}

void setDriveModeImmediate(bool reverse) {
  reverse_mode = reverse;
  if (!SIMULATION_MODE) {
    digitalWrite(FWD_PIN, reverse ? LOW  : HIGH);
    digitalWrite(REV_PIN, reverse ? HIGH : LOW);
  }
  Serial.print("[DRIVE] -> "); Serial.println(reverse ? "REVERSE" : "FORWARD");
}

void requestDriveMode(bool reverse) {
  if (reverse_mode == reverse && interlockState == INTERLOCK_IDLE) return;
  if (current_voltage <= INTERLOCK_ZERO_V && interlockState == INTERLOCK_IDLE) {
    setDriveModeImmediate(reverse);
    return;
  }
  if (interlockState == INTERLOCK_IDLE || interlockReverse != reverse) {
    interlockReverse = reverse;
    interlockState   = INTERLOCK_RAMPING_DOWN;
    target_voltage   = 0.0f;
    Serial.print("[INTERLOCK] Requested -> "); Serial.println(reverse ? "REVERSE" : "FORWARD");
    Serial.println("[INTERLOCK] Ramping voltage to zero...");
  }
}

void setBrake(bool engaged) {
  brake_active = engaged;
  if (engaged) {
    target_voltage = 0.0f;
    setDAC(0.0f);
    if (!SIMULATION_MODE) { digitalWrite(FWD_PIN, LOW); digitalWrite(REV_PIN, LOW); }
  } else {
    setDriveModeImmediate(reverse_mode);
  }
  if (!SIMULATION_MODE) {
    digitalWrite(BRAKE_PIN_1, engaged ? HIGH : LOW);
    digitalWrite(BRAKE_PIN_2, engaged ? HIGH : LOW);
  }
  Serial.print("[BRAKE] "); Serial.println(engaged ? "ENGAGED" : "RELEASED");
}

void emergencyStop() {
  target_voltage   = 0.0f;
  setDAC(0.0f);
  interlockState   = INTERLOCK_IDLE;
  steerDirection   = 0;
  autoTargetActive = false;
  centerRequested  = false;
  if (!SIMULATION_MODE) {
    digitalWrite(FWD_PIN,  LOW);
    digitalWrite(REV_PIN,  LOW);
    digitalWrite(STEP_PIN, LOW);
    stepPinState = false;
  }
  setBrake(true);
  autonomousMode = false;
  restartWebServer();
  Serial.println("[ESTOP] All stopped - MANUAL mode");
}

bool verifyDAC() {
  if (SIMULATION_MODE) return true;
  Wire.beginTransmission(DAC_ADDRESS);
  return (Wire.endTransmission() == 0);
}

void restartWebServer() {
  server.stop(); delay(100); server.begin();
  Serial.println("[SERVER] Restarted");
}

void printSystemStatus() {
  float ang = (float)current_step_position / STEPS_PER_DEGREE;
  Serial.println("--- STATUS ---");
  Serial.print("Mode:           "); Serial.println(autonomousMode ? "AUTONOMOUS" : "MANUAL");
  Serial.print("SimMode:        "); Serial.println(SIMULATION_MODE ? "YES" : "NO");
  Serial.print("Sys OK:         "); Serial.println(system_ok ? "YES" : "NO");
  Serial.print("Voltage:        "); Serial.print(current_voltage, 3); Serial.println(" V");
  Serial.print("Target V:       "); Serial.print(target_voltage,  3); Serial.println(" V");
  Serial.print("Direction:      "); Serial.println(reverse_mode ? "REVERSE" : "FORWARD");
  Serial.print("Brake:          "); Serial.println(brake_active ? "ENGAGED" : "RELEASED");
  Serial.print("Steer angle:    "); Serial.print(ang, 2); Serial.print(" deg wheel  (");
  Serial.print(current_step_position); Serial.println(" steps)");
  Serial.print("PULSES_TO_LOCK: "); Serial.println(PULSES_TO_LOCK);
  Serial.print("WHEEL_LOCK_DEG: "); Serial.println(WHEEL_LOCK_DEG, 2);
  Serial.print("STEPS/DEG:      "); Serial.println(STEPS_PER_DEGREE, 2);
  Serial.printf("STEER_CRUISE:   %.1f deg/s  CREEP: %.1f deg/s  zone: %.1f deg\n",
               STEER_SPEED_CRUISE, STEER_SPEED_CREEP, STEER_CREEP_ZONE);
  Serial.print("WiFi:           "); Serial.println(WiFi.localIP());
  Serial.println("--- END ---");
}
