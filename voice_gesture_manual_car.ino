/*
  Robust Bluetooth car (AFMotor)
  - Commands:
      U = forward, D = backward, L = left, R = right, S = stop, X = EMERGENCY STOP
      '0'..'9' = speed level (0..max)
      V### = set speed 0..255 (e.g., V200)
      T#### = set watchdog timeout ms (e.g., T800)
      ? = status query (reports dir/speed/timeout)
  - Features:
      • Smooth acceleration/deceleration ramp
      • Auto-stop if no command within watchdog window
      • Per-motor trim for straight tracking
      • Clean handling of noisy/partial serial input
  - Uses hardware Serial at 9600 (HC-05/HC-06). If you need USB debugging,
    move the BT module to SoftwareSerial and map prints accordingly.
*/

#include <AFMotor.h>
// #include <SoftwareSerial.h> // Uncomment if you want to move BT off hardware Serial

// -------------------- Config --------------------
static const uint8_t  DEFAULT_SPEED   = 170;   // 0..255
static const uint16_t DEFAULT_TIMEOUT = 800;   // ms, auto-stop if no cmd
static const uint8_t  RAMP_STEP       = 6;     // speed change per ramp tick
static const uint16_t RAMP_INTERVAL   = 20;    // ms between ramp steps

// Motor trims (0..255). Use to correct drift: 255 = full, 230 ≈ -10% power, etc.
static const uint8_t TRIM_M1 = 255;
static const uint8_t TRIM_M2 = 255;
static const uint8_t TRIM_M3 = 255;
static const uint8_t TRIM_M4 = 255;

// If you really must use SoftwareSerial for BT, set this to 1 and wire BT to pins below
#define USE_BT_SOFTWARESERIAL 0
#if USE_BT_SOFTWARESERIAL
#include <SoftwareSerial.h>
static const uint8_t BT_RX = 2;   // Module TX -> Arduino 2
static const uint8_t BT_TX = 3;   // Module RX <- Arduino 3
SoftwareSerial BT(BT_RX, BT_TX);
#define BT_PORT BT
#else
#define BT_PORT Serial
#endif

// ----------------- Motor objects ----------------
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

// ------------------- State ----------------------
enum Motion : uint8_t { STOPPED=0, FWD, BACK, LEFT, RIGHT };
volatile Motion targetMotion = STOPPED;

uint8_t  targetSpeed  = DEFAULT_SPEED;   // desired speed (0..255)
uint8_t  currentSpeed = 0;               // actual applied speed (ramps to target)
uint16_t watchdogMs   = DEFAULT_TIMEOUT;

unsigned long lastCmdMs     = 0;
unsigned long lastRampTick  = 0;

// ----------------- Helpers ----------------------
static inline uint8_t scaleTrim(uint8_t base, uint8_t trim) {
  // Multiply base*trim/255 without overflow
  return (uint16_t(base) * uint16_t(trim)) / 255;
}

void setAllSpeeds(uint8_t spd) {
  M1.setSpeed(scaleTrim(spd, TRIM_M1));
  M2.setSpeed(scaleTrim(spd, TRIM_M2));
  M3.setSpeed(scaleTrim(spd, TRIM_M3));
  M4.setSpeed(scaleTrim(spd, TRIM_M4));
}

void applyMotion(Motion m) {
  switch (m) {
    case FWD:
      M1.run(FORWARD);  M2.run(FORWARD);  M3.run(FORWARD);  M4.run(FORWARD);
      break;
    case BACK:
      M1.run(BACKWARD); M2.run(BACKWARD); M3.run(BACKWARD); M4.run(BACKWARD);
      break;
    case LEFT: // tank turn left
      M1.run(BACKWARD); M2.run(FORWARD);  M3.run(FORWARD);  M4.run(BACKWARD);
      break;
    case RIGHT: // tank turn right
      M1.run(FORWARD);  M2.run(BACKWARD); M3.run(BACKWARD); M4.run(FORWARD);
      break;
    case STOPPED:
    default:
      M1.run(RELEASE);  M2.run(RELEASE);  M3.run(RELEASE);  M4.run(RELEASE);
      break;
  }
}

// Smoothly ramp currentSpeed towards targetSpeed
void serviceRamping() {
  unsigned long now = millis();
  if (now - lastRampTick < RAMP_INTERVAL) return;
  lastRampTick = now;

  if (currentSpeed == targetSpeed) return;

  if (currentSpeed < targetSpeed) {
    uint16_t s = currentSpeed + RAMP_STEP;
    currentSpeed = (s > targetSpeed) ? targetSpeed : s;
  } else {
    int16_t s = int16_t(currentSpeed) - int16_t(RAMP_STEP);
    currentSpeed = (s < int16_t(targetSpeed)) ? targetSpeed : uint8_t(s);
  }

  setAllSpeeds(currentSpeed);
  if (currentSpeed == 0) applyMotion(STOPPED);
}

// Force immediate stop (no ramp)
void emergencyStop() {
  targetMotion = STOPPED;
  targetSpeed  = 0;
  currentSpeed = 0;
  setAllSpeeds(0);
  applyMotion(STOPPED);
}

// Apply motion with ramp-aware speed
void setMotion(Motion m) {
  targetMotion = m;
  if (targetSpeed == 0 && m != STOPPED) {
    // if speed is 0 but a motion is requested, bump to a safe minimum
    targetSpeed = 60;
  }
  if (m == STOPPED) {
    // let it ramp down cleanly; if you want hard brake, call emergencyStop()
  } else {
    applyMotion(m);
  }
}

// ----------------- Command parsing -----------------
void onCommandChar(char c) {
  if (c >= 'a' && c <= 'z') c = char(c - 'a' + 'A'); // to upper
  lastCmdMs = millis();

  if (c >= '0' && c <= '9') {
    // map '0'..'9' to 0..255 (0=stop)
    uint8_t level = uint8_t(c - '0');             // 0..9
    targetSpeed = (level == 0) ? 0 : (uint8_t)map(level, 1, 9, 40, 255);
    // keep current direction; if targetSpeed==0, we’ll ramp to halt
    BT_PORT.print(F("SPD=")); BT_PORT.println(targetSpeed);
    return;
  }

  switch (c) {
    case 'U': setMotion(FWD);    BT_PORT.println(F("DIR=FWD"));    break;
    case 'D': setMotion(BACK);   BT_PORT.println(F("DIR=BACK"));   break;
    case 'L': setMotion(LEFT);   BT_PORT.println(F("DIR=LEFT"));   break;
    case 'R': setMotion(RIGHT);  BT_PORT.println(F("DIR=RIGHT"));  break;
    case 'S': setMotion(STOPPED);BT_PORT.println(F("DIR=STOP"));   break;
    case 'X': emergencyStop();   BT_PORT.println(F("EMERG=STOP")); break;
    case '?': {
      BT_PORT.print(F("STATUS speed=")); BT_PORT.print(targetSpeed);
      BT_PORT.print(F(" current=")); BT_PORT.print(currentSpeed);
      BT_PORT.print(F(" dir="));
      BT_PORT.print((targetMotion==FWD)?"FWD":(targetMotion==BACK)?"BACK":(targetMotion==LEFT)?"LEFT":(targetMotion==RIGHT)?"RIGHT":"STOP");
      BT_PORT.print(F(" timeout=")); BT_PORT.println(watchdogMs);
    } break;
    default: /* ignore */ break;
  }
}

// Buffer numeric commands like V200 / T800
char lineBuf[12];
uint8_t lineLen = 0;

void processLineBuffer() {
  lineBuf[lineLen] = '\0';
  if (lineLen == 0) return;

  // Expect forms: V### or T####
  if (lineBuf[0] == 'V' || lineBuf[0] == 'v') {
    int v = atoi(lineBuf + 1);
    if (v < 0) v = 0; if (v > 255) v = 255;
    targetSpeed = uint8_t(v);
    BT_PORT.print(F("SPD=")); BT_PORT.println(targetSpeed);
  } else if (lineBuf[0] == 'T' || lineBuf[0] == 't') {
    long v = atol(lineBuf + 1);
    if (v < 100) v = 100; if (v > 5000) v = 5000;
    watchdogMs = (uint16_t)v;
    BT_PORT.print(F("TIMEOUT=")); BT_PORT.println(watchdogMs);
  }
  lineLen = 0;
}

void readSerialCommands(Stream& s) {
  while (s.available() > 0) {
    char c = (char)s.read();

    // accept simple single-char commands immediately
    if (c == 'U' || c == 'u' || c == 'D' || c == 'd' ||
        c == 'L' || c == 'l' || c == 'R' || c == 'r' ||
        c == 'S' || c == 's' || c == 'X' || c == 'x' ||
        (c >= '0' && c <= '9') || c == '?' ) {
      onCommandChar(c);
      // reset any partial line
      lineLen = 0;
      continue;
    }

    // line-based extended commands: V### / T####
    if (c == '\n' || c == '\r') {
      processLineBuffer();
      continue;
    }

    // accumulate [A-Za-z0-9] up to buffer size
    if (lineLen < sizeof(lineBuf) - 1) {
      if ((c >= '0' && c <= '9') ||
          (c >= 'A' && c <= 'Z') ||
          (c >= 'a' && c <= 'z')) {
        lineBuf[lineLen++] = c;
      }
    }
  }
}

// ----------------- Arduino core -------------------
void setup() {
#if USE_BT_SOFTWARESERIAL
  BT.begin(9600);
#else
  Serial.begin(9600);
#endif

  // Safe start
  emergencyStop();
  setAllSpeeds(0);

  // Set initial desired state
  targetSpeed = DEFAULT_SPEED;
  lastCmdMs   = millis();

  BT_PORT.println(F("READY"));
  BT_PORT.println(F("Cmds: U/D/L/R/S, 0..9, V###, T####, X, ?"));
}

void loop() {
  // Read commands (BT on BT_PORT)
  readSerialCommands(BT_PORT);

  // Watchdog auto-stop
  if (watchdogMs > 0 && (millis() - lastCmdMs) > watchdogMs) {
    setMotion(STOPPED);      // gentle stop (ramps down)
    // reset timer so we don't spam repeatedly
    lastCmdMs = millis();
  }

  // Service speed ramping
  serviceRamping();
}
