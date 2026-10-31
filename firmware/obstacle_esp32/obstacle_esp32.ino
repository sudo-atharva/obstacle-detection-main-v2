// Obstacle Detection - ESP32 + L293D motor driver
//
// The PC app decides the speed and sends it over USB serial (115200 baud), one line per command:
//   S<speed>     speed from 0.00 to 1.00, e.g. "S0.30" = 30 %, "S0" = stop
//   C<hard stop cm>,<min duty>,<reverse A 0/1>,<reverse B 0/1>   settings from the GUI, e.g. "C15,120,0,0"
//                (the app resends them every second; until then the defaults below apply)
// Every SONAR_EVERY_MS it sends the two ultrasonic readings back:
//   U<left cm>,<right cm>   e.g. "U123,85"; 999 = no echo (nothing within ~4 m)
// GPS module (NEO-6M or similar, 9600 baud) on Serial2: its $..RMC and $..GGA sentences are
// passed through unchanged, one per line; the app parses them.
// The app decides when to stop (limit set in its GUI). As a backstop, the ESP32 itself
// refuses to drive while either sensor reads under HARD_STOP_CM.
// Safety: if no command arrives for WATCHDOG_MS (app closed or crashed, cable unplugged),
// the motors stop.
// You can also type commands in the Arduino Serial Monitor to test the motors without the app.
//
// Board: any ESP32 dev board (e.g. ESP32-WROOM-32 DevKit). Works with Arduino-ESP32 core 2.x and 3.x.
// Wiring: see ../../docs/wiring.svg

// ---- Pins (L293D pin numbers in brackets) ----
const int ENA = 32;  // [1]  EN1  - speed of motor A (PWM)
const int IN1 = 33;  // [2]  IN1  - direction of motor A
const int IN2 = 14;  // [7]  IN2  - direction of motor A
const int ENB = 25;  // [9]  EN2  - speed of motor B (PWM)
const int IN3 = 27;  // [10] IN3  - direction of motor B
const int IN4 = 26;  // [15] IN4  - direction of motor B
const int STATUS_LED = 2;  // on-board LED: lit while the app is sending commands
// HC-SR04 ultrasonic sensors (VCC 5 V). ECHO outputs 5 V: put a divider (1k + 2k) before the ESP32 pin.
const int TRIG_L = 5, ECHO_L = 18;
const int TRIG_R = 19, ECHO_R = 21;
// GPS: module TX -> RX2 (GPIO16), module RX -> TX2 (GPIO17). VCC 3.3 V or 5 V per your module.
const int GPS_RX = 16, GPS_TX = 17;
const long GPS_BAUD = 9600;
// Active buzzer (+ to pin, - to GND): sounds while either sensor reads under HARD_STOP_CM.
const int BUZZER = 23;

// ---- Tuning ----
const int PWM_FREQ = 1000;               // Hz
const int PWM_BITS = 8;                  // duty 0..255
const unsigned long WATCHDOG_MS = 500;   // stop if the app goes quiet this long
const unsigned long SONAR_EVERY_MS = 100;
const float NO_ECHO_CM = 999;
// Defaults; the app's GUI overrides these with the C command.
int MIN_DUTY = 120;                      // small DC motors stall below this; raise it if yours only hum
bool REVERSE_A = false;                  // true if motor A spins backwards (or swap its wires)
bool REVERSE_B = false;
float HARD_STOP_CM = 15;                 // backstop + buzzer: motors stop below this whatever the app says

// ---- PWM helpers: the LEDC API changed in Arduino-ESP32 core 3.0 ----
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmSetup(int pin, int channel) { ledcAttach(pin, PWM_FREQ, PWM_BITS); }
void pwmWrite(int pin, int channel, int duty) { ledcWrite(pin, duty); }
#else
void pwmSetup(int pin, int channel) { ledcSetup(channel, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, channel); }
void pwmWrite(int pin, int channel, int duty) { ledcWrite(channel, duty); }
#endif

unsigned long lastCommandMs = 0;
float currentSpeed = 0;
unsigned long lastSonarMs = 0;
bool tooClose = false;
String line;
String gpsLine;

float readCm(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 25000);  // 25 ms timeout ~ 4 m
  return us ? us / 58.0 : NO_ECHO_CM;
}

void setMotor(int en, int channel, int inA, int inB, bool reverse, float speed) {
  if (speed <= 0.01) {
    // Brake: both inputs equal with EN high stops the motor faster than letting it coast.
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    pwmWrite(en, channel, 255);
    return;
  }
  digitalWrite(inA, reverse ? LOW : HIGH);
  digitalWrite(inB, reverse ? HIGH : LOW);
  int duty = MIN_DUTY + (int)((255 - MIN_DUTY) * min(speed, 1.0f));
  pwmWrite(en, channel, duty);
}

void setSpeed(float speed) {
  speed = constrain(speed, 0.0f, 1.0f);
  setMotor(ENA, 0, IN1, IN2, REVERSE_A, speed);
  setMotor(ENB, 1, IN3, IN4, REVERSE_B, speed);
  if (speed != currentSpeed) {
    Serial.printf("speed %.2f\n", speed);
  }
  currentSpeed = speed;
}

void handleLine(const String &cmd) {
  if (cmd.length() >= 2 && (cmd[0] == 'S' || cmd[0] == 's')) {
    setSpeed(tooClose ? 0 : cmd.substring(1).toFloat());
    lastCommandMs = millis();
  } else if (cmd[0] == 'C' || cmd[0] == 'c') {
    float stopCm;
    int duty, revA, revB;
    if (sscanf(cmd.c_str() + 1, "%f,%d,%d,%d", &stopCm, &duty, &revA, &revB) == 4) {
      HARD_STOP_CM = constrain(stopCm, 0.0f, 400.0f);
      MIN_DUTY = constrain(duty, 0, 255);
      REVERSE_A = revA;
      REVERSE_B = revB;
    } else {
      Serial.println("bad settings (use C<stop cm>,<min duty>,<reverse A 0/1>,<reverse B 0/1>)");
    }
  } else if (cmd.length() > 0) {
    Serial.println("unknown command (use S0.00 .. S1.00 or C15,120,0,0)");
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  for (int pin : {IN1, IN2, IN3, IN4, STATUS_LED, TRIG_L, TRIG_R, BUZZER}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  pinMode(ECHO_L, INPUT);
  pinMode(ECHO_R, INPUT);
  pwmSetup(ENA, 0);
  pwmSetup(ENB, 1);
  setSpeed(0);
  Serial.println("obstacle_esp32 ready");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line.trim();
      handleLine(line);
      line = "";
    } else if (line.length() < 32) {
      line += c;
    }
  }

  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      gpsLine.trim();
      if (gpsLine.startsWith("$") && (gpsLine.indexOf("RMC,") == 3 || gpsLine.indexOf("GGA,") == 3)) {
        Serial.println(gpsLine);
      }
      gpsLine = "";
    } else if (gpsLine.length() < 100) {
      gpsLine += c;
    }
  }

  if (millis() - lastSonarMs >= SONAR_EVERY_MS) {
    lastSonarMs = millis();
    float left = readCm(TRIG_L, ECHO_L);
    float right = readCm(TRIG_R, ECHO_R);
    Serial.printf("U%.0f,%.0f\n", left, right);
    tooClose = min(left, right) < HARD_STOP_CM;
    digitalWrite(BUZZER, tooClose ? HIGH : LOW);
    if (tooClose && currentSpeed > 0) {
      setSpeed(0);
    }
  }

  bool alive = millis() - lastCommandMs < WATCHDOG_MS;
  if (!alive && currentSpeed > 0) {
    setSpeed(0);
    Serial.println("watchdog: no commands, motors stopped");
  }
  digitalWrite(STATUS_LED, alive ? HIGH : LOW);
}
