#include <Servo.h>

// ═══════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════

// Motor driver - L298N or similar
const int L_EN_FOR_ONE  = 3;
const int R_EN_FOR_ONE  = 4;
const int L_PWM_FOR_ONE = 5;
const int R_PWM_FOR_ONE = 6;
const int L_EN_FOR_TWO  = 8;
const int R_EN_FOR_TWO  = 12;
const int L_PWM_FOR_TWO = 10;
const int R_PWM_FOR_TWO = 11;

// Arm servos (free Mega PWM pins)
const int PIN_BASE    = 44;   // Base / shoulder rotation
const int PIN_ELBOW   = 45;   // Elbow joint
const int PIN_GRIPPER = 46;   // Gripper open/close

// ═══════════════════════════════════════════════════════
//  OBJECTS
// ═══════════════════════════════════════════════════════
Servo baseJoint;
Servo elbowJoint;
Servo gripperServo;

// ═══════════════════════════════════════════════════════
//  ROVER STATE
// ═══════════════════════════════════════════════════════
const int SPEED_MIN = 155;
const int SPEED_MAX = 250;
int speed_left  = SPEED_MAX;
int speed_right = SPEED_MAX;

// ═══════════════════════════════════════════════════════
//  ARM STATE
// ═══════════════════════════════════════════════════════
int baseAngle  = 0;
int elbowAngle = 0;
bool gripperClosed = false; // track open/closed state

// Gripper — CONTINUOUS ROTATION servo
// 90 = stop | 110 = close direction | 80 = open direction
const int GRIPPER_STOP      = 90;
const int GRIPPER_CLOSE_DIR = 110;
const int GRIPPER_OPEN_DIR  = 80;
const int GRIPPER_CLOSE_MS  = 600; // ms to run when closing
const int GRIPPER_OPEN_MS   = 800; // ms to run when opening

void gripperClose(Stream& port) {
  gripperServo.write(GRIPPER_CLOSE_DIR);
  delay(GRIPPER_CLOSE_MS);
  gripperServo.write(GRIPPER_STOP);
  gripperClosed = true;
  port.println(F("GRIPPER: CLOSED"));
}

void gripperOpen(Stream& port) {
  gripperServo.write(GRIPPER_OPEN_DIR);
  delay(GRIPPER_OPEN_MS);
  gripperServo.write(GRIPPER_STOP);
  gripperClosed = false;
  port.println(F("GRIPPER: OPEN"));
}

// ═══════════════════════════════════════════════════════
//  FUNCTION DECLARATIONS
// ═══════════════════════════════════════════════════════
void processCommand(char c, Stream& port);
void handleArmCommand(String cmd, Stream& port);
void handleSpeedCommand(String cmd, Stream& port);
void forward();
void backward();
void turnLeft();
void turnRight();
void forwardLeft();
void forwardRight();
void backLeft();
void backRight();
void stopMotors();

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // HC-05 Bluetooth on pins 18(RX1) / 19(TX1)

  // Motor driver enable pins
  pinMode(L_EN_FOR_ONE,  OUTPUT); pinMode(R_EN_FOR_ONE,  OUTPUT);
  pinMode(L_PWM_FOR_ONE, OUTPUT); pinMode(R_PWM_FOR_ONE, OUTPUT);
  pinMode(L_EN_FOR_TWO,  OUTPUT); pinMode(R_EN_FOR_TWO,  OUTPUT);
  pinMode(L_PWM_FOR_TWO, OUTPUT); pinMode(R_PWM_FOR_TWO, OUTPUT);

  digitalWrite(L_EN_FOR_ONE, HIGH);
  digitalWrite(R_EN_FOR_ONE, HIGH);
  digitalWrite(L_EN_FOR_TWO, HIGH);
  digitalWrite(R_EN_FOR_TWO, HIGH);

  // Arm servos
  baseJoint.attach(PIN_BASE);
  elbowJoint.attach(PIN_ELBOW);
  gripperServo.attach(PIN_GRIPPER);

  baseJoint.write(baseAngle);
  elbowJoint.write(elbowAngle);
  gripperServo.write(GRIPPER_STOP); // continuous servo: stop on startup

  delay(500);

  Serial.println(F("=== Rover + Arm Controller Ready ==="));
  Serial.println(F("Rover : W=Stop  F=Fwd  B=Back  R=Left  L=Right"));
  Serial.println(F("        G H I J = diagonal moves"));
  Serial.println(F("Arm   : ARM:<base>,<elbow>,<gripper>  (0-180 each)"));
  Serial.println(F("Example: ARM:90,45,30"));
  Serial.println(F("====================================="));
}

// ═══════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════
void loop() {

  // ── Serial Monitor input ──────────────────────────────
  if (Serial.available() > 0) {
    char peek = (char)Serial.peek();
    if (peek == 'A' || peek == 'S') {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if      (line.startsWith("ARM:")) handleArmCommand(line, Serial);
      else if (line.startsWith("SPD:")) handleSpeedCommand(line, Serial);
    } else {
      char c = Serial.read();
      processCommand(c, Serial);
    }
  }

  // ── Bluetooth (Serial1) input ─────────────────────────
  if (Serial1.available() > 0) {
    char peek = (char)Serial1.peek();
    if (peek == 'A' || peek == 'S') {
      String line = Serial1.readStringUntil('\n');
      line.trim();
      if      (line.startsWith("ARM:")) handleArmCommand(line, Serial1);
      else if (line.startsWith("SPD:")) handleSpeedCommand(line, Serial1);
    } else {
      char c = Serial1.read();
      processCommand(c, Serial1);
    }
  }
}

// ═══════════════════════════════════════════════════════
//  COMMAND PROCESSOR  (works for both Serial & Serial1)
// ═══════════════════════════════════════════════════════
void processCommand(char c, Stream& port) {
  switch (c) {

    // ── Rover movement ──────────────────────────────────
    case 'W': stopMotors();
              port.println(F("STOP"));              break;
    case 'F': forward();
              port.println(F("FORWARD"));           break;
    case 'B': backward();
              port.println(F("BACKWARD"));          break;
    case 'R': turnLeft();
              port.println(F("LEFT"));              break;
    case 'L': turnRight();
              port.println(F("RIGHT"));             break;

    // ── Diagonal movement ───────────────────────────────
    case 'I': backRight();
              port.println(F("BACK-RIGHT"));        break;
    case 'H': forwardLeft();
              port.println(F("FWD-LEFT"));          break;
    case 'J': forwardRight();
              port.println(F("FWD-RIGHT"));         break;
    case 'G': backLeft();
              port.println(F("BACK-LEFT"));         break;

    // ── Arm preset positions ────────────────────────────
    case 'P': // Home position
      baseJoint.write(0); elbowJoint.write(0); gripperServo.write(GRIPPER_STOP);
      baseAngle = elbowAngle = 0;
      port.println(F("ARM: HOME"));                 break;

    case 'O': // Gripper open
      gripperOpen(port);                            break;

    case 'C': // Gripper close
      gripperClose(port);                           break;

    default: break;
  }
}

// ═══════════════════════════════════════════════════════
//  ARM COMMAND PARSER
//  Expects: "ARM:<base>,<elbow>,<gripper>"
//  Example: "ARM:90,45,30"
// ═══════════════════════════════════════════════════════
void handleArmCommand(String cmd, Stream& port) {
  String values = cmd.substring(4); // strip "ARM:"

  int c1 = values.indexOf(',');
  int c2 = values.indexOf(',', c1 + 1);

  if (c1 == -1 || c2 == -1) {
    port.println(F("ERR: Format must be ARM:<base>,<elbow>,<gripper>"));
    return;
  }

  int b = constrain(values.substring(0,      c1).toInt(), 0, 180);
  int e = constrain(values.substring(c1 + 1, c2).toInt(), 0, 180);
  int g = constrain(values.substring(c2 + 1).toInt(),     0, 180);

  baseJoint.write(b);
  elbowJoint.write(e);
  baseAngle  = b;
  elbowAngle = e;

  // Gripper toggle: >=90 → CLOSE, <90 → OPEN
  // Only act if state actually changes (avoids re-running the motor)
  bool wantClose = (g >= 90);
  if (wantClose && !gripperClosed)  gripperClose(port);
  else if (!wantClose && gripperClosed) gripperOpen(port);

  port.print(F("ARM SET → Base:"));
  port.print(b);
  port.print(F(" Elbow:"));
  port.print(e);
  port.print(F(" Gripper:"));
  port.println(gripperClosed ? F("CLOSED") : F("OPEN"));
}

// ═══════════════════════════════════════════════════════
//  SPEED COMMAND PARSER
//  Expects: "SPD:<0-255>"  sets drive motor PWM
// ═══════════════════════════════════════════════════════
void handleSpeedCommand(String cmd, Stream& port) {
  String value = cmd.substring(4);
  int v = constrain(value.toInt(), 0, 255);
  speed_left  = v;
  speed_right = v;
  port.print(F("SPD SET → "));
  port.println(v);
}

// ═══════════════════════════════════════════════════════
//  ROVER MOTION FUNCTIONS
// ═══════════════════════════════════════════════════════
void forward() {
  analogWrite(R_PWM_FOR_ONE, speed_left);  analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, 0);           analogWrite(L_PWM_FOR_TWO, speed_right);
}
void backward() {
  analogWrite(R_PWM_FOR_ONE, 0);           analogWrite(L_PWM_FOR_ONE, speed_left);
  analogWrite(R_PWM_FOR_TWO, speed_right); analogWrite(L_PWM_FOR_TWO, 0);
}
void turnRight() {
  analogWrite(R_PWM_FOR_ONE, 0);           analogWrite(L_PWM_FOR_ONE, speed_left);
  analogWrite(R_PWM_FOR_TWO, 0);           analogWrite(L_PWM_FOR_TWO, speed_right);
}
void turnLeft() {
  analogWrite(R_PWM_FOR_ONE, speed_left);  analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, speed_right); analogWrite(L_PWM_FOR_TWO, 0);
}
void stopMotors() {
  analogWrite(R_PWM_FOR_ONE, 0); analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, 0); analogWrite(L_PWM_FOR_TWO, 0);
}
void forwardLeft() {
  analogWrite(R_PWM_FOR_ONE, 0); analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, speed_right); analogWrite(L_PWM_FOR_TWO, 0);
}
void forwardRight() {
  analogWrite(R_PWM_FOR_ONE, 0); analogWrite(L_PWM_FOR_ONE, speed_left);
  analogWrite(R_PWM_FOR_TWO, 0); analogWrite(L_PWM_FOR_TWO, 0);
}
void backLeft() {
  analogWrite(R_PWM_FOR_ONE, 0); analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, 0); analogWrite(L_PWM_FOR_TWO, speed_right);
}
void backRight() {
  analogWrite(R_PWM_FOR_ONE, speed_left); analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, 0);          analogWrite(L_PWM_FOR_TWO, 0);
}
