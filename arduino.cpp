#include <Servo.h>

// ═══════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════

// Motor driver (L298N) – two H-bridge modules
const int L_EN_FOR_ONE  = 3;
const int R_EN_FOR_ONE  = 4;
const int L_PWM_FOR_ONE = 5;   // Driver ONE: right-side BACKWARD
const int R_PWM_FOR_ONE = 6;   // Driver ONE: right-side FORWARD
const int L_EN_FOR_TWO  = 8;
const int R_EN_FOR_TWO  = 12;
const int L_PWM_FOR_TWO = 10;  // Driver TWO: left-side FORWARD
const int R_PWM_FOR_TWO = 11;  // Driver TWO: left-side BACKWARD

// Arm servos (Mega PWM pins 44/45/46)
const int PIN_BASE    = 44;
const int PIN_ELBOW   = 45;
const int PIN_GRIPPER = 46;

// ═══════════════════════════════════════════════════════════════
//  OBJECTS
// ═══════════════════════════════════════════════════════════════
Servo baseJoint;
Servo elbowJoint;
Servo gripperServo;

// ═══════════════════════════════════════════════════════════════
//  DRIVE STATE
// ═══════════════════════════════════════════════════════════════
const int SPEED_MIN = 155;
const int SPEED_MAX = 250;
int speed_left  = SPEED_MAX;
int speed_right = SPEED_MAX;

// ═══════════════════════════════════════════════════════════════
//  ARM STATE
// ═══════════════════════════════════════════════════════════════
int  baseAngle    = 0;
int  elbowAngle   = 0;
bool gripperClosed = false;

// Gripper is a CONTINUOUS ROTATION servo
//   90 = stop  |  110 = close direction  |  80 = open direction
const int GRIPPER_STOP      = 90;
const int GRIPPER_CLOSE_DIR = 110;
const int GRIPPER_OPEN_DIR  = 80;
const int GRIPPER_CLOSE_MS  = 600;
const int GRIPPER_OPEN_MS   = 800;

// Non-blocking gripper timer
unsigned long gripperEndMs   = 0;
bool          gripperActive  = false;
bool          gripperClosing = false;
Stream*       gripperPort    = nullptr;

// ═══════════════════════════════════════════════════════════════
//  SERIAL INPUT BUFFERS  (char-by-char — never blocks the loop)
// ═══════════════════════════════════════════════════════════════
String usb_buf = "";   // Serial  (USB / Python app)
String bt_buf  = "";   // Serial1 (HC-05 Bluetooth)

// ═══════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════
void processLine(const String& line, Stream& port);
void processChar(char c, Stream& port);
void handleArmCommand(const String& cmd, Stream& port);
void handleSpeedCommand(const String& cmd, Stream& port);
void startGripperClose(Stream& port);
void startGripperOpen(Stream& port);
void updateGripper();
void forward();
void backward();
void turnLeft();
void turnRight();
void forwardLeft();
void forwardRight();
void backLeft();
void backRight();
void stopMotors();

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);          // USB → Python app / Serial Monitor
  Serial1.begin(9600);         // HC-05 on Mega pins 18(RX1) / 19(TX1)

  // Motor enable pins (always HIGH)
  pinMode(L_EN_FOR_ONE,  OUTPUT); pinMode(R_EN_FOR_ONE,  OUTPUT);
  pinMode(L_PWM_FOR_ONE, OUTPUT); pinMode(R_PWM_FOR_ONE, OUTPUT);
  pinMode(L_EN_FOR_TWO,  OUTPUT); pinMode(R_EN_FOR_TWO,  OUTPUT);
  pinMode(L_PWM_FOR_TWO, OUTPUT); pinMode(R_PWM_FOR_TWO, OUTPUT);

  digitalWrite(L_EN_FOR_ONE, HIGH);
  digitalWrite(R_EN_FOR_ONE, HIGH);
  digitalWrite(L_EN_FOR_TWO, HIGH);
  digitalWrite(R_EN_FOR_TWO, HIGH);

  stopMotors();

  // Servos
  baseJoint.attach(PIN_BASE);
  elbowJoint.attach(PIN_ELBOW);
  gripperServo.attach(PIN_GRIPPER);

  baseJoint.write(baseAngle);
  elbowJoint.write(elbowAngle);
  gripperServo.write(GRIPPER_STOP);

  delay(500);

  Serial.println(F("=== Rover + Arm Ready ==="));
  Serial.println(F("Drive: F=Fwd  B=Back  L=Left  R=Right  W=Stop"));
  Serial.println(F("Diag : H=FwdLeft  J=FwdRight  G=BackLeft  I=BackRight"));
  Serial.println(F("Arm  : ARM:<base>,<elbow>,<gripper>  (0-180 each)"));
  Serial.println(F("       gripper>=90 closes, <90 opens"));
  Serial.println(F("       P=ArmHome  O=GripOpen  C=GripClose"));
  Serial.println(F("Speed: SPD:<0-255>"));
  Serial.println(F("========================="));
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP  — fully non-blocking
// ═══════════════════════════════════════════════════════════════
void loop() {
  // Gripper non-blocking timer
  updateGripper();

  // USB / Python serial
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      usb_buf.trim();
      if (usb_buf.length() > 0) {
        processLine(usb_buf, Serial);
        usb_buf = "";
      }
    } else if (usb_buf.length() < 64) {
      usb_buf += c;
    }
  }

  // Bluetooth
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      bt_buf.trim();
      if (bt_buf.length() > 0) {
        processLine(bt_buf, Serial1);
        bt_buf = "";
      }
    } else if (bt_buf.length() < 64) {
      bt_buf += c;
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  COMMAND ROUTER
//  Routes complete lines to the right handler.
//  Python sends: single char (F/B/L/R/W/G/H/I/J/P/O/C)
//                ARM:<base>,<elbow>,<gripper>\n
//                SPD:<0-255>\n
// ═══════════════════════════════════════════════════════════════
void processLine(const String& line, Stream& port) {
  if (line.startsWith(F("ARM:"))) {
    handleArmCommand(line, port);
  } else if (line.startsWith(F("SPD:"))) {
    handleSpeedCommand(line, port);
  } else if (line.length() == 1) {
    processChar(line.charAt(0), port);
  }
  // Unknown commands are silently dropped
}

// ─────────────────────────────────────────────────────────────
//  Single-character commands
//  Key mapping matches app.py:  A/ArrowLeft='L'  D/ArrowRight='R'
// ─────────────────────────────────────────────────────────────
void processChar(char c, Stream& port) {
  switch (c) {
    case 'F': forward();      port.println(F("FORWARD"));    break;
    case 'B': backward();     port.println(F("BACKWARD"));   break;
    case 'L': turnLeft();     port.println(F("LEFT"));       break;
    case 'R': turnRight();    port.println(F("RIGHT"));      break;
    case 'W': stopMotors();   port.println(F("STOP"));       break;
    case 'H': forwardLeft();  port.println(F("FWD-LEFT"));   break;
    case 'J': forwardRight(); port.println(F("FWD-RIGHT"));  break;
    case 'G': backLeft();     port.println(F("BACK-LEFT"));  break;
    case 'I': backRight();    port.println(F("BACK-RIGHT")); break;

    case 'P':  // Arm home position
      baseJoint.write(0);
      elbowJoint.write(0);
      baseAngle = elbowAngle = 0;
      port.println(F("ARM:HOME"));
      break;

    case 'O': startGripperOpen(port);  break;
    case 'C': startGripperClose(port); break;

    default: break;
  }
}

// ═══════════════════════════════════════════════════════════════
//  ARM COMMAND PARSER
//  Format: "ARM:<base>,<elbow>,<gripper>"   values 0-180
//  gripper value: >=90 → close,  <90 → open
// ═══════════════════════════════════════════════════════════════
void handleArmCommand(const String& cmd, Stream& port) {
  String val = cmd.substring(4);  // strip "ARM:"

  int c1 = val.indexOf(',');
  int c2 = val.indexOf(',', c1 + 1);

  if (c1 < 0 || c2 < 0) {
    port.println(F("ERR:ARM format ARM:<base>,<elbow>,<gripper>"));
    return;
  }

  int b = constrain(val.substring(0,      c1).toInt(), 0, 180);
  int e = constrain(val.substring(c1 + 1, c2).toInt(), 0, 180);
  int g = constrain(val.substring(c2 + 1).toInt(),     0, 180);

  baseJoint.write(b);
  elbowJoint.write(e);
  baseAngle  = b;
  elbowAngle = e;

  bool wantClose = (g >= 90);
  if      (wantClose  && !gripperClosed) startGripperClose(port);
  else if (!wantClose &&  gripperClosed) startGripperOpen(port);

  port.print(F("ARM:B="));  port.print(b);
  port.print(F(",E="));     port.print(e);
  port.print(F(",G="));     port.println(gripperClosed ? F("CLOSED") : F("OPEN"));
}

// ═══════════════════════════════════════════════════════════════
//  SPEED COMMAND PARSER
//  Format: "SPD:<0-255>"
// ═══════════════════════════════════════════════════════════════
void handleSpeedCommand(const String& cmd, Stream& port) {
  int v = constrain(cmd.substring(4).toInt(), 0, 255);
  speed_left  = v;
  speed_right = v;
  port.print(F("SPD:"));
  port.println(v);
}

// ═══════════════════════════════════════════════════════════════
//  NON-BLOCKING GRIPPER STATE MACHINE
//  Start functions kick off the motor; updateGripper() stops it.
// ═══════════════════════════════════════════════════════════════
void startGripperClose(Stream& port) {
  if (gripperClosed) {
    port.println(F("GRIP:ALREADY_CLOSED"));
    return;
  }
  gripperServo.write(GRIPPER_CLOSE_DIR);
  gripperEndMs   = millis() + GRIPPER_CLOSE_MS;
  gripperActive  = true;
  gripperClosing = true;
  gripperPort    = &port;
  port.println(F("GRIP:CLOSING"));
}

void startGripperOpen(Stream& port) {
  if (!gripperClosed) {
    port.println(F("GRIP:ALREADY_OPEN"));
    return;
  }
  gripperServo.write(GRIPPER_OPEN_DIR);
  gripperEndMs   = millis() + GRIPPER_OPEN_MS;
  gripperActive  = true;
  gripperClosing = false;
  gripperPort    = &port;
  port.println(F("GRIP:OPENING"));
}

void updateGripper() {
  if (!gripperActive) return;
  if (millis() >= gripperEndMs) {
    gripperServo.write(GRIPPER_STOP);
    gripperClosed = gripperClosing;
    gripperActive = false;
    if (gripperPort) {
      gripperPort->println(gripperClosed ? F("GRIP:CLOSED") : F("GRIP:OPEN"));
      gripperPort = nullptr;
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  ROVER MOTION FUNCTIONS
//
//  Physical wiring (determined from motor orientation):
//    Driver ONE → RIGHT side motors
//      R_PWM_FOR_ONE = forward    L_PWM_FOR_ONE = backward
//    Driver TWO → LEFT side motors
//      L_PWM_FOR_TWO = forward    R_PWM_FOR_TWO = backward
//  (asymmetry because one motor side is physically reversed)
// ═══════════════════════════════════════════════════════════════
void forward() {
  analogWrite(R_PWM_FOR_ONE, speed_right); analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, 0);           analogWrite(L_PWM_FOR_TWO, speed_left);
}

void backward() {
  analogWrite(R_PWM_FOR_ONE, 0);           analogWrite(L_PWM_FOR_ONE, speed_right);
  analogWrite(R_PWM_FOR_TWO, speed_left);  analogWrite(L_PWM_FOR_TWO, 0);
}

void turnLeft() {
  // Right side forward + left side backward → spins left in place
  analogWrite(R_PWM_FOR_ONE, speed_right); analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, speed_left);  analogWrite(L_PWM_FOR_TWO, 0);
}

void turnRight() {
  // Left side forward + right side backward → spins right in place
  analogWrite(R_PWM_FOR_ONE, 0);           analogWrite(L_PWM_FOR_ONE, speed_right);
  analogWrite(R_PWM_FOR_TWO, 0);           analogWrite(L_PWM_FOR_TWO, speed_left);
}

void stopMotors() {
  analogWrite(R_PWM_FOR_ONE, 0); analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, 0); analogWrite(L_PWM_FOR_TWO, 0);
}

void forwardLeft() {
  // Right side forward, left side stopped → arcs forward-left
  analogWrite(R_PWM_FOR_ONE, speed_right); analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, 0);           analogWrite(L_PWM_FOR_TWO, 0);
}

void forwardRight() {
  // Left side forward, right side stopped → arcs forward-right
  analogWrite(R_PWM_FOR_ONE, 0); analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, 0); analogWrite(L_PWM_FOR_TWO, speed_left);
}

void backLeft() {
  // Right side backward, left side stopped → arcs backward-left
  analogWrite(R_PWM_FOR_ONE, 0);           analogWrite(L_PWM_FOR_ONE, speed_right);
  analogWrite(R_PWM_FOR_TWO, 0);           analogWrite(L_PWM_FOR_TWO, 0);
}

void backRight() {
  // Left side backward, right side stopped → arcs backward-right
  analogWrite(R_PWM_FOR_ONE, 0); analogWrite(L_PWM_FOR_ONE, 0);
  analogWrite(R_PWM_FOR_TWO, speed_left);  analogWrite(L_PWM_FOR_TWO, 0);
}
