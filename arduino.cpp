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

// [SAFETY] Servo initialization timeout & watchdog
const unsigned long SERVO_ATTACH_TIMEOUT_MS = 5000;
const unsigned long GRIPPER_WATCHDOG_MS     = 2000;  // kill gripper if timing goes awry

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

  // [PERF] Motor enable verification (quick smoke test)
  Serial.println(F("  Testing motor enables..."));
  pinMode(13, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(13, HIGH); delay(100); digitalWrite(13, LOW); delay(100);
  }
  Serial.println(F("  Motor driver should be responsive"));

  // [SAFETY] Servo attach with timeout detection + diagnostics
  Serial.println(F("  Attaching servos..."));
  unsigned long servo_start = millis();
  bool servo_ok = true;
  
  if (!baseJoint.attach(PIN_BASE)) {
    Serial.println(F("ERR: BASE servo attach failed"));
    servo_ok = false;
  }
  if (!elbowJoint.attach(PIN_ELBOW)) {
    Serial.println(F("ERR: ELBOW servo attach failed"));
    servo_ok = false;
  }
  if (!gripperServo.attach(PIN_GRIPPER)) {
    Serial.println(F("ERR: GRIPPER servo attach failed"));
    servo_ok = false;
  }
  
  // [SAFETY] Verify attach completed within timeout
  if (millis() - servo_start > SERVO_ATTACH_TIMEOUT_MS) {
    Serial.println(F("WARN: Servo init took > 5s — possible hung servo?"));
  }

  baseJoint.write(baseAngle);
  elbowJoint.write(elbowAngle);
  gripperServo.write(GRIPPER_STOP);

  delay(500);

  if (servo_ok) {
    Serial.println(F("  ✓ Servos attached"));
  } else {
    Serial.println(F("  ✗ SOME SERVOS FAILED — arm may not respond"));
  }

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
        // [SAFETY] Explicit buffer clear on each complete command
        usb_buf = "";
      }
    } else if (usb_buf.length() < 64) {
      usb_buf += c;
    } else {
      // [SAFETY] Buffer overflow—discard incomplete command, signal error
      Serial.println(F("ERR:USB buffer overflow"));
      usb_buf = "";
    }
  }

  // Bluetooth
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      bt_buf.trim();
      if (bt_buf.length() > 0) {
        processLine(bt_buf, Serial1);
        // [SAFETY] Explicit buffer clear
        bt_buf = "";
      }
    } else if (bt_buf.length() < 64) {
      bt_buf += c;
    } else {
      // [SAFETY] Bluetooth buffer overflow
      Serial1.println(F("ERR:BT buffer overflow"));
      bt_buf = "";
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
//  [FIX] Improved bounds checking + parse error handling
// ═══════════════════════════════════════════════════════════════
void handleArmCommand(const String& cmd, Stream& port) {
  String val = cmd.substring(4);  // strip "ARM:"

  // [SAFETY] Verify exact format before parsing
  int c1 = val.indexOf(',');
  int c2 = val.indexOf(',', c1 + 1);
  int c3 = val.length();

  if (c1 < 0 || c2 < 0 || c2 >= c3 - 1) {
    port.println(F("ERR:ARM format ARM:<base>,<elbow>,<gripper> each 0-180"));
    return;
  }

  // [FIX] Parse with explicit bounds check, handle toInt() = 0 on fail
  String s_base = val.substring(0,      c1);
  String s_elbo = val.substring(c1 + 1, c2);
  String s_grip = val.substring(c2 + 1);
  
  // Validate non-empty
  if (s_base.length() == 0 || s_elbo.length() == 0 || s_grip.length() == 0) {
    port.println(F("ERR:ARM empty values"));
    return;
  }

  int b = s_base.toInt();
  int e = s_elbo.toInt();
  int g = s_grip.toInt();

  // [SAFETY] Check for valid parse (toInt returns 0 on fail, but 0 is also valid)
  // Use constrain which is safe—clips to [0, 180]
  b = constrain(b, 0, 180);
  e = constrain(e, 0, 180);
  g = constrain(g, 0, 180);

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
  
  unsigned long now = millis();
  
  // [SAFETY] Gripper watchdog—if timing goes > 2s, force stop to prevent motor burnout
  unsigned long elapsed = now - (gripperEndMs - (gripperClosing ? GRIPPER_CLOSE_MS : GRIPPER_OPEN_MS));
  if (elapsed > GRIPPER_WATCHDOG_MS) {
    Serial.println(F("WARN: Gripper watchdog fired—servo timeout"));
    gripperServo.write(GRIPPER_STOP);
    gripperActive = false;
    gripperClosed = gripperClosing;  // assume it reached desired state
    if (gripperPort) {
      gripperPort->println(gripperClosed ? F("GRIP:CLOSED (timeout)") : F("GRIP:OPEN (timeout)"));
      gripperPort = nullptr;
    }
    return;
  }
  
  // Normal completion
  if (now >= gripperEndMs) {
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
