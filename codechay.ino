// Line follower tối ưu: weighted sensors + PID + lost-line search
// Giữ mapping chân theo manual / code trước
// IN1=11, IN2=6, IN3=5, IN4=3
// IR: s1=8, s2=10, s3=2, s4=12, s5=13
// SR04: TRIG=A0, ECHO=A1
// Servo: grip D7, rot D4

#include <Servo.h>

// ---------- Pin mapping ----------
#define IN1 11
#define IN2 6
#define IN3 5
#define IN4 3

#define S1 8
#define S2 10
#define S3 2
#define S4 12
#define S5 13

#define TRIG A0
#define ECHO A1

#define SERVO_GRIP_PIN 7
#define SERVO_ROT_PIN 4

Servo servoGrip, servoRot;

// ---------- PID parameters (tune these) ----------
float Kp = 55.0;   // proportional
float Ki = 0.8;    // integral
float Kd = 12.0;   // derivative

float integral = 0.0;
float lastError = 0.0;
unsigned long lastTime = 0;
float integralLimit = 300.0; // anti-windup

// ---------- Motion / speed ----------
int baseSpeed = 170;    // speed when on line (0-255)
int minSpeed = 90;      // minimal PWM to overcome friction
int maxSpeed = 255;

int searchSpeed = 150;  // speed when searching (rotate)

// weights for 5 sensors (leftmost -> rightmost)
// center = 0; left negative, right positive
const int weights[5] = {-2, -1, 0, 1, 2};

// lastKnownDirection used when lost (positive -> last seen to right)
float lastKnownError = 0.0;

// ---------- Function prototypes ----------
void moveLeftWheel(int pwmForward, int pwmBackward);
void moveRightWheel(int pwmForward, int pwmBackward);
void setMotorPWM(int leftPWM, int rightPWM);
void stopMotors();

int getDistance(int trigPin, int echoPin);
void grabSequence(); // reuse from previous code (simple version)

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // sensors
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  // ultrasonic
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // servos
  servoGrip.attach(SERVO_GRIP_PIN);
  servoRot.attach(SERVO_ROT_PIN);
  servoGrip.write(0);
  servoRot.write(0);

  lastTime = millis();
}

// ---------- Main loop (non-blocking-ish) ----------
void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // seconds
  if (dt <= 0) dt = 0.001;

  // 1) Check ultrasonic first (gắp khi <= 10 cm)
  int dist = getDistance(TRIG, ECHO);
  if (dist <= 10) {
    stopMotors();
    delay(50);
    grabSequence();
    delay(600); // tránh spam grab
    lastTime = millis();
    return;
  }

  // 2) Read sensors and compute weighted position
  int r1 = digitalRead(S1);
  int r2 = digitalRead(S2);
  int r3 = digitalRead(S3);
  int r4 = digitalRead(S4);
  int r5 = digitalRead(S5);
  // manual: 1 = white, 0 = black (line = black)
  // we want value=1 when sensor sees the line (black), so invert:
  int v1 = 1 - r1;
  int v2 = 1 - r2;
  int v3 = 1 - r3;
  int v4 = 1 - r4;
  int v5 = 1 - r5;

  int vals[5] = {v1, v2, v3, v4, v5};
  int sumVal = v1 + v2 + v3 + v4 + v5;

  bool lost = (sumVal == 0);

  float position = 0.0; // weighted average position (-2 .. +2)
  if (!lost) {
    int wsum = 0;
    for (int i = 0; i < 5; ++i) {
      wsum += weights[i] * vals[i];
    }
    position = (float)wsum / (float)sumVal; // e.g. -2..2
    lastKnownError = position;
  }

  // Desired position is 0 (center). Error = position - desired
  float error = position; // desired = 0
  // PID
  integral += error * dt;
  // anti-windup
  if (integral > integralLimit) integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;
  float derivative = (error - lastError) / dt;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  // When inverted sign might be needed depending on robot mapping.
  // We'll use: left = base + correction, right = base - correction
  // so positive correction turns robot right.
  int leftPWM, rightPWM;

  if (!lost) {
    float leftF = baseSpeed + correction;
    float rightF = baseSpeed - correction;

    // map to obey minSpeed and maxSpeed with sign
    leftPWM = constrain((int)leftF, -maxSpeed, maxSpeed);
    rightPWM = constrain((int)rightF, -maxSpeed, maxSpeed);

    // ensure minimum forward speed if both positive
    if (leftPWM > 0 && leftPWM < minSpeed) leftPWM = minSpeed;
    if (rightPWM > 0 && rightPWM < minSpeed) rightPWM = minSpeed;

    // If both sides went negative (rare), clip to small backward speed or stop
  } else {
    // LOST: rotate toward last known direction
    if (lastKnownError > 0.1) {
      // last seen to the right -> rotate right in place
      leftPWM = searchSpeed;
      rightPWM = -searchSpeed;
    } else if (lastKnownError < -0.1) {
      // last seen to the left -> rotate left
      leftPWM = -searchSpeed;
      rightPWM = searchSpeed;
    } else {
      // unknown -> spin slowly right
      leftPWM = searchSpeed;
      rightPWM = -searchSpeed;
    }
    // small timeout handling could be added: if lost for long -> stop
  }

  // Apply motor PWM (handle negative for reverse)
  setMotorPWM(leftPWM, rightPWM);

  lastError = error;
  lastTime = now;

  // Debug (comment out if too verbose)
  Serial.print("s:");
  Serial.print(v1); Serial.print(v2); Serial.print(v3); Serial.print(v4); Serial.print(v5);
  Serial.print(" pos:"); Serial.print(position);
  Serial.print(" err:"); Serial.print(error);
  Serial.print(" cor:"); Serial.print(correction);
  Serial.print(" L:"); Serial.print(leftPWM); Serial.print(" R:"); Serial.println(rightPWM);

  delay(15); // small loop delay for stability
}

// ---------- Motor helper functions ----------
void setMotorPWM(int leftVal, int rightVal) {
  // leftVal/rightVal range can be negative (reverse)
  // left wheel uses IN1 (forward) and IN2 (back)
  if (leftVal >= 0) {
    analogWrite(IN1, constrain(leftVal, 0, 255));
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, constrain(-leftVal, 0, 255));
  }

  // right wheel uses IN3 (forward) and IN4 (back)
  if (rightVal >= 0) {
    analogWrite(IN3, constrain(rightVal, 0, 255));
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, constrain(-rightVal, 0, 255));
  }
}

void stopMotors() {
  analogWrite(IN1, 0); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, 0);
}

// ---------- Ultrasonic ----------
int getDistance(int trigPin, int echoPin) {
  long duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999;
  return int(duration * 0.034 / 2);
}

// ---------- Simple grab sequence ----------
void grabSequence() {
  Serial.println("GRAB start");
  stopMotors();
  delay(80);
  // example sequence (tune angles and timing to physical setup)
  servoRot.write(90);   // xoay vào vị trí lấy
  delay(300);
  servoGrip.write(70);  // kẹp
  delay(350);
  // nâng lên / lùi nhẹ nếu cần
  moveLeftWheel(100,0); moveRightWheel(100,0);
  delay(150);
  stopMotors();
  // giữ vật, xoay về vị trí chứa
  servoRot.write(0);
  delay(300);
  // mở nếu muốn thả (servoGrip.write(0))
  Serial.println("GRAB done");
}

// Utility small motor helpers (direct)
void moveLeftWheel(int pwmF, int pwmB) {
  analogWrite(IN1, pwmF);
  analogWrite(IN2, pwmB);
}
void moveRightWheel(int pwmF, int pwmB) {
  analogWrite(IN3, pwmF);
  analogWrite(IN4, pwmB);
}
