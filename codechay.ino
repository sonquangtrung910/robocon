// ROBOT dò line + gắp khi siêu âm <= 10 cm
// Dựa trên HUSC Shield manual (pin mapping & ví dụ)
// Servo SE1 (gắp) -> D7, SE2 (xoay) -> D4
// SR04 TRIG -> A0, ECHO -> A1
// Motor: IN1=11, IN2=6, IN3=5, IN4=3
// IR: ir1=8, ir2=10, ir3=2, ir4=12, ir5=13

#include <Servo.h>

#define IN1 11
#define IN2 6
#define IN3 5
#define IN4 3

#define ir1 8
#define ir2 10
#define ir3 2   // chú ý: manual chuyển IR3 sang D2 khi đổi IN1 -> D11
#define ir4 12
#define ir5 13

#define TRIG A0
#define ECHO A1

#define SERVO_GRIP_PIN 7  // servo1: mở/kẹp
#define SERVO_ROT_PIN  4  // servo2: xoay

Servo servoGrip; // SE1
Servo servoRot;  // SE2

int speedDefault = 180; // 0-255, chỉnh tuỳ robot

// ---------- Hàm cơ bản điều khiển motor ----------
void moveForward(int sp) {
  analogWrite(IN1, sp); analogWrite(IN2, 0);
  analogWrite(IN3, 0);  analogWrite(IN4, sp);
}
void moveBackward(int sp) {
  analogWrite(IN1, 0);  analogWrite(IN2, sp);
  analogWrite(IN3, sp);  analogWrite(IN4, 0);
}
void stopMotors() {
  analogWrite(IN1, 0); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, 0);
}
void turnLeft(int sp) {
  analogWrite(IN1, 0); analogWrite(IN2, sp);
  analogWrite(IN3, 0); analogWrite(IN4, sp);
}
void turnRight(int sp) {
  analogWrite(IN1, sp); analogWrite(IN2, 0);
  analogWrite(IN3, sp); analogWrite(IN4, 0);
}

// ---------- Hàm đọc siêu âm (theo manual) ----------
int getDistance(int trigPin, int echoPin) {
  long duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // thêm timeout 30000us để tránh treo nếu không có phản hồi
  duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999; // không nhận tín hiệu -> very far
  return int(duration * 0.034 / 2); // cm
}

// ---------- Hàm gắp (sequence) ----------
void grabSequence() {
  Serial.println("START GRAB SEQUENCE");
  stopMotors();
  delay(100);

  // 1) Đưa servo gắp vào vị trí kẹp (servo1 ~ 70 deg)
  servoGrip.write(70); // kẹp (manual gợi ý 70-80)
  delay(500);

  // 2) Xoay tay (servo2) về vị trí lấy vật (90 độ là ví dụ)
  servoRot.write(90);
  delay(500);

  // 3) Lùi nhẹ để tách vật/hỗ trợ xoay
  moveBackward(150);
  delay(300);
  stopMotors();

  // (Nếu muốn thả vật sau thao tác, thêm servoGrip.write(0) để mở)
  Serial.println("GRAB DONE");
}

// ---------- Setup & Loop ----------
void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // IR pins
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  // Ultrasonic
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Servos
  servoGrip.attach(SERVO_GRIP_PIN);
  servoRot.attach(SERVO_ROT_PIN);

  // Init positions (manual recommends servo1=0 open, servo2=0/180 depending)
  servoGrip.write(0);   // mở
  servoRot.write(0);    // vị trí ban đầu
  delay(200);
}

void loop() {
  // 1) Đọc khoảng cách trước
  int dist = getDistance(TRIG, ECHO);
  Serial.print("Distance: "); Serial.print(dist); Serial.println(" cm");

  // Nếu có vật ở gần <= 10cm -> thực hiện gắp
  if (dist <= 10) {
    grabSequence();
    // chờ một chút để tránh gắp lặp lại liên tục
    delay(1000);
    return; // quay lại loop
  }

  // 2) Đọc IR để dò line (logic đơn giản tham khảo manual)
  int s1 = digitalRead(ir1);
  int s2 = digitalRead(ir2);
  int s3 = digitalRead(ir3);
  int s4 = digitalRead(ir4);
  int s5 = digitalRead(ir5);

  // manual: 1 = white, 0 = black (line = black)
  // Nếu line ở giữa -> đi thẳng
  if (s1 == 1 && s2 == 1 && s3 == 0 && s4 == 1 && s5 == 1) {
    moveForward(speedDefault);
  }
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
    // giao điểm ngang / checkpoint -> dừng & có thể chạy gắp theo ví dụ
    stopMotors();
    delay(200);
    // (tùy nhu cầu: có thể gọi grabSequence() ở đây khi muốn gắp khi gặp checkpoint)
    moveForward(120);
    delay(200);
  }
  else if ( (s1==1 && s2==1 && s3==1 && s4==0) || (s4==0 && s5==1) ) {
    // rẽ phải nhẹ
    turnRight(150);
  }
  else if ( (s1==0 && s2==1 && s3==1 && s4==1) || (s1==1 && s2==0) ) {
    // rẽ trái nhẹ
    turnLeft(150);
  }
  else {
    // mặc định: đi chậm
    moveForward(140);
  }

  delay(50); // loop nhỏ
}
