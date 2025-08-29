#include <Servo.h>

#define IN1 11
#define IN2 6
#define IN3 5
#define IN4 3

// Ultrasonic pins
#define TRIG_PIN A0
#define ECHO_PIN A1

// Servo pins
#define SERVO_GRIP_PIN 7
#define SERVO_ROT_PIN 4

unsigned long thoigian;
unsigned long hientai = 0;
int timecho = 1000; // ms chờ sau khi gắp

// 2 chân enable đã được nối với 5V rồi
// ta sẽ điều khiển tốc độ động cơ trực tiếp
// qua 4 chân này
int toc_do = 200; // biến toc_độ giá trị nằm trong khoảng(0-255)

Servo servoGrip;
Servo servoRot;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600); // Bắt đầu giao tiếp Serial với baudrate 9600

  servoGrip.attach(SERVO_GRIP_PIN);
  servoRot.attach(SERVO_ROT_PIN);

  // vị trí khởi tạo servo (mở tay, xoay về 0)
  servoGrip.write(0);  // mở
  servoRot.write(0);   // vị trí ban đầu

  thoigian = millis();
}

// ------- Hàm di chuyển (bạn cung cấp) -------
void moveForward(int speed) { // Hàm tiến
  analogWrite(IN1, speed); // bên trái quay tiến
  analogWrite(IN2, 0);
  analogWrite(IN3, 0); // bên phải quay tiến
  analogWrite(IN4, speed);
}
void turnLeft(int speed) { // hàm quay trái tại chỗ
  analogWrite(IN1, 0); // bên trái quay lùi
  analogWrite(IN2, speed);
  analogWrite(IN3, 0); // bên phải quay tiến
  analogWrite(IN4, speed);
}
void turnLeft1(int speed) { // hàm rẽ trái
  analogWrite(IN1, 0); // bên trái đứng yên
  analogWrite(IN2, 0);
  analogWrite(IN3, 0); // bên phải quay tiến
  analogWrite(IN4, speed);
}
void turnRight(int speed) { // hàm quay phải tại chỗ
  analogWrite(IN1, speed); // bên trái quay tiến
  analogWrite(IN2, 0);
  analogWrite(IN3, speed); // bên phải quay lùi
  analogWrite(IN4, 0);
}
void turnRight1(int speed) { // hàm rẽ phải
  analogWrite(IN1, speed); // Bên trái quay tiến
  analogWrite(IN2, 0);
  analogWrite(IN3, 0); // Bên phải đứng yên
  analogWrite(IN4, 0);
}
void moveBackward(int speed) { // Hàm đi lùi
  analogWrite(IN1, 0); // bên trái quay lùi
  analogWrite(IN2, speed);
  analogWrite(IN3, speed); // bên phải quay lùi
  analogWrite(IN4, 0);
}
void stopMotors() { // Hàm stop
  analogWrite(IN1, 0); // bên trái đứng yên
  analogWrite(IN2, 0);
  analogWrite(IN3, 0); // bên phải đứng yên
  analogWrite(IN4, 0);
}

// ---------- Hàm đo siêu âm (1 lần) ----------
long measureDistanceCmOnce() {
  // gửi xung 10 microseconds
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // đọc thời gian (timeout 30000 us)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1; // timeout / không nhận
  // dùng hệ số 0.01715 (duration(us) * 0.01715 = cm)
  long dist = (long)(duration * 0.01715);
  return dist;
}

// ---------- Lấy median của 3 lần đo (ổn định hơn) ----------
long measureDistanceCm() {
  const int S = 3;
  long arr[S];
  int valid = 0;
  for (int i = 0; i < S; ++i) {
    long d = measureDistanceCmOnce();
    if (d > 0) {
      arr[valid++] = d;
    }
    delay(20);
  }
  if (valid == 0) return -1;
  // sắp xếp nhỏ (valid <=3)
  for (int i = 1; i < valid; ++i) {
    long key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
  // median
  return arr[valid / 2];
}

// ---------- Chuỗi gắp đơn giản ----------
void grabSequence() {
  Serial.println("Start grab sequence");
  stopMotors();
  delay(80);

  // xoay tay vào vị trí lấy (tùy cơ cấu của bạn)
  servoRot.write(90);   // xoay ra giữa
  delay(300);

  // đóng kẹp (gáo kẹp) - chỉnh góc phù hợp servo của bạn
  servoGrip.write(70);  // kẹp (thay 70 bằng góc phù hợp)
  delay(400);

  // có thể lùi nhẹ để cố định
  moveBackward(150);
  delay(200);
  stopMotors();

  // xoay tay về vị trí chứa (nếu cần)
  servoRot.write(0);
  delay(300);

  // nếu muốn mở kẹp để thả, gọi servoGrip.write(0);
  Serial.println("Grab done");
}

void loop() {
  // di chuyển tiến liên tục
  moveForward(toc_do);

  // đọc siêu âm không quá thường xuyên
  if (millis() - thoigian >= 100) { // mỗi 100 ms đo một lần
    thoigian = millis();
    long d = measureDistanceCm();
    if (d < 0) {
      Serial.println("No echo / out of range");
    } else {
      Serial.print("Distance (cm): ");
      Serial.println(d);
      // nếu <= 10 cm -> dừng và gắp
      if (d <= 10) {
        stopMotors();
        grabSequence();
        // tránh lặp lại liên tục: chờ timecho
        delay(timecho);
      }
    }
  }

  // bạn có thể thêm delay nhỏ hoặc làm việc khác
  // delay(10); // optional
}
