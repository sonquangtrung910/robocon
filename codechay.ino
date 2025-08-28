-
#include <Servo.h> // Thư viện bắt buộc để điều khiển servo
// --- KHAI BÁO ĐỐI TƯỢNG SERVO ---
Servo servoKep; // Servo 1 điều khiển kẹp/mở, nối chân D7
Servo servoXoay; // Servo 2 điều khiển xoay tay gắp, nối chân D4

// --- KHAI BÁO CÁC CHÂN KẾT NỐI (THEO ĐÚNG TÀI LIỆU) ---

// 1. Cảm biến dò line (5 mắt)
#define IR1_PIN 8  // Mắt trái ngoài cùng
#define IR2_PIN 10 // Mắt trái
#define IR3_PIN 2  // Mắt giữa (ĐÃ ĐỔI TỪ D11 SANG D2)
#define IR4_PIN 12 // Mắt phải
#define IR5_PIN 13 // Mắt phải ngoài cùng

// 2. Động cơ DC (qua HUSC Shield)
#define MOTOR_L_IN1 11 // (ĐÃ ĐỔI TỪ D9 SANG D11)
#define MOTOR_L_IN2 6
#define MOTOR_R_IN1 5  // Tương ứng IN3 trong tài liệu
#define MOTOR_R_IN2 3  // Tương ứng IN4 trong tài liệu

// 3. Nút nhấn
#define BUTTON_START_PIN A2 // Nút nhấn bắt đầu được nối vào chân A2

// --- KHAI BÁO CÁC BIẾN TOÀN CỤC ---
int tocDoCoSo = 150;     // Tốc độ di chuyển cơ bản (0-255)
int tocDoRe = 180;       // Tốc độ khi rẽ, cần nhanh hơn để đáp ứng
bool daBatDau = false;   // Biến cờ để bắt đầu robot
int demVachNgang = 0;    // Biến đếm số lần gặp vạch ngang

// ==========================================================================
// HÀM SETUP - CHẠY 1 LẦN KHI KHỞI ĐỘNG
// ==========================================================================
void setup() {
  Serial.begin(9600);

  // Cấu hình các chân động cơ là OUTPUT
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  // Cấu hình các chân cảm biến dò line là INPUT
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(IR4_PIN, INPUT);
  pinMode(IR5_PIN, INPUT);

  // Cấu hình chân nút nhấn là INPUT.
  // Module nút nhấn trong bộ kit thường có sẵn điện trở kéo lên,
  // nên chỉ cần dùng INPUT là đủ.
  pinMode(BUTTON_START_PIN, INPUT);

  // Gắn đối tượng servo vào các chân tương ứng
  servoKep.attach(7);
  servoXoay.attach(4);

  // Thiết lập trạng thái ban đầu cho tay gắp: Mở và ở vị trí ban đầu
  servoKep.write(0);  // 0 độ: Mở tối đa
  servoXoay.write(0); // Quay về vị trí 0 độ
  
  Serial.println("Robot san sang! Nhan nut (A2) de bat dau...");
}

// ==========================================================================
// HÀM LOOP - CHẠY LẶP LẠI VÔ HẠN
// ==========================================================================
void loop() {
  // Robot chỉ bắt đầu khi biến daBatDau là true
  if (!daBatDau) {
    // Đọc trạng thái của nút nhấn. Nhấn là LOW (0)
    int trangThaiNutNhan = digitalRead(BUTTON_START_PIN);
    
    if (trangThaiNutNhan == LOW) {
      delay(50); // Chống dội đơn giản
      // Kiểm tra lại lần nữa để chắc chắn đây là một lần nhấn
      if (digitalRead(BUTTON_START_PIN) == LOW) {
        Serial.println("BAT DAU!");
        daBatDau = true;
        // Đi thẳng một đoạn ngắn để vào line
        diThang(tocDoCoSo);
        delay(300);
      }
    }
    return; // Thoát khỏi hàm loop và chờ lần kiểm tra tiếp theo
  }

  // --- Chương trình chính khi robot đã bắt đầu ---
  
  // 1. Đọc giá trị từ 5 cảm biến dò line
  int s1 = digitalRead(IR1_PIN);
  int s2 = digitalRead(IR2_PIN);
  int s3 = digitalRead(IR3_PIN);
  int s4 = digitalRead(IR4_PIN);
  int s5 = digitalRead(IR5_PIN);

  // In trạng thái cảm biến để debug
  Serial.print("Cam bien: ");
  Serial.print(s1); Serial.print(s2); Serial.print(s3); Serial.print(s4); Serial.print(s5);
  Serial.print(" | Vach ngang: "); Serial.println(demVachNgang);

  // 2. Logic xử lý các trường hợp dò line
  
  // Gặp vạch ngang
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
    xuLyVachNgang();
  }
  // Đi thẳng
  else if (s1 == 1 && s2 == 1 && s3 == 0 && s4 == 1 && s5 == 1) {
    diThang(tocDoCoSo);
  }
  // Lệch trái nhẹ
  else if (s1 == 1 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 1) {
    reTrai(tocDoRe);
  }
  // Lệch phải nhẹ
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 1) {
    rePhai(tocDoRe);
  }
  // Lệch trái nhiều
  else if (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1) {
    reTrai(tocDoRe);
  }
  // Lệch phải nhiều
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 0) {
    rePhai(tocDoRe);
  }
  // Gặp góc cua vuông trái
  else if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 1) {
    diThang(tocDoCoSo);
    delay(150);
    while(digitalRead(IR3_PIN) == 1) { reTrai(tocDoRe); }
  }
  // Gặp góc cua vuông phải
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0) {
    diThang(tocDoCoSo);
    delay(150);
    while(digitalRead(IR3_PIN) == 1) { rePhai(tocDoRe); }
  }
  // Mất line hoàn toàn
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1) {
    dungLai();
  }
}

// ==========================================================================
// CÁC HÀM CHỨC NĂNG PHỤ
// ==========================================================================

void xuLyVachNgang() {
  dungLai();
  delay(200);

  if (demVachNgang == 0) {
    Serial.println("Gap vach ngang lan 1 -> Re trai");
    diThang(tocDoCoSo);
    delay(200);
    reTrai(tocDoRe);
    delay(1000);
  } 
  else if (demVachNgang == 1) {
    Serial.println("Gap vach ngang lan 2 -> Thuc hien gap vat");
    servoKep.write(70);
    delay(500);
    servoXoay.write(90);
    delay(500);
    diLui(tocDoCoSo);
    delay(300);
    reTrai(tocDoRe);
    delay(1100);
    while(digitalRead(IR3_PIN) == 1) {
        reTrai(tocDoRe);
    }
  }
  
  demVachNgang++;
  dungLai();
  delay(500);
}

// --- CÁC HÀM ĐIỀU KHIỂN ĐỘNG CƠ ---

void diThang(int speed) {
  analogWrite(MOTOR_L_IN1, speed);
  analogWrite(MOTOR_L_IN2, 0);
  analogWrite(MOTOR_R_IN1, 0);
  analogWrite(MOTOR_R_IN2, speed);
}

void diLui(int speed) {
  analogWrite(MOTOR_L_IN1, 0);
  analogWrite(MOTOR_L_IN2, speed);
  analogWrite(MOTOR_R_IN1, speed);
  analogWrite(MOTOR_R_IN2, 0);
}

void rePhai(int speed) {
  analogWrite(MOTOR_L_IN1, speed);
  analogWrite(MOTOR_L_IN2, 0);
  analogWrite(MOTOR_R_IN1, speed);
  analogWrite(MOTOR_R_IN2, 0);
}

void reTrai(int speed) {
  analogWrite(MOTOR_L_IN1, 0);
  analogWrite(MOTOR_L_IN2, speed);
  analogWrite(MOTOR_R_IN1, 0);
  analogWrite(MOTOR_R_IN2, speed);
}

void dungLai() {
  analogWrite(MOTOR_L_IN1, 0);
  analogWrite(MOTOR_L_IN2, 0);
  analogWrite(MOTOR_R_IN1, 0);
  analogWrite(MOTOR_R_IN2, 0);
}