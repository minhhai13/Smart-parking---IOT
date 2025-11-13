// --- Thư viện ---
#include "SPI.h"
#include "MFRC522.h"
#include "ESP32Servo.h"
#include <string.h> // Cần cho memcpy

// --- Cấu hình UART (Giao tiếp với ESP-IN) ---
#define UART_BAUD 115200
#define RXD2 16 // Nối với TXD2 (17) của Master
#define TXD2 17 // Nối với RXD2 (16) của Master

// --- Định nghĩa gói tin UART mới ---
#define UART_COMMIT_PACKET 0xFF // Gói tin xác nhận xe đã ra
#define UART_CANCEL_PACKET 0xEE // Gói tin hủy bỏ (xe lùi/timeout)

// --- Cấu hình RC522 (Cổng Ra) ---
#define SS_PIN 5
#define RST_PIN 4
MFRC522 mfrc522(SS_PIN, RST_PIN);

// --- Cấu hình Servo (Cổng Ra) ---
#define SERVO_PIN 13
Servo barrierServo;
const int SERVO_OPEN = 90;
const int SERVO_CLOSED = 0;

// --- Cấu hình Cảm biến (Cổng Ra) ---
#define TRIG1 32 // Sensor trước barrier (S1)
#define ECHO1 33
#define TRIG2 25 // Sensor sau barrier (S2)
#define ECHO2 26

// --- Cấu hình Buzzer (Cổng Ra) ---
#define BUZZER_PIN 27

// --- Thông số ---
const int VEHICLE_THRESHOLD = 10;
const unsigned long BARRIER_TIMEOUT = 15000;
const unsigned long UART_TIMEOUT = 2000; 

// --- State Machine ---
enum VehicleState {
    STATE_0_IDLE = 0,   
    STATE_1_WAITING = 1,
    STATE_2_PASSING = 2,
    STATE_3_CLEARING = 3
};
VehicleState currentState = STATE_0_IDLE;
bool barrierOpen = false;
unsigned long barrierOpenTime = 0;

// --- Biến mới cho logic Commit/Cancel ---
bool exitRequestPending = false; // true nếu Master đã cho phép ra
byte lastAuthorizedUID[4] = {0, 0, 0, 0}; // Lưu UID của thẻ vừa được phép ra


// ===================================
// HÀM CHỨC NĂNG
// ===================================

void beep(int duration) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
}

long getDistance(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH, 30000);
    if (duration == 0) return 999;
    return duration / 58;
}

void openBarrier() {
    if (!barrierOpen) {
        Serial.println(">>> BARRIER OPEN (Exit) <<<");
        barrierServo.write(SERVO_OPEN);
        barrierOpen = true;
        barrierOpenTime = millis();
    }
}

void closeBarrier() {
    if (barrierOpen) {
        Serial.println("<<< BARRIER CLOSED (Exit) >>>");
        barrierServo.write(SERVO_CLOSED);
        barrierOpen = false;
        // KHÔNG reset exitRequestPending ở đây vội
    }
}

// HÀM QUAN TRỌNG NHẤT: Xử lý RFID tại Cổng Ra
void handleRFID_Exit() {
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        byte *uid = mfrc522.uid.uidByte;
        byte size = mfrc522.uid.size;

        Serial.print("Card UID (Exit): ");
        for (byte i = 0; i < size; i++) {
            Serial.print(uid[i] < 0x10 ? " 0" : " ");
            Serial.print(uid[i], HEX);
        }
        Serial.println();

        if (currentState == STATE_1_WAITING && !exitRequestPending) {
            
            // ✅ XÓA BUFFER TRƯỚC KHI GỬI!
            while (Serial2.available()) {
                Serial2.read(); // Xóa dữ liệu cũ
            }
            delay(10); // Đợi một chút cho chắc
            
            // 1. Gửi 4 bytes UID (Yêu cầu) cho Master
            Serial.println("UART SEND (to Master): Requesting permission...");
            Serial2.write(uid, 4);
            Serial2.flush(); // ✅ Đảm bảo dữ liệu được gửi hết

            // 2. Chờ Master phản hồi (với timeout)
            unsigned long waitStart = millis();
            bool received = false;
            while (millis() - waitStart < UART_TIMEOUT) {
                if (Serial2.available()) {
                    char response = Serial2.read();
                    Serial.print("UART RECV (from Master): '");
                    Serial.print(response); Serial.println("'");

                    if (response == 'O') { // 'O' = OK
                        Serial.println("✅ ACCESS GRANTED (from Master)");
                        exitRequestPending = true;
                        memcpy(lastAuthorizedUID, uid, 4);
                        openBarrier();
                    } else if (response == 'E') { // 'E' = Error
                        Serial.println("⛔ ACCESS DENIED - Vehicle Not Inside");
                        beep(500);
                    } else if (response == 'I') { // 'I' = Invalid
                        Serial.println("⛔ ACCESS DENIED - Invalid Card");
                        beep(500);
                    }
                    received = true;
                    break;
                }
            }

            if (!received) {
                Serial.println("⛔ ERROR - Master not responding (Timeout)");
                beep(1000);
            }
        }

        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
        delay(300);
    }
}

VehicleState determineState(bool s1, bool s2) {
    if (!s1 && !s2) return STATE_0_IDLE;
    if (s1 && !s2) return STATE_1_WAITING;
    if (s1 && s2) return STATE_2_PASSING;
    if (!s1 && s2) return STATE_3_CLEARING;
    return STATE_0_IDLE; // Fallback
}

void handleStateTransition(VehicleState newState) {
    if (newState == currentState) return; 

    Serial.print("State transition (Exit): ");
    Serial.print(currentState);
    Serial.print(" -> ");
    Serial.println(newState);

    VehicleState oldState = currentState;
    currentState = newState;

    switch (newState) {
        case STATE_0_IDLE:
            if (oldState == STATE_3_CLEARING) {
                // Xe đã ra thành công
                Serial.println("✓ Vehicle exited successfully (Exit)");
                if (exitRequestPending) {
                    // Gửi gói tin COMMIT (0xFF)
                    Serial.println("UART SEND (to Master): COMMIT packet (0xFF)");
                    Serial2.write(UART_COMMIT_PACKET);
                    Serial2.write(lastAuthorizedUID, 4);
                    exitRequestPending = false; // Hoàn thành giao dịch
                }
            } else if (oldState == STATE_1_WAITING) {
                // Xe lùi ra
                Serial.println("⚠️ Vehicle reversed - Exit cancelled");
                if (exitRequestPending) {
                    // Gửi gói tin CANCEL (0xEE)
                    Serial.println("UART SEND (to Master): CANCEL packet (0xEE)");
                    Serial2.write(UART_CANCEL_PACKET);
                    Serial2.write(lastAuthorizedUID, 4);
                    exitRequestPending = false; // Hủy giao dịch
                }
            }
            closeBarrier();
            break;

        case STATE_1_WAITING:
            Serial.println("Vehicle waiting at exit - Please scan card");
            break;

        case STATE_2_PASSING:
            Serial.println("Vehicle passing through exit barrier");
            break;

        case STATE_3_CLEARING:
            Serial.println("Vehicle clearing the exit gate");
            break;
    }
}

// ===================================
// SETUP & LOOP
// ===================================

void setup() {
    Serial.begin(115200); 
    
    Serial2.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2); 
    Serial.println("\n=== Exit Gate System (SLAVE - FIXED) ===");

    SPI.begin(18, 19, 23, SS_PIN);
    mfrc522.PCD_Init();
    Serial.println("RFID OK");

    pinMode(TRIG1, OUTPUT);
    pinMode(ECHO1, INPUT);
    pinMode(TRIG2, OUTPUT);
    pinMode(ECHO2, INPUT);
    Serial.println("Sensors OK");

    barrierServo.attach(SERVO_PIN);
    barrierServo.write(SERVO_CLOSED);
    Serial.println("Servo OK");
    
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    beep(100); 
    Serial.println("Buzzer OK");

    Serial.println("System ready - Waiting for vehicles...");
}

void loop() {
    // 1. Đọc cảm biến (Cổng Ra)
    long dist1 = getDistance(TRIG1, ECHO1);
    long dist2 = getDistance(TRIG2, ECHO2);
    bool s1 = (dist1 < VEHICLE_THRESHOLD);
    bool s2 = (dist2 < VEHICLE_THRESHOLD);

    // 2. Cập nhật State Machine (Cổng Ra)
    VehicleState newState = determineState(s1, s2);
    handleStateTransition( newState);

    // 3. Xử lý RFID (Cổng Ra)
    if (currentState == STATE_1_WAITING) {
        handleRFID_Exit();
    }

    // 4. Xử lý Timeout (Cổng ra)
    if (barrierOpen && (millis() - barrierOpenTime > BARRIER_TIMEOUT)) {
        if (currentState == STATE_0_IDLE || currentState == STATE_1_WAITING) {
            Serial.println("⏱️ Barrier timeout (Exit) - Closing for safety");
            
            if (exitRequestPending) {
                // Xe không đi qua -> Gửi gói tin CANCEL (0xEE)
                Serial.println("UART SEND (to Master): CANCEL packet (0xEE) - Timeout");
                Serial2.write(UART_CANCEL_PACKET);
                Serial2.write(lastAuthorizedUID, 4);
                exitRequestPending = false; // Hủy giao dịch
            }
            closeBarrier();
        }
    }

    delay(100); 
}