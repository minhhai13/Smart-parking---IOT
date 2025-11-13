#include <Wire.h>

#include <LiquidCrystal_I2C.h>

#include <RTClib.h>

#include "SPI.h"

#include "MFRC522.h"

#include "ESP32Servo.h"

#include <WiFi.h>

#include <HTTPClient.h>



// --- THAY ĐỔI THÔNG TIN CỦA BẠN TẠI ĐÂY ---

const char* WIFI_SSID = "TANG 3.5G";

const char* WIFI_PASS = "1234567890";

String GOOGLE_SCRIPT_URL = "https://script.google.com/macros/s/AKfycby8S1JUhxNVxMc74WWl9dVVjWGv4MjHM0DJ9e5Bqsl1V77prceYlJIUedCALnP7M_6T/exec";

// --- KẾT THÚC THAY ĐỔI ---



// --- Cấu hình UART (Giao tiếp với ESP-OUT) ---

#define UART_BAUD 115200

#define RXD2 16

#define TXD2 17



// --- Định nghĩa gói tin UART mới ---

#define UART_COMMIT_PACKET 0xFF // Gói tin xác nhận xe đã ra

#define UART_CANCEL_PACKET 0xEE // Gói tin hủy bỏ (xe lùi/timeout)



// --- Cấu hình RC522 (Cổng Vào) ---

#define SS_PIN 5

#define RST_PIN 4

MFRC522 mfrc522(SS_PIN, RST_PIN);



// --- Cấu hình Servo (Cổng Vào) ---

#define SERVO_PIN 13

Servo barrierServo;

const int SERVO_OPEN = 90;

const int SERVO_CLOSED = 0;



// --- Cấu hình Cảm biến (Cổng Vào) ---

#define TRIG1 32 // Sensor trước barrier (S1)

#define ECHO1 33

#define TRIG2 25 // Sensor sau barrier (S2)

#define ECHO2 26



// --- Cấu hình Buzzer (Cổng Vào) ---

#define BUZZER_PIN 27



// --- Cấu hình LCD 1602 I2C ---

LiquidCrystal_I2C lcd(0x27, 16, 2);



// --- Cấu hình RTC & EEPROM ---

RTC_DS3231 rtc;

#define EEPROM_ADDR 0x57

#define EEPROM_SLOTS_ADDR 0

#define EEPROM_STATE_START_ADDR 1

const int TOTAL_SLOTS = 9;



// --- Danh sách UID hợp lệ ---

const byte allowedUIDs[][4] = {

    {0x41, 0x7B, 0xF7, 0x16}, // Index 0

    {0xF7, 0x13, 0x73, 0x06}, // Index 1

    {0x0E, 0x77, 0xF9, 0x05}, // Index 2

    {0xFE, 0x91, 0xC9, 0x01}  // Index 3

};

const int ALLOWED_COUNT = 4;

int availableSlots = TOTAL_SLOTS;



// --- Thông số ---

const int VEHICLE_THRESHOLD = 10;

const unsigned long BARRIER_TIMEOUT = 15000;



// --- State Machine ---

enum VehicleState {

    STATE_0_IDLE = 0,

    STATE_1_WAITING = 1,

    STATE_2_PASSING = 2,

    STATE_3_CLEARING = 3

};

VehicleState currentState = STATE_0_IDLE;

bool barrierOpen = false;

bool cardAuthorized = false;

unsigned long barrierOpenTime = 0;



// ===================================

// HÀM EEPROM

// ===================================



void writeEEPROM(int i2c_addr, unsigned int eeaddress, byte data) {

    Wire.beginTransmission(i2c_addr);

    Wire.write((int)(eeaddress >> 8));   // MSB

    Wire.write((int)(eeaddress & 0xFF)); // LSB

    Wire.write(data);

    Wire.endTransmission();

    delay(5);

}



byte readEEPROM(int i2c_addr, unsigned int eeaddress) {

    byte rdata = 0xFF;

    Wire.beginTransmission(i2c_addr);

    Wire.write((int)(eeaddress >> 8));   // MSB

    Wire.write((int)(eeaddress & 0xFF)); // LSB

    Wire.endTransmission();

    Wire.requestFrom(i2c_addr, 1);

    if (Wire.available()) rdata = Wire.read();

    return rdata;

}



int getCardState(int cardIndex) {

    if (cardIndex < 0 || cardIndex >= ALLOWED_COUNT) return -1;

    return readEEPROM(EEPROM_ADDR, EEPROM_STATE_START_ADDR + cardIndex);

}



void setCardState(int cardIndex, int state) {

    if (cardIndex < 0 || cardIndex >= ALLOWED_COUNT) return;

    writeEEPROM(EEPROM_ADDR, EEPROM_STATE_START_ADDR + cardIndex, (byte)state);

    Serial.print("SET STATE: Card index "); Serial.print(cardIndex);

    Serial.print(" = "); Serial.println(state == 1 ? "IN" : "OUT");

}



void updateSlots(int slots) {

    availableSlots = slots;

    if (availableSlots < 0) availableSlots = 0;

    if (availableSlots > TOTAL_SLOTS) availableSlots = TOTAL_SLOTS;

    writeEEPROM(EEPROM_ADDR, EEPROM_SLOTS_ADDR, (byte)availableSlots);

    Serial.print("SLOTS UPDATED: "); Serial.println(availableSlots);

    updateLCD();

}



// ===================================

// HÀM LCD & BUZZER

// ===================================



void setupLCD() {

    lcd.init();

    lcd.backlight();

    lcd.clear();

    lcd.setCursor(0, 0);

    lcd.print("Smart Parking");

    lcd.setCursor(0, 1);

    lcd.print("Loading...");

}



void updateLCD() {

    char line1[17];

    sprintf(line1, "Slots: %d/%d", availableSlots, TOTAL_SLOTS);

    lcd.clear(); // Xóa toàn bộ để tránh lỗi

    lcd.setCursor(0, 0);

    lcd.print(line1);

}



void lcdMessage(String msg) {

    lcd.setCursor(0, 1);

    lcd.print("                "); // Clear line 2

    lcd.setCursor(0, 1);

    lcd.print(msg);

}



void beep(int duration) {

    digitalWrite(BUZZER_PIN, HIGH);

    delay(duration);

    digitalWrite(BUZZER_PIN, LOW);

}



// ===================================

// HÀM GỐC (Đã sửa đổi)

// ===================================



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

        Serial.println(">>> BARRIER OPEN (Entry) <<<");

        lcdMessage("Welcome!");

        barrierServo.write(SERVO_OPEN);

        barrierOpen = true;

        barrierOpenTime = millis();

    }

}



void closeBarrier() {

    if (barrierOpen) {

        Serial.println("<<< BARRIER CLOSED (Entry) >>>");

        barrierServo.write(SERVO_CLOSED);

        barrierOpen = false;

        cardAuthorized = false;

        updateLCD(); // Quay về màn hình slot

        lcdMessage("Scan card...");

    }

}



int getCardIndex(byte *uid, byte size) {

    for (int i = 0; i < ALLOWED_COUNT; i++) {

        bool match = true;

        for (byte j = 0; j < size; j++) {

            if (uid[j] != allowedUIDs[i][j]) {

                match = false;

                break;

            }

        }

        if (match) {

            return i; // Trả về index

        }

    }

    return -1; // Không tìm thấy

}



void printUID(byte *uid, byte size) {

    for (byte i = 0; i < size; i++) {

        Serial.print(uid[i] < 0x10 ? " 0" : " ");

        Serial.print(uid[i], HEX);

    }

}



// ===================================

// HÀM GHI LOG (GOOGLE SHEETS) - MỚI

// ===================================



// Hàm chuyển đổi UID (byte array) sang String (dạng Hex)

String getUIDString(byte *uid, byte size) {

  String uidStr = "";

  for (byte i = 0; i < size; i++) {

    if (uid[i] < 0x10) uidStr += "0"; // Thêm '0' nếu < 16

    uidStr += String(uid[i], HEX);

  }

  uidStr.toUpperCase(); // In hoa cho đẹp (VD: 417BF716)

  return uidStr;

}



// Hàm gửi log lên Google Sheet

// *** LƯU Ý: HÀM NÀY SẼ GÂY TRỄ VÀI GIÂY KHI CHẠY ***

void logToGoogleSheet(String uid, String action) {

  if (WiFi.status() != WL_CONNECTED) {

    Serial.println("GCS: WiFi not connected, skipping log.");

    return;

  }



  HTTPClient http;

 

  // Tạo URL đầy đủ với tham số

  // Ví dụ: .../exec?uid=417BF716&action=Entry

  String url = GOOGLE_SCRIPT_URL + "?uid=" + uid + "&action=" + action;



  Serial.print("GCS: Sending request to: ");

  Serial.println(url);



  http.begin(url);

  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS); // Rất quan trọng!

  int httpCode = http.GET();



  if (httpCode > 0) {

    Serial.printf("GCS: HTTP Code: %d\n", httpCode);

    if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {

      String payload = http.getString();

      Serial.print("GCS: Response: ");

      Serial.println(payload);

    }

  } else {

    Serial.printf("GCS: GET failed, error: %s\n", http.errorToString(httpCode).c_str());

  }



  http.end();

}





// Xử lý logic CỔNG VÀO

void handleRFID_Entry() {

    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {

        byte *uid = mfrc522.uid.uidByte;

        byte size = mfrc522.uid.size;



        Serial.print("Card UID (Entry): ");

        printUID(uid, size);

        Serial.println();



        if (currentState == STATE_1_WAITING && !cardAuthorized) {

           

            int cardIndex = getCardIndex(uid, size);



            if (cardIndex != -1) {

                // Thẻ hợp lệ, kiểm tra logic bãi xe

               

                if (availableSlots == 0) {

                    Serial.println("⛔ ACCESS DENIED - Parking Full");

                    lcdMessage("Loi: Bai Da Day");

                    beep(500);

                } else {

                    int state = getCardState(cardIndex);

                    if (state == 1) { // 1 = IN

                        Serial.println("⛔ ACCESS DENIED - Vehicle Already Inside");

                        lcdMessage("Loi: Xe Da O Trong");

                        beep(500);

                    } else {

                        // HỢP LỆ HOÀN TOÀN

                        Serial.println("✅ ACCESS GRANTED - Entry allowed");

                        cardAuthorized = true;

                       

                        // Cập nhật logic:

                        setCardState(cardIndex, 1); // 1 = IN

                        updateSlots(availableSlots - 1);

                       

                        // *** GỌI HÀM LOG TẠI ĐÂY ***

                        logToGoogleSheet(getUIDString(uid, size), "Entry");

                        // *** KẾT THÚC GỌI LOG ***

                       

                        openBarrier(); // Mở barrier

                    }

                }

            } else {

                // Thẻ không hợp lệ

                Serial.println("⛔ ACCESS DENIED - Invalid card");

                lcdMessage("Loi: The Khong Hop Le");

                beep(500);

            }

        }



        mfrc522.PICC_HaltA();

        mfrc522.PCD_StopCrypto1();

        delay(300);

    }

}



// HÀM MỚI: Xử lý YÊU CẦU từ Cổng Ra (4 bytes)

void processUART_Request() {

    byte uidFromExit[4];

    Serial2.readBytes(uidFromExit, 4);



    Serial.print("UART RECV (from Exit): REQUEST packet ");

    printUID(uidFromExit, 4);

    Serial.println();

   

    int cardIndex = getCardIndex(uidFromExit, 4);



    if (cardIndex != -1) {

        int state = getCardState(cardIndex);

        if (state == 1) { // 1 = IN (Hợp lệ để đi ra)

            Serial.println("UART SEND (to Exit): 'O' (OK)");

            Serial2.write('O'); // 'O' = OK

            // *** KHÔNG CẬP NHẬT EEPROM TẠI ĐÂY NỮA ***

        } else {

            // Xe đang "Ở NGOÀI" nhưng lại đòi đi "RA"

            Serial.println("UART SEND (to Exit): 'E' (Error - Not Inside)");

            Serial2.write('E'); // 'E' = Error (Lỗi logic)

        }

    } else {

        // Thẻ không có trong hệ thống

        Serial.println("UART SEND (to Exit): 'I' (Invalid)");

        Serial2.write('I'); // 'I' = Invalid (Thẻ không hợp lệ)

    }

}





// HÀM MỚI: Xử lý logic CỔNG RA (từ ESP-OUT)

// Hàm này giờ sẽ phân loại gói tin

void handleUART_Exit() {

    if (Serial2.available() >= 1) {

        byte firstByte = Serial2.peek();

       

        // Kiểm tra xem có phải COMMIT/CANCEL không

        if (firstByte == UART_COMMIT_PACKET || firstByte == UART_CANCEL_PACKET) {

            if (Serial2.available() >= 5) { // Cần đủ 5 bytes

                byte command = Serial2.read(); // Đọc command byte

                byte uid[4];

                Serial2.readBytes(uid, 4);

               

                if (command == UART_COMMIT_PACKET) {

                    Serial.print("UART RECV: COMMIT ");

                    printUID(uid, 4);

                    Serial.println();

                   

                    int cardIndex = getCardIndex(uid, 4);

                    if (cardIndex != -1) {

                        setCardState(cardIndex, 0); // OUT

                        updateSlots(availableSlots + 1);

                       

                        // *** GỌI HÀM LOG TẠI ĐÂY ***

                        logToGoogleSheet(getUIDString(uid, 4), "Exit");

                        // *** KẾT THÚC GỌI LOG ***

                    }

                }

                else if (command == UART_CANCEL_PACKET) {

                    Serial.print("UART RECV: CANCEL ");

                    printUID(uid, 4);

                    Serial.println();

                }

            }

        }

        // Nếu không phải 0xFF/0xEE -> là REQUEST (4 bytes UID)

        else if (Serial2.available() >= 4) {

            processUART_Request();

        }

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



    Serial.print("State transition (Entry): ");

    Serial.print(currentState);

    Serial.print(" -> ");

    Serial.println(newState);



    VehicleState oldState = currentState;

    currentState = newState;



    switch (newState) {

        case STATE_0_IDLE:

            if (oldState == STATE_3_CLEARING) {

                Serial.println("✓ Vehicle entered successfully (Entry)");

                lcdMessage("Da Vao. Scan card...");

            } else if (oldState == STATE_1_WAITING) {

                Serial.println("⚠️ Vehicle reversed - Entry cancelled");

                lcdMessage("Da Huy. Scan card...");

                if (cardAuthorized) {

                    // Xe đã quẹt thẻ nhưng lại lùi ra

                    // HỦY BỎ GIAO DỊCH

                    Serial.println("CANCEL TRANSACTION!");

                   

                    // Cần lấy lại UID đã quẹt

                    // Hạn chế: hàm getCardIndex dùng mfrc522.uid...

                    // Tạm thời chấp nhận rủi ro nếu thẻ bị lấy ra nhanh

                    int cardIndex = getCardIndex(mfrc522.uid.uidByte, mfrc522.uid.size);

                    if (cardIndex != -1) {

                         setCardState(cardIndex, 0); // Trả về 0 = OUT

                         updateSlots(availableSlots + 1); // Trả lại slot

                    }

                }

            }

            closeBarrier();

            break;



        case STATE_1_WAITING:

            Serial.println("Vehicle waiting at entry - Please scan card");

            lcdMessage("Moi quet the...");

            break;



        case STATE_2_PASSING:

            Serial.println("Vehicle passing through entry barrier");

            lcdMessage("Dang di qua...");

            break;



        case STATE_3_CLEARING:

            Serial.println("Vehicle clearing the entry gate");

            break;

    }

}



// ===================================

// SETUP & LOOP

// ===================================



void setup() {

    Serial.begin(115200);

    Wire.begin();

   

    Serial2.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);

    Serial.println("\n=== Entry Gate System (MASTER - FIXED) ===");



    setupLCD();



    if (!rtc.begin()) {

        Serial.println("Couldn't find RTC");

        lcdMessage("Loi: Khong tim thay RTC");

        while (1);

    }

    Serial.println("RTC OK");



    SPI.begin(18, 19, 23, SS_PIN);

    mfrc522.PCD_Init();

    Serial.println("RFID OK");



    pinMode(TRIG1, OUTPUT);

    pinMode(ECHO1, INPUT);

    pinMode(TRIG2, OUTPUT);

    pinMode(ECHO2, INPUT);



    barrierServo.attach(SERVO_PIN);

    barrierServo.write(SERVO_CLOSED);

    Serial.println("Servo OK");

   

    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(BUZZER_PIN, LOW);

    beep(100);

    Serial.println("Buzzer OK");



    // Đọc trạng thái từ EEPROM

    byte slotsFromEEPROM = readEEPROM(EEPROM_ADDR, EEPROM_SLOTS_ADDR);

    if (slotsFromEEPROM > TOTAL_SLOTS || slotsFromEEPROM == 0xFF) { // Kiểm tra kỹ hơn

        Serial.println("First boot or EEPROM reset. Setting slots to TOTAL.");

        updateSlots(TOTAL_SLOTS);

       

        for(int i=0; i<ALLOWED_COUNT; i++) {

          setCardState(i, 0); // 0 = OUT

        }

    } else {

        availableSlots = slotsFromEEPROM;

        Serial.print("Loaded from EEPROM: "); Serial.print(availableSlots); Serial.println(" slots");

    }



    // --- BẮT ĐẦU CODE KẾT NỐI WIFI ---

    Serial.println("Connecting to WiFi...");

    lcdMessage("Dang ket noi WiFi");

   

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int retries = 0;

    // Chờ tối đa 10 giây (20 * 500ms)

    while (WiFi.status() != WL_CONNECTED && retries < 20) {

      delay(500);

      Serial.print(".");

      retries++;

    }



    if (WiFi.status() == WL_CONNECTED) {

      Serial.println("\nWiFi connected!");

      lcdMessage("WiFi Da Ket Noi");

      delay(1000);

    } else {

      Serial.println("\nWiFi connect failed!");

      lcdMessage("Loi WiFi");

      delay(1000);

    }

    // --- KẾT THÚC CODE WIFI ---



    updateLCD();

    lcdMessage("San sang!");

    Serial.println("System ready - Waiting for vehicles...");

}



void loop() {

    // 1. Đọc cảm biến (Cổng Vào)

    long dist1 = getDistance(TRIG1, ECHO1);

    long dist2 = getDistance(TRIG2, ECHO2);

    bool s1 = (dist1 < VEHICLE_THRESHOLD);

    bool s2 = (dist2 < VEHICLE_THRESHOLD);



    // 2. Cập nhật State Machine (Cổng Vào)

    VehicleState newState = determineState(s1, s2);

    handleStateTransition(newState);



    // 3. Xử lý RFID (Cổng Vào)

    if (currentState == STATE_1_WAITING) {

        handleRFID_Entry();

    }



    // 4. Lắng nghe yêu cầu từ Cổng Ra (MỚI)

    handleUART_Exit();



    // 5. Xử lý Timeout (Cổng vào)

    if (barrierOpen && (millis() - barrierOpenTime > BARRIER_TIMEOUT)) {

        if (currentState == STATE_0_IDLE || currentState == STATE_1_WAITING) {

            Serial.println("⏱️ Barrier timeout (Entry) - Closing for safety");

           

            if (cardAuthorized) {

                 Serial.println("TIMEOUT - CANCEL TRANSACTION!");

                 int cardIndex = getCardIndex(mfrc522.uid.uidByte, mfrc522.uid.size);

                 if (cardIndex != -1) {

                     setCardState(cardIndex, 0); // Trả về 0 = OUT

                     updateSlots(availableSlots + 1); // Trả lại slot

                 }

                 cardAuthorized = false;

            }

            closeBarrier(); // Đóng barrier sau khi đã xử lý

        }

    }

   

    delay(100);

}