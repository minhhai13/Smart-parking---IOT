#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include "SPI.h"
#include "MFRC522.h"
#include "ESP32Servo.h"

#define UART_BAUD 115200
#define RXD2 16
#define TXD2 17

#define UART_COMMIT_PACKET 0xFF
#define UART_CANCEL_PACKET 0xEE

#define SS_PIN 5
#define RST_PIN 4
MFRC522 mfrc522(SS_PIN, RST_PIN);

#define SERVO_PIN 13
Servo barrierServo;
const int SERVO_OPEN = 90;
const int SERVO_CLOSED = 0;

#define TRIG1 32
#define ECHO1 33
#define TRIG2 25
#define ECHO2 26

#define BUZZER_PIN 27

LiquidCrystal_I2C lcd(0x27, 16, 2);

RTC_DS3231 rtc;
#define EEPROM_ADDR 0x57
#define EEPROM_SLOTS_ADDR 0
#define EEPROM_STATE_START_ADDR 1
const int TOTAL_SLOTS = 9;

const byte allowedUIDs[][4] = {
    {0x41, 0x7B, 0xF7, 0x16},
    {0xF7, 0x13, 0x73, 0x06},
    {0x0E, 0x77, 0xF9, 0x05},
    {0xFE, 0x91, 0xC9, 0x01}
};
const int ALLOWED_COUNT = 4;
int availableSlots = TOTAL_SLOTS;

const int VEHICLE_THRESHOLD = 10;
const unsigned long BARRIER_TIMEOUT = 15000;

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

void writeEEPROM(int i2c_addr, unsigned int eeaddress, byte data) {
    Wire.beginTransmission(i2c_addr);
    Wire.write((int)(eeaddress >> 8));
    Wire.write((int)(eeaddress & 0xFF));
    Wire.write(data);
    Wire.endTransmission();
    delay(5);
}

byte readEEPROM(int i2c_addr, unsigned int eeaddress) {
    byte rdata = 0xFF;
    Wire.beginTransmission(i2c_addr);
    Wire.write((int)(eeaddress >> 8));
    Wire.write((int)(eeaddress & 0xFF));
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
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
}

void lcdMessage(String msg) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(msg);
}

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
        updateLCD();
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
            return i;
        }
    }
    return -1;
}

void printUID(byte *uid, byte size) {
    for (byte i = 0; i < size; i++) {
        Serial.print(uid[i] < 0x10 ? " 0" : " ");
        Serial.print(uid[i], HEX);
    }
}

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
                
                if (availableSlots == 0) {
                    Serial.println("⛔ ACCESS DENIED - Parking Full");
                    lcdMessage("Loi: Bai Da Day");
                    beep(500);
                } else {
                    int state = getCardState(cardIndex);
                    if (state == 1) {
                        Serial.println("⛔ ACCESS DENIED - Vehicle Already Inside");
                        lcdMessage("Loi: Xe Da O Trong");
                        beep(500);
                    } else {
                        Serial.println("✅ ACCESS GRANTED - Entry allowed");
                        cardAuthorized = true;
                        
                        setCardState(cardIndex, 1);
                        updateSlots(availableSlots - 1);
                        
                        openBarrier();
                    }
                }
            } else {
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

void processUART_Request() {
    byte uidFromExit[4];
    Serial2.readBytes(uidFromExit, 4);

    Serial.print("UART RECV (from Exit): REQUEST packet ");
    printUID(uidFromExit, 4);
    Serial.println();
    
    int cardIndex = getCardIndex(uidFromExit, 4);

    if (cardIndex != -1) {
        int state = getCardState(cardIndex);
        if (state == 1) {
            Serial.println("UART SEND (to Exit): 'O' (OK)");
            Serial2.write('O');
        } else {
            Serial.println("UART SEND (to Exit): 'E' (Error - Not Inside)");
            Serial2.write('E');
        }
    } else {
        Serial.println("UART SEND (to Exit): 'I' (Invalid)");
        Serial2.write('I');
    }
}

void handleUART_Exit() {
    if (Serial2.available() >= 1) {
        byte firstByte = Serial2.peek();
        
        if (firstByte == UART_COMMIT_PACKET || firstByte == UART_CANCEL_PACKET) {
            if (Serial2.available() >= 5) {
                byte command = Serial2.read();
                byte uid[4];
                Serial2.readBytes(uid, 4);
                
                if (command == UART_COMMIT_PACKET) {
                    Serial.print("UART RECV: COMMIT ");
                    printUID(uid, 4);
                    Serial.println();
                    
                    int cardIndex = getCardIndex(uid, 4);
                    if (cardIndex != -1) {
                        setCardState(cardIndex, 0);
                        updateSlots(availableSlots + 1);
                    }
                }
                else if (command == UART_CANCEL_PACKET) {
                    Serial.print("UART RECV: CANCEL ");
                    printUID(uid, 4);
                    Serial.println();
                }
            }
        }
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
    return STATE_0_IDLE;
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
                    Serial.println("CANCEL TRANSACTION!");
                    
                    int cardIndex = getCardIndex(mfrc522.uid.uidByte, mfrc522.uid.size);
                    if (cardIndex != -1) {
                         setCardState(cardIndex, 0);
                         updateSlots(availableSlots + 1);
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

    byte slotsFromEEPROM = readEEPROM(EEPROM_ADDR, EEPROM_SLOTS_ADDR);
    if (slotsFromEEPROM > TOTAL_SLOTS || slotsFromEEPROM < 0) {
        Serial.println("First boot or EEPROM reset. Setting slots to TOTAL.");
        updateSlots(TOTAL_SLOTS);
        
        for(int i=0; i<ALLOWED_COUNT; i++) {
          setCardState(i, 0);
        }
    } else {
        availableSlots = slotsFromEEPROM;
        Serial.print("Loaded from EEPROM: "); Serial.print(availableSlots); Serial.println(" slots");
    }

    updateLCD();
    lcdMessage("San sang!");
    Serial.println("System ready - Waiting for vehicles...");
}

void loop() {
    long dist1 = getDistance(TRIG1, ECHO1);
    long dist2 = getDistance(TRIG2, ECHO2);
    bool s1 = (dist1 < VEHICLE_THRESHOLD);
    bool s2 = (dist2 < VEHICLE_THRESHOLD);

    VehicleState newState = determineState(s1, s2);
    handleStateTransition(newState);

    if (currentState == STATE_1_WAITING) {
        handleRFID_Entry();
    }

    handleUART_Exit();

    if (barrierOpen && (millis() - barrierOpenTime > BARRIER_TIMEOUT)) {
        if (currentState == STATE_0_IDLE || currentState == STATE_1_WAITING) {
            Serial.println("⏱️ Barrier timeout (Entry) - Closing for safety");
            
            if (cardAuthorized) {
                 Serial.println("TIMEOUT - CANCEL TRANSACTION!");
                 int cardIndex = getCardIndex(mfrc522.uid.uidByte, mfrc522.uid.size);
                 if (cardIndex != -1) {
                       setCardState(cardIndex, 0);
                       updateSlots(availableSlots + 1);
                 }
                 cardAuthorized = false;
            }
            closeBarrier();
        }
    }
    
    delay(100);
}
#include "SPI.h"
#include "MFRC522.h"
#include "ESP32Servo.h"
#include <string.h>

#define UART_BAUD 115200
#define RXD2 16
#define TXD2 17

#define UART_COMMIT_PACKET 0xFF
#define UART_CANCEL_PACKET 0xEE

#define SS_PIN 5
#define RST_PIN 4
MFRC522 mfrc522(SS_PIN, RST_PIN);

#define SERVO_PIN 13
Servo barrierServo;
const int SERVO_OPEN = 90;
const int SERVO_CLOSED = 0;

#define TRIG1 32
#define ECHO1 33
#define TRIG2 25
#define ECHO2 26

#define BUZZER_PIN 27

const int VEHICLE_THRESHOLD = 10;
const unsigned long BARRIER_TIMEOUT = 15000;
const unsigned long UART_TIMEOUT = 2000;

enum VehicleState {
    STATE_0_IDLE = 0,
    STATE_1_WAITING = 1,
    STATE_2_PASSING = 2,
    STATE_3_CLEARING = 3
};
VehicleState currentState = STATE_0_IDLE;
bool barrierOpen = false;
unsigned long barrierOpenTime = 0;

bool exitRequestPending = false;
byte lastAuthorizedUID[4] = {0, 0, 0, 0};

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
    }
}

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
            
            while (Serial2.available()) {
                Serial2.read();
            }
            delay(10);
            
            Serial.println("UART SEND (to Master): Requesting permission...");
            Serial2.write(uid, 4);
            Serial2.flush();

            unsigned long waitStart = millis();
            bool received = false;
            while (millis() - waitStart < UART_TIMEOUT) {
                if (Serial2.available()) {
                    char response = Serial2.read();
                    Serial.print("UART RECV (from Master): '");
                    Serial.print(response); Serial.println("'");

                    if (response == 'O') {
                        Serial.println("✅ ACCESS GRANTED (from Master)");
                        exitRequestPending = true;
                        memcpy(lastAuthorizedUID, uid, 4);
                        openBarrier();
                    } else if (response == 'E') {
                        Serial.println("⛔ ACCESS DENIED - Vehicle Not Inside");
                        beep(500);
                    } else if (response == 'I') {
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
    return STATE_0_IDLE;
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
                Serial.println("✓ Vehicle exited successfully (Exit)");
                if (exitRequestPending) {
                    Serial.println("UART SEND (to Master): COMMIT packet (0xFF)");
                    Serial2.write(UART_COMMIT_PACKET);
                    Serial2.write(lastAuthorizedUID, 4);
                    exitRequestPending = false;
                }
            } else if (oldState == STATE_1_WAITING) {
                Serial.println("⚠️ Vehicle reversed - Exit cancelled");
                if (exitRequestPending) {
                    Serial.println("UART SEND (to Master): CANCEL packet (0xEE)");
                    Serial2.write(UART_CANCEL_PACKET);
                    Serial2.write(lastAuthorizedUID, 4);
                    exitRequestPending = false;
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
    long dist1 = getDistance(TRIG1, ECHO1);
    long dist2 = getDistance(TRIG2, ECHO2);
    bool s1 = (dist1 < VEHICLE_THRESHOLD);
    bool s2 = (dist2 < VEHICLE_THRESHOLD);

    VehicleState newState = determineState(s1, s2);
    handleStateTransition( newState);

    if (currentState == STATE_1_WAITING) {
        handleRFID_Exit();
    }

    if (barrierOpen && (millis() - barrierOpenTime > BARRIER_TIMEOUT)) {
        if (currentState == STATE_0_IDLE || currentState == STATE_1_WAITING) {
            Serial.println("⏱️ Barrier timeout (Exit) - Closing for safety");
            
            if (exitRequestPending) {
                Serial.println("UART SEND (to Master): CANCEL packet (0xEE) - Timeout");
                Serial2.write(UART_CANCEL_PACKET);
                Serial2.write(lastAuthorizedUID, 4);
                exitRequestPending = false;
            }
            closeBarrier();
        }
    }

    delay(100);
}