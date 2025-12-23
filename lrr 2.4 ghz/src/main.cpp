

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include "pins_config.h"

#ifdef ROLE_MASTER
#define IS_MASTER true
#define DEVICE_NAME "MASTER"
#else
#define IS_MASTER false
#define DEVICE_NAME "SLAVE"
#endif


U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

SPIClass radioSPI(HSPI);

LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, radioSPI);

volatile bool operationDone = false;
bool transmitFlag = false;
int transmissionState = RADIOLIB_ERR_NONE;

uint32_t txCount = 0;
uint32_t rxCount = 0;
uint32_t errorCount = 0;

float lastRSSI = 0;
float lastSNR = 0;
String lastMessage = "";

unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL = 2000; // мс

void initDisplay();
void initRadio();
void updateDisplay();
void setFlag();
void transmitPacket(const char* data);
void processReceivedPacket();
void handleMaster();
void handleSlave();

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag() {
    operationDone = true;
}


void setup() {
    Serial.begin(115200);
    delay(2000); 
    
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  LilyGO T3-S3 LR1121 Ping-Pong"));
    Serial.printf("  Режим: %s\n", DEVICE_NAME);
    Serial.println(F("========================================"));
    
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_OFF);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    
    initDisplay();
    
    initRadio();
    
    if (IS_MASTER) {
        Serial.println(F("[MASTER] Начинаю передачу PING..."));
        transmitFlag = true;
    } else {
        Serial.println(F("[SLAVE] Жду PING..."));
        int state = radio.startReceive();
        if (state != RADIOLIB_ERR_NONE) {
            Serial.printf("[ERROR] Ошибка запуска приёма: %d\n", state);
        }
    }
    
    updateDisplay();
}

void initDisplay() {
    display.begin();
    display.enableUTF8Print();
    display.setFont(u8g2_font_6x10_tf);
    display.clearBuffer();
    
    display.setCursor(0, 12);
    display.print(F("T3-S3 LR1121"));
    display.setCursor(0, 24);
    display.printf("Mode: %s", DEVICE_NAME);
    display.setCursor(0, 36);
    display.print(F("Initializing..."));
    
    display.sendBuffer();
    delay(500);
}

void initRadio() {
    Serial.println(F("[RADIO] Инициализация SPI..."));
    
    // Инициализация SPI для радио
    radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
    
    Serial.println(F("[RADIO] Инициализация LR1121..."));
    
    // Инициализация LR1121
    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Ошибка инициализации LR1121: %d\n", state);
        display.clearBuffer();
        display.setCursor(0, 30);
        display.printf("RADIO ERROR: %d", state);
        display.sendBuffer();
        while (true) {
            digitalWrite(BOARD_LED, !digitalRead(BOARD_LED));
            delay(200);
        }
    }
    Serial.println(F("[RADIO] LR1121 инициализирован!"));
    
    // Установка частоты
    Serial.printf("[RADIO] Частота: %.1f MHz\n", CONFIG_RADIO_FREQ);
    state = radio.setFrequency(CONFIG_RADIO_FREQ);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Ошибка установки частоты: %d\n", state);
    }
    
    // Установка полосы пропускания
    Serial.printf("[RADIO] Bandwidth: %.1f kHz\n", CONFIG_RADIO_BW);
    state = radio.setBandwidth(CONFIG_RADIO_BW);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Ошибка установки BW: %d\n", state);
    }
    
    // Установка Spreading Factor
    Serial.printf("[RADIO] SF: %d\n", CONFIG_RADIO_SF);
    state = radio.setSpreadingFactor(CONFIG_RADIO_SF);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Ошибка установки SF: %d\n", state);
    }
    
    // Установка Coding Rate
    Serial.printf("[RADIO] CR: 4/%d\n", CONFIG_RADIO_CR);
    state = radio.setCodingRate(CONFIG_RADIO_CR);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Ошибка установки CR: %d\n", state);
    }
    
    // Установка мощности передачи
    Serial.printf("[RADIO] TX Power: %d dBm\n", CONFIG_RADIO_OUTPUT_POWER);
    state = radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Ошибка установки мощности: %d\n", state);
    }
    
    // Установка Sync Word
    state = radio.setSyncWord(CONFIG_RADIO_SYNC_WORD);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Ошибка установки Sync Word: %d\n", state);
    }
    
    // Установка длины преамбулы
    state = radio.setPreambleLength(CONFIG_RADIO_PREAMBLE);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Ошибка установки преамбулы: %d\n", state);
    }
    
    state = radio.setCRC(2); 
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Ошибка установки CRC: %d\n", state);
    }
    
    state = radio.setTCXO(3.0);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[WARN] Ошибка TCXO: %d (может быть нормально)\n", state);
    }
    
    radio.setPacketReceivedAction(setFlag);
    radio.setPacketSentAction(setFlag);
    
    Serial.println(F("[RADIO] Настройка завершена!"));
}


void updateDisplay() {
    display.clearBuffer();
    
    // Заголовок
    display.setFont(u8g2_font_6x10_tf);
    display.setCursor(0, 10);
    display.printf("LR1121 %s", DEVICE_NAME);
    display.drawHLine(0, 12, 128);
    
    // Частота
    display.setCursor(0, 24);
    display.printf("F:%.1fMHz BW:%.0fk", CONFIG_RADIO_FREQ, CONFIG_RADIO_BW);
    
    // Счётчики
    display.setCursor(0, 36);
    display.printf("TX:%lu RX:%lu E:%lu", txCount, rxCount, errorCount);
    
    // Последний пакет
    display.setCursor(0, 48);
    if (lastMessage.length() > 0) {
        display.printf("Last: %s", lastMessage.c_str());
    } else {
        display.print(F("Last: ---"));
    }
    
    // RSSI/SNR
    display.setCursor(0, 60);
    if (rxCount > 0) {
        display.printf("RSSI:%.0f SNR:%.1f", lastRSSI, lastSNR);
    } else {
        display.print(F("Waiting..."));
    }
    
    display.sendBuffer();
}


void transmitPacket(const char* data) {
    Serial.printf("[TX] Отправка: %s\n", data);
    
    digitalWrite(BOARD_LED, LED_ON);
    
    transmissionState = radio.startTransmit(data);
    if (transmissionState != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Ошибка начала передачи: %d\n", transmissionState);
        errorCount++;
        digitalWrite(BOARD_LED, LED_OFF);
        
        radio.startReceive();
    }
}


void processReceivedPacket() {
    String str;
    int state = radio.readData(str);
    
    if (state == RADIOLIB_ERR_NONE) {
        rxCount++;
        lastRSSI = radio.getRSSI();
        lastSNR = radio.getSNR();
        lastMessage = str;
        
        Serial.printf("[RX] Получено: '%s' (RSSI: %.1f dBm, SNR: %.1f dB)\n", 
                      str.c_str(), lastRSSI, lastSNR);
        
        digitalWrite(BOARD_LED, LED_ON);
        delay(50);
        digitalWrite(BOARD_LED, LED_OFF);
        
        if (IS_MASTER) {
            if (str == "PONG") {
                Serial.println(F("[MASTER] Получен PONG, жду перед следующим PING"));
                transmitFlag = true;
                lastTransmitTime = millis();
            }
        } else {
            if (str == "PING") {
                Serial.println(F("[SLAVE] Получен PING, отправляю PONG"));
                delay(100);
                transmitPacket("PONG");
                txCount++;
            }
        }
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println(F("[ERROR] Ошибка CRC!"));
        errorCount++;
    } else {
        Serial.printf("[ERROR] Ошибка чтения данных: %d\n", state);
        errorCount++;
    }
}


void handleMaster() {
    if (operationDone) {
        operationDone = false;
        
        if (transmitFlag) {
            if (transmissionState == RADIOLIB_ERR_NONE) {
                Serial.println(F("[MASTER] PING отправлен, жду PONG..."));
                txCount++;
            }
            transmitFlag = false;
            
            radio.startReceive();
        } else {
            processReceivedPacket();
        }
        
        updateDisplay();
    }
    
    if (transmitFlag && (millis() - lastTransmitTime >= TRANSMIT_INTERVAL)) {
        transmitPacket("PING");
        transmitFlag = false;
    }
    
    static unsigned long waitStartTime = 0;
    if (!transmitFlag) {
        if (waitStartTime == 0) {
            waitStartTime = millis();
        } else if (millis() - waitStartTime > 5000) {
            Serial.println(F("[MASTER] Таймаут, повторяю PING..."));
            waitStartTime = 0;
            transmitPacket("PING");
        }
    } else {
        waitStartTime = 0;
    }
}


void handleSlave() {
    if (operationDone) {
        operationDone = false;
        
        if (transmitFlag) {
            if (transmissionState == RADIOLIB_ERR_NONE) {
                Serial.println(F("[SLAVE] PONG отправлен, жду PING..."));
            }
            transmitFlag = false;
            
            radio.startReceive();
        } else {
            processReceivedPacket();
        }
        
        updateDisplay();
    }
}


void loop() {
    if (IS_MASTER) {
        handleMaster();
    } else {
        handleSlave();
    }
    
    static unsigned long lastButtonCheck = 0;
    if (digitalRead(BUTTON_PIN) == LOW && millis() - lastButtonCheck > 500) {
        lastButtonCheck = millis();
        Serial.println(F("\n=== Статистика ==="));
        Serial.printf("TX: %lu, RX: %lu, Errors: %lu\n", txCount, rxCount, errorCount);
        Serial.printf("RSSI: %.1f dBm, SNR: %.1f dB\n", lastRSSI, lastSNR);
        Serial.printf("Freq: %.1f MHz, BW: %.1f kHz, SF: %d\n", 
                      CONFIG_RADIO_FREQ, CONFIG_RADIO_BW, CONFIG_RADIO_SF);
        Serial.println(F("==================\n"));
    }
}
