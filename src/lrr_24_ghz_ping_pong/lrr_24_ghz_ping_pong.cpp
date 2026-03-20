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
int transmissionState = RADIOLIB_ERR_NONE;

// Флаг: true = ждём завершения TX, false = в режиме RX
bool waitingForTxDone = false;

uint32_t txCount = 0;
uint32_t rxCount = 0;
uint32_t errorCount = 0;

float lastRSSI = 0;
float lastSNR = 0;
String lastMessage = "";

unsigned long lastActionTime = 0;
const unsigned long TIMEOUT_INTERVAL = 5000; // Таймаут ожидания ответа

// Прототипы функций
void initDisplay();
void initRadio();
void updateDisplay();
void setFlag();
void transmitPacket(const char* data);
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
    Serial.printf("  Mode: %s\n", DEVICE_NAME);
    Serial.println(F("========================================"));
    
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_OFF);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    
    initDisplay();
    initRadio();
    
    if (IS_MASTER) {
        Serial.println(F("[MASTER] Starting..."));
        // Отправляем первый PING
        transmitPacket("PING");
        txCount++;
        lastActionTime = millis();
    } else {
        Serial.println(F("[SLAVE] Waiting for PING..."));
        radio.startReceive();
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
    Serial.println(F("[RADIO] Init SPI..."));
    radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
    
    Serial.println(F("[RADIO] Init LR1121..."));
    
    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] LR1121 init failed: %d\n", state);
        display.clearBuffer();
        display.setCursor(0, 30);
        display.printf("RADIO ERROR: %d", state);
        display.sendBuffer();
        while (true) {
            digitalWrite(BOARD_LED, !digitalRead(BOARD_LED));
            delay(200);
        }
    }
    Serial.println(F("[RADIO] LR1121 OK!"));
    
    Serial.printf("[RADIO] Freq: %.1f MHz\n", CONFIG_RADIO_FREQ);
    radio.setFrequency(CONFIG_RADIO_FREQ);
    
    Serial.printf("[RADIO] BW: %.1f kHz\n", CONFIG_RADIO_BW);
    radio.setBandwidth(CONFIG_RADIO_BW);
    
    Serial.printf("[RADIO] SF: %d\n", CONFIG_RADIO_SF);
    radio.setSpreadingFactor(CONFIG_RADIO_SF);
    
    Serial.printf("[RADIO] CR: 4/%d\n", CONFIG_RADIO_CR);
    radio.setCodingRate(CONFIG_RADIO_CR);
    
    Serial.printf("[RADIO] TX Power: %d dBm\n", CONFIG_RADIO_OUTPUT_POWER);
    radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
    
    radio.setSyncWord(CONFIG_RADIO_SYNC_WORD);
    radio.setPreambleLength(CONFIG_RADIO_PREAMBLE);
    radio.setCRC(2);
    radio.setTCXO(3.0);
    
    radio.setPacketReceivedAction(setFlag);
    radio.setPacketSentAction(setFlag);
    
    Serial.println(F("[RADIO] Setup complete!"));
}

void updateDisplay() {
    display.clearBuffer();
    
    display.setFont(u8g2_font_6x10_tf);
    display.setCursor(0, 10);
    display.printf("LR1121 %s", DEVICE_NAME);
    display.drawHLine(0, 12, 128);
    
    display.setCursor(0, 24);
    display.printf("F:%.0fMHz BW:%.0fk", CONFIG_RADIO_FREQ, CONFIG_RADIO_BW);
    
    display.setCursor(0, 36);
    display.printf("TX:%lu RX:%lu E:%lu", txCount, rxCount, errorCount);
    
    display.setCursor(0, 48);
    if (lastMessage.length() > 0) {
        display.printf("Last: %s", lastMessage.c_str());
    } else {
        display.print(F("Last: ---"));
    }
    
    display.setCursor(0, 60);
    if (rxCount > 0) {
        display.printf("RSSI:%.0f SNR:%.1f", lastRSSI, lastSNR);
    } else {
        display.print(F("Waiting..."));
    }
    
    display.sendBuffer();
}

void transmitPacket(const char* data) {
    Serial.printf("[TX] Sending: %s\n", data);
    
    digitalWrite(BOARD_LED, LED_ON);
    waitingForTxDone = true;
    
    transmissionState = radio.startTransmit(data);
    if (transmissionState != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] startTransmit failed: %d\n", transmissionState);
        errorCount++;
        digitalWrite(BOARD_LED, LED_OFF);
        waitingForTxDone = false;
        radio.startReceive();
    }
}

void handleMaster() {
    if (operationDone) {
        operationDone = false;
        digitalWrite(BOARD_LED, LED_OFF);
        
        if (waitingForTxDone) {
            // TX завершён — переключаемся на приём
            Serial.println(F("[MASTER] PING sent, waiting for PONG..."));
            waitingForTxDone = false;
            radio.startReceive();
            lastActionTime = millis();
        } else {
            // Приняли пакет
            String str;
            int state = radio.readData(str);
            
            if (state == RADIOLIB_ERR_NONE && str.length() > 0) {
                rxCount++;
                lastRSSI = radio.getRSSI();
                lastSNR = radio.getSNR();
                lastMessage = str;
                
                Serial.printf("[RX] Received: '%s' (RSSI: %.1f dBm, SNR: %.1f dB)\n",
                              str.c_str(), lastRSSI, lastSNR);
                
                if (str == "PONG") {
                    Serial.println(F("[MASTER] Got PONG! Sending next PING..."));
                    // Сразу отправляем следующий PING
                    transmitPacket("PING");
                    txCount++;
                    lastActionTime = millis();
                } else {
                    // Не PONG — продолжаем слушать
                    radio.startReceive();
                }
            } else {
                // Ошибка или пустой пакет
                if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                    Serial.println(F("[ERROR] CRC mismatch!"));
                    errorCount++;
                }
                radio.startReceive();
            }
        }
        
        updateDisplay();
    }
    
    // Таймаут — если долго нет PONG, повторяем PING
    if (!waitingForTxDone && (millis() - lastActionTime > TIMEOUT_INTERVAL)) {
        Serial.println(F("[MASTER] Timeout! Retrying PING..."));
        transmitPacket("PING");
        txCount++;
        lastActionTime = millis();
    }
}

void handleSlave() {
    if (operationDone) {
        operationDone = false;
        digitalWrite(BOARD_LED, LED_OFF);
        
        if (waitingForTxDone) {
            // TX завершён — переключаемся на приём
            Serial.println(F("[SLAVE] PONG sent, waiting for PING..."));
            waitingForTxDone = false;
            radio.startReceive();
        } else {
            // Приняли пакет
            String str;
            int state = radio.readData(str);
            
            if (state == RADIOLIB_ERR_NONE && str.length() > 0) {
                rxCount++;
                lastRSSI = radio.getRSSI();
                lastSNR = radio.getSNR();
                lastMessage = str;
                
                Serial.printf("[RX] Received: '%s' (RSSI: %.1f dBm, SNR: %.1f dB)\n",
                              str.c_str(), lastRSSI, lastSNR);
                
                if (str == "PING") {
                    Serial.println(F("[SLAVE] Got PING! Sending PONG..."));
                    // Отправляем PONG
                    transmitPacket("PONG");
                    txCount++;
                } else {
                    // Не PING — продолжаем слушать
                    radio.startReceive();
                }
            } else {
                // Ошибка или пустой пакет
                if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                    Serial.println(F("[ERROR] CRC mismatch!"));
                    errorCount++;
                }
                radio.startReceive();
            }
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
    
    // Кнопка для вывода статистики
    static unsigned long lastButtonCheck = 0;
    if (digitalRead(BUTTON_PIN) == LOW && millis() - lastButtonCheck > 500) {
        lastButtonCheck = millis();
        Serial.println(F("\n=== Statistics ==="));
        Serial.printf("TX: %lu, RX: %lu, Errors: %lu\n", txCount, rxCount, errorCount);
        Serial.printf("RSSI: %.1f dBm, SNR: %.1f dB\n", lastRSSI, lastSNR);
        Serial.printf("Freq: %.1f MHz, BW: %.1f kHz, SF: %d\n",
                      CONFIG_RADIO_FREQ, CONFIG_RADIO_BW, CONFIG_RADIO_SF);
        Serial.println(F("==================\n"));
    }
}
