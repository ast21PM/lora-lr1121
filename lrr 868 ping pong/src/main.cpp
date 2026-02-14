
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include "pins_config.h"

// Определения для управления светодиодом
#define LED_ON                      HIGH
#define LED_OFF                     LOW

// Определение роли устройства (MASTER или SLAVE) на основе флагов из platformio.ini
#ifdef ROLE_MASTER
#define IS_MASTER true
#define DEVICE_NAME "MASTER"
#else
#define IS_MASTER false
#define DEVICE_NAME "SLAVE"
#endif

// Инициализация дисплея
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// Инициализация SPI для радиомодуля
SPIClass radioSPI(HSPI);

// Создание экземпляра класса модуля LR1121
LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, radioSPI);

// Глобальные переменные состояния
volatile bool operationDone = false; // Флаг завершения операции (TX или RX)
int transmissionState = RADIOLIB_ERR_NONE; // Статус последней операции

// Флаг: true = ожидаем завершения передачи (TX), false = в режиме приема (RX)
bool waitingForTxDone = false;

// Статистика
uint32_t txCount = 0;
uint32_t rxCount = 0;
uint32_t errorCount = 0;
long rtt = 0; // Round-trip time

// Данные последнего принятого пакета
float lastRSSI = 0;
float lastSNR = 0;
String lastMessage = "";

// Таймер для отслеживания таймаутов
unsigned long lastActionTime = 0;
const unsigned long TIMEOUT_INTERVAL = 5000; // Таймаут ожидания ответа 5 секунд

// Прототипы функций
void initDisplay();
void initRadio();
void updateDisplay();
void setFlag();
void transmitPacket(const char* data);
void handleMaster();
void handleSlave();
void startReceive();

// Функция обратного вызова для прерывания от LR1121.
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag() {
    operationDone = true;
}

/**
 * @brief Переводит радиомодуль в режим приема.
 */
void startReceive() {
    int state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] startReceive failed: %d\n", state);
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  LilyGO T3-S3 LR1121 Ping-Pong 868MHz"));
    Serial.printf("  Mode: %s\n", DEVICE_NAME);
    Serial.println(F("========================================"));
    
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_OFF);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    
    initDisplay();
    initRadio();
    
    if (IS_MASTER) {
        Serial.println(F("[MASTER] Starting... First PING will be sent shortly."));
        delay(1000); // Небольшая задержка перед первой отправкой
        transmitPacket("PING");
        txCount++;
    } else {
        Serial.println(F("[SLAVE] Waiting for PING..."));
        startReceive();
    }
    
    updateDisplay();
}

void initDisplay() {
    display.begin();
    display.enableUTF8Print();
    display.setFont(u8g2_font_6x10_tf);
    display.clearBuffer();
    
    display.setCursor(0, 12);
    display.print(F("T3-S3 LR1121 868MHz"));
    display.setCursor(0, 24);
    display.printf("Mode: %s", DEVICE_NAME);
    display.setCursor(0, 36);
    display.print(F("Initializing..."));
    
    display.sendBuffer();
    delay(500);
}

void initRadio() {
    // 1. Инициализация шины SPI
    Serial.println(F("[RADIO] Init SPI..."));
    radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
    
    Serial.println(F("[RADIO] Init LR1121..."));
    
    // 2. Базовая инициализация чипа LR1121
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

    // 3. Настройка RF-переключателя для 868 МГц (SMA)
    Serial.println(F("[RF_SWITCH] Configuring for 868 MHz (SMA)..."));
    pinMode(ANT_SW_VDD, OUTPUT);
    pinMode(ANT_SW_RX, OUTPUT);
    pinMode(ANT_SW_TX, OUTPUT);
    
    digitalWrite(ANT_SW_VDD, HIGH); // Подаем питание на свитч
    delay(10);
    
    // Для 868 MHz: LOW/LOW = SMA connector
    digitalWrite(ANT_SW_RX, LOW);
    digitalWrite(ANT_SW_TX, LOW);
    Serial.println(F("[RF_SWITCH] SMA antenna active (868MHz)"));
    
    // 4. Применение параметров LoRa из platformio.ini
    Serial.printf("[RADIO] Freq: %.1f MHz\n", CONFIG_RADIO_FREQ);
    state = radio.setFrequency(CONFIG_RADIO_FREQ);

    Serial.printf("[RADIO] BW: %.1f kHz\n", CONFIG_RADIO_BW);
    state += radio.setBandwidth(CONFIG_RADIO_BW);
    
    Serial.printf("[RADIO] SF: %d\n", CONFIG_RADIO_SF);
    state += radio.setSpreadingFactor(CONFIG_RADIO_SF);
    
    Serial.printf("[RADIO] CR: 4/%d\n", CONFIG_RADIO_CR);
    state += radio.setCodingRate(CONFIG_RADIO_CR);
    
    Serial.printf("[RADIO] Sync Word: 0x%02X\n", CONFIG_RADIO_SYNC_WORD);
    state += radio.setSyncWord(CONFIG_RADIO_SYNC_WORD);

    Serial.printf("[RADIO] Preamble: %d\n", CONFIG_RADIO_PREAMBLE);
    state += radio.setPreambleLength(CONFIG_RADIO_PREAMBLE);

    Serial.printf("[RADIO] TX Power: %d dBm\n", CONFIG_RADIO_OUTPUT_POWER);
    state += radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
    
    // ДОБАВЛЕНО: Явная настройка критически важных параметров
    Serial.println(F("[RADIO] Setting TCXO voltage to 3.3V..."));
    state += radio.setTCXO(3.3); // Установка напряжения для TCXO, согласно схеме платы
    
    Serial.println(F("[RADIO] Enabling CRC check..."));
    state += radio.setCRC(true); // Явное включение проверки CRC

    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("[ERROR] One of radio settings failed! Code: %d\n", state);
      while(true);
    }
    
    // 5. Установка обработчиков прерываний
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
    display.printf("F:%.0fMHz BW:%.0fk SF%d", CONFIG_RADIO_FREQ, CONFIG_RADIO_BW, CONFIG_RADIO_SF);
    
    display.setCursor(0, 36);
    display.printf("TX:%lu RX:%lu E:%lu", txCount, rxCount, errorCount);
    
    display.setCursor(0, 48);
    if (lastMessage.length() > 0) {
        display.printf("Last: %s RTT:%ldms", lastMessage.c_str(), rtt);
    } else {
        display.print(F("Last: ---"));
    }
    
    display.setCursor(0, 60);
    if (rxCount > 0 || lastRSSI != 0) {
        display.printf("RSSI:%.0f SNR:%.1f", lastRSSI, lastSNR);
    } else {
        display.print(F("Waiting for signal..."));
    }
    
    display.sendBuffer();
}

void transmitPacket(const char* data) {
    Serial.printf("[TX] Sending: %s\n", data);
    
    digitalWrite(BOARD_LED, LED_ON);
    waitingForTxDone = true;
    
    transmissionState = radio.startTransmit(data);
    lastActionTime = millis();

    if (transmissionState != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] startTransmit failed: %d\n", transmissionState);
        errorCount++;
        digitalWrite(BOARD_LED, LED_OFF);
        waitingForTxDone = false;
        startReceive();
    }
}

void handleMaster() {
    if (operationDone) {
        operationDone = false;
        digitalWrite(BOARD_LED, LED_OFF);
        
        if (waitingForTxDone) {
            Serial.println(F("[MASTER] PING sent, waiting for PONG..."));
            waitingForTxDone = false;
            startReceive();
        } else {
            String str;
            int state = radio.readData(str);
            
            if (state == RADIOLIB_ERR_NONE) {
                rtt = millis() - lastActionTime;
                rxCount++;
                lastRSSI = radio.getRSSI();
                lastSNR = radio.getSNR();
                lastMessage = str;
                
                Serial.printf("[RX] Received: '%s' (RSSI: %.1f dBm, SNR: %.1f dB, RTT: %ld ms)\n",
                              str.c_str(), lastRSSI, lastSNR, rtt);
                
                if (str == "PONG") {
                    Serial.println(F("[MASTER] Got PONG! Sending next PING after delay..."));
                    delay(1000);
                    transmitPacket("PING");
                    txCount++;
                } else {
                    Serial.printf("[WARN] Received unexpected packet: %s\n", str.c_str());
                    startReceive();
                }
            } else {
                errorCount++;
                if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                    Serial.println(F("[ERROR] CRC mismatch!"));
                } else {
                    Serial.printf("[ERROR] readData failed: %d\n", state);
                }
                startReceive();
            }
        }
        updateDisplay();
    }
    
    if (!waitingForTxDone && (millis() - lastActionTime > TIMEOUT_INTERVAL)) {
        Serial.println(F("[MASTER] Timeout! Retrying PING..."));
        errorCount++;
        rtt = 0;
        transmitPacket("PING");
        txCount++;
        updateDisplay();
    }
}

void handleSlave() {
    if (operationDone) {
        operationDone = false;
        digitalWrite(BOARD_LED, LED_OFF);
        
        if (waitingForTxDone) {
            Serial.println(F("[SLAVE] PONG sent, waiting for PING..."));
            waitingForTxDone = false;
            startReceive();
        } else {
            String str;
            int state = radio.readData(str);
            
            if (state == RADIOLIB_ERR_NONE) {
                rxCount++;
                lastRSSI = radio.getRSSI();
                lastSNR = radio.getSNR();
                lastMessage = str;
                
                Serial.printf("[RX] Received: '%s' (RSSI: %.1f dBm, SNR: %.1f dB)\n",
                              str.c_str(), lastRSSI, lastSNR);
                
                if (str == "PING") {
                    Serial.println(F("[SLAVE] Got PING! Sending PONG..."));
                    transmitPacket("PONG");
                    txCount++;
                } else {
                     Serial.printf("[WARN] Received unexpected packet: %s\n", str.c_str());
                    startReceive();
                }
            } else {
                errorCount++;
                if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                    Serial.println(F("[ERROR] CRC mismatch!"));
                } else {
                    Serial.printf("[ERROR] readData failed: %d\n", state);
                }
                startReceive();
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
    
    static unsigned long lastButtonCheck = 0;
    if (digitalRead(BUTTON_PIN) == LOW && millis() - lastButtonCheck > 500) {
        lastButtonCheck = millis();
        Serial.println(F("\n=== Statistics ==="));
        Serial.printf("TX: %lu, RX: %lu, Errors: %lu\n", txCount, rxCount, errorCount);
        Serial.printf("Last RSSI: %.1f dBm, SNR: %.1f dB\n", lastRSSI, lastSNR);
        Serial.printf("Freq: %.1f MHz, BW: %.1f kHz, SF: %d, CR: 4/%d\n",
                      CONFIG_RADIO_FREQ, CONFIG_RADIO_BW, CONFIG_RADIO_SF, CONFIG_RADIO_CR);
        Serial.println(F("==================\n"));
    }
}
