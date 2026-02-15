
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include "pins_config.h"

// ==================== Определение роли устройства ====================
#ifdef ROLE_MASTER
#define IS_MASTER true
#define DEVICE_NAME "MASTER"
#else
#define IS_MASTER false
#define DEVICE_NAME "SLAVE"
#endif

// ==================== Глобальные объекты ====================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
SPIClass radioSPI(HSPI);
LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, radioSPI);

// ==================== Состояния и флаги ====================
volatile bool operationDone = false;
bool waitingForTxDone = false;

// Перечисление для текущего диапазона частот
enum RadioBand {
    BAND_2400,
    BAND_868
};
RadioBand currentBand = BAND_2400; // Начинаем с 2.4 ГГц

// ==================== Статистика ====================
uint32_t txCount = 0;
uint32_t rxCount = 0;
uint32_t errorCount = 0;
float lastRSSI = 0;
float lastSNR = 0;
String lastMessage = "";

// ==================== Тайминги ====================
unsigned long lastPacketTime = 0;
const unsigned long MASTER_TX_INTERVAL = 1000; // Мастер шлет пакеты каждую секунду
const unsigned long SLAVE_TIMEOUT = 5000;      // Слейв ждет 5 секунд перед сменой частоты

// ==================== Прототипы функций ====================
void initDisplay();
void initRadio();
void updateDisplay();
void setFlag();
bool switchTo868();
bool switchTo2400();
void transmitPacket(String data);
void handleMaster();
void handleSlave();
void startReceive();

// ==================== Функции ====================

// Функция обратного вызова для прерывания от LR1121
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
        Serial.printf("[ERROR] startReceive failed on current band. Code: %d\n", state);
    }
}

/**
 * @brief Настраивает радио и антенный свитч для работы на 868 МГц.
 * @return true, если настройка прошла успешно, иначе false.
 */
bool switchTo868() {
    Serial.println(F("\n[SWITCH] === Switching to 868 MHz ==="));
    
    // 1. Антенный свитч (SMA)
    digitalWrite(ANT_SW_RX, LOW);
    digitalWrite(ANT_SW_TX, LOW);
    
    // 2. Инициализация (перезагрузка) чипа на 868 МГц
    // Параметры: Freq, BW, SF, CR, SyncWord, Power, Preamble
    // ИСПРАВЛЕНО: Мощность увеличена до 20 dBm для стабильной связи
    int state = radio.begin(868.0, 125.0, 9, 5, 0x12, 20, 8);
    
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Init 868 failed: %d\n", state);
        return false;
    }
    
    // 3. ОБЯЗАТЕЛЬНО восстанавливаем настройки платы после ребута чипа
    radio.setTCXO(3.3);       // Без этого не будет работать!
    radio.setCRC(true);
    radio.setPacketReceivedAction(setFlag);
    radio.setPacketSentAction(setFlag);
    
    currentBand = BAND_868;
    Serial.println(F("[SWITCH] OK (868 MHz)"));
    updateDisplay();
    return true;
}

/**
 * @brief Настраивает радио и антенный свитч для работы на 2.4 ГГц.
 * @return true, если настройка прошла успешно, иначе false.
 */
bool switchTo2400() {
    Serial.println(F("\n[SWITCH] === Switching to 2.4 GHz ==="));

    // 1. Антенный свитч (PCB / WiFi path)
    digitalWrite(ANT_SW_RX, HIGH);
    digitalWrite(ANT_SW_TX, LOW);
    
    // 2. Инициализация (перезагрузка) чипа на 2.4 ГГц
    // RadioLib сам поймет по частоте 2450.0, что нужно включить 2.4G модем
    // BW ставим пошире (406.25 кГц), это стандарт для LoRa 2.4
    int state = radio.begin(2450.0, 406.25, 9, 5, 0x12, 10, 8);
    
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] Init 2.4 failed: %d\n", state);
        return false;
    }

    // 3. ОБЯЗАТЕЛЬНО восстанавливаем настройки платы
    radio.setTCXO(3.3);       // Критично!
    radio.setCRC(true);
    radio.setPacketReceivedAction(setFlag);
    radio.setPacketSentAction(setFlag);

    currentBand = BAND_2400;
    Serial.println(F("[SWITCH] OK (2.4 GHz)"));
    updateDisplay();
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println(F("\n========================================"));
    Serial.println(F("     Dual-Band Switch LR1121"));
    Serial.printf("      Mode: %s\n", DEVICE_NAME);
    Serial.println(F("========================================"));
    
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_OFF);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    
    initDisplay();
    initRadio(); // Инициализируем только пины
    
    // Стартуем на 2.4 ГГц
    if (!switchTo2400()) {
        display.clearBuffer();
        display.setCursor(0, 30);
        display.print("RADIO FATAL ERROR!");
        display.sendBuffer();
        while(true);
    }
    
    if (IS_MASTER) {
        Serial.println(F("[MASTER] Starting..."));
        lastPacketTime = millis();
        transmitPacket("CMD_FLY_2.4");
    } else {
        Serial.println(F("[SLAVE] Waiting for commands..."));
        lastPacketTime = millis();
        startReceive();
    }
}

void initDisplay() {
    display.begin();
    display.enableUTF8Print();
    display.setFont(u8g2_font_6x10_tf);
    display.clearBuffer();
    
    display.setCursor(0, 12);
    display.print(F("Dual-Band LR1121"));
    display.setCursor(0, 24);
    display.printf("Mode: %s", DEVICE_NAME);
    display.setCursor(0, 36);
    display.print(F("Initializing..."));
    
    display.sendBuffer();
}

/**
 * @brief Инициализирует только пины, связанные с радио.
 * Полная инициализация модема происходит в функциях switchTo...
 */
void initRadio() {
    Serial.println(F("[RADIO] Initializing GPIOs..."));
    radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
    
    // Инициализация пинов антенного свитча
    pinMode(ANT_SW_VDD, OUTPUT);
    pinMode(ANT_SW_RX, OUTPUT);
    pinMode(ANT_SW_TX, OUTPUT);
    digitalWrite(ANT_SW_VDD, HIGH); // Подаем питание на свитч
    delay(10);
}

void updateDisplay() {
    display.clearBuffer();
    
    display.setFont(u8g2_font_6x10_tf);
    display.setCursor(0, 10);
    display.printf("LR1121 %s", DEVICE_NAME);
    
    display.setCursor(85, 10);
    display.print(currentBand == BAND_868 ? "868MHz" : "2.4GHz");

    display.drawHLine(0, 12, 128);
    
    display.setCursor(0, 24);
    display.printf("TX:%lu RX:%lu E:%lu", txCount, rxCount, errorCount);
    
    display.setCursor(0, 36);
    if (lastMessage.length() > 0) {
        display.printf("Last: %s", lastMessage.c_str());
    } else {
        display.print(F("Last: ---"));
    }
    
    display.setCursor(0, 48);
    if (rxCount > 0 || txCount > 0) {
        display.printf("RSSI:%.0f SNR:%.1f", lastRSSI, lastSNR);
    } else {
        display.print(F("Waiting..."));
    }
    
    display.sendBuffer();
}

void transmitPacket(String data) {
    Serial.printf("[TX] Sending on %s: %s\n", (currentBand == BAND_868 ? "868MHz" : "2.4GHz"), data.c_str());
    
    digitalWrite(BOARD_LED, LED_ON);
    waitingForTxDone = true;
    
    int state = radio.startTransmit(data);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] startTransmit failed: %d\n", state);
        errorCount++;
        digitalWrite(BOARD_LED, LED_OFF);
        waitingForTxDone = false;
        startReceive();
    } else {
        txCount++;
    }
    updateDisplay();
}

void processReceivedPacket() {
    String str;
    int state = radio.readData(str);
    
    if (state == RADIOLIB_ERR_NONE && str.length() > 0) {
        rxCount++;
        lastRSSI = radio.getRSSI();
        lastSNR = radio.getSNR();
        lastMessage = str;
        lastPacketTime = millis(); // Сбрасываем таймер таймаута ТОЛЬКО при успешном приеме
        
        Serial.printf("[RX] Received on %s: '%s' (RSSI: %.1f, SNR: %.1f)\n",
                      (currentBand == BAND_868 ? "868MHz" : "2.4GHz"),
                      str.c_str(), lastRSSI, lastSNR);
        
        // Логика ответа для Слейва
        if (!IS_MASTER) {
            if (str == "CMD_FLY_2.4") {
                transmitPacket("ACK_2.4");
            } else if (str == "CMD_FLY_868") {
                transmitPacket("ACK_868");
            }
        }

    } else {
        errorCount++;
        if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            Serial.println(F("[ERROR] CRC mismatch!"));
        } else {
            Serial.printf("[ERROR] readData failed: %d\n", state);
        }
    }
    updateDisplay();
}

void handleMaster() {
    // 1. Проверка нажатия кнопки для смены диапазона
    static unsigned long lastButtonPress = 0;
    if (digitalRead(BUTTON_PIN) == LOW && millis() - lastButtonPress > 500) {
        lastButtonPress = millis();
        if (currentBand == BAND_2400) {
            switchTo868();
        } else {
            switchTo2400();
        }
        // Сразу отправляем пакет на новой частоте и сбрасываем таймер
        lastPacketTime = millis();
        transmitPacket(currentBand == BAND_868 ? "CMD_FLY_868" : "CMD_FLY_2.4");
        return; 
    }

    // 2. Обработка завершения операций (TX done или RX received)
    if (operationDone) {
        operationDone = false;
        digitalWrite(BOARD_LED, LED_OFF);
        
        if (waitingForTxDone) { // Передача завершена, ждем ответа
            waitingForTxDone = false;
            startReceive();
        } else { // Приняли пакет (ACK от слейва)
            processReceivedPacket(); 
            // ИСПРАВЛЕНО: ОБЯЗАТЕЛЬНО возвращаемся в режим приема после получения ACK
            startReceive();
        }
    }

    // 3. Отправка пакетов по таймеру
    if (!waitingForTxDone && (millis() - lastPacketTime > MASTER_TX_INTERVAL)) {
        // Сбрасываем таймер прямо перед отправкой для стабильного интервала
        lastPacketTime = millis();
        transmitPacket(currentBand == BAND_868 ? "CMD_FLY_868" : "CMD_FLY_2.4");
    }
}

void handleSlave() {
    // 1. Обработка завершения операций (TX done или RX received)
    if (operationDone) {
        operationDone = false;
        digitalWrite(BOARD_LED, LED_OFF);
        
        if (waitingForTxDone) { // Отправка ACK завершена
            waitingForTxDone = false;
            startReceive(); // Возвращаемся в режим приема
        } else { // Приняли пакет от мастера
            processReceivedPacket();
        }
    }
    
    // 2. Проверка таймаута и поиск мастера на другой частоте
    if (!waitingForTxDone && (millis() - lastPacketTime > SLAVE_TIMEOUT)) {
        Serial.println(F("[SLAVE] Timeout! Searching on other band..."));
        errorCount++; // Считаем таймаут за ошибку
        if (currentBand == BAND_2400) {
            switchTo868();
        } else {
            switchTo2400();
        }
        startReceive(); // Начинаем слушать на новой частоте
        lastPacketTime = millis(); // Сбрасываем таймер, чтобы не переключаться постоянно
        updateDisplay();
    }
}

void loop() {
    if (IS_MASTER) {
        handleMaster();
    } else {
        handleSlave();
    }
}
