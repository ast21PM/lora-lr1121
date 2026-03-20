 #include <Arduino.h>
 #include <SPI.h>
 #include <Wire.h>
 #include <RadioLib.h>
 #include <U8g2lib.h>
 #include "pins_config.h"
 
 // ==================== Определение роли ====================
 #ifdef ROLE_MASTER
     #define IS_RECEIVER true
     #define DEVICE_NAME "RECEIVER"
 #else
     #define IS_RECEIVER false
     #define DEVICE_NAME "SENSOR"
 #endif
 
 // ==================== Типы событий ====================
 #define EVENT_BUTTON_PRESS  "BTN"
 #define EVENT_BUTTON_LONG   "BTN_LONG"
 #define EVENT_HEARTBEAT     "HB"
 
 // ==================== Структура для лога ====================
 #define MAX_LOG_ENTRIES 5
 
 struct LogEntry {
     uint32_t eventNum;
     String eventType;
     String sensorId;
     float rssi;
     float snr;
     unsigned long timestamp;
 };
 
 // ==================== Глобальные переменные ====================
 U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
 SPIClass radioSPI(HSPI);
 LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, radioSPI);
 
 volatile bool operationDone = false;
 bool waitingForTxDone = false;
 
 // Счётчики
 uint32_t txCount = 0;
 uint32_t rxCount = 0;
 uint32_t eventCounter = 0;
 
 // Последние данные
 float lastRSSI = 0;
 float lastSNR = 0;
 
 // Лог событий (для RECEIVER)
 LogEntry eventLog[MAX_LOG_ENTRIES];
 int logIndex = 0;
 
 // Для SENSOR - используем значение из pins_config.h
 // SENSOR_ID определён как макрос в pins_config.h
 unsigned long lastHeartbeat = 0;
 const unsigned long HEARTBEAT_INTERVAL = 30000;  // Heartbeat каждые 30 сек
 
 // Кнопка
 bool lastButtonState = HIGH;
 unsigned long buttonPressTime = 0;
 const unsigned long LONG_PRESS_TIME = 1000;  // 1 сек для длинного нажатия
 bool buttonHandled = false;
 
 // ==================== Прототипы функций ====================
 void initDisplay();
 void initRadio();
 void updateDisplayReceiver();
 void updateDisplaySensor();
 void setFlag();
 void transmitEvent(const char* eventType);
 void handleReceiver();
 void handleSensor();
 void addToLog(const String& eventType, const String& sensorId, float rssi, float snr);
 String formatTime(unsigned long ms);
 
 // ==================== Callback для прерываний ====================
 #if defined(ESP8266) || defined(ESP32)
 ICACHE_RAM_ATTR
 #endif
 void setFlag() {
     operationDone = true;
 }
 
 // ==================== Setup ====================
 void setup() {
     Serial.begin(115200);
     delay(2000);
     
     Serial.println();
     Serial.println(F("========================================"));
     Serial.println(F("  LoRa Sensor Logger System"));
     Serial.printf("  Role: %s\n", DEVICE_NAME);
     Serial.println(F("========================================"));
     
     // Инициализация пинов
     pinMode(BOARD_LED, OUTPUT);
     digitalWrite(BOARD_LED, LED_OFF);
     pinMode(BUTTON_PIN, INPUT_PULLUP);
     
     // I2C для дисплея
     Wire.begin(I2C_SDA, I2C_SCL);
     
     initDisplay();
     initRadio();
     
     if (IS_RECEIVER) {
         // RECEIVER: сразу переходим в режим приёма
         Serial.println(F("[RECEIVER] Waiting for sensor events..."));
         radio.setPacketReceivedAction(setFlag);
         radio.startReceive();
     } else {
         // SENSOR: готов отправлять события
         Serial.println(F("[SENSOR] Ready. Press BOOT button to send event."));
         Serial.println(F("[SENSOR] Long press (1s) for LONG event."));
     }
     
     if (IS_RECEIVER) {
         updateDisplayReceiver();
     } else {
         updateDisplaySensor();
     }
 }
 
 // ==================== Инициализация дисплея ====================
 void initDisplay() {
     display.begin();
     display.enableUTF8Print();
     display.setFont(u8g2_font_6x10_tf);
     display.clearBuffer();
     
     display.setCursor(0, 12);
     display.print(F("LoRa Sensor Logger"));
     display.setCursor(0, 24);
     display.printf("Role: %s", DEVICE_NAME);
     display.setCursor(0, 36);
     display.print(F("Initializing..."));
     
     display.sendBuffer();
     delay(500);
 }
 
 // ==================== Инициализация радио ====================
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
     
     // Настройка параметров
     radio.setFrequency(CONFIG_RADIO_FREQ);
     radio.setBandwidth(CONFIG_RADIO_BW);
     radio.setSpreadingFactor(CONFIG_RADIO_SF);
     radio.setCodingRate(CONFIG_RADIO_CR);
     radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
     radio.setSyncWord(CONFIG_RADIO_SYNC_WORD);
     radio.setPreambleLength(CONFIG_RADIO_PREAMBLE);
     radio.setCRC(2);
     radio.setTCXO(3.0);
     
     Serial.printf("[RADIO] Freq: %.1f MHz, BW: %.1f kHz, SF: %d\n", 
                   CONFIG_RADIO_FREQ, CONFIG_RADIO_BW, CONFIG_RADIO_SF);
     Serial.println(F("[RADIO] Setup complete!"));
 }
 
 // ==================== Обновление дисплея RECEIVER ====================
 void updateDisplayReceiver() {
     display.clearBuffer();
     
     // Заголовок
     display.setFont(u8g2_font_6x10_tf);
     display.setCursor(0, 10);
     display.print(F("EVENT LOG ="));
     
     // Статистика
     display.setCursor(80, 10);
     display.printf("RX:%lu", rxCount);
     
     display.drawHLine(0, 12, 128);
     
     // Показываем последние события (новые сверху)
     int y = 24;
     int count = min(logIndex, MAX_LOG_ENTRIES);  // Сколько записей показать
     
     if (count == 0) {
         display.setCursor(0, 36);
         display.print(F("Waiting for events..."));
     } else {
         // Идём от последнего к первому (новые сверху)
         for (int i = 0; i < count && y <= 60; i++) {
             // logIndex указывает на СЛЕДУЮЩУЮ позицию для записи
             // Последняя запись: (logIndex - 1)
             // Предпоследняя: (logIndex - 2) и т.д.
             int idx = (logIndex - 1 - i + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
             LogEntry& entry = eventLog[idx];
             
             // Пропускаем пустые записи
             if (entry.eventNum == 0) continue;
             
             display.setCursor(0, y);
             // Формат: #1 S01 BTN -35dB
             display.printf("#%lu %s %s %.0fdB", 
                           entry.eventNum,
                           entry.sensorId.c_str(),
                           entry.eventType.c_str(),
                           entry.rssi);
             y += 12;
         }
     }
     
     display.sendBuffer();
 }
 
 // ==================== Обновление дисплея SENSOR ====================
 void updateDisplaySensor() {
     display.clearBuffer();
     
     // Заголовок
     display.setFont(u8g2_font_6x10_tf);
     display.setCursor(0, 10);
     display.printf("SENSOR [%s]", SENSOR_ID);
     display.drawHLine(0, 12, 128);
     
     // Статус
     display.setCursor(0, 26);
     display.printf("TX Count: %lu", txCount);
     
     display.setCursor(0, 40);
     display.print(F("Press BOOT to send"));
     
     display.setCursor(0, 52);
     display.print(F("Hold 1s for LONG"));
     
     // Индикатор готовности
     display.setCursor(0, 64);
     if (waitingForTxDone) {
         display.print(F("Status: SENDING..."));
     } else {
         display.print(F("Status: READY"));
     }
     
     display.sendBuffer();
 }
 
 // ==================== Отправка события ====================
 void transmitEvent(const char* eventType) {
     // Формат пакета: "SENSOR_ID:EVENT_TYPE:EVENT_NUM"
     // Например: "S01:BTN:42"
     char packet[32];
     eventCounter++;
     snprintf(packet, sizeof(packet), "%s:%s:%lu", SENSOR_ID, eventType, eventCounter);
     
     Serial.printf("[TX] Sending event: %s\n", packet);
     
     digitalWrite(BOARD_LED, LED_ON);
     waitingForTxDone = true;
     
     radio.setPacketSentAction(setFlag);
     int state = radio.startTransmit(packet);
     
     if (state != RADIOLIB_ERR_NONE) {
         Serial.printf("[ERROR] startTransmit failed: %d\n", state);
         digitalWrite(BOARD_LED, LED_OFF);
         waitingForTxDone = false;
     } else {
         txCount++;
     }
     
     updateDisplaySensor();
 }
 
 // ==================== Добавление в лог ====================
 void addToLog(const String& eventType, const String& sensorId, float rssi, float snr) {
     int idx = logIndex % MAX_LOG_ENTRIES;
     
     eventLog[idx].eventNum = rxCount;
     eventLog[idx].eventType = eventType;
     eventLog[idx].sensorId = sensorId;
     eventLog[idx].rssi = rssi;
     eventLog[idx].snr = snr;
     eventLog[idx].timestamp = millis();
     
     logIndex++;
 }
 
 // ==================== Обработчик RECEIVER ====================
 void handleReceiver() {
     if (operationDone) {
         operationDone = false;
         digitalWrite(BOARD_LED, LED_ON);
         
         String str;
         int state = radio.readData(str);
         
         if (state == RADIOLIB_ERR_NONE && str.length() > 0) {
             rxCount++;
             lastRSSI = radio.getRSSI();
             lastSNR = radio.getSNR();
             
             Serial.printf("[RX] Received: '%s' (RSSI: %.1f dBm, SNR: %.1f dB)\n",
                           str.c_str(), lastRSSI, lastSNR);
             
             // Парсим пакет: "S01:BTN:42"
             int firstColon = str.indexOf(':');
             int secondColon = str.indexOf(':', firstColon + 1);
             
             if (firstColon > 0 && secondColon > firstColon) {
                 String sensorId = str.substring(0, firstColon);
                 String eventType = str.substring(firstColon + 1, secondColon);
                 String eventNum = str.substring(secondColon + 1);
                 
                 Serial.printf("[EVENT] Sensor: %s, Type: %s, Num: %s\n",
                               sensorId.c_str(), eventType.c_str(), eventNum.c_str());
                 
                 // Добавляем в лог
                 addToLog(eventType, sensorId, lastRSSI, lastSNR);
                 
                 // Мигаем LED в зависимости от типа события
                 if (eventType == EVENT_BUTTON_LONG) {
                     // Длинное мигание для LONG press
                     for (int i = 0; i < 3; i++) {
                         digitalWrite(BOARD_LED, LED_ON);
                         delay(100);
                         digitalWrite(BOARD_LED, LED_OFF);
                         delay(100);
                     }
                 }
             } else {
                 Serial.println(F("[WARN] Invalid packet format"));
             }
             
             updateDisplayReceiver();
         } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
             Serial.println(F("[ERROR] CRC mismatch!"));
         }
         
         digitalWrite(BOARD_LED, LED_OFF);
         
         // Возвращаемся в режим приёма
         radio.setPacketReceivedAction(setFlag);
         radio.startReceive();
     }
 }
 
 // ==================== Обработчик SENSOR ====================
 void handleSensor() {
     // Обработка завершения передачи
     if (operationDone && waitingForTxDone) {
         operationDone = false;
         waitingForTxDone = false;
         digitalWrite(BOARD_LED, LED_OFF);
         
         Serial.println(F("[TX] Event sent successfully!"));
         updateDisplaySensor();
     }
     
     // Чтение состояния кнопки
     bool currentButtonState = digitalRead(BUTTON_PIN);
     
     // Кнопка нажата (LOW)
     if (currentButtonState == LOW && lastButtonState == HIGH) {
         // Начало нажатия
         buttonPressTime = millis();
         buttonHandled = false;
     }
     
     // Кнопка удерживается
     if (currentButtonState == LOW && !buttonHandled) {
         unsigned long pressDuration = millis() - buttonPressTime;
         
         // Проверяем длинное нажатие
         if (pressDuration >= LONG_PRESS_TIME) {
             Serial.println(F("[BUTTON] Long press detected!"));
             transmitEvent(EVENT_BUTTON_LONG);
             buttonHandled = true;
         }
     }
     
     // Кнопка отпущена (HIGH)
     if (currentButtonState == HIGH && lastButtonState == LOW) {
         // Если ещё не обработано — это короткое нажатие
         if (!buttonHandled && !waitingForTxDone) {
             unsigned long pressDuration = millis() - buttonPressTime;
             
             if (pressDuration < LONG_PRESS_TIME && pressDuration > 50) {  // debounce
                 Serial.println(F("[BUTTON] Short press detected!"));
                 transmitEvent(EVENT_BUTTON_PRESS);
             }
         }
         buttonHandled = false;
     }
     
     lastButtonState = currentButtonState;
     
     // Heartbeat (опционально)
     #ifdef ENABLE_HEARTBEAT
     if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL && !waitingForTxDone) {
         Serial.println(F("[HEARTBEAT] Sending..."));
         transmitEvent(EVENT_HEARTBEAT);
         lastHeartbeat = millis();
     }
     #endif
 }
 
 // ==================== Main Loop ====================
 void loop() {
     if (IS_RECEIVER) {
         handleReceiver();
     } else {
         handleSensor();
     }
 }
 