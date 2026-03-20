
/**
 * @file main.cpp
 * @author AI Expert
 * @brief Прошивка для LilyGo T3-S3 LR1121 для надежной передачи бинарных файлов по LoRa 868 MHz.
 * @version 1.1 (Bugfix)
 * @date 2024-08-02
 *
 * @details
 * Версия 1.1: Исправлена критическая ошибка переполнения буфера (stack smashing) в Master,
 * повышена надежность логики приема в Slave.
 * Этот код объединяет стабильный 868 MHz радиоканал с протоколом надежной пакетной
 * передачи файла, включающим фрагментацию, сборку и проверку целостности по CRC16.
 */

 #include <Arduino.h>
 #include <SPI.h>
 #include <Wire.h>
 #include <RadioLib.h>
 #include <U8g2lib.h>
 #include "pins_config.h"
 #include "file_data.h"
 
 // ==================== ПАРАМЕТРЫ ПРОТОКОЛА ====================
 #define MAX_PAYLOAD_SIZE 250
 #define PACKET_HEADER_SIZE 2
 #define FIRST_PACKET_HEADER_SIZE 4
 
 // ==================== РОЛЬ УСТРОЙСТВА ====================
 #ifdef ROLE_MASTER
   #define IS_MASTER true
   #define DEVICE_NAME "MASTER"
 #else
   #define IS_MASTER false
   #define DEVICE_NAME "SLAVE"
 #endif
 
 // ==================== ОБЪЕКТЫ ====================
 U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
 SPIClass radioSPI(HSPI);
 Module radioModule(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, radioSPI);
 LR1121 radio(&radioModule);
 
 // ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
 volatile bool packetReceivedFlag = false;
 
 #if !IS_MASTER
   #define FILE_BUFFER_SIZE 4096 
   uint8_t receivedFileBuffer[FILE_BUFFER_SIZE];
   size_t receivedBytesCount = 0;
   uint16_t expectedCRC = 0;
 #endif
 
 // ==================== ISR & АНТЕННА ====================
 #if defined(ESP8266) || defined(ESP32)
 ICACHE_RAM_ATTR
 #endif
 void onPacketReceived() {
   packetReceivedFlag = true;
 }
 
 void configureAntenna(bool isTransmit) {
   digitalWrite(ANT_SW_TX, isTransmit);
   digitalWrite(ANT_SW_RX, !isTransmit);
 }
 
 // ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================
 uint16_t calculateCRC16(const uint8_t *data, size_t length) {
   uint16_t crc = 0xFFFF;
   for (size_t i = 0; i < length; i++) {
     crc ^= (uint16_t)data[i] << 8;
     for (uint8_t j = 0; j < 8; j++) {
       crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
     }
   }
   return crc;
 }
 
 String formatHEX(uint16_t value) {
     char buffer[7];
     sprintf(buffer, "0x%04X", value);
     return String(buffer);
 }
 
 // ==================== ФУНКЦИИ ДИСПЛЕЯ (U8g2) ====================
 void displayStatus(const String& line1, const String& line2 = "", const String& line3 = "", const String& line4 = "") {
   display.clearBuffer();
   display.setFont(u8g2_font_6x10_tf);
   display.setCursor(0, 10);
   display.print(line1);
   if (line2.length() > 0) { display.setCursor(0, 24); display.print(line2); }
   if (line3.length() > 0) { display.setCursor(0, 38); display.print(line3); }
   if (line4.length() > 0) { display.setCursor(0, 52); display.print(line4); }
   display.sendBuffer();
 }
 
 void displayPacketInfo(bool isSending, int current, int total, float rssi = 0.0) {
     String role = isSending ? "Sending file:" : "Receiving file:";
     String progress = String(current) + "/" + String(total);
     String rssiStr = isSending ? "" : "RSSI: " + String(rssi, 1) + " dBm";
     displayStatus(role, progress, rssiStr);
 }
 
 // ==================== ИНИЦИАЛИЗАЦИЯ ====================
 void initDisplay() {
   Wire.begin(I2C_SDA, I2C_SCL);
   display.begin();
   display.enableUTF8Print();
   Serial.println(F("[DISPLAY] Инициализация OK"));
   displayStatus("LR1121 File Transfer", "Initializing...");
 }
 
 void initRadio() {
   radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
   
   Serial.print(F("[RADIO] Инициализация... "));
   int state = radio.begin();
   if (state != RADIOLIB_ERR_NONE) {
     Serial.printf("ОШИБКА %d\n", state);
     displayStatus("RADIO INIT FAILED!", "Error code: " + String(state));
     while (true) { digitalWrite(BOARD_LED, !digitalRead(BOARD_LED)); delay(100); }
   }
   Serial.println(F("OK"));
 
   pinMode(ANT_SW_VDD, OUTPUT);
   pinMode(ANT_SW_RX, OUTPUT);
   pinMode(ANT_SW_TX, OUTPUT);
   digitalWrite(ANT_SW_VDD, HIGH);
   delay(10);
   configureAntenna(false);
 
   Serial.print(F("[RADIO] Установка напряжения TCXO... "));
   state = radio.setTCXO(3.3);
   if (state != RADIOLIB_ERR_NONE) {
     Serial.printf("ОШИБКА %d\n", state);
     displayStatus("TCXO CONFIG FAILED!", "Error code: " + String(state));
     while (true) { digitalWrite(BOARD_LED, !digitalRead(BOARD_LED)); delay(100); }
   }
   Serial.println(F("OK"));
   
   Serial.println(F("[RADIO] Установка параметров..."));
   radio.setFrequency(CONFIG_RADIO_FREQ);
   radio.setBandwidth(CONFIG_RADIO_BW);
   radio.setSpreadingFactor(CONFIG_RADIO_SF);
   radio.setCodingRate(CONFIG_RADIO_CR);
   radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
   radio.setSyncWord(CONFIG_RADIO_SYNC_WORD);
   radio.setPreambleLength(CONFIG_RADIO_PREAMBLE);
   radio.setCRC(true);
   
   Serial.println(F("[RADIO] Конфигурация завершена."));
 
   #if !IS_MASTER
     radio.setPacketReceivedAction(onPacketReceived);
   #endif
 }
 
 // ==================== MASTER: ОТПРАВКА ФАЙЛА ====================
 #if IS_MASTER
 void sendFile(const uint8_t* data, size_t dataLen) {
   if (dataLen == 0) return;
 
   uint16_t originalCRC = calculateCRC16(data, dataLen);
   uint8_t totalPackets = (dataLen + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;
   
   Serial.printf("\n[TX] Отправка файла: %u байт, %u пакетов...\n", dataLen, totalPackets);
   Serial.printf("[TX] CRC16 оригинала: %s\n", formatHEX(originalCRC).c_str());
 
   uint8_t packetBuffer[MAX_PAYLOAD_SIZE + FIRST_PACKET_HEADER_SIZE];
 
   for (uint8_t i = 0; i < totalPackets; i++) {
     size_t offset = i * MAX_PAYLOAD_SIZE;
     
     // !!! ИСПРАВЛЕНИЕ КРИТИЧЕСКОЙ ОШИБКИ !!!
     // Корректное вычисление размера полезной нагрузки для текущего пакета.
     size_t payloadLen = (dataLen - offset) > MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : (dataLen - offset);
     
     packetBuffer[0] = i;
     packetBuffer[1] = totalPackets;
     size_t packetSize;
 
     if (i == 0) {
       packetBuffer[2] = (originalCRC >> 8) & 0xFF;
       packetBuffer[3] = originalCRC & 0xFF;
       memcpy(packetBuffer + FIRST_PACKET_HEADER_SIZE, data + offset, payloadLen);
       packetSize = payloadLen + FIRST_PACKET_HEADER_SIZE;
     } else {
       memcpy(packetBuffer + PACKET_HEADER_SIZE, data + offset, payloadLen);
       packetSize = payloadLen + PACKET_HEADER_SIZE;
     }
     
     Serial.printf("\n[TX] Отправка пакета %d/%d (Размер: %u байт)... ", i + 1, totalPackets, packetSize);
     displayPacketInfo(true, i + 1, totalPackets);
 
     radio.standby();
     configureAntenna(true);
     delay(5);
     
     digitalWrite(BOARD_LED, HIGH);
     int state = radio.transmit(packetBuffer, packetSize);
     digitalWrite(BOARD_LED, LOW);
     
     if (state == RADIOLIB_ERR_NONE) Serial.println(F("УСПЕХ!"));
     else Serial.printf("ОШИБКА %d\n", state);
 
     delay(200 + random(0, 51));
   }
   Serial.println(F("\n[TX] Отправка завершена. Возврат в режим ожидания."));
   displayStatus(DEVICE_NAME, "Press BOOT to send");
   configureAntenna(false);
   radio.standby();
 }
 #endif
 
 // ==================== SLAVE: ПРИЕМ ФАЙЛА ====================
 #if !IS_MASTER
 void handleReception() {
   packetReceivedFlag = false;
   digitalWrite(BOARD_LED, HIGH);
   Serial.println(F("\n[RX] Прерывание сработало! Проверка..."));
 
   size_t packetLen = radio.getPacketLength();
   float rssi = radio.getRSSI();
   float snr = radio.getSNR();
   
   Serial.printf("[DIAG] Статистика: RSSI=%.2f dBm, SNR=%.2f dB, Len=%u\n", rssi, snr, packetLen);
 
   if (packetLen == 0) {
      Serial.println(F("[RX] ⚠️ ОШИБКА: Длина пакета 0!"));
      digitalWrite(BOARD_LED, LOW);
      radio.startReceive();
      return;
   }
 
   uint8_t packetBuffer[MAX_PAYLOAD_SIZE + FIRST_PACKET_HEADER_SIZE];
   int state = radio.readData(packetBuffer, packetLen);
 
   if (state == RADIOLIB_ERR_NONE) {
     Serial.printf("[RX] ✓ Чтение успешно! Длина: %u байт\n", packetLen);
     
     if (packetLen < PACKET_HEADER_SIZE) {
         Serial.printf("[RX] ⚠️ ОШИБКА: Пакет слишком короткий!\n");
     } else {
       uint8_t currentPacket = packetBuffer[0];
       uint8_t totalPackets = packetBuffer[1];
       displayPacketInfo(false, currentPacket + 1, totalPackets, rssi);
       Serial.printf("[RX] Принят пакет %d/%d\n", currentPacket + 1, totalPackets);
       
       uint8_t* payloadPtr;
       size_t payloadLen;
 
       if (currentPacket == 0) {
         // ПЕРВЫЙ ПАКЕТ: сбрасываем состояние и сохраняем CRC
         receivedBytesCount = 0; 
         if (packetLen >= FIRST_PACKET_HEADER_SIZE) {
             expectedCRC = ((uint16_t)packetBuffer[2] << 8) | packetBuffer[3];
             Serial.printf("[RX] Ожидаемый CRC: %s\n", formatHEX(expectedCRC).c_str());
             payloadPtr = packetBuffer + FIRST_PACKET_HEADER_SIZE;
             payloadLen = packetLen - FIRST_PACKET_HEADER_SIZE;
         } else {
             Serial.println("[RX] ОШИБКА: Первый пакет не содержит CRC!");
             payloadLen = 0;
         }
       } else {
         // ПОСЛЕДУЮЩИЙ ПАКЕТ: проверяем, что мы уже начали прием
         if (receivedBytesCount == 0) {
           Serial.println("[RX] ⚠️ ОШИБКА: Пропущен первый пакет! Ожидание новой передачи...");
           displayStatus("RX Error!", "Missed packet #0");
           digitalWrite(BOARD_LED, LOW);
           radio.startReceive();
           return; // Прерываем обработку этого невалидного пакета
         }
         payloadPtr = packetBuffer + PACKET_HEADER_SIZE;
         payloadLen = packetLen - PACKET_HEADER_SIZE;
       }
       
       if (payloadLen > 0 && receivedBytesCount + payloadLen <= FILE_BUFFER_SIZE) {
         memcpy(receivedFileBuffer + receivedBytesCount, payloadPtr, payloadLen);
         receivedBytesCount += payloadLen;
       }
       
       if (currentPacket == totalPackets - 1) {
         Serial.println(F("\n[RX] ПОЛНЫЙ ФАЙЛ ПРИНЯТ!"));
         Serial.printf("[RX] Всего байт: %u\n", receivedBytesCount);
         uint16_t receivedCRC = calculateCRC16(receivedFileBuffer, receivedBytesCount);
         Serial.printf("[RX] CRC принятого файла: %s\n", formatHEX(receivedCRC).c_str());
 
         if (receivedCRC == expectedCRC) {
             Serial.println("[RX] ПРОВЕРКА УСПЕШНА: CRC СОВПАЛИ!");
             displayStatus("File OK!", "CRC Matched", formatHEX(receivedCRC));
 
             if (receivedBytesCount < FILE_BUFFER_SIZE) {
                 receivedFileBuffer[receivedBytesCount] = '\0';
                 Serial.println(F("\n--- Содержимое принятого файла ---"));
                 Serial.println((const char*)receivedFileBuffer);
                 Serial.println(F("------------------------------------"));
             }
             
         } else {
             Serial.println("[RX] ПРОВЕРКА ПРОВАЛЕНА: CRC НЕ СОВПАЛИ!");
             displayStatus("File CORRUPTED!", "CRC Mismatch!", "Got:" + formatHEX(receivedCRC), "Exp:" + formatHEX(expectedCRC));
         }
         Serial.println("\n>>> Ожидание следующего файла... <<<");
       }
     }
   } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
     Serial.println("[RX] ⚠️ ОШИБКА CRC! Пакет поврежден.");
     displayStatus("RX Error!", "CRC Mismatch!");
   } else {
     Serial.printf("[RX] ✗ Ошибка чтения данных: %d\n", state);
     displayStatus("RX Error!", "Code: " + String(state));
   }
 
   digitalWrite(BOARD_LED, LOW);
   radio.startReceive(); // Всегда возвращаемся в режим приема
 }
 #endif
 
 // ==================== SETUP & LOOP ====================
 void setup() {
   Serial.begin(115200);
   delay(500);
   
   pinMode(BOARD_LED, OUTPUT);
   digitalWrite(BOARD_LED, LOW);
   pinMode(BUTTON_PIN, INPUT_PULLUP);
   
   initDisplay();
   initRadio();
   
   String status = IS_MASTER ? "Press BOOT to send" : "Waiting for file...";
   Serial.printf("\n=== LR1121 Reliable File Transfer (868MHz) ===\n");
   Serial.printf("Роль устройства: %s\n", DEVICE_NAME);
   displayStatus(DEVICE_NAME, status);
 
   if (IS_MASTER) {
     Serial.println(">>> Нажмите кнопку BOOT для отправки файла <<<");
     radio.standby();
   } else {
     Serial.println(">>> Ожидание файла... <<<");
     int rxState = radio.startReceive();
     if (rxState != RADIOLIB_ERR_NONE) {
         Serial.printf("[SETUP] ОШИБКА: Не удалось запустить режим приема! Код: %d\n", rxState);
     }
   }
 }
 
 void loop() {
   #if IS_MASTER
     if (digitalRead(BUTTON_PIN) == LOW) {
       delay(50);
       if (digitalRead(BUTTON_PIN) == LOW) {
         sendFile(file_data, sizeof(file_data));
         while(digitalRead(BUTTON_PIN) == LOW);
       }
     }
   #else
     if (packetReceivedFlag) {
       handleReception();
     }
   #endif
   delay(1);
 }
 