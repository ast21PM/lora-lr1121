/**
 * @file main.cpp
 * @brief LoRa ELRS Simulation: 4-Role System (v4.9 - Aggressor FHSS Sync)
 *
 * CHANGELOG v4.9:
 *  [FIX] Агрессор переработан как полудуплекс — ключевое исправление:
 *        Раньше агрессор инкрементировал fhssIndex сам → уходил на другой канал
 *        Теперь агрессор: 1) слушает 10мс на текущем канале,
 *                         2) ловит пакет пилота → берёт его fhssIndex,
 *                         3) передаёт DISARM на ТОМ ЖЕ канале,
 *                         4) прыгает на следующий канал по таблице пилота.
 *        Дрон видит агрессора потому что оба на одном канале.
 *  [FIX] RSSI дрона: обновляется для ОБОИХ источников, pilotBound не блокирует
 *  [FIX] tickDisplay() убран из горячего пути STATE_TRACK
 *  [FIX] Якорь от millis() после jumpToNextChannel()
 *  [KEEP] Binding phrase → UID → FHSS таблица
 *  [KEEP] Экран binding при старте + Serial UID диагностика
 *
 * Временная диаграмма (агрессор v4.9):
 *   PILOT:    t=0   |==TX ARM==|  t=20ms  |==TX ARM==|
 *   AGGRESSOR:       |listen 10ms| → |==TX DISARM==|
 *   DRONE:    ждёт пакет на канале N...
 *             принимает PILOT (RSSI=-60), принимает AGGR (RSSI=-55)
 *             aggrRssi > pilotRssi+2 → ATTACK!
 *
 * Структура пакета (10 байт):
 *  [0]    = следующий fhssIndex
 *  [1]    = TX_ID (Pilot=0xBB, Aggressor=0xAA)
 *  [2..7] = 4x11bit каналов + 1bit AUX1 (ARM/DISARM)
 *  [8..9] = CRC14 (big-endian)
 */

 #include <Arduino.h>
 #include <SPI.h>
 #include <Wire.h>
 #include <RadioLib.h>
 #include <U8g2lib.h>
 #include "pins_config.h"
 
 // ==================== КОНФИГУРАЦИЯ ПРОТОКОЛА ====================
 #define FHSS_CHANNELS        80
 #define FHSS_BASE_FREQ       863.0f
 #define FHSS_STEP            0.1f
 #define FHSS_RENDEZVOUS_IDX  0
 
 #define ELRS_PAYLOAD_SIZE    8
 #define CRC_SIZE             2
 #define ELRS_PACKET_SIZE     (ELRS_PAYLOAD_SIZE + CRC_SIZE)
 
 #define SYNC_TIMEOUT_MS      2000
 #define DISPLAY_UPDATE_MS    200
 #define DISARM_THRESHOLD     3
 #ifndef RX_WINDOW_EXTRA_MS
   #define RX_WINDOW_EXTRA_MS   100
 #endif
 
 // ==================== Имя роли ====================
 #if defined(ROLE_AGGRESSOR)
   #define DEVICE_ROLE "AGGRESSOR (TX)"
 #elif defined(ROLE_MONITOR)
   #define DEVICE_ROLE "MONITOR (RX)"
 #elif defined(ROLE_PILOT)
   #define DEVICE_ROLE "PILOT (TX)"
 #elif defined(ROLE_DRONE)
   #define DEVICE_ROLE "DRONE (RX)"
 #else
   #error "Please select a valid Environment in PlatformIO!"
 #endif
 
 // ==================== ОБЪЕКТЫ ====================
 U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
 SPIClass radioSPI(HSPI);
 Module radioModule(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, radioSPI);
 LR1121 radio(&radioModule);
 
 // ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
 volatile bool packetReceivedFlag = false;
 uint8_t       systemUID[6];
 uint8_t       fhssTable[FHSS_CHANNELS];
 uint8_t       fhssIndex = 0;
 
 String        displayLine2  = "";
 String        displayLine3  = "";
 unsigned long lastDisplayMs = 0;
 bool          displayDirty  = false;
 
 #ifdef ROLE_DRONE
   bool          ledBlinking       = false;
   int           ledBlinkCount     = 0;
   unsigned long lastLedMs         = 0;
   #define LED_BLINK_INTERVAL      60
   #define LED_BLINK_TOTAL         20
 
   // Конечный автомат дрона
   enum DroneState { DS_ARMED, DS_DISARMED };
   DroneState    droneState        = DS_ARMED;
 
   uint8_t       consecutiveDisarm = 0;
   bool          pilotBound        = false;
   uint8_t       pilotID           = PILOT_ID;
 
   float         pilotRssi         = -150.0f;
   float         aggrRssi          = -150.0f;
   unsigned long lastPilotMs       = 0;
   unsigned long lastAggrMs        = 0;
 #endif
 
 // ==================== ISR ====================
 #if defined(ESP8266) || defined(ESP32)
 ICACHE_RAM_ATTR
 #endif
 void onPacketReceived() { packetReceivedFlag = true; }
 
 #ifdef ROLE_AGGRESSOR
 volatile bool aggrPktFlag = false;
 void IRAM_ATTR onAggrPkt() { aggrPktFlag = true; }
 #endif
 
 void configureAntenna(bool isTransmit) {
   digitalWrite(ANT_SW_TX,  isTransmit);
   digitalWrite(ANT_SW_RX, !isTransmit);
 }
 
 // ==================== CORE: UID / FHSS / CRC ====================
 
 void generateUID(const char* phrase, uint8_t* output) {
   size_t len = strlen(phrase);
   uint32_t hash = 5381;
   for (size_t i = 0; i < len; i++) hash = ((hash << 5) + hash) + phrase[i];
   for (int i = 0; i < 6; i++) {
     output[i] = (hash >> (i * 4)) & 0xFF;
     output[i] ^= phrase[i % len];
   }
 }
 
 void generateFHSSTable(const uint8_t* uid, uint8_t* table) {
   for (int i = 0; i < FHSS_CHANNELS; i++) table[i] = i;
   uint32_t seed = (uint32_t)uid[0] << 24 | (uint32_t)uid[1] << 16 |
                   (uint32_t)uid[2] << 8  | uid[3];
   for (int i = FHSS_CHANNELS - 1; i > 0; i--) {
     seed = seed * 1664525 + 1013904223;
     int j = (seed >> 16) % (i + 1);
     uint8_t tmp = table[i]; table[i] = table[j]; table[j] = tmp;
   }
 }
 
 uint16_t calculateCRC14(const uint8_t* data, size_t length) {
   uint16_t crc = 0;
   for (size_t i = 0; i < length; i++) {
     crc ^= (uint16_t)data[i] << 6;
     for (uint8_t j = 0; j < 8; j++)
       crc = (crc & 0x2000) ? (crc << 1) ^ 0x3D65 : crc << 1;
   }
   return crc & 0x3FFF;
 }
 
 // ==================== ELRS 11-BIT PACKING ====================
 
 uint16_t mapRcTo11Bit(uint16_t rcValue) {
   return (uint16_t)map((long)rcValue, 988, 2011, 172, 1811);
 }
 
 void packELRSData_11bit(uint8_t* payload, bool isArmed) {
   uint16_t ch0 = mapRcTo11Bit(1500);
   uint16_t ch1 = mapRcTo11Bit(1500);
   uint16_t ch2 = mapRcTo11Bit(1500);
   uint16_t ch3 = mapRcTo11Bit(988);
   uint8_t  ch4 = isArmed ? 1 : 0;
 
   uint64_t packed = 0;
   packed |= (uint64_t)ch0;
   packed |= (uint64_t)ch1 << 11;
   packed |= (uint64_t)ch2 << 22;
   packed |= (uint64_t)ch3 << 33;
   packed |= (uint64_t)ch4 << 44;
 
   memcpy(&payload[2], &packed, 6);
 }
 
 bool unpackIsArmed_11bit(const uint8_t* packet) {
   return (packet[7] >> 4) & 0x01;
 }
 
 // ==================== DISPLAY ====================
 
 void setDisplay(String line2, String line3 = "") {
   displayLine2 = line2;
   displayLine3 = line3;
   displayDirty = true;
 }
 
 void tickDisplay() {
   if (!displayDirty) return;
 #ifdef ROLE_DRONE
   if (droneState == DS_DISARMED) return; // экран заморожен навсегда
 #endif
   if (millis() - lastDisplayMs < DISPLAY_UPDATE_MS) return;
   lastDisplayMs = millis();
   displayDirty  = false;
   display.clearBuffer();
   display.setFont(u8g2_font_6x10_tf);
   display.setCursor(0, 10); display.print(DEVICE_ROLE);
   display.drawHLine(0, 12, 128);
   display.setCursor(0, 25); display.print(displayLine2);
   if (displayLine3.length() > 0) { display.setCursor(0, 40); display.print(displayLine3); }
 
 #ifdef ROLE_DRONE
   display.setCursor(0, 55);
   if (!pilotBound) {
     display.print("NOT BOUND");
   } else {
     display.print("BND:0x");
     display.print(String(pilotID, HEX));
     display.print(droneState == DS_ARMED ? " ARMED" : " DISARMED");
   }
 #endif
 
   display.sendBuffer();
 }
 
 void forceDisplay(String line2, String line3 = "") {
   displayLine2  = line2;
   displayLine3  = line3;
   displayDirty  = true;
   lastDisplayMs = 0;
   tickDisplay();
 }
 
 void showBindingScreen() {
   display.clearBuffer();
   display.setFont(u8g2_font_6x10_tf);
   display.setCursor(0, 10); display.print(DEVICE_ROLE);
   display.drawHLine(0, 12, 128);
   display.setCursor(0, 24); display.print("BINDING PHRASE:");
   String phrase = String(BINDING_PHRASE);
   display.setCursor(0, 36); display.print(phrase.substring(0, 16));
   display.setCursor(0, 48); display.print("UID:");
   char uidStr[13];
   snprintf(uidStr, sizeof(uidStr), "%02X%02X%02X%02X%02X%02X",
            systemUID[0], systemUID[1], systemUID[2],
            systemUID[3], systemUID[4], systemUID[5]);
   display.print(uidStr);
   display.sendBuffer();
   delay(1500);
 }
 
 // ==================== ИНИЦИАЛИЗАЦИЯ ====================
 void setup() {
   Serial.begin(115200);
   pinMode(BOARD_LED,  OUTPUT); digitalWrite(BOARD_LED, LOW);
   pinMode(ANT_SW_VDD, OUTPUT); pinMode(ANT_SW_RX, OUTPUT); pinMode(ANT_SW_TX, OUTPUT);
   digitalWrite(ANT_SW_VDD, HIGH);
 
   Wire.begin(I2C_SDA, I2C_SCL);
   display.begin();
   forceDisplay("Initializing...");
 
   generateUID(BINDING_PHRASE, systemUID);
   generateFHSSTable(systemUID, fhssTable);
 
   Serial.println("=== BINDING PHRASE ===");
   Serial.printf("[BIND] Phrase : \"%s\"\n", BINDING_PHRASE);
   Serial.printf("[BIND] UID    : %02X:%02X:%02X:%02X:%02X:%02X\n",
                 systemUID[0], systemUID[1], systemUID[2],
                 systemUID[3], systemUID[4], systemUID[5]);
   Serial.printf("[BIND] FHSS[0..4]: %d %d %d %d %d\n",
                 fhssTable[0], fhssTable[1], fhssTable[2],
                 fhssTable[3], fhssTable[4]);
   Serial.println("======================");
 
   showBindingScreen();
 
   radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
   int state = radio.begin();
   if (state != RADIOLIB_ERR_NONE) {
     Serial.printf("[RADIO] Init Failed: %d\n", state);
     forceDisplay("RADIO FAIL:", String(state));
     while (1);
   }
 
   radio.setTCXO(3.3);
   radio.setFrequency(CONFIG_RADIO_FREQ);
   radio.setBandwidth(CONFIG_RADIO_BW);
   radio.setSpreadingFactor(CONFIG_RADIO_SF);
   radio.setCodingRate(CONFIG_RADIO_CR);
   radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
   radio.setSyncWord(CONFIG_RADIO_SYNC_WORD);
   radio.setPreambleLength(CONFIG_RADIO_PREAMBLE);
   radio.setCRC(false);
 
 #if defined(ROLE_MONITOR) || defined(ROLE_DRONE)
   radio.setPacketReceivedAction(onPacketReceived);
   configureAntenna(false);
   fhssIndex = FHSS_RENDEZVOUS_IDX;
   float rendezvousFreq = FHSS_BASE_FREQ + (fhssTable[FHSS_RENDEZVOUS_IDX] * FHSS_STEP);
   radio.setFrequency(rendezvousFreq);
   radio.startReceive();
   forceDisplay("Phase: SEARCH", "Wait " + String(rendezvousFreq, 1) + " MHz");
   Serial.printf("[RX] Waiting on rendezvous %.1f MHz\n", rendezvousFreq);
 #elif defined(ROLE_AGGRESSOR)
   // Агрессор стартует в режиме прослушки на rendezvous канале
   radio.setPacketReceivedAction(onAggrPkt);
   configureAntenna(false);
   fhssIndex = FHSS_RENDEZVOUS_IDX;
   float aggrFreq = FHSS_BASE_FREQ + (fhssTable[FHSS_RENDEZVOUS_IDX] * FHSS_STEP);
   radio.setFrequency(aggrFreq);
   radio.startReceive();
   forceDisplay("AGGR SEARCH", "Listening pilot...");
   Serial.printf("[AGGR] Listening for pilot on %.1f MHz\n", aggrFreq);
 #else
   // PILOT
   configureAntenna(false);
   forceDisplay("TX Ready", "Phrase OK");
 #endif
 
 #ifdef ROLE_DRONE
   Serial.printf("[DRONE] Pilot ID: 0x%02X\n", pilotID);
 #endif
   Serial.println("[SYSTEM] Ready. Role: " DEVICE_ROLE);
 }
 
 // =============================================================================
 // PILOT TX LOOP
 // Кнопка BOOT (GPIO 0) — переключает ARM/DISARM вручную.
 // Нажать = DISARM, отпустить = ARM
 // =============================================================================
 #if defined(ROLE_PILOT)
 
 #define PILOT_BTN_PIN  0   // BOOT кнопка на LilyGO T3-S3
 bool pilotArmed = true;    // текущее состояние ARM/DISARM пилота
 
 void loop() {
   static unsigned long nextTxTime  = 0;
   static bool          lastBtnState = HIGH;
   if (nextTxTime == 0) {
     nextTxTime = millis();
     pinMode(PILOT_BTN_PIN, INPUT_PULLUP);
   }
 
   // Читаем кнопку — нажата = LOW (подтяжка к VCC)
   bool btnState = digitalRead(PILOT_BTN_PIN);
   if (btnState == LOW && lastBtnState == HIGH) {
     // Фронт нажатия — переключаем
     pilotArmed = !pilotArmed;
     Serial.printf("[PILOT] Button → %s\n", pilotArmed ? "ARM" : "DISARM");
   }
   lastBtnState = btnState;
 
   while ((long)(millis() - nextTxTime) < 0) { tickDisplay(); }
   nextTxTime += TX_INTERVAL_MS;
 
   uint8_t payload[ELRS_PAYLOAD_SIZE];
   uint8_t packet[ELRS_PACKET_SIZE];
 
   float freq = FHSS_BASE_FREQ + (fhssTable[fhssIndex] * FHSS_STEP);
   radio.setFrequency(freq);
 
   payload[0] = (fhssIndex + 1) % FHSS_CHANNELS;
   payload[1] = DEVICE_ID; // 0xBB
   packELRSData_11bit(payload, pilotArmed);
   setDisplay(pilotArmed ? "FLYING ARMED" : "PILOT DISARM",
              "F:" + String(freq, 1) + " i:" + String(fhssIndex));
 
   uint16_t crc = calculateCRC14(payload, ELRS_PAYLOAD_SIZE);
   memcpy(packet, payload, ELRS_PAYLOAD_SIZE);
   packet[ELRS_PAYLOAD_SIZE]     = (crc >> 8) & 0xFF;
   packet[ELRS_PAYLOAD_SIZE + 1] = crc & 0xFF;
 
   configureAntenna(true);
   digitalWrite(BOARD_LED, pilotArmed ? HIGH : LOW);
   radio.transmit(packet, ELRS_PACKET_SIZE);
   digitalWrite(BOARD_LED, LOW);
 
   fhssIndex = (fhssIndex + 1) % FHSS_CHANNELS;
   radio.standby();
   configureAntenna(false);
 }
 #endif // PILOT
 
 // =============================================================================
 // AGGRESSOR LOOP
 //
 // Два режима работы:
 //
 // 1. ПИЛОТ ЕСТЬ (AGGR_SYNC):
 //    Слушаем TX_SLOT_OFFSET_MS, ловим пакет пилота → берём его fhssIndex.
 //    Передаём DISARM на том же канале сразу после пилота.
 //    Дрон ещё слушает этот канал → получает DISARM.
 //
 // 2. ПИЛОТА НЕТ (AGGR_SOLO):
 //    Работаем как обычный TX — просто шлём DISARM каждые TX_INTERVAL_MS.
 //    Дрон синхронизируется с агрессором как с единственным TX в эфире.
 //    Агрессор сразу становится "пилотом" для дрона по FHSS.
 // =============================================================================
 #if defined(ROLE_AGGRESSOR)
 
 enum AggrState { AGGR_SOLO, AGGR_SYNC };
 AggrState     aggrState       = AGGR_SOLO;
 unsigned long aggrLastPilotMs = 0;
 uint8_t       aggrSyncIdx     = 0;
 
 // Для SOLO режима — абсолютный TX якорь как у пилота
 static unsigned long aggrNextTxTime = 0;
 
 void loop() {
   uint8_t rxBuf[ELRS_PACKET_SIZE];
   bool    gotPilot = false;
 
   // ---- Потеря синхронизации с пилотом → возврат в SOLO ----
   if (aggrState == AGGR_SYNC && millis() - aggrLastPilotMs > 500) {
     aggrState = AGGR_SOLO;
     aggrNextTxTime = millis(); // сразу начинаем TX
     Serial.println("[AGGR] Pilot lost — SOLO mode");
     setDisplay("AGGR SOLO", "No pilot");
   }
 
   if (aggrState == AGGR_SOLO) {
     // ================================================================
     // SOLO режим — просто TX как пилот, только DISARM
     // Дрон синхронизируется с нами напрямую
     // ================================================================
     if (aggrNextTxTime == 0) aggrNextTxTime = millis();
     while ((long)(millis() - aggrNextTxTime) < 0) { tickDisplay(); }
     aggrNextTxTime += TX_INTERVAL_MS;
 
     // Пока ждём — слушаем не появился ли пилот
     {
       float f = FHSS_BASE_FREQ + (fhssTable[fhssIndex] * FHSS_STEP);
       radio.standby();
       radio.setFrequency(f);
       configureAntenna(false);
       aggrPktFlag = false;
       radio.startReceive();
       unsigned long listenUntil = millis() + 5; // 5мс быстрая проверка
       while ((long)(listenUntil - millis()) > 0) {
         if (aggrPktFlag) {
           aggrPktFlag = false;
           if (radio.getPacketLength() == ELRS_PACKET_SIZE) {
             radio.readData(rxBuf, ELRS_PACKET_SIZE);
             uint16_t pktCRC  = (uint16_t)rxBuf[ELRS_PAYLOAD_SIZE] << 8 | rxBuf[ELRS_PAYLOAD_SIZE + 1];
             uint16_t calcCRC = calculateCRC14(rxBuf, ELRS_PAYLOAD_SIZE);
             if (calcCRC == pktCRC && rxBuf[1] == 0xBB) {
               aggrSyncIdx     = rxBuf[0];
               aggrLastPilotMs = millis();
               aggrState       = AGGR_SYNC;
               gotPilot        = true;
               Serial.printf("[AGGR] Pilot appeared — SYNC mode ch->%d\n", aggrSyncIdx);
               break;
             }
           }
         }
         delayMicroseconds(100);
       }
     }
 
   } else {
     // ================================================================
     // SYNC режим — полудуплекс, слушаем пилота TX_SLOT_OFFSET_MS
     // ================================================================
     float f = FHSS_BASE_FREQ + (fhssTable[fhssIndex] * FHSS_STEP);
     radio.standby();
     radio.setFrequency(f);
     configureAntenna(false);
     aggrPktFlag = false;
     radio.startReceive();
 
     unsigned long listenUntil = millis() + TX_SLOT_OFFSET_MS;
     while ((long)(listenUntil - millis()) > 0) {
       if (aggrPktFlag) {
         aggrPktFlag = false;
         if (radio.getPacketLength() == ELRS_PACKET_SIZE) {
           radio.readData(rxBuf, ELRS_PACKET_SIZE);
           uint16_t pktCRC  = (uint16_t)rxBuf[ELRS_PAYLOAD_SIZE] << 8 | rxBuf[ELRS_PAYLOAD_SIZE + 1];
           uint16_t calcCRC = calculateCRC14(rxBuf, ELRS_PAYLOAD_SIZE);
           if (calcCRC == pktCRC && rxBuf[1] == 0xBB) {
             aggrSyncIdx     = rxBuf[0];
             aggrLastPilotMs = millis();
             gotPilot        = true;
             break;
           }
         }
       }
       delayMicroseconds(100);
     }
   }
 
   // ---- Передаём DISARM ----
   uint8_t payload[ELRS_PAYLOAD_SIZE];
   uint8_t packet[ELRS_PACKET_SIZE];
 
   float txFreq = FHSS_BASE_FREQ + (fhssTable[fhssIndex] * FHSS_STEP);
   radio.standby();
   radio.setFrequency(txFreq);
 
   uint8_t nextCh = (aggrState == AGGR_SYNC && gotPilot)
                    ? aggrSyncIdx
                    : (fhssIndex + 1) % FHSS_CHANNELS;
 
   payload[0] = nextCh;
   payload[1] = DEVICE_ID; // 0xAA
   packELRSData_11bit(payload, false); // DISARM
 
   uint16_t crc = calculateCRC14(payload, ELRS_PAYLOAD_SIZE);
   memcpy(packet, payload, ELRS_PAYLOAD_SIZE);
   packet[ELRS_PAYLOAD_SIZE]     = (crc >> 8) & 0xFF;
   packet[ELRS_PAYLOAD_SIZE + 1] = crc & 0xFF;
 
   configureAntenna(true);
   digitalWrite(BOARD_LED, HIGH);
   int txState = radio.transmit(packet, ELRS_PACKET_SIZE);
   digitalWrite(BOARD_LED, LOW);
 
   if (txState == RADIOLIB_ERR_NONE) {
     String modeStr = (aggrState == AGGR_SYNC) ? (gotPilot ? "SYNC" : "SYNC?") : "SOLO";
     Serial.printf("[AGGR] DISARM | F:%.1f ch%d->%d | %s\n",
                   txFreq, fhssIndex, nextCh, modeStr.c_str());
     setDisplay(aggrState == AGGR_SYNC ? "ATTACKING" : "AGGR SOLO",
                "F:" + String(txFreq, 1) + " " + modeStr);
   } else {
     Serial.printf("[AGGR] TX Err:%d\n", txState);
   }
 
   fhssIndex = nextCh;
   configureAntenna(false);
   tickDisplay();
 }
 #endif // AGGRESSOR
 
 // =============================================================================
 // RX LOGIC: MONITOR и DRONE — общие утилиты + логика дрона
 // =============================================================================
 #if defined(ROLE_MONITOR) || defined(ROLE_DRONE)
 
 enum RxState { STATE_SEARCH, STATE_TRACK };
 RxState       rxState              = STATE_SEARCH;
 unsigned long lastPacketMs         = 0;
 unsigned long nextExpectedPacketMs = 0;
 
 bool tryReceivePacket(uint8_t* outPacket) {
   if (!packetReceivedFlag) return false;
   packetReceivedFlag = false;
   if (radio.getPacketLength() != ELRS_PACKET_SIZE) return false;
   radio.readData(outPacket, ELRS_PACKET_SIZE);
   uint16_t pktCRC  = (uint16_t)outPacket[ELRS_PAYLOAD_SIZE] << 8 | outPacket[ELRS_PAYLOAD_SIZE + 1];
   uint16_t calcCRC = calculateCRC14(outPacket, ELRS_PAYLOAD_SIZE);
   return (calcCRC == pktCRC);
 }
 
 unsigned long jumpToNextChannel() {
   radio.standby();
   delay(2);
   float nextFreq = FHSS_BASE_FREQ + (fhssTable[fhssIndex] * FHSS_STEP);
   radio.setFrequency(nextFreq);
   delay(1);
   radio.startReceive();
   return millis(); // ПОСЛЕ startReceive
 }
 
 bool waitUntil(unsigned long deadline) {
   while ((long)(deadline - millis()) > 0) {
     if (packetReceivedFlag) return true;
     delayMicroseconds(50);
   }
   return packetReceivedFlag;
 }
 
 void handleValidPacket(const uint8_t* packet, float freq) {
   bool    isArmed = unpackIsArmed_11bit(packet);
   uint8_t txID    = packet[1];
   float   rssi    = radio.getRSSI();
 
 #ifdef ROLE_MONITOR
   // Монитор просто логирует — вся логика в отдельном loop ниже
   if (isArmed) {
     setDisplay("PILOT ARM", "ID:" + String(txID, HEX) + " R:" + String((int)rssi));
     Serial.printf("[MON] ARM  | ID:0x%02X | %.1fMHz | RSSI:%.1f\n", txID, freq, rssi);
   } else {
     setDisplay("AGGR DARM", "ID:" + String(txID, HEX) + " R:" + String((int)rssi));
     Serial.printf("[MON] DARM | ID:0x%02X | %.1fMHz | RSSI:%.1f\n", txID, freq, rssi);
   }
 #endif
 
 #ifdef ROLE_DRONE
   unsigned long now = millis();
 
   // ШАГ 1: Обновляем RSSI для обоих источников ВСЕГДА
   if (txID == pilotID) {
     pilotRssi   = rssi;
     lastPilotMs = now;
     if (!pilotBound) {
       pilotBound = true;
       Serial.printf("[DRONE] BOUND 0x%02X RSSI:%.1f\n", pilotID, rssi);
       for (int i = 0; i < 6; i++) {
         digitalWrite(BOARD_LED, i % 2 ? HIGH : LOW);
         delay(80);
       }
       digitalWrite(BOARD_LED, LOW);
     }
   } else {
     aggrRssi   = rssi;
     lastAggrMs = now;
     Serial.printf("[DRONE] AGG | ID:0x%02X | RSSI:%.1f\n", txID, rssi);
   }
 
   if (lastPilotMs != 0 && now - lastPilotMs > 1000) pilotRssi = -150.0f;
   if (lastAggrMs  != 0 && now - lastAggrMs  > 1000) aggrRssi  = -150.0f;
 
   bool isAggressorCloser = (aggrRssi > pilotRssi);
 
   // ШАГ 2: Конечный автомат состояний дрона
   switch (droneState) {
 
     // -----------------------------------------------------------
     case DS_ARMED:
       if (isAggressorCloser && txID != pilotID && !isArmed) {
         consecutiveDisarm++;
         Serial.printf("[DRONE] ATTACK! %d/%d | A:%.1f\n",
                       consecutiveDisarm, DISARM_THRESHOLD, aggrRssi);
         if (consecutiveDisarm >= DISARM_THRESHOLD) {
           droneState        = DS_DISARMED;
           consecutiveDisarm = 0;
           Serial.println("!!! DRONE DISARMED — REBOOT TO RECOVER !!!");
           // Финальный экран — больше не меняется
           display.clearBuffer();
           display.setFont(u8g2_font_6x10_tf);
           display.setCursor(0, 12); display.print("DRONE (RX)");
           display.drawHLine(0, 14, 128);
           display.setCursor(0, 30); display.print("!! DISARMED !!");
           display.setCursor(0, 44); display.print(String((int)aggrRssi) + " dBm");
           display.setCursor(0, 58); display.print("REBOOT TO RECOVER");
           display.sendBuffer();
           ledBlinking = true; ledBlinkCount = 0; lastLedMs = now;
         } else {
           setDisplay("ATTACK " + String(consecutiveDisarm) + "/" + String(DISARM_THRESHOLD),
                      String((int)aggrRssi) + " dBm");
         }
       } else if (txID == pilotID) {
         consecutiveDisarm = 0;
         setDisplay("ARMED [PILOT]", String((int)pilotRssi) + " dBm");
         Serial.printf("[DRONE] OK A:%.1f\n", aggrRssi);
       }
       break;
 
     // -----------------------------------------------------------
     case DS_DISARMED:
       // Дрон дизармлен навсегда — только логируем, экран не меняем
       Serial.printf("[DRONE] DISARMED (ignoring) | ID:0x%02X | A:%.1f\n",
                     txID, aggrRssi);
       break;
   }
 #endif
 }
 
 #ifdef ROLE_DRONE
 void tickLed() {
   if (!ledBlinking) return;
   if (millis() - lastLedMs < LED_BLINK_INTERVAL) return;
   lastLedMs = millis();
   digitalWrite(BOARD_LED, (ledBlinkCount % 2 == 0) ? HIGH : LOW);
   if (++ledBlinkCount >= LED_BLINK_TOTAL) {
     ledBlinking = false;
     digitalWrite(BOARD_LED, LOW);
   }
 }
 #endif
 
 // =============================================================================
 // MONITOR LOOP — простой сканер без FHSS-слежки
 // Прыгает по всем 80 каналам, на каждом ждёт TX_INTERVAL_MS.
 // Не теряет пакеты из-за дрейфа якоря — просто сканирует всё подряд.
 // Показывает ВСЕ пакеты: и от пилота и от агрессора.
 // =============================================================================
 #ifdef ROLE_MONITOR
 void loop() {
   uint8_t packet[ELRS_PACKET_SIZE];
 
   // Прыгаем на текущий канал
   float freq = FHSS_BASE_FREQ + (fhssTable[fhssIndex] * FHSS_STEP);
   radio.standby();
   radio.setFrequency(freq);
   radio.startReceive();
 
   // Слушаем TX_INTERVAL_MS — за это время пилот точно передаст если он на этом канале
   // Плюс агрессор придёт через ~10мс после него — оба в окне
   unsigned long listenUntil = millis() + TX_INTERVAL_MS;
   packetReceivedFlag = false;
 
   while ((long)(listenUntil - millis()) > 0) {
     if (packetReceivedFlag) {
       // Поймали что-то — читаем не выходя из цикла (может прийти ещё один пакет)
       if (radio.getPacketLength() == ELRS_PACKET_SIZE) {
         radio.readData(packet, ELRS_PACKET_SIZE);
         uint16_t pktCRC  = (uint16_t)packet[ELRS_PAYLOAD_SIZE] << 8 | packet[ELRS_PAYLOAD_SIZE + 1];
         uint16_t calcCRC = calculateCRC14(packet, ELRS_PAYLOAD_SIZE);
         if (calcCRC == pktCRC) {
           bool    isArmed = unpackIsArmed_11bit(packet);
           uint8_t txID    = packet[1];
           float   rssi    = radio.getRSSI();
           if (isArmed) {
             setDisplay("PILOT ARM", "ID:0x" + String(txID, HEX) + " R:" + String((int)rssi));
             Serial.printf("[MON] ARM  | ID:0x%02X | %.1fMHz | RSSI:%.1f\n", txID, freq, rssi);
           } else {
             setDisplay("AGGR DARM", "ID:0x" + String(txID, HEX) + " R:" + String((int)rssi));
             Serial.printf("[MON] DARM | ID:0x%02X | %.1fMHz | RSSI:%.1f\n", txID, freq, rssi);
           }
           tickDisplay();
           // Продолжаем слушать этот канал — агрессор может прийти чуть позже
           radio.startReceive();
         }
       }
       packetReceivedFlag = false;
     }
     delayMicroseconds(100);
   }
 
   // Переходим на следующий канал
   fhssIndex = (fhssIndex + 1) % FHSS_CHANNELS;
 }
 #endif // MONITOR
 
 // =============================================================================
 // DRONE LOOP — FHSS-слежка за пилотом
 // =============================================================================
 #ifdef ROLE_DRONE
 void loop() {
   uint8_t packet[ELRS_PACKET_SIZE];
 
   tickLed();
 
   if (rxState == STATE_SEARCH) {
     if (packetReceivedFlag) {
       if (tryReceivePacket(packet)) {
         float rFreq = FHSS_BASE_FREQ + (fhssTable[FHSS_RENDEZVOUS_IDX] * FHSS_STEP);
         Serial.printf("[RX] SYNC | ID:0x%02X | %.1fMHz\n", packet[1], rFreq);
         fhssIndex = packet[0];
         unsigned long jumpedAt = jumpToNextChannel();
         lastPacketMs         = jumpedAt;
         nextExpectedPacketMs = jumpedAt + TX_INTERVAL_MS;
         rxState = STATE_TRACK;
         handleValidPacket(packet, rFreq);
         tickDisplay();
       }
     } else {
       tickDisplay();
     }
     return;
   }
 
   if (rxState == STATE_TRACK) {
 
     if (millis() - lastPacketMs > SYNC_TIMEOUT_MS) {
       Serial.println("[RX] Sync LOST");
       pilotBound = false; pilotRssi = -150.0f; aggrRssi = -150.0f;
       rxState   = STATE_SEARCH;
       fhssIndex = FHSS_RENDEZVOUS_IDX;
       float rFreq = FHSS_BASE_FREQ + (fhssTable[FHSS_RENDEZVOUS_IDX] * FHSS_STEP);
       radio.standby(); delay(2);
       radio.setFrequency(rFreq);
       radio.startReceive();
       setDisplay("SEARCH", "Sync lost!");
       tickDisplay();
       return;
     }
 
     unsigned long deadline = nextExpectedPacketMs + RX_WINDOW_EXTRA_MS;
     if ((long)(deadline - millis()) < 0) deadline = millis() + 5;
 
     if ((long)(deadline - millis()) > 20) tickDisplay();
 
     waitUntil(deadline);
     tickLed();
 
     if (tryReceivePacket(packet)) {
       fhssIndex = packet[0];
       unsigned long jumpedAt = jumpToNextChannel();
       lastPacketMs         = jumpedAt;
       nextExpectedPacketMs = jumpedAt + TX_INTERVAL_MS;
       float curFreq = FHSS_BASE_FREQ +
         (fhssTable[(fhssIndex == 0 ? FHSS_CHANNELS-1 : fhssIndex-1)] * FHSS_STEP);
       handleValidPacket(packet, curFreq);
 
     } else {
       uint8_t missedIdx = fhssIndex;
       fhssIndex = (fhssIndex + 1) % FHSS_CHANNELS;
       unsigned long jumpedAt = jumpToNextChannel();
       nextExpectedPacketMs   = jumpedAt + TX_INTERVAL_MS;
       Serial.printf("[RX] Miss ch%d->ch%d\n", missedIdx, fhssIndex);
     }
   }
 }
 #endif // DRONE
 #endif // RX