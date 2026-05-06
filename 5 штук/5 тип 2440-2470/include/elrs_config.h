#pragma once

// ===========================================================================
// ELRS DISARM System — конфигурация протокола
// ===========================================================================

#ifndef FHSS_CHANNELS
#define FHSS_CHANNELS 40
#endif
#ifndef FHSS_BASE_FREQ
#define FHSS_BASE_FREQ 903.5f
#endif
#ifndef FHSS_STEP_FREQ
#define FHSS_STEP_FREQ 0.6f
#endif
#ifndef FHSS_RENDEZVOUS_IDX
#define FHSS_RENDEZVOUS_IDX 0
#endif

#define ELRS_FHSS_HOP_INTERVAL 4
#define ELRS_SYNC_INTERVAL 16
#define ELRS_NONCE_WINDOW 2
#define ELRS_SEQUENCE_LEN (FHSS_CHANNELS * (256 / FHSS_CHANNELS))

#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ 902.0
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW 500.0
#endif
#ifndef CONFIG_RADIO_SF
#define CONFIG_RADIO_SF 6
#endif
#ifndef CONFIG_RADIO_CR
#define CONFIG_RADIO_CR 5
#endif
#ifndef CONFIG_RADIO_SYNC_WORD
#define CONFIG_RADIO_SYNC_WORD 0x12
#endif
#ifndef CONFIG_RADIO_PREAMBLE
#define CONFIG_RADIO_PREAMBLE 6
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER 10
#endif

#ifndef BINDING_PHRASE
#define BINDING_PHRASE "TEST11"
#endif

// ---------------------------------------------------------------------------
// Тайминги в миллисекундах (для RadioLib API и OLED — не критично по точности)
// ---------------------------------------------------------------------------
#ifndef TX_INTERVAL_MS
#define TX_INTERVAL_MS 10 // 100Hz — один слот = 10 мс
#endif
#ifndef TX_SLOT_OFFSET_MS
#define TX_SLOT_OFFSET_MS 0
#endif
#ifndef RX_WINDOW_EXTRA_MS
#define RX_WINDOW_EXTRA_MS 3 // окно приёма = 10 + 3 = 13 мс
#endif
// RX_TIMEOUT_MS — суммарный таймаут ожидания пакета для RadioLib receive()
// RadioLib принимает таймаут в МС (int), поэтому оставляем здесь в мс.
#ifndef RX_TIMEOUT_MS
#define RX_TIMEOUT_MS (TX_INTERVAL_MS + RX_WINDOW_EXTRA_MS)
#endif
#ifndef SYNC_TIMEOUT_MS
#define SYNC_TIMEOUT_MS 2000
#endif

// ---------------------------------------------------------------------------
// Тайминги в МИКРОСЕКУНДАХ для esp_timer_get_time() — используются в radioTask
// Тип int64_t, суффикс LL обязателен.
// ---------------------------------------------------------------------------
#define TX_INTERVAL_US 10000LL // 10 мс = 10 000 мкс (100 Hz)
#define TX_SLOT_OFFSET_US 0LL
#define RX_WINDOW_EXTRA_US ((int64_t)(RX_WINDOW_EXTRA_MS) * 1000LL)
// RX_TIMEOUT_US — передаётся в radio.receive() НЕ напрямую (RadioLib принимает
// мс). Используется только для проверки таймаутов через esp_timer_get_time().
#define RX_TIMEOUT_US ((int64_t)(TX_INTERVAL_MS + RX_WINDOW_EXTRA_MS) * 1000LL)
#define SYNC_TIMEOUT_US ((int64_t)(SYNC_TIMEOUT_MS) * 1000LL)
#define TRACK_TIMEOUT_US (500LL * 1000LL) // 500 мс в мкс

// ---------------------------------------------------------------------------
// Гибридное ожидание слота для radioTask (PILOT и AGGRESSOR SOLO):
// Если до слота > 2 мс — уступаем ядро через vTaskDelay.
// Последние мкс крутимся в жёстком busy-wait с NOP для точного тайминга.
// Порог для перехода к busy-wait (мкс):
// ---------------------------------------------------------------------------
#define TX_BUSYWAIT_THRESHOLD_US 2000LL // менее 2 мс — переходим в busy-wait

// ---------------------------------------------------------------------------
// Протокол
// ---------------------------------------------------------------------------
#ifndef PILOT_ID
#define PILOT_ID 0xBB
#endif
#ifndef DISARM_THRESHOLD
#define DISARM_THRESHOLD 3
#endif

#if defined(ROLE_PILOT)
#define DEVICE_ROLE "PILOT (TX)"
#elif defined(ROLE_DRONE)
#define DEVICE_ROLE "DRONE (RX)"
#elif defined(ROLE_AGGRESSOR)
#define DEVICE_ROLE "AGGRESSOR"
#elif defined(ROLE_MONITOR)
#define DEVICE_ROLE "MONITOR (RX)"
#else
#define DEVICE_ROLE "UNKNOWN"
#endif

#define FHSS_SYNC_CHANNEL (FHSS_CHANNELS / 2)
#define FHSS_SYNC_FREQ_MHZ                                                     \
  (FHSS_BASE_FREQ + (FHSS_SYNC_CHANNEL * FHSS_STEP_FREQ))

#define ELRS_SHOULD_HOP(nonce) (((nonce) + 1) % ELRS_FHSS_HOP_INTERVAL == 0)
#define ELRS_SHOULD_SYNC(nonce) ((nonce) % ELRS_SYNC_INTERVAL == 0)
#define ELRS_CRC_HIGH_INIT(nonce) (((nonce) % ELRS_FHSS_HOP_INTERVAL) + 1)
