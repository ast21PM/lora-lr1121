#pragma once
#include <stdint.h>

// ===========================================================================
// ELRS FHSS — алгоритмически точная копия официального ExpressLRS
// Источник: ExpressLRS/src/lib/FHSS/FHSS.cpp (FHSSrandomiseFHSSsequenceBuild)
//           ExpressLRS/src/lib/random.h (rngSeed, rngN)
// Все внешние зависимости (logging.h, options.h, targets.h) убраны.
// ===========================================================================

// ---------------------------------------------------------------------------
// Параметры домена FCC915 (берутся из platformio.ini через -D флаги)
// Значения из официального ELRS: freq_start=903500000, freq_stop=926900000,
// freq_count=40
// ---------------------------------------------------------------------------
#ifndef FHSS_CHANNELS
  #define FHSS_CHANNELS     40
#endif
#ifndef FHSS_BASE_FREQ
  #define FHSS_BASE_FREQ    903.5f   // МГц
#endif
#ifndef FHSS_STEP_FREQ
  #define FHSS_STEP_FREQ    0.6f     // МГц, (926.9-903.5)/39 = 0.6
#endif
#ifndef FHSS_RENDEZVOUS_IDX
  #define FHSS_RENDEZVOUS_IDX 0
#endif

// Длина последовательности — кратна FHSS_CHANNELS
// ELRS использует 256, нам хватит одного периода
#define FHSS_SEQUENCE_LEN   FHSS_CHANNELS

// ---------------------------------------------------------------------------
// Публичный интерфейс
// ---------------------------------------------------------------------------

/**
 * @brief Генерирует UID из binding phrase через MD5
 *        Алгоритм: MD5(phrase), берём байты [1..6] (байт [0] пропускается)
 *        Это точное поведение официального ELRS.
 *
 * @param phrase  binding phrase (одинакова на всех платах)
 * @param uid     выходной буфер 6 байт
 */
void generateUID(const char* phrase, uint8_t* uid);

/**
 * @brief Генерирует FHSS последовательность из seed
 *        Алгоритм: FHSSrandomiseFHSSsequenceBuild() из официального ELRS
 *        - sync_channel = FHSS_CHANNELS / 2
 *        - каждый FHSS_CHANNELS-й элемент = sync_channel
 *        - Fisher-Yates shuffle блоками по FHSS_CHANNELS
 *        - RNG: LCG из random.h ELRS (множитель 2891336453, инкремент 1)
 *
 * @param uid       6-байтовый UID от generateUID()
 * @param sequence  выходной массив длиной FHSS_SEQUENCE_LEN
 */
void generateFHSSsequence(const uint8_t* uid, uint8_t* sequence);

/**
 * @brief Возвращает частоту в МГц для заданного индекса в последовательности
 *
 * @param sequence  массив от generateFHSSsequence()
 * @param idx       текущий указатель в последовательности
 * @return float    частота в МГц
 */
float FHSSgetFreq(const uint8_t* sequence, uint8_t idx);

/**
 * @brief Возвращает следующий индекс в последовательности (с wrap-around)
 */
static inline uint8_t FHSSnextIndex(uint8_t current)
{
    return (current + 1) % FHSS_SEQUENCE_LEN;
}

/**
 * @brief Индекс sync-канала (середина диапазона — как в ELRS)
 */
static inline uint8_t FHSSsyncChannel()
{
    return FHSS_CHANNELS / 2;
}
