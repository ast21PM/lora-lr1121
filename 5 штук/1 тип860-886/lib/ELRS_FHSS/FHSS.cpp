#include "FHSS.h"
#include <string.h>

// ===========================================================================
// LCG Random Number Generator
// Точная копия из ExpressLRS/src/lib/random.h
// state = state * 2891336453 + 1
// result = (state >> 16) % n
// ===========================================================================

static uint32_t rng_state = 0;

static void rngSeed(uint32_t seed)
{
    rng_state = seed;
}

// Возвращает случайное число в диапазоне [0, max-1]
static uint32_t rngN(uint32_t max) {
    rng_state = rng_state * 214013U + 2531011U;
    return (rng_state >> 16) % max;
}

// ===========================================================================
// UID generation via MD5
// Официальный ELRS: MD5(bindingPhrase), uid = result[1..6]
// Используем встроенный MD5Builder из ESP32 Arduino framework
// ===========================================================================

#include <MD5Builder.h>

void generateUID(const char* phrase, uint8_t* uid)
{
    MD5Builder md5;
    md5.begin();
    md5.add((uint8_t*)phrase, (uint16_t)strlen(phrase));
    md5.calculate();

    uint8_t result[16];
    md5.getBytes(result);

    // ELRS намеренно пропускает байт [0], берёт [1..6]
    memcpy(uid, result + 1, 6);
}

// ===========================================================================
// FHSS Sequence Generation
// Точная копия FHSSrandomiseFHSSsequenceBuild() из FHSS.cpp
//
// Алгоритм:
//   1. Заполнить массив: позиции кратные FHSS_CHANNELS = sync_channel,
//      позиция sync_channel внутри блока = 0, остальные = i % FHSS_CHANNELS
//   2. Fisher-Yates shuffle: для каждой не-sync позиции менять местами
//      со случайной позицией в том же блоке
// ===========================================================================

void generateFHSSsequence(const uint8_t* uid, uint8_t* sequence)
{
    const uint8_t  freqCount   = FHSS_CHANNELS;
    const uint8_t  syncChannel = FHSSsyncChannel(); // FHSS_CHANNELS / 2

    // Seed из первых 4 байт UID — как в официальном ELRS
    uint32_t seed = ((uint32_t)uid[0] << 24)
                  | ((uint32_t)uid[1] << 16)
                  | ((uint32_t)uid[2] <<  8)
                  |  (uint32_t)uid[3];

    rngSeed(seed);

    // --- Шаг 1: инициализация массива ---
    for (uint16_t i = 0; i < FHSS_SEQUENCE_LEN; i++)
    {
        if (i % freqCount == 0) {
            // Начало каждого блока = sync канал
            sequence[i] = syncChannel;
        } else if (i % freqCount == syncChannel) {
            // Позиция sync_channel внутри блока = 0
            sequence[i] = 0;
        } else {
            sequence[i] = i % freqCount;
        }
    }

    // --- Шаг 2: Fisher-Yates shuffle по блокам ---
    for (uint16_t i = 0; i < FHSS_SEQUENCE_LEN; i++)
    {
        // Пропускаем позиции sync_channel (начало каждого блока)
        if (i % freqCount != 0)
        {
            uint16_t blockStart = (i / freqCount) * freqCount;
            uint8_t  randPos    = rngN(freqCount - 1) + 1; // 1..freqCount-1

            // Меняем местами текущий элемент с случайным в том же блоке
            uint8_t tmp                    = sequence[i];
            sequence[i]                    = sequence[blockStart + randPos];
            sequence[blockStart + randPos] = tmp;
        }
    }
}

// ===========================================================================
// Frequency lookup
// ===========================================================================

float FHSSgetFreq(const uint8_t* sequence, uint8_t idx)
{
    return FHSS_BASE_FREQ + (sequence[idx % FHSS_SEQUENCE_LEN] * FHSS_STEP_FREQ);
}
