#pragma once

// ===========================================================================
// ELRS OTA Packet Format — минимальная версия для 900MHz RC пакетов
// Источник: ExpressLRS/src/lib/OTA/OTA.h (OTA_Packet4_s, OTA4_PACKET_SIZE)
// Убраны зависимости: crsf_protocol.h, telemetry_protocol.h, FIFO.h
// Оставлен только OTA4 (8 байт) — стандартный формат ELRS 900MHz
// ===========================================================================

#include <stdint.h>

// Макрос упаковки структур без padding — критично для правильного sizeof()
#ifndef PACKED
  #define PACKED __attribute__((packed))
#endif

// ---------------------------------------------------------------------------
// Размеры пакета
// ---------------------------------------------------------------------------
#define OTA4_PACKET_SIZE      8U   // полный пакет: 7 байт данных + 1 байт CRC (младший)
#define OTA4_CRC_CALC_LEN     7U   // сколько байт входит в расчёт CRC (всё кроме crcLow)

// ---------------------------------------------------------------------------
// Типы пакетов — биты [1:0] первого байта
// ---------------------------------------------------------------------------
#define PACKET_TYPE_RCDATA    0b00  // RC каналы (основной тип TX→RX)
#define PACKET_TYPE_DATA      0b01  // MSP / телеметрия
#define PACKET_TYPE_SYNC      0b10  // синхронизация (TX→RX при старте)
#define PACKET_TYPE_LINKSTATS 0b00  // статистика канала (RX→TX, downlink)

// ---------------------------------------------------------------------------
// CRC полиномы (из OTA.h оригинального ELRS)
// ELRS_CRC14_POLY используется для OTA4 пакетов на 900MHz
// ---------------------------------------------------------------------------
#define ELRS_CRC_POLY      0x07    // CRC8  (для CRSF фреймов)
#define ELRS_CRC14_POLY    0x2E57  // CRC14 (для OTA4 пакетов) — Koopman: 0x372b
#define ELRS_CRC16_POLY    0x3D65  // CRC16 (для OTA8 пакетов)

// ---------------------------------------------------------------------------
// RC каналы — 4 канала по 10 бит (5 байт)
// Используется внутри OTA_Packet4_s.rc.ch
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t raw[5]; // 4x 10-bit каналов упакованных в 5 байт
} PACKED OTA_Channels_4x10;

// ---------------------------------------------------------------------------
// Sync пакет — передаётся TX при инициализации связи
// RX использует fhssIndex и nonce для синхронизации
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t fhssIndex;    // текущий индекс в FHSS последовательности
    uint8_t nonce;        // счётчик пакетов TX (0..255, цикличный)
    uint8_t rfRateEnum;   // индекс скоростного режима (50Hz=0, 100Hz=1, ...)
    uint8_t switchEncMode:1,
            newTlmRatio:3,
            geminiMode:1,
            otaProtocol:2,
            free:1;
    uint8_t UID4;         // байт [4] UID — для верификации binding phrase
    uint8_t UID5;         // байт [5] UID — для верификации binding phrase
} PACKED OTA_Sync_s;

// ---------------------------------------------------------------------------
// OTA4 пакет — основной формат ELRS 900MHz (8 байт итого)
//
// Байт [0]:  type(2бит) + crcHigh(6бит)
// Байты [1..6]: payload (RC данные или sync)
// Байт [7]:  crcLow (младший байт CRC14)
//
// CRC14 считается от байт [0..6] (OTA4_CRC_CALC_LEN = 7)
// seed для CRC = (uid[4] << 8) | uid[5]
// ---------------------------------------------------------------------------
typedef struct {
    // Первый байт: тип пакета (биты 1:0) + старшие 6 бит CRC
    uint8_t type:    2,
            crcHigh: 6;

    union {
        // PACKET_TYPE_RCDATA — RC каналы (основной тип)
        struct {
            OTA_Channels_4x10 ch;      // 5 байт: CH0..CH3 по 10 бит
            uint8_t switches: 7,       // AUX каналы в упакованном виде
                    isArmed:  1;       // AUX1: 1=ARM, 0=DISARM
        } PACKED rc;

        // PACKET_TYPE_SYNC — синхронизация при старте
        OTA_Sync_s sync;

        // Сырые байты payload для прямого доступа
        uint8_t raw[6];
    };

    uint8_t crcLow; // младший байт CRC14
} PACKED OTA_Packet4_s;

// ---------------------------------------------------------------------------
// Вспомогательные функции упаковки/распаковки RC каналов
// Источник: OTA.cpp — PackChannelData / UnpackChannelData для OTA4
// ---------------------------------------------------------------------------

/**
 * @brief Упаковывает 4 канала (10 бит каждый) в 5 байт OTA_Channels_4x10
 *        Диапазон входных значений: 0..1023 (10 бит)
 *        Стандарт ELRS: 988мкс → 0, 1500мкс → 512, 2012мкс → 1023
 *
 * @param ch      выходная структура
 * @param ch0..3  значения каналов 0..1023
 */
static inline void OTA4_packChannels(OTA_Channels_4x10* ch,
                                     uint16_t ch0, uint16_t ch1,
                                     uint16_t ch2, uint16_t ch3)
{
    uint32_t b0 = ((uint32_t)ch0 & 0x3FF)
                | (((uint32_t)ch1 & 0x3FF) << 10)
                | (((uint32_t)ch2 & 0x3FF) << 20)
                | (((uint32_t)ch3 & 0x03F) << 30); // младшие 6 бит ch3

    uint8_t b4 = (ch3 >> 6) & 0x0F; // старшие 4 бита ch3

    ch->raw[0] = (b0)       & 0xFF;
    ch->raw[1] = (b0 >>  8) & 0xFF;
    ch->raw[2] = (b0 >> 16) & 0xFF;
    ch->raw[3] = (b0 >> 24) & 0xFF;
    ch->raw[4] = b4;
}

/**
 * @brief Распаковывает 5 байт OTA_Channels_4x10 обратно в 4 канала
 */
static inline void OTA4_unpackChannels(const OTA_Channels_4x10* ch,
                                       uint16_t* ch0, uint16_t* ch1,
                                       uint16_t* ch2, uint16_t* ch3)
{
    uint32_t b = ((uint32_t)ch->raw[0])
               | ((uint32_t)ch->raw[1] <<  8)
               | ((uint32_t)ch->raw[2] << 16)
               | ((uint32_t)ch->raw[3] << 24);

    *ch0 = (b)        & 0x3FF;
    *ch1 = (b >> 10)  & 0x3FF;
    *ch2 = (b >> 20)  & 0x3FF;
    *ch3 = ((b >> 30) & 0x3F) | ((uint16_t)ch->raw[4] << 6);
}

/**
 * @brief Конвертирует PWM мкс (988..2012) в 10-битное ELRS значение (0..1023)
 */
static inline uint16_t OTA4_usToCh(uint16_t us)
{
    if (us < 988)  us = 988;
    if (us > 2012) us = 2012;
    return (uint16_t)(((uint32_t)(us - 988) * 1023) / 1024);
}

/**
 * @brief Конвертирует 10-битное ELRS значение обратно в PWM мкс
 */
static inline uint16_t OTA4_chToUs(uint16_t ch)
{
    return (uint16_t)(988 + ((uint32_t)ch * 1024) / 1023);
}
