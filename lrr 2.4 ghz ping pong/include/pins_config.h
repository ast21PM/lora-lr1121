#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

// ==================== I2C (OLED SSD1306) ====================
#define I2C_SDA                     18
#define I2C_SCL                     17
#define OLED_ADDR                   0x3C

// ==================== SPI (LoRa LR1121) ====================
#define RADIO_SCLK_PIN              5
#define RADIO_MISO_PIN              3
#define RADIO_MOSI_PIN              6
#define RADIO_CS_PIN                7

// ==================== LoRa LR1121 Control ====================
#define RADIO_RST_PIN               8
#define RADIO_DIO9_PIN              36      // IRQ pin для LR1121
#define RADIO_BUSY_PIN              34

// ==================== SD Card ====================
#define SDCARD_MOSI                 11
#define SDCARD_MISO                 2
#define SDCARD_SCLK                 14
#define SDCARD_CS                   13

// ==================== LED & Button ====================
#define BOARD_LED                   37
#define LED_ON                      HIGH
#define LED_OFF                     LOW
#define BUTTON_PIN                  0       // BOOT button

// ==================== Battery ====================
#define ADC_PIN                     1

// ==================== Конфигурация радио ====================
// Частоты для LR1121:
// Sub-GHz: 150.0 - 960.0 MHz (LoRa/FSK)
// 2.4 GHz: 2400.0 - 2500.0 MHz (LoRa)

#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           868.0   // MHz (EU ISM band)
#endif

#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   14      // dBm (max 22 для Sub-GHz, max 13 для 2.4GHz)
#endif

#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0   // kHz (62.5, 125.0, 250.0, 500.0)
#endif

#ifndef CONFIG_RADIO_SF
#define CONFIG_RADIO_SF             10      // Spreading Factor (5-12)
#endif

#ifndef CONFIG_RADIO_CR
#define CONFIG_RADIO_CR             5       // Coding Rate (5-8)
#endif

#define CONFIG_RADIO_PREAMBLE       16
#define CONFIG_RADIO_SYNC_WORD      0x34    // Private network

#if !defined(ROLE_MASTER) && !defined(ROLE_SLAVE)
#define ROLE_MASTER                 1       // По умолчанию - мастер
#endif

#endif // PINS_CONFIG_H
