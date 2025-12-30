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

// ==================== Конфигурация радио (значения по умолчанию) ====================
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           2450.0  // MHz (2.4 GHz band)
#endif

#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   10      // dBm
#endif

#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             406.25  // kHz
#endif

#ifndef CONFIG_RADIO_SF
#define CONFIG_RADIO_SF             9       // Spreading Factor
#endif

#ifndef CONFIG_RADIO_CR
#define CONFIG_RADIO_CR             5       // Coding Rate 4/5
#endif

#define CONFIG_RADIO_PREAMBLE       16
#define CONFIG_RADIO_SYNC_WORD      0x34    // Private network

// ==================== Идентификатор датчика ====================
#ifndef SENSOR_ID
#define SENSOR_ID                   "S01"   // По умолчанию S01
#endif

#endif // PINS_CONFIG_H
