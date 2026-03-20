#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

// ==================== I2C (OLED SSD1306) ====================
#define I2C_SDA                     18
#define I2C_SCL                     17

// ==================== SPI (LoRa LR1121) ====================
#define RADIO_SCLK_PIN              5
#define RADIO_MISO_PIN              3
#define RADIO_MOSI_PIN              6
#define RADIO_CS_PIN                7

// ==================== LoRa LR1121 Control ====================
#define RADIO_RST_PIN               8
// ПРОВЕРКА: Пин прерывания (IRQ) для LR1121.
// В логах видно, что прерывания срабатывают, значит пин 36 для DIO9 настроен корректно.
#define RADIO_DIO9_PIN              36      // IRQ pin
#define RADIO_BUSY_PIN              34

// ==================== Управление антенным переключателем ====================
#define ANT_SW_VDD                  21
#define ANT_SW_RX                   10
#define ANT_SW_TX                   9

// ==================== LED & Button ====================
#define BOARD_LED                   37
#define BUTTON_PIN                  0       // BOOT button

#endif // PINS_CONFIG_H