
#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

/*
 * =================================================================================
 *      Конфигурация пинов для платы LilyGO T3-S3 (проверено по схеме и рабочему коду)
 * =================================================================================
 */

// ==================== I2C (OLED SSD1306) ====================
#define I2C_SDA                     18
#define I2C_SCL                     17

// ==================== SPI (LoRa LR1121) ====================
#define RADIO_SCLK_PIN              5   // SPI Clock
#define RADIO_MISO_PIN              3   // Master In, Slave Out
#define RADIO_MOSI_PIN              6   // Master Out, Slave In
#define RADIO_CS_PIN                7   // Chip Select (NSS)

// ==================== LoRa LR1121 Control ====================
#define RADIO_RST_PIN               8   // Пин сброса (Reset)
#define RADIO_DIO9_PIN              36  // Пин прерывания (IRQ), на схеме как DIO1
#define RADIO_BUSY_PIN              34  // Пин статуса "занят" (Busy)

// ==================== Управление антенным переключателем (RF Switch) ====================
// ИСПРАВЛЕННЫЕ ПИНЫ И ЛОГИКА на основе рабочего примера
//
// Логика работы (для T3-S3 v1.2):
// - Для работы в диапазоне Sub-GHz (868 МГц) через SMA-коннектор:
//   - ANT_SW_VDD (GPIO21) должен быть HIGH (питание)
//   - ANT_SW_RX  (GPIO10) должен быть LOW
//   - ANT_SW_TX  (GPIO9)  должен быть LOW
//
#define ANT_SW_VDD                  21  // Питание для RF-переключателя
#define ANT_SW_RX                   10  // Управляющий пин 1
#define ANT_SW_TX                   9   // Управляющий пин 2

// ==================== LED & Button ====================
#define BOARD_LED                   37  // Пользовательский светодиод (синий)
#define BUTTON_PIN                  0   // Кнопка BOOT/USER

#endif // PINS_CONFIG_H
