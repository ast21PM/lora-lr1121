/*
 * SUPER MAX - OLED Demo
 * Тайминги по реальной песне!
 * 
 * Просто весёлая демка для LilyGO T3-S3
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <U8g2lib.h>
 
 #define I2C_SDA   18
 #define I2C_SCL   17
 #define BOARD_LED 37
 
 U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
 
 void setup() {
     Serial.begin(115200);
     pinMode(BOARD_LED, OUTPUT);
     
     Wire.begin(I2C_SDA, I2C_SCL);
     display.begin();
     display.enableUTF8Print();
     
     Serial.println(F("SUPER MAX DEMO!"));
 }
 
 void flash() {
     digitalWrite(BOARD_LED, HIGH);
     delay(30);
     digitalWrite(BOARD_LED, LOW);
 }
 
 void showText(const char* text, bool big = false) {
     display.clearBuffer();
     
     if (big) {
         display.setFont(u8g2_font_ncenB18_tr);
     } else {
         display.setFont(u8g2_font_ncenB12_tr);
     }
     
     int width = display.getStrWidth(text);
     int x = (128 - width) / 2;
     int y = big ? 45 : 40;
     
     display.setCursor(x, y);
     display.print(text);
     display.sendBuffer();
     flash();
 }
 
 void clear() {
     display.clearBuffer();
     display.sendBuffer();
 }
 
 // Один цикл песни по твоим onset'ам
 // Onset 0: 69.7 ms    - TU
 // Onset 1: 139.3 ms   - TU  
 // Onset 2: 510.8 ms   - DU
 // Onset 3: 859.1 ms   - DU
 // Onset 5: 2043.4 ms  - MAX
 // Onset 6: 2229.1 ms  - VER
 // Onset 7: 2275.5 ms  - STAP
 // Onset 8: 2716.7 ms  - PEN
 
 void superMaxCycle() {
     unsigned long start = millis();
     
     // TU @ 70ms
     while(millis() - start < 70) delay(1);
     showText("TU", true);
     
     // TU @ 139ms
     while(millis() - start < 139) delay(1);
     showText("TU", true);
     
     // Пауза
     while(millis() - start < 400) delay(1);
     clear();
     
     // DU @ 511ms
     while(millis() - start < 511) delay(1);
     showText("DU", true);
     
     // DU @ 859ms
     while(millis() - start < 859) delay(1);
     showText("DU", true);
     
     // Пауза перед MAX
     while(millis() - start < 1500) delay(1);
     clear();
     
     // MAX @ 2043ms
     while(millis() - start < 2043) delay(1);
     showText("MAX", true);
     
     // VERSTAPPEN @ 2229ms
     while(millis() - start < 2229) delay(1);
     showText("VERSTAPPEN!", false);
     
     // Держим до конца цикла ~3400ms (следующий onset 11)
     while(millis() - start < 3200) delay(1);
     clear();
     
     // === ВТОРОЙ КУПЛЕТ ===
     // Onset 11: 3436.6 ms - TU
     // Onset 12: 3901.0 ms - сдвигаем относительно
     
     start = millis(); // Новый отсчёт
     
     // TU 
     while(millis() - start < 70) delay(1);
     showText("TU", true);
     
     // TU
     while(millis() - start < 139) delay(1);
     showText("TU", true);
     
     while(millis() - start < 400) delay(1);
     clear();
     
     // DU
     while(millis() - start < 511) delay(1);
     showText("DU", true);
     
     // DU
     while(millis() - start < 859) delay(1);
     showText("DU", true);
     
     while(millis() - start < 1500) delay(1);
     clear();
     
     // MAX VERSTAPPEN
     while(millis() - start < 2043) delay(1);
     showText("MAX", true);
     
     while(millis() - start < 2229) delay(1);
     showText("VERSTAPPEN!", false);
     
     while(millis() - start < 3200) delay(1);
     clear();
 }
 
 void intro() {
     display.clearBuffer();
     display.setFont(u8g2_font_ncenB12_tr);
     display.setCursor(8, 25);
     display.print("SUPER MAX!");
     
     display.setFont(u8g2_font_6x10_tf);
     display.setCursor(15, 50);
     display.print("Press BOOT to start");
     
     display.sendBuffer();
 }
 
 void winner() {
     display.clearBuffer();
     display.setFont(u8g2_font_ncenB12_tr);
     display.setCursor(25, 25);
     display.print("WINNER!");
     
     display.setFont(u8g2_font_ncenB10_tr);
     display.setCursor(15, 50);
     display.print("#1 VERSTAPPEN");
     
     display.sendBuffer();
     
     for (int i = 0; i < 10; i++) {
         flash();
         delay(100);
     }
 }
 
 void loop() {
     intro();
     
     // Ждём нажатия кнопки BOOT для старта
     while (digitalRead(0) == HIGH) {
         delay(10);
     }
     delay(200); // debounce
     
     // 4 повтора
     for (int i = 0; i < 4; i++) {
         superMaxCycle();
     }
     
     winner();
     delay(3000);
 }
 