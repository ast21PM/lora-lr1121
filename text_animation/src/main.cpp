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
     
     // Держим до конца цикла ~3200ms
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
 
 void f1Car(int x) {
     // Болид F1 вид сбоку (более реалистичный)
     
     // Заднее антикрыло (высокое)
     display.drawBox(x - 8, 22, 2, 10);     // Стойка
     display.drawBox(x - 12, 20, 12, 2);    // Плоскость крыла
     
     // Корпус (сужается к носу)
     display.drawBox(x - 5, 32, 8, 5);      // Задняя часть (мотор)
     display.drawBox(x + 3, 33, 15, 4);     // Средняя часть
     display.drawBox(x + 18, 34, 12, 3);    // Нос (сужается)
     display.drawBox(x + 30, 35, 6, 2);     // Кончик носа
     
     // Кокпит + Гало
     display.drawBox(x + 5, 29, 6, 4);      // Кокпит
     display.drawFrame(x + 4, 27, 8, 3);    // Гало (рамка)
     
     // Понтоны (боковины)
     display.drawBox(x + 2, 37, 12, 2);     // Днище
     
     // Переднее крыло (широкое)
     display.drawBox(x + 28, 38, 10, 2);    // Плоскость
     display.drawBox(x + 32, 36, 2, 2);     // Стойка
     
     // Колёса (побольше)
     display.drawDisc(x, 40, 4);            // Заднее
     display.drawDisc(x + 28, 40, 3);       // Переднее
     
     // Воздухозаборник
     display.drawBox(x + 8, 26, 3, 3);
 }
 
 void redBullLogo(int ox, int oy) {
     // Red Bull logo из пиксель-арта
     // Масштаб 1:1, размер ~50x25 пикселей
     
     // Солнце (центр) - круг с лучами
     display.drawDisc(ox + 25, oy + 8, 7);  // Основной круг
     // Лучи солнца
     display.drawPixel(ox + 25, oy - 1);
     display.drawPixel(ox + 18, oy + 1);
     display.drawPixel(ox + 32, oy + 1);
     display.drawPixel(ox + 16, oy + 5);
     display.drawPixel(ox + 34, oy + 5);
     display.drawPixel(ox + 15, oy + 10);
     display.drawPixel(ox + 35, oy + 10);
     
     // Левый бык
     // Рога
     display.drawLine(ox + 2, oy + 8, ox + 5, oy + 5);
     display.drawLine(ox + 5, oy + 5, ox + 7, oy + 7);
     display.drawPixel(ox + 1, oy + 7);
     display.drawPixel(ox, oy + 8);
     
     // Голова левого быка
     display.drawBox(ox + 5, oy + 9, 4, 3);
     display.drawPixel(ox + 4, oy + 10);
     
     // Тело левого быка
     display.drawBox(ox + 8, oy + 10, 10, 6);
     display.drawBox(ox + 6, oy + 12, 3, 4);
     display.drawBox(ox + 17, oy + 12, 4, 5);
     
     // Ноги левого быка
     display.drawBox(ox + 7, oy + 16, 2, 4);
     display.drawBox(ox + 12, oy + 16, 2, 4);
     display.drawPixel(ox + 6, oy + 19);
     display.drawPixel(ox + 11, oy + 19);
     
     // Хвост левого быка
     display.drawLine(ox + 5, oy + 14, ox + 3, oy + 17);
     display.drawPixel(ox + 2, oy + 18);
     
     // Правый бык (зеркально)
     // Рога
     display.drawLine(ox + 48, oy + 8, ox + 45, oy + 5);
     display.drawLine(ox + 45, oy + 5, ox + 43, oy + 7);
     display.drawPixel(ox + 49, oy + 7);
     display.drawPixel(ox + 50, oy + 8);
     
     // Голова правого быка
     display.drawBox(ox + 41, oy + 9, 4, 3);
     display.drawPixel(ox + 46, oy + 10);
     
     // Тело правого быка
     display.drawBox(ox + 32, oy + 10, 10, 6);
     display.drawBox(ox + 41, oy + 12, 3, 4);
     display.drawBox(ox + 29, oy + 12, 4, 5);
     
     // Ноги правого быка
     display.drawBox(ox + 41, oy + 16, 2, 4);
     display.drawBox(ox + 36, oy + 16, 2, 4);
     display.drawPixel(ox + 43, oy + 19);
     display.drawPixel(ox + 38, oy + 19);
     
     // Хвост правого быка
     display.drawLine(ox + 45, oy + 14, ox + 47, oy + 17);
     display.drawPixel(ox + 48, oy + 18);
 }
 
 void f1CarFront(int y) {
     // Болид F1 вид спереди
     int cx = 64;  // Центр экрана
     
     // Колёса (широкие)
     display.drawBox(cx - 35, y + 10, 8, 16);   // Левое колесо
     display.drawBox(cx + 27, y + 10, 8, 16);   // Правое колесо
     
     // Подвеска
     display.drawLine(cx - 27, y + 18, cx - 15, y + 15);  // Левая
     display.drawLine(cx + 27, y + 18, cx + 15, y + 15);  // Правая
     
     // Корпус
     display.drawBox(cx - 15, y + 8, 30, 18);   // Основной
     
     // Понтоны
     display.drawBox(cx - 22, y + 12, 7, 10);   // Левый
     display.drawBox(cx + 15, y + 12, 7, 10);   // Правый
     
     // Кокпит (тёмный)
     display.drawFrame(cx - 5, y + 5, 10, 8);   // Гало
     
     // Воздухозаборник
     display.drawBox(cx - 3, y, 6, 6);
     
     // Переднее крыло
     display.drawBox(cx - 30, y + 26, 60, 3);
     display.drawBox(cx - 2, y + 22, 4, 4);     // Нос
 }
 
 void carAnimation() {
     // Машинка едет сбоку
     for (int x = -40; x < 130; x += 5) {
         display.clearBuffer();
         f1Car(x);
         
         // Выхлоп
         if (x > 0) {
             display.drawPixel(x - 15, 35);
             display.drawPixel(x - 20, 34);
             display.drawPixel(x - 24, 36);
         }
         
         display.sendBuffer();
         delay(25);
     }
 }
 
 void logoAnimation() {
     // Лого Red Bull появляется
     for (int y = -30; y <= 20; y += 2) {
         display.clearBuffer();
         redBullLogo(39, y);  // Центрируем (128-50)/2 = 39
         display.sendBuffer();
         delay(30);
     }
     delay(1500);
 }
 
 void carFrontAnimation() {
     // Машинка едет на нас (приближается)
     for (int scale = 0; scale < 20; scale++) {
         display.clearBuffer();
         f1CarFront(10 + scale);
         display.sendBuffer();
         delay(80);
     }
 }
 
 void winner() {
     // Сначала машинка сбоку
     carAnimation();
     
     // Потом лого Red Bull
     logoAnimation();
     
     // Потом машинка спереди едет на нас
     carFrontAnimation();
     
     // Финал — VER World Champion
     display.clearBuffer();
     display.setFont(u8g2_font_ncenB14_tr);
     
     int w1 = display.getStrWidth("VER");
     display.setCursor((128 - w1) / 2, 25);
     display.print("VER");
     
     display.setFont(u8g2_font_ncenB10_tr);
     int w2 = display.getStrWidth("World Champion");
     display.setCursor((128 - w2) / 2, 50);
     display.print("World Champion");
     
     display.sendBuffer();
     
     for (int i = 0; i < 10; i++) {
         flash();
         delay(150);
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
     delay(5000);
 }