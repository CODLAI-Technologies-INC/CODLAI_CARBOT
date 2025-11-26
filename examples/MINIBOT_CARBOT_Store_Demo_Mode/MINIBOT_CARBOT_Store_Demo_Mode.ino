/*
 * CARBOT Store Demo Mode
 * 
 * This example demonstrates the capabilities of the Carbot in a loop.
 * It is intended for display or testing purposes.
 * 
 * Features:
 * - Forward/Backward movement
 * - Steering Left/Right
 * - LED blinking
 * - Horn/Buzzer usage
 */

#include <MINIBOT.h>
#include <CARBOT.h>

MINIBOT minibot;
CARBOT carbot;

void setup() {
  minibot.begin();
  carbot.begin();
  
  // Intro sound
  carbot.buzzerPlay(1000, 100);
  delay(100);
  carbot.buzzerPlay(1500, 100);
}

void loop() {
  // 1. Forward / İleri
  carbot.steer(90); // Center / Ortala
  carbot.controlLED(true);
  carbot.moveForward();
  delay(1000);
  carbot.stop();
  carbot.controlLED(false);
  delay(500);

  // 2. Backward / Geri
  carbot.controlLED(true);
  carbot.moveBackward();
  delay(1000);
  carbot.stop();
  carbot.controlLED(false);
  delay(500);

  // 3. Steer Left & Forward / Sola Dön & İleri
  carbot.steer(45);
  carbot.moveForward();
  delay(1000);
  carbot.stop();
  delay(200);

  // 4. Steer Right & Backward / Sağa Dön & Geri
  carbot.steer(135);
  carbot.moveBackward();
  delay(1000);
  carbot.stop();
  delay(200);
  
  // Center Steering / Direksiyonu Ortala
  carbot.steer(90);

  // 5. Horn & LED Show / Korna ve LED Şov
  for(int i=0; i<3; i++) {
    carbot.controlLED(true);
    carbot.buzzerPlay(2000, 50);
    delay(100);
    carbot.controlLED(false);
    delay(100);
  }
  
  delay(2000); // Wait before repeating / Tekrarlamadan önce bekle
}
