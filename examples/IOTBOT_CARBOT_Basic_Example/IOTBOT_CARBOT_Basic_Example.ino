#include <IOTBOT.h>
#include <CARBOT.h>

IOTBOT iotbot;
CARBOT carbot;

void setup() {
  iotbot.begin();
  carbot.begin();
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "IOTBOT CARBOT");
  iotbot.lcdWriteCR(0, 1, "Basic Example");
}

void loop() {
  // Simple movement demo
  carbot.moveForward();
  delay(1000);
  carbot.stop();
  delay(1000);
  carbot.moveBackward();
  delay(1000);
  carbot.stop();
  delay(1000);
  
  // Steering
  carbot.steer(45); // Left
  delay(1000);
  carbot.steer(135); // Right
  delay(1000);
  carbot.steer(90); // Center
  delay(1000);
}
