#include <CARBOT.h>

CARBOT carBot;

void setup()
{
  carBot.serialStart(115200); // Initialize serial communication
  carBot.begin(); // Initialize the car bot / Araç botunu başlat
  carBot.serialWrite("CARBOT Basic Example Started");
}

void loop()
{
  // Turn on LED headlights / LED farları aç
  carBot.serialWrite("LED ON");
  carBot.controlLED(true);
  delay(2000);

  // Turn off LED headlights / LED farları kapat
  carBot.serialWrite("LED OFF");
  carBot.controlLED(false);
  delay(2000);

  // Move the car forward / Aracı ileri hareket ettir
  carBot.serialWrite("Moving Forward");
  carBot.moveForward();
  delay(2000);

  // Stop the car / Aracı durdur
  carBot.serialWrite("Stopping");
  carBot.stop();
  delay(2000);

  // Steer left / Direksiyonu sola çevir
  carBot.serialWrite("Steering Left");
  carBot.steer(45);
  delay(2000);

  // Center the steering / Direksiyonu ortala
  carBot.serialWrite("Steering Center");
  carBot.steer(90);
  delay(2000);

  // Move the car backward / Aracı geri hareket ettir
  carBot.serialWrite("Moving Backward");
  carBot.moveBackward();
  delay(2000);

  // Stop the car / Aracı durdur
  carBot.serialWrite("Stopping");
  carBot.stop();
  delay(2000);

  // Steer right / Direksiyonu sağa çevir
  carBot.serialWrite("Steering Right");
  carBot.steer(135);
  delay(2000);

  // Center the steering / Direksiyonu ortala
  carBot.serialWrite("Steering Center");
  carBot.steer(90);
  delay(2000);

  // Play a sound with the buzzer / Buzzer çal
  carBot.serialWrite("Beeping");
  carBot.buzzerPlay(1000, 500);
  delay(2000);
}
