#include <CARBOT.h>

CARBOT carBot;

void runFullCarbotDemo()
{
  carBot.serialWrite("LED ON");
  carBot.controlLED(true);
  delay(2000);

  carBot.serialWrite("LED OFF");
  carBot.controlLED(false);
  delay(2000);

  carBot.serialWrite("Moving Forward");
  carBot.moveForward();
  delay(2000);

  carBot.serialWrite("Stopping");
  carBot.stop();
  delay(2000);

  carBot.serialWrite("Steering Left");
  carBot.steer(45);
  delay(2000);

  carBot.serialWrite("Steering Center");
  carBot.steer(90);
  delay(2000);

  carBot.serialWrite("Moving Backward");
  carBot.moveBackward();
  delay(2000);

  carBot.serialWrite("Stopping");
  carBot.stop();
  delay(2000);

  carBot.serialWrite("Steering Right");
  carBot.steer(135);
  delay(2000);

  carBot.serialWrite("Steering Center");
  carBot.steer(90);
  delay(2000);

  carBot.serialWrite("Beeping");
  carBot.buzzerPlay(1000, 500);
  delay(2000);

  carBot.serialWrite("Istiklal Marsi");
  carBot.istiklalMarsiCal();
}

void setup()
{
  carBot.serialStart(115200);
  carBot.begin();
  carBot.serialWrite("CARBOT Full Test Started");
}

void loop()
{
  static bool demoDone = false;
  static unsigned long sampleCount = 0;

  if (!demoDone)
  {
    runFullCarbotDemo();
    demoDone = true;
    carBot.serialWrite("Ultrasonik okuma basladi (otomatik mod).");
  }

  float distance = carBot.readUltrasonicCM();
  if (distance < 0)
  {
    carBot.serialWrite("Ultrasonik sensorden okuma yok.");
  }
  else
  {
    carBot.serialWrite(String("Mesafe: ") + String(distance, 1) + " cm");
  }

  sampleCount++;
  if (sampleCount % 10 == 0)
  {
    carBot.serialWrite("LED/Buzzer test (ultrasonik otomatik kapanacak).");
    carBot.controlLED(true);
    delay(200);
    carBot.controlLED(false);
    carBot.buzzerPlay(1000, 200);
  }

  delay(500);
}
