// TR: GERCEK PROJE - Bekci Arac Modu. CARBOT bu sefer surmez, YERINDE
// DURUR ve on taraftaki ultrasonik sensoruyle etrafini "gozetler". Bir
// sey (ornegin bir el) belirlenen mesafeden daha yakina gelirse: farlar
// hizla yanip soner, korna (buzzer) alarm calar ve arac KISA BIR SURE
// GERI KACAR - tipki ürkmüş bir hayvan gibi. Tehlike gectikten sonra
// tekrar nobetine doner.
// EN: A REAL PROJECT - Guard Car Mode. This time CARBOT doesn't drive
// around - it STAYS PUT and "watches" its surroundings with the front
// ultrasonic sensor. If something (e.g. a hand) gets closer than the set
// distance: the headlights flash rapidly, the horn (buzzer) sounds an
// alarm, and the car BACKS AWAY BRIEFLY - just like a startled animal.
// Once the danger passes, it goes back to watch duty.

#include <CARBOT.h>

#define LED_PIN 16 // Minibot mavi LED pini / Minibot blue LED pin
#define B1_PIN 0   // Minibot uzerindeki dahili buton (nobeti ac/kapa) / Minibot's built-in button (arm/disarm guard duty)

CARBOT carBot;

// TR/EN: Bu degeri false yapip yeniden yukleyerek dili degistirebilirsiniz.
// Change this to false and re-upload to switch the language.
bool turkish = true;

namespace {
  constexpr float kIntruderDistanceCm = 25.0f; // Bu mesafenin altinda alarm / alarm triggers below this distance
  bool guarding = true;
  bool lastButtonState = true; // digitalRead: HIGH = birakilmis / released

  void say(const String &tr, const String &en) {
    carBot.serialWrite(turkish ? tr : en);
  }
}

void setup() {
  carBot.serialStart(115200);
  carBot.begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(B1_PIN, INPUT_PULLUP);

  carBot.stop();
  carBot.steer(90);
  say("Bekci modu AKTIF. Nobeti acmak/kapatmak icin butona basin.",
      "Guard mode ACTIVE. Press the button to arm/disarm.");
}

void loop() {
  bool buttonState = (digitalRead(B1_PIN) == HIGH);

  if (lastButtonState == true && buttonState == false) {
    guarding = !guarding;
    digitalWrite(LED_PIN, LOW);
    carBot.controlLED(false);
    say(guarding ? "Nobet basladi." : "Nobet durduruldu.",
        guarding ? "Guard duty started." : "Guard duty stopped.");
    carBot.buzzerPlay(guarding ? 1200 : 700, 150);
    delay(300); // debounce
  }
  lastButtonState = buttonState;

  if (!guarding) {
    delay(50);
    return;
  }

  float distance = carBot.readUltrasonicCM();
  bool intruderDetected = (distance > 0 && distance < kIntruderDistanceCm);

  if (intruderDetected) {
    say("ALARM! Bir sey yaklasti, mesafe: " + String(distance, 1) + "cm",
        "ALARM! Something got close, distance: " + String(distance, 1) + "cm");

    // Farlari hizla yanip sondur + korna cal / flash headlights + sound the horn
    for (int i = 0; i < 4; ++i) {
      digitalWrite(LED_PIN, HIGH);
      carBot.controlLED(true);
      carBot.buzzerPlay(1500, 80);
      delay(80);
      digitalWrite(LED_PIN, LOW);
      carBot.controlLED(false);
      delay(80);
    }

    // Kisa bir sure geri kac / back away briefly
    carBot.steer(90);
    carBot.moveBackward();
    delay(600);
    carBot.stop();

    delay(1000); // Sakinlesmek icin kisa bir bekleme / brief pause to settle down
  } else {
    say("Nobette... mesafe: " + String(distance > 0 ? String(distance, 1) : String("okuma yok")) + "cm",
        "On watch... distance: " + String(distance > 0 ? String(distance, 1) : String("no reading")) + "cm");
    delay(300);
  }
}
