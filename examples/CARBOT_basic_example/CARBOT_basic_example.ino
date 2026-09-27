#include <CARBOT.h>

// Uncomment the following line if testing on IOTBOT to enable LCD feedback (IOTBOT ekranında durum görmek için aşağıdaki satırı aktif edin)
// #define USE_IOTBOT_SCREEN

#ifdef USE_IOTBOT_SCREEN
#include <IOTBOT.h>
IOTBOT iotbot;
#endif

#define LED_PIN 16 // Minibot blue LED pin (Minibot mavi led pini)
#define B1_PIN 0   // Minibot built-in button

CARBOT carBot;

enum Mode { IDLE_MODE, DEMO_MODE, AUTO_MODE };
Mode currentMode = IDLE_MODE; // Default is idle / Varsayılan mod durgun (IDLE)
bool demoDone = false;
unsigned long sampleCount = 0;
unsigned long afkTimer = 0; // AFK zamanlayıcısı

// Button logic variables
bool b1State = false;
unsigned long b1Timer = 0;
int b1Clicks = 0;
bool b1LongPressed = false;

// Helper function to print status to Serial and IOTBOT LCD (Seri porta ve IOTBOT LCD'ye durum yazdıran yardımcı fonksiyon)
void showStatus(String title, String detail = "")
{
  if (detail == "") {
    carBot.serialWrite(title);
  } else {
    carBot.serialWrite(title + ": " + detail);
  }

#ifdef USE_IOTBOT_SCREEN
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, title);
  if (detail != "") {
    iotbot.lcdWriteCR(0, 1, detail);
  }
#endif
}

// Sesli bildirim fonksiyonu (Audible Feedback for Modes)
void playModeSound(Mode m) {
    if (m == IDLE_MODE) {
        // IDLE: Durgun mod (Tek kalın tok ses)
        carBot.buzzerPlay(200, 150);
    } else if (m == AUTO_MODE) {
        // AUTO: Otonom mod (Artan iki savaşçı sesi)
        carBot.buzzerPlay(500, 100);
        delay(50);
        carBot.buzzerPlay(800, 150);
    } else if (m == DEMO_MODE) {
        // DEMO: Demo başliyor cıngılı (3 neşeli nota)
        carBot.buzzerPlay(400, 100);
        delay(50);
        carBot.buzzerPlay(500, 100);
        delay(50);
        carBot.buzzerPlay(650, 200);
    }
}

bool isB1Pressed() {
#ifdef USE_IOTBOT_SCREEN
  return iotbot.button1Read();
#else
  return (digitalRead(B1_PIN) == LOW);
#endif
}

bool isB2Pressed() {
#ifdef USE_IOTBOT_SCREEN
  return iotbot.button2Read();
#else
  return false;
#endif
}

// Checks buttons dynamically. Returns true if a mode switch occurred so callers can abort their sequence.
bool checkButtons() {
    bool b1Pressed = isB1Pressed();
    bool modeChanged = false;
    
    // Check for double click timeout (wait 400ms for second click)
    if (!b1Pressed && b1Clicks > 0 && (millis() - b1Timer > 400)) {
        if (b1Clicks == 2) {
            // Double click action: Toggle DEMO_MODE
            if (currentMode == DEMO_MODE) {
                currentMode = IDLE_MODE;
                showStatus("Mod Degisti", "IDLE (Durgun)");
                playModeSound(IDLE_MODE);
            } else {
                currentMode = DEMO_MODE;
                demoDone = false; // Restart demo
                showStatus("Mod Degisti", "DEMO MODE");
                playModeSound(DEMO_MODE);
            }
            carBot.stop();
            afkTimer = millis(); // Kullanıcı aktifliği AFK'yi sıfırlar
            modeChanged = true;
        }
        b1Clicks = 0; // Reset clicks
    }

    if (b1Pressed && !b1State) {
        // Button just pressed
        b1State = true;
        b1Timer = millis();
        b1LongPressed = false;
        afkTimer = millis(); // Kullanıcı aktifliği AFK'yi sıfırlar
    } else if (b1Pressed && b1State) {
        // Button held
        if (!b1LongPressed && (millis() - b1Timer > 1000)) {
            b1LongPressed = true;
            // Hold action: Toggle AUTO_MODE
            if (currentMode == AUTO_MODE) {
               currentMode = IDLE_MODE;
               showStatus("Mod Degisti", "IDLE (Durgun)");
               playModeSound(IDLE_MODE);
            } else {
               currentMode = AUTO_MODE;
               showStatus("Mod Degisti", "AUTO (Otonom)");
               playModeSound(AUTO_MODE);
            }
            carBot.stop();
            modeChanged = true;
            b1Clicks = 0; // Cancel any pending clicks
        }
    } else if (!b1Pressed && b1State) {
        // Button just released
        b1State = false;
        if (!b1LongPressed) {
            b1Clicks++;
            b1Timer = millis(); // Refresh timer for double click window
        }
    }

    // For IOTBOT users: B2 functions as a panic button returning to IDLE
    bool b2Pressed = isB2Pressed();
    if (b2Pressed && currentMode != IDLE_MODE) {
        currentMode = IDLE_MODE;
        showStatus("Mod Degisti", "IDLE (Durgun)");
        playModeSound(IDLE_MODE);
        carBot.stop();
        afkTimer = millis();
        modeChanged = true;
    }

    return modeChanged;
}

// Macro to replace delay() with a non-blocking delay that periodically checks for button presses
#define SMART_DELAY(ms) \
  do { \
    unsigned long s = millis(); \
    while(millis() - s < ms) { \
      if (checkButtons()) return; \
      delay(50); \
    } \
  } while(0)

void runFullCarbotDemo()
{
  showStatus("CARBOT LED", "ON");
  digitalWrite(LED_PIN, HIGH);
  carBot.controlLED(true);
  SMART_DELAY(2000); if(currentMode != DEMO_MODE) return;

  showStatus("CARBOT LED", "OFF");
  digitalWrite(LED_PIN, LOW);
  carBot.controlLED(false);
  SMART_DELAY(2000); if(currentMode != DEMO_MODE) return;

  showStatus("Hareket", "Ileri (Forward)");
  digitalWrite(LED_PIN, HIGH);
  carBot.moveForward();
  SMART_DELAY(2000); if(currentMode != DEMO_MODE) return;

  showStatus("Hareket", "Durdu");
  digitalWrite(LED_PIN, LOW);
  carBot.stop();
  SMART_DELAY(2000); if(currentMode != DEMO_MODE) return;

  showStatus("Direksiyon", "Sola (135 derece)");
  digitalWrite(LED_PIN, HIGH);
  carBot.steer(135);
  SMART_DELAY(2000); if(currentMode != DEMO_MODE) return;

  showStatus("Direksiyon", "Merkez (90)");
  digitalWrite(LED_PIN, LOW);
  carBot.steer(90);
  SMART_DELAY(2000); if(currentMode != DEMO_MODE) return;

  showStatus("Hareket", "Geri (Backward)");
  digitalWrite(LED_PIN, HIGH);
  carBot.moveBackward();
  for (int i = 0; i < 8; i++) {
      carBot.buzzerPlay(1200, 100);
      SMART_DELAY(150); if(currentMode != DEMO_MODE) return;
  }

  showStatus("Hareket", "Durdu");
  digitalWrite(LED_PIN, LOW);
  carBot.stop();
  SMART_DELAY(2000); if(currentMode != DEMO_MODE) return;

  showStatus("Direksiyon", "Saga (45 derece)");
  digitalWrite(LED_PIN, HIGH);
  carBot.steer(45);
  SMART_DELAY(2000); if(currentMode != DEMO_MODE) return;

  showStatus("Direksiyon", "Merkez (90)");
  digitalWrite(LED_PIN, LOW);
  carBot.steer(90);
  SMART_DELAY(2000); if(currentMode != DEMO_MODE) return;

  showStatus("Buzzer", "Uyari Sesi");
  digitalWrite(LED_PIN, HIGH);
  carBot.buzzerPlay(1000, 500);
  digitalWrite(LED_PIN, LOW);
  SMART_DELAY(1500); if(currentMode != DEMO_MODE) return;

  showStatus("Buzzer", "Istiklal Marsi");
  digitalWrite(LED_PIN, HIGH);
  carBot.istiklalMarsiCal();
  digitalWrite(LED_PIN, LOW);
}

void runAutoMode() 
{
  float distance = carBot.readUltrasonicCM();
  
  if (distance > 0) {
      showStatus("Otonom Mesafe", String(distance, 1) + " cm");
  } else {
      showStatus("Otonom Mesafe", "Sensorden okuma yok");
  }
  
  if (distance > 0 && distance < 15.0) {
      showStatus("Engel Tespit!", "Mesafe: " + String(distance, 1) + "cm");
      digitalWrite(LED_PIN, HIGH);
      
      // Dur
      carBot.stop();
      SMART_DELAY(500); if(currentMode != AUTO_MODE) return;
      
      // Biraz geri git. Giderken mesafenin 30cm uzerine cikip cikmadigini denetle. Kamyon sesi cikar.
      showStatus("Kacis Manevrasi", "Geri Gidiliyor");
      carBot.steer(90);
      carBot.moveBackward();
      
      int backCount = 0;
      int maxBackSteps = 12; // En fazla ~3 saniye dener (zorlamaması icin timeout)
      while (backCount < maxBackSteps) {
          carBot.buzzerPlay(1200, 100);
          SMART_DELAY(150); if(currentMode != AUTO_MODE) return;
          
          float currentDist = carBot.readUltrasonicCM();
          if (currentDist > 30.0) {
              // Yeterli donus mesafesi (30cm) elde edildiyse geri gitmeyi aninda kes
              break; 
          }
          backCount++;
      }
      
      // Akıllı Kaçış: Rastgele sağa veya sola dön
      int rndTurn = random(0, 2); // 0 veya 1
      int turnAngle = (rndTurn == 0) ? 135 : 45; // Carbotta 135=Sol, 45=Sag
      String yonStr = (turnAngle == 135) ? "Sol (135)" : "Sag (45)";
      
      showStatus("Kacis Manevrasi", "Donus: " + yonStr);
      carBot.steer(turnAngle);
      carBot.moveForward();
      SMART_DELAY(1500); if(currentMode != AUTO_MODE) return;
      
      // Direksiyonu topla
      carBot.steer(90);
      digitalWrite(LED_PIN, LOW);
  } else {
      // Engel yoksa düz git
      showStatus("Otonom", "Ileri Gidiliyor");
      carBot.steer(90);
      carBot.moveForward();
      
      // Sensörü anlık okuyabilmek için kısa parça bekleme
      SMART_DELAY(150); if(currentMode != AUTO_MODE) return;
  }
}

void runIdleMode() 
{
  float distance = carBot.readUltrasonicCM();
  if (distance < 0) {
    showStatus("IDLE Mesafe", "Okuma yok");
  } else {
    showStatus("IDLE Mesafe", String(distance, 1) + " cm");
  }

  // AFK (Durgunluk) Kontrolü - 15 saniyeden uzun süredir butona falan basılmadıysa
  if (millis() - afkTimer > 15000) {
    showStatus("Durum", "AFK! Buradayim.");
    
    // Küçük iki hızlı flaşör ile robot kendine dikkat çeker
    digitalWrite(LED_PIN, HIGH);
    carBot.controlLED(true);
    SMART_DELAY(75); if(currentMode != IDLE_MODE) return;
    digitalWrite(LED_PIN, LOW);
    carBot.controlLED(false);
    SMART_DELAY(75); if(currentMode != IDLE_MODE) return;
    digitalWrite(LED_PIN, HIGH);
    carBot.controlLED(true);
    SMART_DELAY(75); if(currentMode != IDLE_MODE) return;
    digitalWrite(LED_PIN, LOW);
    carBot.controlLED(false);
    
    // Minimal uyanış sesi (uyarıcı ping)
    carBot.buzzerPlay(1500, 50);
    
    afkTimer = millis(); // Bir 15 saniye daha geri saymak üzere zamanlayıcıyı tazeler
  }

  // Cihaz durgun kalmalı
  carBot.stop();
  carBot.steer(90);

  // Buton dinlemeyi seri tutmak için küçük süreli delay
  SMART_DELAY(250);
}

void setup()
{
  carBot.serialStart(115200);
  carBot.begin();
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(B1_PIN, INPUT_PULLUP);

#ifdef USE_IOTBOT_SCREEN
  iotbot.begin();
#endif
  
  // Basit rastgelelik tohumlaması (Random Seed)
  randomSeed(analogRead(14)); // Boş olması muhtemel kalibre pini örneği

  // Açılış Durumu
  showStatus("CARBOT Ready", "IDLE MODE (Durgun)");
  playModeSound(IDLE_MODE); // Açılış onayı sesi
  afkTimer = millis(); 
}

void loop()
{
  // Hangi durum çalışırsa çalışsın butona basıldıysa checkButtons içinden mod güncellenecek
  if (checkButtons()) return;

  if (currentMode == DEMO_MODE) {
    if (!demoDone) {
      runFullCarbotDemo();
      demoDone = true;
      // Demo bittikten sonra tekrar durgun moda geçer
      if (currentMode == DEMO_MODE) {
        currentMode = IDLE_MODE;
        afkTimer = millis(); 
        showStatus("Demo Bitti", "IDLE (Durgun)");
        playModeSound(IDLE_MODE);
      }
    }
  } 
  else if (currentMode == AUTO_MODE) {
    runAutoMode();
  }
  else if (currentMode == IDLE_MODE) {
    runIdleMode();
  }
}
