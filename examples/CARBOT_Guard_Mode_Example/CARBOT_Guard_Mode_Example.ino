/*
 * TR: GERÇEK PROJE - Bekçi Araç Modu (Otomatik nöbet + Manuel sürüş)
 *  - OTOMATİK mod = NÖBET: CARBOT sürmez, YERİNDE DURUR ve öndeki ultrasonik
 *    sensörüyle etrafını "gözetler". Bir şey (örneğin bir el) belirlenen
 *    mesafeden daha yakına gelirse: farlar hızla yanıp söner, korna (buzzer)
 *    alarm çalar ve araç KISA BİR SÜRE GERİ KAÇAR - tıpkı ürkmüş bir hayvan
 *    gibi. Tehlike geçince tekrar nöbetine döner. Açılışta nöbet başlar.
 *  - MINIBOT üzerindeki butona (B1 / GPIO0) basınca nöbet biter ve MANUEL
 *    moda geçer: aracı seri komutlarla sürersiniz. Butona tekrar basınca nöbet
 *    yeniden başlar.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim     / help          -> komut listesi
 *      oto        / auto          -> nöbeti başlat (otomatik mod)
 *      manuel     / manual        -> nöbeti bitir (manuel mod)
 *      esik 25    / threshold 25  -> alarm mesafesi (cm, 5-200)
 *      mesafe     / distance      -> mesafeyi şimdi ölç
 *      ileri 2    / forward 2     -> 2 saniye ileri (en fazla 10)
 *      geri 2     / back 2        -> 2 saniye geri
 *      sol / sag / duz  (left / right / straight) -> direksiyon
 *      hiz 180    / speed 180     -> sürüş hızı (PWM 0-255)
 *      dur        / stop          -> HER ŞEYİ HEMEN DURDUR (manuel moda geçer)
 *      dil        / lang          -> dili değiştir (Türkçe <-> English)
 *
 * EN: A REAL PROJECT - Guard Car Mode (Automatic watch + Manual driving)
 *  - AUTO mode = GUARD DUTY: CARBOT doesn't drive around - it STAYS PUT and
 *    "watches" its surroundings with the front ultrasonic sensor. If something
 *    (e.g. a hand) gets closer than the set distance: the headlights flash
 *    rapidly, the horn (buzzer) sounds an alarm, and the car BACKS AWAY
 *    BRIEFLY - just like a startled animal. Once the danger passes, it goes back
 *    to watch duty. Guard duty starts at power-up.
 *  - Press the button on the MINIBOT (B1 / GPIO0) to end guard duty and switch
 *    to MANUAL mode: you drive the car with serial commands. Press the button
 *    again to restart guard duty.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help        / yardim       -> command list
 *      auto        / oto          -> start guard duty (auto mode)
 *      manual      / manuel       -> end guard duty (manual mode)
 *      threshold 25 / esik 25     -> alarm distance (cm, 5-200)
 *      distance    / mesafe       -> measure the distance now
 *      forward 2   / ileri 2      -> forward for 2 seconds (max 10)
 *      back 2      / geri 2       -> backward for 2 seconds
 *      left / right / straight (sol / sag / duz) -> steering
 *      speed 180   / hiz 180      -> driving speed (PWM 0-255)
 *      stop        / dur          -> STOP EVERYTHING NOW (switches to manual)
 *      lang        / dil          -> switch language (Turkish <-> English)
 *
 * NOT / NOTE: Ultrasonik sensör far ve korna ile aynı pinleri kullanır; alarm
 * sırasında kütüphane otomatik geçiş yapar ve seri porta kısa bir uyarı yazar.
 * / The ultrasonic sensor shares its pins with the headlights and horn; during an
 * alarm the library switches automatically and prints a short warning.
 *
 * Bağlantı / Wiring: CARBOT'un üzerindeki MINIBOT'a yükleyin (ESP8266).
 *   Ultrasonik sensör / ultrasonic sensor: Echo GPIO4 (far / headlights),
 *   Trig GPIO5 (buzzer). Direksiyon / steering GPIO13, motor GPIO12 + GPIO14,
 *   buton / button GPIO0, mavi LED / blue LED GPIO16.
 */

#include <CARBOT.h> // CARBOT kütüphanesi / CARBOT library

CARBOT carBot; // CARBOT nesnesi / CARBOT object

#define LED_PIN 16   // MINIBOT mavi LED / MINIBOT blue LED
#define BUTTON_PIN 0 // MINIBOT B1 butonu (basılıyken LOW) / MINIBOT B1 button (LOW while pressed)

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Direksiyon açıları. Aracınız ters dönüyorsa SOL ve SAĞ değerlerini yer değiştirin.
// Steering angles. If your car turns the wrong way, swap LEFT and RIGHT.
const int STEER_CENTER = 90;
const int STEER_LEFT = 45;
const int STEER_RIGHT = 135;
const int ESCAPE_SPEED = 255;       // Geri kaçış hızı / back-away speed
const uint32_t ESCAPE_MS = 600;     // Geri kaçış süresi / back-away time
const uint32_t CALM_MS = 1000;      // Alarmdan sonra sakinleşme / settle time after an alarm
const uint32_t READ_MS = 100;       // Sensör okuma aralığı / sensor read interval
const uint32_t REPORT_MS = 1000;    // Seri porta mesafe yazma aralığı / distance print interval

float intruderDistanceCm = 25.0f;   // Bu mesafenin altında alarm ("esik" ile değişir) / alarm below this ("threshold")

// ---------------------------------------------------------------------------
// Motor ve direksiyon / Motor and steering
// ---------------------------------------------------------------------------
int driveSpeed = 180;
int targetSpeed = 0;   // İstenen hız, işaretli / requested speed, signed
int appliedSpeed = 0;  // Motora verilen hız / applied speed
uint32_t lastRampMs = 0;
uint32_t driveUntilMs = 0;
bool timedDrive = false;

void applyMotor(int speed) {
  if (speed > 0) carBot.moveForward(speed);
  else if (speed < 0) carBot.moveBackward(-speed);
  else carBot.stop();
}

void stopNow() { // Rampa yok, hemen dur / no ramp, stop at once
  targetSpeed = 0;
  appliedSpeed = 0;
  timedDrive = false;
  carBot.stop();
}

// Hızı her 15 ms'de biraz değiştir; yön tersse önce 0'a in.
// Change the speed a little every 15 ms; if the direction flips, go to 0 first.
void updateRamp(uint32_t now) {
  if (now - lastRampMs < 15) return;
  lastRampMs = now;
  if (appliedSpeed == targetSpeed) return;
  int goal = targetSpeed;
  if ((appliedSpeed > 0 && targetSpeed < 0) || (appliedSpeed < 0 && targetSpeed > 0)) goal = 0;
  if (appliedSpeed < goal) appliedSpeed = min(appliedSpeed + 15, goal);
  else appliedSpeed = max(appliedSpeed - 15, goal);
  applyMotor(appliedSpeed);
}

// ---------------------------------------------------------------------------
// Nöbet durum makinesi (bloklamaz) / Guard state machine (non-blocking)
// ---------------------------------------------------------------------------
enum GuardState { WATCH, FLASH, ESCAPE, CALM };
GuardState guardState = WATCH;
uint32_t stateMs = 0;      // Durumun başlama zamanı / time the state started
int flashCount = 0;
uint32_t lastReadMs = 0;
uint32_t lastReportMs = 0;
float lastDistance = -1;

void lightsAndLed(bool on) {
  digitalWrite(LED_PIN, on ? HIGH : LOW);
  carBot.controlLED(on);
}

void runGuard(uint32_t now) {
  switch (guardState) {
    case WATCH:
      if (now - lastReadMs >= READ_MS) {
        lastReadMs = now;
        lastDistance = carBot.readUltrasonicCM();
        if (lastDistance > 0 && lastDistance < intruderDistanceCm) {
          Serial.println(String(L("ALARM! Bir şey yaklaştı, mesafe: ", "ALARM! Something got close, distance: ")) +
                         String(lastDistance, 1) + " cm");
          guardState = FLASH;
          flashCount = 0;
          stateMs = now - 80; // İlk yanıp sönme hemen / first flash right away
        } else if (now - lastReportMs >= REPORT_MS) {
          lastReportMs = now;
          if (lastDistance > 0) Serial.println(String(L("Nöbette... mesafe: ", "On watch... distance: ")) + String(lastDistance, 1) + " cm");
          else Serial.println(L("Nöbette... mesafe: okuma yok", "On watch... distance: no reading"));
        }
      }
      break;

    case FLASH: // Farları hızla yak-söndür + korna / flash the headlights + horn
      if (now - stateMs >= 80) {
        stateMs = now;
        if (flashCount % 2 == 0) {
          lightsAndLed(true);
          carBot.buzzerPlay(1500, 80); // Kısa bip (80 ms) / short beep (80 ms)
        } else {
          lightsAndLed(false);
        }
        flashCount++;
        if (flashCount >= 8) {
          lightsAndLed(false);
          carBot.steer(STEER_CENTER);
          targetSpeed = -ESCAPE_SPEED;
          appliedSpeed = -ESCAPE_SPEED; // Kaçış anında başlar (araç zaten duruyor) / escape starts at once (car is stopped)
          applyMotor(appliedSpeed);
          guardState = ESCAPE;
          stateMs = now;
          Serial.println(L("Geri kaçıyor!", "Backing away!"));
        }
      }
      break;

    case ESCAPE: // Kısa bir süre geri kaç / back away briefly
      if (now - stateMs >= ESCAPE_MS) {
        stopNow();
        guardState = CALM;
        stateMs = now;
      }
      break;

    case CALM: // Sakinleşmek için kısa bekleme / short pause to settle down
      if (now - stateMs >= CALM_MS) {
        guardState = WATCH;
        Serial.println(L("Nöbete dönüldü.", "Back on watch."));
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "EŞİK" -> "esik"
// Lower-cases and simplifies Turkish letters: "EŞİK" -> "esik"
String normalizeCommand(String s) {
  s.trim();
  s.replace("İ", "i"); s.replace("I", "i"); s.replace("ı", "i");
  s.replace("Ş", "s"); s.replace("ş", "s");
  s.replace("Ğ", "g"); s.replace("ğ", "g");
  s.replace("Ü", "u"); s.replace("ü", "u");
  s.replace("Ö", "o"); s.replace("ö", "o");
  s.replace("Ç", "c"); s.replace("ç", "c");
  s.toLowerCase();
  return s;
}

bool readCommand(String &cmd) {
  while (Serial.available() > 0) {
    char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() == 0) continue;
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 40) cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150) {
    cmd = normalizeCommand(cmdBuffer);
    cmdBuffer = "";
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Mesajlar ve modlar / Messages and modes
// ---------------------------------------------------------------------------
bool manualMode = false; // false = NÖBET (otomatik), true = MANUEL / false = GUARD (auto), true = MANUAL

void printHelp() {
  Serial.println(L("---- BEKÇİ ARAÇ - Komutlar ----", "---- GUARD CAR - Commands ----"));
  Serial.println(L("  yardim          : bu liste", "  help            : this list"));
  Serial.println(L("  oto / manuel    : nöbet / manuel mod", "  auto / manual   : guard / manual mode"));
  Serial.println(L("  esik 5-200      : alarm mesafesi (cm)", "  threshold 5-200 : alarm distance (cm)"));
  Serial.println(L("  mesafe          : mesafeyi ölç", "  distance        : measure the distance"));
  Serial.println(L("  ileri/geri [sn] : sür (varsayılan 2 sn)", "  forward/back [s]: drive (default 2 s)"));
  Serial.println(L("  sol / sag / duz : direksiyon", "  left / right / straight : steering"));
  Serial.println(L("  hiz 0-255       : sürüş hızı", "  speed 0-255     : driving speed"));
  Serial.println(L("  dur             : HER ŞEYİ DURDUR", "  stop            : STOP EVERYTHING"));
  Serial.println(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  Serial.println(L("  Buton (B1)      : NÖBET <-> MANUEL", "  Button (B1)     : GUARD <-> MANUAL"));
}

void setMode(bool manual) {
  manualMode = manual;
  stopNow();
  carBot.steer(STEER_CENTER);
  lightsAndLed(false);
  guardState = WATCH;
  carBot.buzzerPlay(manual ? 700 : 1200, 150);
  Serial.println(manual ? L(">> Nöbet durduruldu. MANUEL mod: aracı seri komutlarla sürün.",
                            ">> Guard duty stopped. MANUAL mode: drive the car with serial commands.")
                        : L(">> Nöbet başladı (OTOMATİK mod).", ">> Guard duty started (AUTO mode)."));
}

void driveCommand(int direction, const String &arg, bool hasValue, uint32_t now) {
  if (!manualMode) setMode(true);
  int seconds = hasValue ? constrain(arg.toInt(), 1, 10) : 2;
  targetSpeed = direction * driveSpeed;
  timedDrive = true;
  driveUntilMs = now + seconds * 1000UL;
  Serial.println(String(direction > 0 ? L("İleri ", "Forward ") : L("Geri ", "Backward ")) + seconds +
                 L(" sn, hız ", " s, speed ") + driveSpeed);
}

void handleCommand(const String &cmd, uint32_t now) {
  int space = cmd.indexOf(' ');
  String word = (space < 0) ? cmd : cmd.substring(0, space);
  String arg = (space < 0) ? String("") : cmd.substring(space + 1);
  arg.trim();
  bool hasValue = arg.length() > 0 && (isDigit(arg[0]) || arg[0] == '-');

  if (word == "dur" || word == "stop") {
    if (!manualMode) setMode(true); // Dur her zaman çalışır / stop always works
    stopNow();
    Serial.println(L("DURDU.", "STOPPED."));
  } else if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false);
  } else if (word == "manuel" || word == "manual") {
    setMode(true);
  } else if ((word == "esik" || word == "threshold") && hasValue) {
    intruderDistanceCm = constrain(arg.toInt(), 5, 200);
    Serial.println(String(L("Alarm mesafesi: ", "Alarm distance: ")) + (int)intruderDistanceCm + " cm");
  } else if (word == "mesafe" || word == "distance") {
    float cm = carBot.readUltrasonicCM();
    if (cm > 0) Serial.println(String(L("Mesafe: ", "Distance: ")) + String(cm, 1) + " cm");
    else Serial.println(L("Mesafe okunamadı (sensör takılı mı?).", "No distance reading (is the sensor plugged in?)."));
  } else if (word == "ileri" || word == "forward") {
    driveCommand(1, arg, hasValue, now);
  } else if (word == "geri" || word == "back" || word == "backward") {
    driveCommand(-1, arg, hasValue, now);
  } else if (word == "sol" || word == "left") {
    if (!manualMode) setMode(true);
    carBot.steer(STEER_LEFT);
    Serial.println(L("Direksiyon sola.", "Steering left."));
  } else if (word == "sag" || word == "right") {
    if (!manualMode) setMode(true);
    carBot.steer(STEER_RIGHT);
    Serial.println(L("Direksiyon sağa.", "Steering right."));
  } else if (word == "duz" || word == "straight" || word == "orta" || word == "center") {
    if (!manualMode) setMode(true);
    carBot.steer(STEER_CENTER);
    Serial.println(L("Direksiyon ortada.", "Steering centered."));
  } else if ((word == "hiz" || word == "speed") && hasValue) {
    driveSpeed = constrain(abs(arg.toInt()), 0, 255);
    if (targetSpeed > 0) targetSpeed = driveSpeed;
    if (targetSpeed < 0) targetSpeed = -driveSpeed;
    Serial.println(String(L("Sürüş hızı: ", "Drive speed: ")) + driveSpeed);
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    Serial.println(L("Dil: Türkçe", "Language: English"));
    printHelp();
  } else {
    Serial.println(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
bool lastButton = false;
uint32_t lastButtonMs = 0;

// Butona yeni basıldıysa true (titreşim süzgeçli) / true on a new press (debounced)
bool buttonPressed(uint32_t now) {
  bool pressed = (digitalRead(BUTTON_PIN) == LOW);
  bool edge = pressed && !lastButton && (now - lastButtonMs) > 200;
  if (edge) lastButtonMs = now;
  lastButton = pressed;
  return edge;
}

void setup() {
  carBot.serialStart(115200); // Seri haberleşme / Serial communication
  carBot.begin();             // CARBOT başlatılıyor / Initialize CARBOT
  carBot.stop();              // Güvenlik: motorlar durgun başlar / safety: motors start stopped
  carBot.steer(STEER_CENTER);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println();
  Serial.println(L("Bekçi araç hazır.", "Guard car ready."));
  printHelp();
  setMode(false); // Nöbetle başla / start on guard duty
}

void loop() {
  uint32_t now = millis();

  // 1) Buton -> nöbet aç/kapat / button -> guard duty on/off
  if (buttonPressed(now)) setMode(!manualMode);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd, now);

  // 3) Nöbet veya süreli seri sürüş / guard duty or timed serial drive
  if (!manualMode) {
    runGuard(now);
  } else {
    if (timedDrive && (int32_t)(now - driveUntilMs) >= 0) {
      timedDrive = false;
      targetSpeed = 0;
      Serial.println(L("Süre doldu, araç duruyor.", "Time is up, the car is stopping."));
    }
    updateRamp(now);
  }
}
