/*
 * TR: CARBOT TEMEL ÖRNEK - Otomatik demo + Manuel kontrol
 *  - Açılışta OTOMATİK mod çalışır: araç sırayla farları yakıp söndürür, kısa
 *    bir süre ileri gider, direksiyonu sola/sağa çevirir, "bip bip" diyerek geri
 *    gider ve korna çalar; sonra baştan başlar.
 *  - MINIBOT üzerindeki butona (B1 / GPIO0) basınca MANUEL moda geçer: araç
 *    hemen durur ve seri komutlarla siz sürersiniz. Butona tekrar basınca
 *    otomatik moda döner.
 *  - Motor hızı yumuşak değişir (rampa) ve yön değiştirirken önce yavaşlar.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim    / help        -> komut listesi
 *      oto       / auto        -> otomatik mod
 *      manuel    / manual      -> manuel mod
 *      ileri 2   / forward 2   -> 2 saniye ileri git (sayı yazmazsanız 2 sn, en fazla 10)
 *      geri 2    / back 2      -> 2 saniye geri git
 *      sol       / left        -> direksiyonu sola çevir
 *      sag       / right       -> direksiyonu sağa çevir
 *      duz       / straight    -> direksiyonu ortala
 *      dur       / stop        -> HER ŞEYİ HEMEN DURDUR (manuel moda geçer)
 *      hiz 180   / speed 180   -> sürüş hızı (PWM 0-255)
 *      far       / lights      -> farları aç / kapat
 *      korna     / horn        -> korna çal
 *      mesafe    / distance    -> ultrasonik sensörle mesafe ölç
 *      durum     / status      -> durum bilgisi
 *      dil       / lang        -> dili değiştir (Türkçe <-> English)
 *    Bir sürüş komutu otomatik moddayken gelirse araç manuel moda geçer.
 *
 * EN: CARBOT BASIC EXAMPLE - Automatic demo + Manual control
 *  - At startup AUTO mode runs: the car turns the headlights on and off, drives
 *    forward briefly, steers left/right, backs up with a "beep beep" and sounds
 *    the horn; then it starts over.
 *  - Press the button on the MINIBOT (B1 / GPIO0) to switch to MANUAL mode: the
 *    car stops at once and you drive it with serial commands. Press the button
 *    again to go back to auto mode.
 *  - The motor speed changes smoothly (ramp) and slows down before reversing.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help      / yardim      -> command list
 *      auto      / oto         -> auto mode
 *      manual    / manuel      -> manual mode
 *      forward 2 / ileri 2     -> drive forward for 2 seconds (default 2 s, max 10)
 *      back 2    / geri 2      -> drive backward for 2 seconds
 *      left      / sol         -> steer left
 *      right     / sag         -> steer right
 *      straight  / duz         -> center the steering
 *      stop      / dur         -> STOP EVERYTHING NOW (switches to manual)
 *      speed 180 / hiz 180     -> driving speed (PWM 0-255)
 *      lights    / far         -> headlights on / off
 *      horn      / korna       -> sound the horn
 *      distance  / mesafe      -> measure the distance with the ultrasonic sensor
 *      status    / durum       -> status info
 *      lang      / dil         -> switch language (Turkish <-> English)
 *    A driving command received in auto mode switches the car to manual mode.
 *
 * NOT / NOTE: Ultrasonik sensör far ve korna ile aynı pinleri kullanır; mesafe
 * ölçülürken far/korna, far/korna kullanılırken mesafe çalışmaz (kütüphane
 * otomatik geçiş yapar ve seri porta uyarı yazar). / The ultrasonic sensor shares
 * its pins with the headlights and horn; the library switches automatically and
 * prints a warning on the serial port.
 *
 * Bağlantı / Wiring: CARBOT'un üzerindeki MINIBOT'a yükleyin (ESP8266).
 *   Direksiyon / steering GPIO13, motor GPIO12 + GPIO14, buzzer (Trig) GPIO5,
 *   far / headlights (Echo) GPIO4, buton / button GPIO0, mavi LED / blue LED GPIO16.
 *   IOTBOT ile kullanım için / to use with an IOTBOT: IOTBOT_CARBOT_Basic_Example.
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
const int DEMO_SPEED = 180; // Demo hızı (PWM, tam hız 255) / demo speed (PWM, full speed 255)

// Tip en üstte olmalı (Arduino fonksiyon prototiplerini ilk fonksiyonun önüne ekler).
// The type must be at the top (Arduino puts function prototypes before the first function).
// Demo adımı: hız (+ ileri, - geri, 0 dur), direksiyon, far (1 aç, 0 kapat, -1 dokunma),
// bip Hz (0 = yok), bip tekrar (geri giderken bip bip), süre (ms), mesaj.
// Demo step: speed (+ forward, - backward, 0 stop), steering, lights (1 on, 0 off, -1 keep),
// beep Hz (0 = none), repeat beep (beep beep while reversing), duration (ms), message.
struct CarStep { int speed; int steer; int8_t led; uint16_t beepHz; bool beepRepeat; uint16_t ms; const char *tr; const char *en; };

// ---------------------------------------------------------------------------
// Motor ve direksiyon / Motor and steering
// ---------------------------------------------------------------------------
int driveSpeed = DEMO_SPEED; // "hiz" komutuyla değişir / changed with the "speed" command
int targetSpeed = 0;         // İstenen hız, işaretli (-255..255) / requested speed, signed
int appliedSpeed = 0;        // Motora verilen hız / speed applied to the motor
int steerAngle = STEER_CENTER;
bool lightsOn = false;
uint32_t lastRampMs = 0;
uint32_t driveUntilMs = 0;   // Seri sürüş komutunun bitiş zamanı / end time of a serial drive command
bool timedDrive = false;

void applyMotor(int speed) {
  if (speed > 0) carBot.moveForward(speed);
  else if (speed < 0) carBot.moveBackward(-speed);
  else carBot.stop();
}

// Acil durdurma: rampa yok, hemen dur / emergency stop: no ramp, stop at once
void stopNow() {
  targetSpeed = 0;
  appliedSpeed = 0;
  timedDrive = false;
  carBot.stop();
}

void setSteer(int angle) {
  steerAngle = constrain(angle, 0, 180);
  carBot.steer(steerAngle);
}

void setLights(bool on) {
  lightsOn = on;
  carBot.controlLED(on);
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
// Otomatik demo / Automatic demo
// ---------------------------------------------------------------------------
const CarStep DEMO[] = {
    {0, STEER_CENTER, 1, 0, false, 1500, "Farlar açık", "Headlights on"},
    {0, STEER_CENTER, 0, 0, false, 1000, "Farlar kapalı", "Headlights off"},
    {DEMO_SPEED, STEER_CENTER, -1, 0, false, 1000, "İleri", "Forward"},
    {0, STEER_CENTER, -1, 0, false, 1000, "Dur", "Stop"},
    {0, STEER_LEFT, -1, 0, false, 1000, "Direksiyon sola", "Steering left"},
    {0, STEER_CENTER, -1, 0, false, 800, "Direksiyon ortada", "Steering center"},
    {-DEMO_SPEED, STEER_CENTER, -1, 1200, true, 1200, "Geri (bip bip)", "Backward (beep beep)"},
    {0, STEER_CENTER, -1, 0, false, 1000, "Dur", "Stop"},
    {0, STEER_RIGHT, -1, 0, false, 1000, "Direksiyon sağa", "Steering right"},
    {0, STEER_CENTER, -1, 0, false, 800, "Direksiyon ortada", "Steering center"},
    {0, STEER_CENTER, -1, 1000, false, 2000, "Korna", "Horn"},
};
const int DEMO_LEN = sizeof(DEMO) / sizeof(DEMO[0]);
int demoIndex = 0;
uint32_t stepStartMs = 0;
uint32_t lastBeepMs = 0;

void startDemoStep(uint32_t now) {
  const CarStep &st = DEMO[demoIndex];
  targetSpeed = st.speed;
  setSteer(st.steer);
  if (st.led >= 0) setLights(st.led == 1);
  if (st.beepHz) carBot.buzzerPlay(st.beepHz, st.beepRepeat ? 80 : 300);
  lastBeepMs = now;
  stepStartMs = now;
  Serial.println(String(L("Demo: ", "Demo: ")) + L(st.tr, st.en));
}

void runAutoDemo(uint32_t now) {
  const CarStep &st = DEMO[demoIndex];
  if (st.beepRepeat && now - lastBeepMs >= 300) { // Geri giderken bip bip / beep beep while reversing
    lastBeepMs = now;
    carBot.buzzerPlay(st.beepHz, 80);
  }
  if (now - stepStartMs >= st.ms) {
    demoIndex = (demoIndex + 1) % DEMO_LEN;
    startDemoStep(now);
  }
}

// ---------------------------------------------------------------------------
// Seri komut okuyucu / Serial command reader
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
uint32_t lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "SAĞ" -> "sag"
// Lower-cases and simplifies Turkish letters: "SAĞ" -> "sag"
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
bool manualMode = false; // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL

void printHelp() {
  Serial.println(L("---- CARBOT - Komutlar ----", "---- CARBOT - Commands ----"));
  Serial.println(L("  yardim          : bu liste", "  help            : this list"));
  Serial.println(L("  oto / manuel    : otomatik / manuel mod", "  auto / manual   : auto / manual mode"));
  Serial.println(L("  ileri [sn]      : ileri git (varsayılan 2 sn)", "  forward [s]     : drive forward (default 2 s)"));
  Serial.println(L("  geri [sn]       : geri git", "  back [s]        : drive backward"));
  Serial.println(L("  sol / sag / duz : direksiyon", "  left / right / straight : steering"));
  Serial.println(L("  dur             : HER ŞEYİ DURDUR", "  stop            : STOP EVERYTHING"));
  Serial.println(L("  hiz 0-255       : sürüş hızı", "  speed 0-255     : driving speed"));
  Serial.println(L("  far / korna     : farlar / korna", "  lights / horn   : headlights / horn"));
  Serial.println(L("  mesafe          : mesafe ölç", "  distance        : measure distance"));
  Serial.println(L("  durum           : durum bilgisi", "  status          : status info"));
  Serial.println(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  Serial.println(L("  Buton (B1)      : OTOMATİK <-> MANUEL", "  Button (B1)     : AUTO <-> MANUAL"));
}

void printStatus() {
  Serial.println(String(L("Mod: ", "Mode: ")) + (manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO")) +
                 L(" | hız ", " | speed ") + appliedSpeed + L(" | direksiyon ", " | steering ") + steerAngle +
                 L("° | far ", "° | lights ") + (lightsOn ? L("açık", "on") : L("kapalı", "off")) +
                 L(" | sürüş hızı ", " | drive speed ") + driveSpeed);
}

void setMode(bool manual, uint32_t now) {
  manualMode = manual;
  stopNow();                 // Mod değişince araç önce durur / the car stops when the mode changes
  setSteer(STEER_CENTER);
  carBot.buzzerPlay(manual ? 1500 : 1000, 60);
  if (manual) {
    Serial.println(L(">> MANUEL mod: aracı seri komutlarla sürün (yardim yazın).",
                     ">> MANUAL mode: drive the car with serial commands (type help)."));
  } else {
    Serial.println(L(">> OTOMATİK mod: araç demoyu kendi kendine yapıyor.", ">> AUTO mode: the car runs the demo by itself."));
    demoIndex = 0;
    startDemoStep(now);
  }
}

void driveCommand(int direction, const String &arg, bool hasValue, uint32_t now) {
  if (!manualMode) setMode(true, now);
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
    // Dur her zaman çalışır / stop always works
    if (!manualMode) setMode(true, now);
    stopNow();
    Serial.println(L("DURDU.", "STOPPED."));
  } else if (word == "yardim" || word == "help" || word == "?") {
    printHelp();
  } else if (word == "oto" || word == "otomatik" || word == "auto") {
    setMode(false, now);
  } else if (word == "manuel" || word == "manual") {
    setMode(true, now);
  } else if (word == "ileri" || word == "forward") {
    driveCommand(1, arg, hasValue, now);
  } else if (word == "geri" || word == "back" || word == "backward") {
    driveCommand(-1, arg, hasValue, now);
  } else if (word == "sol" || word == "left") {
    if (!manualMode) setMode(true, now);
    setSteer(STEER_LEFT);
    Serial.println(L("Direksiyon sola.", "Steering left."));
  } else if (word == "sag" || word == "right") {
    if (!manualMode) setMode(true, now);
    setSteer(STEER_RIGHT);
    Serial.println(L("Direksiyon sağa.", "Steering right."));
  } else if (word == "duz" || word == "straight" || word == "orta" || word == "center") {
    if (!manualMode) setMode(true, now);
    setSteer(STEER_CENTER);
    Serial.println(L("Direksiyon ortada.", "Steering centered."));
  } else if ((word == "hiz" || word == "speed") && hasValue) {
    driveSpeed = constrain(abs(arg.toInt()), 0, 255);
    if (targetSpeed > 0) targetSpeed = driveSpeed;   // Giderken yeni hız hemen geçerli / applies at once while driving
    if (targetSpeed < 0) targetSpeed = -driveSpeed;
    Serial.println(String(L("Sürüş hızı: ", "Drive speed: ")) + driveSpeed);
  } else if (word == "far" || word == "lights") {
    if (!manualMode) setMode(true, now);
    setLights(!lightsOn);
    Serial.println(lightsOn ? L("Farlar açık.", "Headlights on.") : L("Farlar kapalı.", "Headlights off."));
  } else if (word == "korna" || word == "horn") {
    carBot.buzzerPlay(1000, 300);
  } else if (word == "mesafe" || word == "distance") {
    lightsOn = false; // Sensör far pinini kullanır / the sensor uses the headlight pin
    float cm = carBot.readUltrasonicCM();
    if (cm > 0) Serial.println(String(L("Mesafe: ", "Distance: ")) + String(cm, 1) + " cm");
    else Serial.println(L("Mesafe okunamadı (sensör takılı mı?).", "No distance reading (is the sensor plugged in?)."));
  } else if (word == "durum" || word == "status") {
    printStatus();
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
  setSteer(STEER_CENTER);
  setLights(false);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println();
  Serial.println(L("CARBOT temel örnek başladı.", "CARBOT basic example started."));
  printHelp();
  setMode(false, millis()); // OTOMATİK modla başla / start in AUTO mode
}

void loop() {
  uint32_t now = millis();

  // 1) Buton -> mod değiştir (sadece basıldığı an) / button -> toggle mode (on press only)
  if (buttonPressed(now)) setMode(!manualMode, now);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd, now);

  // 3) Otomatik demo veya süreli seri sürüş / automatic demo or timed serial drive
  if (!manualMode) {
    runAutoDemo(now);
  } else if (timedDrive && (int32_t)(now - driveUntilMs) >= 0) {
    timedDrive = false;
    targetSpeed = 0;
    Serial.println(L("Süre doldu, araç duruyor.", "Time is up, the car is stopping."));
  }

  // 4) Motor rampası / motor ramp
  updateRamp(now);

  // 5) Araç hareket ederken mavi LED yanar / blue LED on while the car moves
  digitalWrite(LED_PIN, appliedSpeed != 0 ? HIGH : LOW);
}
