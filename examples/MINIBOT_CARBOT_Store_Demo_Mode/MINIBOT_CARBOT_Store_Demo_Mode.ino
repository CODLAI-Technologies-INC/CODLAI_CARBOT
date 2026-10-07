/*
 * TR: CARBOT MAĞAZA (GÖSTERİ) MODU - kumanda gerekmez
 *  - Açılışta OTOMATİK mod çalışır: araç sürekli bir gösteri yapar (ileri,
 *    geri, sola dönüp ileri, sağa dönüp geri, korna ve far şovu) ve baştan
 *    başlar. Gösteri adım adım (bloklamadan) çalışır.
 *    DİKKAT: Araç gösteride gerçekten sürer; masa kenarından uzak tutun.
 *  - MINIBOT üzerindeki butona (B1 / GPIO0) basınca MANUEL moda geçer: araç
 *    hemen durur ve seri komutlarla siz sürersiniz. Butona tekrar basınca
 *    gösteri baştan başlar.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim / help, oto / auto, manuel / manual
 *      ileri 2 / forward 2, geri 2 / back 2  -> 2 saniye sür (en fazla 10)
 *      sol / left, sag / right, duz / straight -> direksiyon
 *      dur / stop       -> HER ŞEYİ HEMEN DURDUR (manuel moda geçer)
 *      hiz 180 / speed 180 -> sürüş hızı (PWM 0-255)
 *      far / lights, korna / horn, dil / lang
 *  - PlatformIO ortamı / environment: MINIBOT_CARBOT_DEMO (src/kontrol/minibot_carbot_demo.cpp
 *    bu dosyayı içeri alır / includes this file).
 *
 * EN: CARBOT STORE (SHOW) MODE - no controller needed
 *  - At startup AUTO mode runs: the car loops a show (forward, backward, turn
 *    left and drive forward, turn right and back up, horn and light show).
 *    The show runs step by step (without blocking).
 *    CAUTION: the car really drives in the show; keep it away from table edges.
 *  - Press the button on the MINIBOT (B1 / GPIO0) to switch to MANUAL mode: the
 *    car stops at once and you drive it with serial commands. Press the button
 *    again and the show starts over.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help / yardim, auto / oto, manual / manuel
 *      forward 2 / ileri 2, back 2 / geri 2  -> drive for 2 seconds (max 10)
 *      left / sol, right / sag, straight / duz -> steering
 *      stop / dur       -> STOP EVERYTHING NOW (switches to manual)
 *      speed 180 / hiz 180 -> driving speed (PWM 0-255)
 *      lights / far, horn / korna, lang / dil
 *
 * Bağlantı / Wiring: CARBOT'un üzerindeki MINIBOT'a yükleyin (ESP8266).
 *   Direksiyon / steering GPIO13, motor GPIO12 + GPIO14, buzzer GPIO5,
 *   far / headlights GPIO4, buton / button GPIO0.
 */

#include <MINIBOT.h> // MINIBOT kütüphanesi / MINIBOT library
#include <CARBOT.h>  // CARBOT kütüphanesi / CARBOT library

MINIBOT minibot; // MINIBOT nesnesi (buton, mavi LED) / MINIBOT object (button, blue LED)
CARBOT carbot;   // CARBOT nesnesi / CARBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Direksiyon açıları. Aracınız ters dönüyorsa SOL ve SAĞ değerlerini yer değiştirin.
// Steering angles. If your car turns the wrong way, swap LEFT and RIGHT.
const int STEER_CENTER = 90;
const int STEER_LEFT = 45;
const int STEER_RIGHT = 135;
const int SHOW_SPEED = 255; // Gösteri hızı (tam hız; masada daha düşük seçin) / show speed (full; use less on a table)

// Gösteri adımı: hız (+ ileri, - geri, 0 dur), direksiyon, far (1 aç, 0 kapat, -1 dokunma),
// bip Hz (0 = yok), süre (ms), mesaj.
// Show step: speed (+ forward, - backward, 0 stop), steering, lights (1 on, 0 off, -1 keep),
// beep Hz (0 = none), duration (ms), message.
struct CarStep { int speed; int steer; int8_t led; uint16_t beepHz; uint16_t ms; const char *tr; const char *en; };

// ---------------------------------------------------------------------------
// Motor ve direksiyon / Motor and steering
// ---------------------------------------------------------------------------
int driveSpeed = 180;  // Manuel sürüş hızı / manual driving speed
int targetSpeed = 0;   // İstenen hız, işaretli / requested speed, signed
int appliedSpeed = 0;  // Motora verilen hız / applied speed
bool lightsOn = false;
uint32_t lastRampMs = 0;
uint32_t driveUntilMs = 0;
bool timedDrive = false;

void applyMotor(int speed) {
  if (speed > 0) carbot.moveForward(speed);
  else if (speed < 0) carbot.moveBackward(-speed);
  else carbot.stop();
}

void stopNow() { // Rampa yok, hemen dur / no ramp, stop at once
  targetSpeed = 0;
  appliedSpeed = 0;
  timedDrive = false;
  carbot.stop();
}

void setLights(bool on) {
  lightsOn = on;
  carbot.controlLED(on);
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
// Gösteri / Show
// ---------------------------------------------------------------------------
const CarStep SHOW[] = {
    {SHOW_SPEED, STEER_CENTER, 1, 0, 1000, "1. İleri", "1. Forward"},
    {0, STEER_CENTER, 0, 0, 500, nullptr, nullptr},
    {-SHOW_SPEED, STEER_CENTER, 1, 0, 1000, "2. Geri", "2. Backward"},
    {0, STEER_CENTER, 0, 0, 500, nullptr, nullptr},
    {SHOW_SPEED, STEER_LEFT, -1, 0, 1000, "3. Sola dön ve ileri", "3. Turn left and forward"},
    {0, STEER_LEFT, -1, 0, 200, nullptr, nullptr},
    {-SHOW_SPEED, STEER_RIGHT, -1, 0, 1000, "4. Sağa dön ve geri", "4. Turn right and backward"},
    {0, STEER_RIGHT, -1, 0, 200, nullptr, nullptr},
    {0, STEER_CENTER, 1, 2000, 100, "5. Korna ve far şovu", "5. Horn and light show"},
    {0, STEER_CENTER, 0, 0, 100, nullptr, nullptr},
    {0, STEER_CENTER, 1, 2000, 100, nullptr, nullptr},
    {0, STEER_CENTER, 0, 0, 100, nullptr, nullptr},
    {0, STEER_CENTER, 1, 2000, 100, nullptr, nullptr},
    {0, STEER_CENTER, 0, 0, 2000, nullptr, nullptr}, // Tekrarlamadan önce bekle / wait before repeating
};
const int SHOW_LEN = sizeof(SHOW) / sizeof(SHOW[0]);
int showIndex = 0;
uint32_t stepStartMs = 0;

void startShowStep(uint32_t now) {
  const CarStep &st = SHOW[showIndex];
  targetSpeed = st.speed;
  carbot.steer(st.steer);
  if (st.led >= 0) setLights(st.led == 1);
  if (st.beepHz) carbot.buzzerPlay(st.beepHz, 50); // Kısa bip (50 ms) / short beep (50 ms)
  if (st.tr) Serial.println(L(st.tr, st.en));
  stepStartMs = now;
}

void runShow(uint32_t now) {
  if (now - stepStartMs >= SHOW[showIndex].ms) {
    showIndex = (showIndex + 1) % SHOW_LEN;
    startShowStep(now);
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
bool manualMode = false; // false = OTOMATİK (gösteri), true = MANUEL / false = AUTO (show), true = MANUAL

void printHelp() {
  Serial.println(L("---- CARBOT MAĞAZA MODU - Komutlar ----", "---- CARBOT STORE MODE - Commands ----"));
  Serial.println(L("  yardim          : bu liste", "  help            : this list"));
  Serial.println(L("  oto / manuel    : gösteri / manuel mod", "  auto / manual   : show / manual mode"));
  Serial.println(L("  ileri/geri [sn] : sür (varsayılan 2 sn)", "  forward/back [s]: drive (default 2 s)"));
  Serial.println(L("  sol / sag / duz : direksiyon", "  left / right / straight : steering"));
  Serial.println(L("  dur             : HER ŞEYİ DURDUR", "  stop            : STOP EVERYTHING"));
  Serial.println(L("  hiz 0-255       : sürüş hızı", "  speed 0-255     : driving speed"));
  Serial.println(L("  far / korna     : farlar / korna", "  lights / horn   : headlights / horn"));
  Serial.println(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  Serial.println(L("  Buton (B1)      : OTOMATİK <-> MANUEL", "  Button (B1)     : AUTO <-> MANUAL"));
}

void setMode(bool manual, uint32_t now) {
  manualMode = manual;
  stopNow(); // Mod değişince araç önce durur / the car stops when the mode changes
  carbot.steer(STEER_CENTER);
  setLights(false);
  carbot.buzzerPlay(manual ? 1500 : 1000, 60);
  if (manual) {
    Serial.println(L(">> MANUEL mod: aracı seri komutlarla sürün (yardim yazın).",
                     ">> MANUAL mode: drive the car with serial commands (type help)."));
  } else {
    Serial.println(L(">> OTOMATİK mod: mağaza gösterisi çalışıyor.", ">> AUTO mode: the store show is running."));
    showIndex = 0;
    startShowStep(now);
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
    if (!manualMode) setMode(true, now); // Dur her zaman çalışır / stop always works
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
    carbot.steer(STEER_LEFT);
    Serial.println(L("Direksiyon sola.", "Steering left."));
  } else if (word == "sag" || word == "right") {
    if (!manualMode) setMode(true, now);
    carbot.steer(STEER_RIGHT);
    Serial.println(L("Direksiyon sağa.", "Steering right."));
  } else if (word == "duz" || word == "straight" || word == "orta" || word == "center") {
    if (!manualMode) setMode(true, now);
    carbot.steer(STEER_CENTER);
    Serial.println(L("Direksiyon ortada.", "Steering centered."));
  } else if ((word == "hiz" || word == "speed") && hasValue) {
    driveSpeed = constrain(abs(arg.toInt()), 0, 255);
    if (targetSpeed > 0 && manualMode) targetSpeed = driveSpeed;
    if (targetSpeed < 0 && manualMode) targetSpeed = -driveSpeed;
    Serial.println(String(L("Sürüş hızı: ", "Drive speed: ")) + driveSpeed);
  } else if (word == "far" || word == "lights") {
    if (!manualMode) setMode(true, now);
    setLights(!lightsOn);
    Serial.println(lightsOn ? L("Farlar açık.", "Headlights on.") : L("Farlar kapalı.", "Headlights off."));
  } else if (word == "korna" || word == "horn") {
    carbot.buzzerPlay(1000, 300);
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

// Butona yeni basıldıysa true (B1 basılıyken LOW okur) / true on a new press (B1 reads LOW while pressed)
bool buttonPressed(uint32_t now) {
  bool pressed = !minibot.button1Read();
  bool edge = pressed && !lastButton && (now - lastButtonMs) > 200;
  if (edge) lastButtonMs = now;
  lastButton = pressed;
  return edge;
}

void setup() {
  minibot.begin();             // MINIBOT başlatılıyor / Initialize MINIBOT
  minibot.serialStart(115200); // Seri haberleşme / Serial communication
  carbot.begin();              // CARBOT başlatılıyor / Initialize CARBOT
  carbot.stop();               // Güvenlik: motorlar durgun başlar / safety: motors start stopped
  carbot.steer(STEER_CENTER);

  // Açılış sesi / intro sound
  carbot.buzzerPlay(1000, 100);
  delay(100);
  carbot.buzzerPlay(1500, 100);

  Serial.println();
  Serial.println(L("CARBOT mağaza modu hazır.", "CARBOT store mode ready."));
  printHelp();
  setMode(false, millis()); // Gösteriyle başla / start with the show
}

void loop() {
  uint32_t now = millis();

  // 1) Buton -> mod değiştir / button -> toggle mode
  if (buttonPressed(now)) setMode(!manualMode, now);

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd, now);

  // 3) Gösteri veya süreli seri sürüş / show or timed serial drive
  if (!manualMode) {
    runShow(now);
  } else if (timedDrive && (int32_t)(now - driveUntilMs) >= 0) {
    timedDrive = false;
    targetSpeed = 0;
    Serial.println(L("Süre doldu, araç duruyor.", "Time is up, the car is stopping."));
  }

  // 4) Motor rampası / motor ramp
  updateRamp(now);

  // 5) Araç hareket ederken mavi LED yanar / blue LED on while the car moves
  minibot.ledWrite(appliedSpeed != 0);
}
