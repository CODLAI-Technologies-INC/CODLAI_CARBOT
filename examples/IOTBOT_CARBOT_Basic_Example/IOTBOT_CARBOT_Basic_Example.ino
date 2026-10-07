/*
 * TR: IOTBOT + CARBOT TEMEL ÖRNEK - Otomatik demo + Manuel kontrol
 *  - Açılışta OTOMATİK mod çalışır: araç 1 sn ileri gider, durur, 1 sn geri
 *    gider, durur, direksiyonu sola / sağa çevirir ve ortalar; sonra baştan başlar.
 *  - B3 butonuna basınca MANUEL moda geçer, aracı kart üzerindeki kontrollerle
 *    siz sürersiniz. B3'e tekrar basınca otomatik moda döner.
 *      Joystick ileri/geri -> ileri / geri hız
 *      Joystick sol/sağ    -> direksiyon
 *      Potansiyometre      -> en yüksek hız (yavaş ... hızlı)
 *      Joystick butonu     -> farları aç / kapat
 *  - Motor hızı yumuşak değişir (rampa) ve yön değiştirirken önce yavaşlar.
 *  - Seri port komutları (115200 baud). Türkçe veya İngilizce yazabilirsiniz:
 *      yardim    / help        -> komut listesi
 *      oto       / auto        -> otomatik mod
 *      manuel    / manual      -> manuel mod
 *      ileri 2   / forward 2   -> 2 saniye ileri git (en fazla 10)
 *      geri 2    / back 2      -> 2 saniye geri git
 *      sol       / left        -> direksiyonu sola çevir
 *      sag       / right       -> direksiyonu sağa çevir
 *      duz       / straight    -> direksiyonu ortala
 *      dur       / stop        -> HER ŞEYİ HEMEN DURDUR (manuel moda geçer)
 *      hiz 180   / speed 180   -> seri sürüş hızı (PWM 0-255)
 *      far       / lights      -> farları aç / kapat
 *      korna     / horn        -> korna çal
 *      dil       / lang        -> dili değiştir (Türkçe <-> English)
 *
 * EN: IOTBOT + CARBOT BASIC EXAMPLE - Automatic demo + Manual control
 *  - At startup AUTO mode runs: the car drives forward for 1 s, stops, drives
 *    backward for 1 s, stops, steers left / right and centers; then it starts over.
 *  - Press B3 to switch to MANUAL mode and drive the car with the onboard
 *    controls. Press B3 again to go back to auto mode.
 *      Joystick forward/back -> forward / backward speed
 *      Joystick left/right   -> steering
 *      Potentiometer         -> top speed (slow ... fast)
 *      Joystick button       -> headlights on / off
 *  - The motor speed changes smoothly (ramp) and slows down before reversing.
 *  - Serial port commands (115200 baud). You can type Turkish or English:
 *      help      / yardim      -> command list
 *      auto      / oto         -> auto mode
 *      manual    / manuel      -> manual mode
 *      forward 2 / ileri 2     -> drive forward for 2 seconds (max 10)
 *      back 2    / geri 2      -> drive backward for 2 seconds
 *      left      / sol         -> steer left
 *      right     / sag         -> steer right
 *      straight  / duz         -> center the steering
 *      stop      / dur         -> STOP EVERYTHING NOW (switches to manual)
 *      speed 180 / hiz 180     -> serial driving speed (PWM 0-255)
 *      lights    / far         -> headlights on / off
 *      horn      / korna       -> sound the horn
 *      lang      / dil         -> switch language (Turkish <-> English)
 *
 * Bağlantı / Wiring: CARBOT kablosunu IOTBOT'un P1-P5 soketlerine takın:
 *   direksiyon / steering IO32, motor IO27 + IO33, buzzer IO25, far / headlights IO26.
 *   Bu soketlere başka modül takmayın. / Plug the CARBOT cable into the IOTBOT's
 *   P1-P5 sockets (pins above). Do not plug other modules into these sockets.
 */

#include <IOTBOT.h> // IoTBot kütüphanesi / IoTBot library
#include <CARBOT.h> // CARBOT kütüphanesi / CARBOT library

IOTBOT iotbot; // IoTBot nesnesi / IoTBot object
CARBOT carbot; // CARBOT nesnesi / CARBOT object

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Direksiyon açıları. Aracınız ters dönüyorsa SOL ve SAĞ değerlerini yer değiştirin.
// Steering angles. If your car turns the wrong way, swap LEFT and RIGHT.
const int STEER_CENTER = 90;
const int STEER_LEFT = 45;
const int STEER_RIGHT = 135;
const int DEMO_SPEED = 180;   // Demo hızı (PWM, tam hız 255) / demo speed (PWM, full speed 255)
const int JOY_DEADZONE = 300; // Joystick ölü bölgesi / joystick dead zone
const int MIN_PWM = 50;       // Motorun kalkabildiği en düşük PWM / lowest PWM that still moves the motor

// Demo adımı: hız (+ ileri, - geri, 0 dur), direksiyon, süre (ms), mesaj
// Demo step: speed (+ forward, - backward, 0 stop), steering, duration (ms), message
struct CarStep { int speed; int steer; uint16_t ms; const char *tr; const char *en; };

// ---------------------------------------------------------------------------
// Motor ve direksiyon / Motor and steering
// ---------------------------------------------------------------------------
int driveSpeed = DEMO_SPEED;  // Seri sürüş hızı / serial driving speed
int targetSpeed = 0;          // İstenen hız, işaretli (-255..255) / requested speed, signed
int appliedSpeed = 0;         // Motora verilen hız / applied speed
int steerAngle = STEER_CENTER;
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

void setSteer(int angle) {
  steerAngle = constrain(angle, 0, 180);
  carbot.steer(steerAngle);
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
// Otomatik demo / Automatic demo
// ---------------------------------------------------------------------------
const CarStep DEMO[] = {
    {DEMO_SPEED, STEER_CENTER, 1000, "İleri", "Forward"},
    {0, STEER_CENTER, 1000, "Dur", "Stop"},
    {-DEMO_SPEED, STEER_CENTER, 1000, "Geri", "Backward"},
    {0, STEER_CENTER, 1000, "Dur", "Stop"},
    {0, STEER_LEFT, 1000, "Direksiyon sola", "Steering left"},
    {0, STEER_RIGHT, 1000, "Direksiyon sağa", "Steering right"},
    {0, STEER_CENTER, 1000, "Direksiyon ortada", "Steering center"},
};
const int DEMO_LEN = sizeof(DEMO) / sizeof(DEMO[0]);
int demoIndex = 0;
uint32_t stepStartMs = 0;

void startDemoStep(uint32_t now) {
  targetSpeed = DEMO[demoIndex].speed;
  setSteer(DEMO[demoIndex].steer);
  stepStartMs = now;
  iotbot.serialWrite(String(L("Demo: ", "Demo: ")) + L(DEMO[demoIndex].tr, DEMO[demoIndex].en));
}

void runAutoDemo(uint32_t now) {
  if (now - stepStartMs >= DEMO[demoIndex].ms) {
    demoIndex = (demoIndex + 1) % DEMO_LEN;
    startDemoStep(now);
  }
}

// ---------------------------------------------------------------------------
// Manuel kontrol: joystick, potansiyometre, joystick butonu
// Manual control: joystick, potentiometer, joystick button
// ---------------------------------------------------------------------------
int joyXCenter = 2048, joyYCenter = 2048;
bool joyDriving = false;   // Joystick şu an sürüyor mu / is the joystick driving right now
bool joySteering = false;  // Joystick şu an direksiyonu tutuyor mu / is the joystick steering right now
bool lastJoyBtn = false;

void calibrateJoystick() { // Joystick'e dokunmayın / do not touch the joystick
  long sx = 0, sy = 0;
  for (int i = 0; i < 20; i++) {
    sx += iotbot.joystickXRead();
    sy += iotbot.joystickYRead();
    delay(5);
  }
  joyXCenter = sx / 20;
  joyYCenter = sy / 20;
  if (abs(joyXCenter - 2048) > 1000) joyXCenter = 2048; // Tuhaf okuma: varsayılan / odd reading: default
  if (abs(joyYCenter - 2048) > 1000) joyYCenter = 2048;
}

void runManual() {
  int dx = iotbot.joystickXRead() - joyXCenter;
  int dy = iotbot.joystickYRead() - joyYCenter;
  int maxSpeed = map(iotbot.potentiometerRead(), 0, 4095, MIN_PWM, 255); // Pot = en yüksek hız / pot = top speed

  // Hız: joystick ileri (+) / geri (-). Kablolu kumandayla aynı yön.
  // Speed: joystick forward (+) / back (-). Same direction as the wired controller.
  if (abs(dy) > JOY_DEADZONE) {
    int s = constrain(map(abs(dy), JOY_DEADZONE, 2048, MIN_PWM, maxSpeed), 0, maxSpeed);
    targetSpeed = dy > 0 ? s : -s;
    joyDriving = true;
    timedDrive = false; // Joystick seri sürüşü geçersiz kılar / the joystick overrides a serial drive
  } else if (joyDriving) {
    targetSpeed = 0;    // Joystick bırakıldı: dur / joystick released: stop
    joyDriving = false;
  }

  // Direksiyon: joystick sol/sağ; bırakınca ortalanır.
  // Steering: joystick left/right; centers when released.
  if (abs(dx) > JOY_DEADZONE) {
    setSteer(constrain(STEER_CENTER + (int)((long)dx * 45 / 2048), 45, 135));
    joySteering = true;
  } else if (joySteering) {
    setSteer(STEER_CENTER);
    joySteering = false;
  }

  // Joystick butonu (basılıyken LOW) -> far / joystick button (LOW while pressed) -> headlights
  bool joyBtn = !iotbot.joystickButtonRead();
  if (joyBtn && !lastJoyBtn) {
    setLights(!lightsOn);
    iotbot.serialWrite(lightsOn ? L("Farlar açık.", "Headlights on.") : L("Farlar kapalı.", "Headlights off."));
  }
  lastJoyBtn = joyBtn;
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
  while (iotbot.serialAvailable() > 0) {
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
// Ekran ve mesajlar / Screen and messages
// ---------------------------------------------------------------------------
bool manualMode = false; // false = OTOMATİK, true = MANUEL / false = AUTO, true = MANUAL
uint32_t lastScreenMs = 0;
bool lastB3 = false;

// lcdWriteFixedTxt Türkçe harfleri LCD'de doğru gösterir ve satırı boşlukla doldurur.
// lcdWriteFixedTxt shows Turkish letters correctly on the LCD and pads the row with spaces.
void lcdRow(int row, const char *text) { iotbot.lcdWriteFixedTxt(0, row, text, 20); }

void printHelp() {
  iotbot.serialWrite(L("---- IOTBOT + CARBOT - Komutlar ----", "---- IOTBOT + CARBOT - Commands ----"));
  iotbot.serialWrite(L("  yardim          : bu liste", "  help            : this list"));
  iotbot.serialWrite(L("  oto / manuel    : otomatik / manuel mod", "  auto / manual   : auto / manual mode"));
  iotbot.serialWrite(L("  ileri [sn]      : ileri git (varsayılan 2 sn)", "  forward [s]     : drive forward (default 2 s)"));
  iotbot.serialWrite(L("  geri [sn]       : geri git", "  back [s]        : drive backward"));
  iotbot.serialWrite(L("  sol / sag / duz : direksiyon", "  left / right / straight : steering"));
  iotbot.serialWrite(L("  dur             : HER ŞEYİ DURDUR", "  stop            : STOP EVERYTHING"));
  iotbot.serialWrite(L("  hiz 0-255       : seri sürüş hızı", "  speed 0-255     : serial driving speed"));
  iotbot.serialWrite(L("  far / korna     : farlar / korna", "  lights / horn   : headlights / horn"));
  iotbot.serialWrite(L("  dil             : English'e geç", "  lang            : switch to Turkish"));
  iotbot.serialWrite(L("  B3 butonu       : OTOMATİK <-> MANUEL", "  B3 button       : AUTO <-> MANUAL"));
  iotbot.serialWrite(L("  Manuel: Joy Y=hız, Joy X=direksiyon, Pot=en yüksek hız, Joy butonu=far",
                       "  Manual: Joy Y=speed, Joy X=steering, Pot=top speed, Joy button=lights"));
}

void drawStaticScreen() {
  lcdRow(0, "  IOTBOT + CARBOT");
  lcdRow(3, manualMode ? L("Joy/Pot/JBtn B3:oto", "Joy/Pot/JBtn B3:auto") : L("B3: manuel kontrol", "B3: manual control"));
  lastScreenMs = 0; // Değerleri hemen çiz / draw the values right away
}

void setMode(bool manual, uint32_t now) {
  manualMode = manual;
  stopNow();                 // Mod değişince araç önce durur / the car stops when the mode changes
  setSteer(STEER_CENTER);
  joyDriving = joySteering = false;
  carbot.buzzerPlay(manual ? 1500 : 1000, 60); // Aracın kendi buzzer'ı / the car's own buzzer
  if (manual) {
    iotbot.serialWrite(L(">> MANUEL mod: joystick ile sürün (veya seri komutlar).",
                         ">> MANUAL mode: drive with the joystick (or serial commands)."));
  } else {
    iotbot.serialWrite(L(">> OTOMATİK mod: araç demoyu kendi kendine yapıyor.", ">> AUTO mode: the car runs the demo by itself."));
    demoIndex = 0;
    startDemoStep(now);
  }
  drawStaticScreen();
}

void driveCommand(int direction, const String &arg, bool hasValue, uint32_t now) {
  if (!manualMode) setMode(true, now);
  int seconds = hasValue ? constrain(arg.toInt(), 1, 10) : 2;
  targetSpeed = direction * driveSpeed;
  timedDrive = true;
  driveUntilMs = now + seconds * 1000UL;
  iotbot.serialWrite(String(direction > 0 ? L("İleri ", "Forward ") : L("Geri ", "Backward ")) + seconds +
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
    iotbot.serialWrite(L("DURDU.", "STOPPED."));
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
    iotbot.serialWrite(L("Direksiyon sola.", "Steering left."));
  } else if (word == "sag" || word == "right") {
    if (!manualMode) setMode(true, now);
    setSteer(STEER_RIGHT);
    iotbot.serialWrite(L("Direksiyon sağa.", "Steering right."));
  } else if (word == "duz" || word == "straight" || word == "orta" || word == "center") {
    if (!manualMode) setMode(true, now);
    setSteer(STEER_CENTER);
    iotbot.serialWrite(L("Direksiyon ortada.", "Steering centered."));
  } else if ((word == "hiz" || word == "speed") && hasValue) {
    driveSpeed = constrain(abs(arg.toInt()), 0, 255);
    if (timedDrive) targetSpeed = (targetSpeed < 0) ? -driveSpeed : driveSpeed;
    iotbot.serialWrite(String(L("Seri sürüş hızı: ", "Serial drive speed: ")) + driveSpeed);
  } else if (word == "far" || word == "lights") {
    setLights(!lightsOn);
    iotbot.serialWrite(lightsOn ? L("Farlar açık.", "Headlights on.") : L("Farlar kapalı.", "Headlights off."));
  } else if (word == "korna" || word == "horn") {
    carbot.buzzerPlay(1000, 300);
  } else if (word == "dil" || word == "lang" || word == "language") {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    drawStaticScreen();
    printHelp();
  } else {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  iotbot.begin();             // IoTBot başlatılıyor / Initialize IoTBot
  iotbot.serialStart(115200); // Seri haberleşme / Serial communication
  carbot.begin();             // CARBOT başlatılıyor / Initialize CARBOT
  carbot.stop();              // Güvenlik: motorlar durgun başlar / safety: motors start stopped
  setSteer(STEER_CENTER);
  setLights(false);
  calibrateJoystick();        // Açılışta joystick'e dokunmayın / do not touch the joystick at startup
  iotbot.lcdClear();
  iotbot.serialWrite(L("IOTBOT + CARBOT temel örnek başladı.", "IOTBOT + CARBOT basic example started."));
  printHelp();
  setMode(false, millis()); // OTOMATİK modla başla / start in AUTO mode
}

void loop() {
  uint32_t now = millis();

  // 1) B3 -> mod değiştir (sadece basıldığı an) / B3 -> toggle mode (on press only)
  bool b3 = iotbot.button3Read();
  if (b3 && !lastB3) setMode(!manualMode, now);
  lastB3 = b3;

  // 2) Seri komutlar / Serial commands
  String cmd;
  if (readCommand(cmd)) handleCommand(cmd, now);

  // 3) Otomatik demo veya manuel kontrol / automatic demo or manual control
  if (!manualMode) {
    runAutoDemo(now);
  } else {
    runManual();
    if (timedDrive && (int32_t)(now - driveUntilMs) >= 0) {
      timedDrive = false;
      targetSpeed = 0;
      iotbot.serialWrite(L("Süre doldu, araç duruyor.", "Time is up, the car is stopping."));
    }
  }

  // 4) Motor rampası / motor ramp
  updateRamp(now);

  // 5) LCD (200 ms'de bir, titremesiz) / LCD (every 200 ms, no flicker)
  if (now - lastScreenMs >= 200) {
    lastScreenMs = now;
    char line[41];
    snprintf(line, sizeof(line), L("Mod: %s", "Mode: %s"), manualMode ? L("MANUEL", "MANUAL") : L("OTOMATİK", "AUTO"));
    lcdRow(1, line);
    snprintf(line, sizeof(line), L("Hız:%+4d Yön:%3d %s", "Spd:%+4d Str:%3d %s"), appliedSpeed, steerAngle,
             lightsOn ? L("FAR", "LED") : "");
    lcdRow(2, line);
  }
}
