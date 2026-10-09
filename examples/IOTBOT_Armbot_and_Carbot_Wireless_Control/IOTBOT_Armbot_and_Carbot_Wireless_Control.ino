/*
 * IOTBOT Armbot and Carbot Wireless Control (ESP-NOW Master)
 *
 * TR: Tek bir IOTBOT ile CARBOT'u ve ARMBOT'u kablosuz (ESP-NOW) kontrol eder.
 * Robotlarin uzerindeki MINIBOT'lara su ornekleri yukleyin:
 *   - CARBOT: MINIBOT_CARBOT_ESP_NOW_Slave_Control.ino
 *   - ARMBOT: MINIBOT_ARMBOT_ESP_NOW_Slave_Control.ino
 * Hepsi kanal 1'de, yayin (broadcast) adresiyle haberlesir.
 *
 * EN: Controls CARBOT and ARMBOT wirelessly (ESP-NOW) from one IOTBOT.
 * Upload these to the MINIBOTs on the robots:
 *   - CARBOT: MINIBOT_CARBOT_ESP_NOW_Slave_Control.ino
 *   - ARMBOT: MINIBOT_ARMBOT_ESP_NOW_Slave_Control.ino
 * All talk on channel 1 using the broadcast address.
 *
 * Kontroller / Controls:
 *   Mod secimi / mode select : Encoder'i cevir, encoder'a bas / turn the encoder, press it
 *   Calisirken / while running: Encoder'a bas = CARBOT <-> ARMBOT
 *   CARBOT: Joystick Y = ileri/geri hiz / forward-backward speed,
 *           Potansiyometre = direksiyon / steering, Joystick butonu = far / lights,
 *           B3 = korna / horn, Joystick butonu + B3 = far otomatik/elle / lights auto/manual
 *   ARMBOT: Joystick X ve encoder = govde donusu / base rotation, Joystick Y = omuz / shoulder,
 *           Potansiyometre = dirsek / elbow, B3 = kiskac ac/kapa / gripper open/close
 *
 * MAGAZA MODU / STORE MODE: Kontrol edilmeyen robot kendiliginden magaza moduna
 * girer - CARBOT surulurken ARMBOT gosteri yapar, ARMBOT kullanilirken CARBOT
 * (yerinde) gosteri yapar. Robot normal komut alinca hemen cikar.
 * / The robot that is NOT being controlled enters store mode by itself - ARMBOT
 * shows off while CARBOT is driven, CARBOT does an (in-place) show while ARMBOT
 * is used. It leaves store mode as soon as a normal command arrives.
 *
 * NOT / NOTE: Joystick X (GPIO15) bir ADC2 pinidir; ESP32'de WiFi/ESP-NOW
 * calisirken ADC2 cogu zaman MESGULDUR. X, okumanin gercekten basarili olup
 * olmadigini soyleyen adc2_get_raw() ile denenir: basarili okuma varsa govde
 * joystick X ile doner, yoksa X ortada sayilir ve encoder calismaya devam eder.
 * Ekranda "X+" = X okunuyor, "X-" = okunamiyor. Mod secimi encoder iledir.
 * / Joystick X (GPIO15) is an ADC2 pin; on the ESP32 ADC2 is usually BUSY while
 * WiFi/ESP-NOW runs. X is tried with adc2_get_raw(), which reports whether the
 * read really succeeded: if it does, the base turns with joystick X, otherwise X
 * counts as centered and the encoder keeps working. Screen: "X+" = X readable,
 * "X-" = not readable. Mode selection uses the encoder.
 *
 * Protokol / Protocol (CodlaiESPNowMessage.deviceType):
 *   1 = ARMBOT komutu / command (axis1..3 + gripper, 0-180 derece / degrees)
 *   2 = CARBOT komutu / command (axis1 = direksiyon / steering 45-135,
 *       axis2 = hiz / speed 0-180: 90 dur / stop, <80 geri / backward, >100 ileri / forward,
 *       action 1 korna / horn, 2 far acik / lights on, 3 far kapali / lights off)
 *   3 = CARBOT telemetrisi / telemetry (axis3 = mesafe cm / distance cm, -1 = bilinmiyor / unknown)
 *   4 = ARMBOT "buradayim" sinyali / heartbeat
 *   Tip 1 veya 2'de action = 10: o robot magaza moduna girsin / that robot enters store mode
 *
 * Seri port (115200 baud), Türkçe veya İngilizce / Serial port, Turkish or English:
 *   yardim / help   -> komut listesi / command list
 *   durum / status  -> mod, robot bağlantıları, açılar / mode, robot links, angles
 *   dil / lang      -> ekran ve seri port dilini değiştir / switch screen and serial language
 * Ekranda / on screen: "*" = robottan sinyal var / signal from the robot, "-" = yok / none,
 * "(O)" / "(A)" = far otomatik (LDR) / lights automatic (LDR).
 */

#define USE_ESPNOW
#define USE_WIFI
#include <IOTBOT.h>
#include <driver/adc.h> // adc2_get_raw: joystick X (GPIO15 = ADC2 kanal 3)

IOTBOT iotbot;

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

// Broadcast Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Data Structures
CodlaiESPNowMessage armData;
CodlaiESPNowMessage carData;

static const uint8_t TYPE_ARM_CMD = 1;
static const uint8_t TYPE_CAR_CMD = 2;
static const uint8_t TYPE_CAR_TELEMETRY = 3;
static const uint8_t TYPE_ARM_HEARTBEAT = 4;
static const uint8_t ACTION_STORE_MODE = 10;       // Kontrol edilmeyen robota: magaza modu / to the idle robot: store mode
static const unsigned long STORE_SEND_MS = 500;    // Magaza komutu araligi / store command interval
static unsigned long lastStoreSendMs = 0;

// Variables for Logic
static int joyYCenter = 2048;
static int joyXCenter = 2048;
static int joyXCache = 2048;              // Son BASARILI X okumasi / last SUCCESSFUL X read
static unsigned long lastXOkMs = 0;       // Son basarili X okumasinin zamani / time of last successful X read
static const unsigned long X_FRESH_MS = 200; // Bundan eski X okumasi kullanilmaz / older X reads are not used
static const int DEADZONE = 500;

// Mode switching
enum Mode
{
  MODE_CARBOT = 0,
  MODE_ARMBOT = 1
};
static Mode currentMode = MODE_CARBOT;
static int modeSelIndex = 0; // 0=CARBOT, 1=ARMBOT
enum AppState
{
  APP_SELECT = 0,
  APP_RUN = 1
};

// Enum parametreli fonksiyonlarin prototipleri: Arduino IDE otomatik
// prototipleri enum tanimindan ONCE yazdigi icin "declared void" hatasi
// veriyordu. / Prototypes of functions taking an enum: the Arduino IDE
// writes its auto-prototypes BEFORE the enum ("declared void" error).
void enterMode(Mode m);
static AppState appState = APP_SELECT;

// Buttons & Debounce
static bool encBtnPrev = false;
static unsigned long encLastMs = 0;
static const unsigned long ENC_DEBOUNCE_MS = 150;
static unsigned long bootMs0 = 0;
static const unsigned long ENC_BOOT_GUARD_MS = 800;
static int lastEncoderVal = 0;

// Joystick button state machine for LED control
static uint8_t joyBtnState = 0; // 0=IDLE, 1=DEBOUNCE_PRESS, 2=PRESSED, 3=WAIT_RELEASE
static unsigned long joyBtnTimer = 0;
static const unsigned long JOY_DEBOUNCE_MS = 30;

// Armbot State
static int armRotAngle = 90;      // Axis1
static int armShoulderAngle = 90; // Axis2
static int armElbowAngle = 50;    // Axis3
static int armGripAngle = 60;     // Gripper
static const int JOY_DEADZONE_ARM = 300;
static const int JOY_STEP_MAX = 5;
static const int JOY_STEP_DIV = 300;
static const int ENCODER_DEG_PER_COUNT = 3;          // Encoder sayimi basina govde donusu / base rotation per encoder count
static const unsigned long ARM_TICK_MS = 20;         // Joystick adimlari sabit hizda (50 Hz) / joystick steps at a fixed rate (50 Hz)
static unsigned long lastArmTickMs = 0;

// Carbot State
static bool ledState = true;
static bool ledAutoMode = true;
static bool b3Prev = false;
static unsigned long b3LastChange = 0;
static const unsigned long B3_DEBOUNCE_MS = 50;

// Link / telemetry
static unsigned long lastCarRxMs = 0;    // Son CARBOT telemetrisi / last CARBOT telemetry
static unsigned long lastArmRxMs = 0;    // Son ARMBOT sinyali / last ARMBOT heartbeat
static unsigned long lastDistanceMs = 0; // Son GECERLI mesafe / last VALID distance
static const unsigned long LINK_TIMEOUT_MS = 1500;
static const unsigned long DISTANCE_TIMEOUT_MS = 1000;
static int carbotDistance = -1;          // -1 = bilinmiyor / unknown

// Sending / screen timers
static unsigned long lastSendMs = 0;
static const unsigned long SEND_INTERVAL_MS = 50;
static unsigned long lastLcdMs = 0;
static const unsigned long LCD_INTERVAL_MS = 150;

bool linkAlive(unsigned long lastRxMs, unsigned long now)
{
  return lastRxMs != 0 && (now - lastRxMs) < LINK_TIMEOUT_MS;
}

// Joystick X'i (GPIO15 = ADC2 kanal 3) okumayi dene. analogRead() ADC2 mesgulken
// sessizce 0 dondurur (bu da "tam sol" ile karisir); adc2_get_raw() ise okumanin
// basarili olup olmadigini soyler. Basariliysa onbellegi tazeler.
// / Try to read joystick X (GPIO15 = ADC2 channel 3). analogRead() silently returns
// 0 while ADC2 is busy (which looks like "full left"); adc2_get_raw() tells whether
// the read really succeeded. On success it refreshes the cache.
void tryReadJoystickX(unsigned long now)
{
  static bool configured = false;
  if (!configured)
  {
    adc2_config_channel_atten(ADC2_CHANNEL_3, ADC_ATTEN_DB_11); // 0-3.3V, analogRead ile ayni / same as analogRead
    configured = true;
  }
  int raw = 0;
  if (adc2_get_raw(ADC2_CHANNEL_3, ADC_WIDTH_BIT_12, &raw) == ESP_OK)
  {
    joyXCache = raw;
    lastXOkMs = now;
  }
}

bool joystickXFresh(unsigned long now)
{
  return lastXOkMs != 0 && (now - lastXOkMs) < X_FRESH_MS;
}

// Kontrol edilmeyen robota magaza modu komutu (500 ms'de bir). Eksen degerleri
// mevcut degerlerdir: magaza modunu bilmeyen eski alicilar da yerinde kalir.
// / Store-mode command to the robot that is NOT being controlled (every 500 ms).
// Axis values are the current ones, so older receivers that don't know store
// mode simply stay put.
void sendStoreModeToIdleRobot(unsigned long now)
{
  if (now - lastStoreSendMs < STORE_SEND_MS)
    return;
  lastStoreSendMs = now;
  if (currentMode == MODE_CARBOT)
  {
    armData.axis1 = armRotAngle;
    armData.axis2 = armShoulderAngle;
    armData.axis3 = armElbowAngle;
    armData.gripper = armGripAngle;
    armData.action = ACTION_STORE_MODE;
    iotbot.sendESPNow(broadcastAddress, (uint8_t *)&armData, sizeof(armData));
  }
  else
  {
    carData.axis1 = 90;
    carData.axis2 = 90; // Dur / stop
    carData.action = ACTION_STORE_MODE;
    iotbot.sendESPNow(broadcastAddress, (uint8_t *)&carData, sizeof(carData));
  }
}

// Satir 1: "FAR:AÇIK   (O)" - far durumu ve otomatik isareti / row 1: light state and auto marker
// (lcdWriteFixedTxt Turkce harfleri dogru gosterir / shows Turkish letters correctly)
void drawLedField()
{
  iotbot.lcdWriteCR(0, 1, L("FAR:", "LED:"));
  iotbot.lcdWriteFixedTxt(4, 1, ledState ? L("AÇIK", "ON") : L("KAPALI", "OFF"), 7);
  iotbot.lcdWriteFixedTxt(11, 1, ledAutoMode ? L("(O)", "(A)") : "", 3);
}

void showCarbotScreen()
{
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "CARBOT (WIFI)");
  drawLedField();
  iotbot.lcdWriteFixedTxt(0, 2, L("Mesafe: ---", "Dist:   ---"), 16);
  iotbot.lcdWriteFixedTxt(0, 3, L("JBtn:Far B3:Korna", "JBtn:LED B3:Horn"), 20);
}

void showArmbotScreen()
{
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "ARMBOT (WIFI)");
  // GÖV=gövde OMZ=omuz DRS=dirsek KSK=kıskaç / ROT=base SHO=shoulder ELB=elbow GRP=gripper
  iotbot.lcdWriteCR(0, 1, L("GÖV:", "ROT:"));
  iotbot.lcdWriteCR(8, 1, L("OMZ:", "SHO:"));
  iotbot.lcdWriteCR(0, 2, L("DRS:", "ELB:"));
  iotbot.lcdWriteCR(8, 2, L("KSK:", "GRP:"));
  iotbot.lcdWriteFixedTxt(0, 3, L("X/ENC:Dön B3:Kıskaç", "X/ENC:Turn B3:Grip"), 20);
}

void updateModeSelectLine()
{
  if (modeSelIndex == 0)
    iotbot.lcdWriteFixedTxt(0, 2, " [CARBOT]   ARMBOT", 20);
  else
    iotbot.lcdWriteFixedTxt(0, 2, "  CARBOT   [ARMBOT]", 20);
}

void showModeSelectScreen()
{
  iotbot.lcdClear();
  iotbot.lcdWriteCR(1, 0, L("Kablosuz Mod Seçin", "Select Wireless Mode"));
  updateModeSelectLine();
  iotbot.lcdWriteFixedTxt(0, 3, L("Çevir:Seç  Bas:Başla", "Turn:Pick Push:Start"), 20);
}

// CARBOT'tan cikarken araci HEMEN durdur (ARMBOT moduna gecince artik araca
// komut gitmez; alici zaman asimi da durdurur ama beklemeyelim).
// / Stop the car RIGHT AWAY when leaving CARBOT mode (no more car commands are
// sent in ARMBOT mode; the receiver's timeout would stop it too, but don't wait).
void sendCarStop()
{
  carData.axis1 = 90;
  carData.axis2 = 90;
  carData.action = ledState ? 2 : 3;
  for (int i = 0; i < 3; i++)
  {
    iotbot.sendESPNow(broadcastAddress, (uint8_t *)&carData, sizeof(carData));
    delay(10);
  }
}

void enterMode(Mode m)
{
  if (currentMode == MODE_CARBOT && m != MODE_CARBOT && appState == APP_RUN)
    sendCarStop();
  currentMode = m;
  lastEncoderVal = iotbot.encoderRead(); // Eski donusleri sayma / ignore old turns
  if (m == MODE_CARBOT)
    showCarbotScreen();
  else
    showArmbotScreen();
  lastLcdMs = 0; // Degerleri hemen ciz / draw values right away
}

// ---------------------------------------------------------------------------
// Seri komutlar (yalnizca yardim/durum/dil; kontrol dongusunu etkilemez)
// Serial commands (help/status/lang only; they don't affect the control loop)
// Seri Monitör'ün satır sonu ayarı ne olursa olsun çalışır (NL, CR, ikisi, hiçbiri).
// Works with any Serial Monitor line-ending setting (NL, CR, both, none).
// ---------------------------------------------------------------------------
String cmdBuffer;
unsigned long lastCharMs = 0;

// Küçük harfe çevirir ve Türkçe harfleri sadeleştirir: "YARDIM" -> "yardim"
// Lower-cases and simplifies Turkish letters: "YARDIM" -> "yardim"
String normalizeCommand(String s)
{
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

bool readCommand(String &cmd)
{
  while (iotbot.serialAvailable() > 0)
  {
    char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r')
    {
      if (cmdBuffer.length() == 0)
        continue;
      cmd = normalizeCommand(cmdBuffer);
      cmdBuffer = "";
      return true;
    }
    if (cmdBuffer.length() < 40)
      cmdBuffer += c;
  }
  // "Satır sonu yok" seçiliyse: 150 ms sessizlikten sonra komutu kabul et.
  // "No line ending" selected: accept the command after 150 ms of silence.
  if (cmdBuffer.length() > 0 && millis() - lastCharMs > 150)
  {
    cmd = normalizeCommand(cmdBuffer);
    cmdBuffer = "";
    return true;
  }
  return false;
}

void printHelp()
{
  iotbot.serialWrite(L("---- KABLOSUZ KUMANDA - Komutlar ----", "---- WIRELESS CONTROLLER - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  durum  : mod, bağlantılar, açılar", "  status : mode, links, angles"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
  iotbot.serialWrite(L("  Encoder: mod seç / CARBOT <-> ARMBOT", "  Encoder: pick mode / CARBOT <-> ARMBOT"));
  iotbot.serialWrite(L("  CARBOT: Joy Y=hız, Pot=direksiyon, Joy butonu=far, B3=korna, Joy butonu+B3=far oto/elle",
                       "  CARBOT: Joy Y=speed, Pot=steering, Joy button=lights, B3=horn, Joy button+B3=lights auto/manual"));
  iotbot.serialWrite(L("  ARMBOT: Joy X/encoder=gövde, Joy Y=omuz, Pot=dirsek, B3=kıskaç",
                       "  ARMBOT: Joy X/encoder=base, Joy Y=shoulder, Pot=elbow, B3=gripper"));
}

void printStatus()
{
  unsigned long now = millis();
  String s = String(L("Mod: ", "Mode: ")) +
             (appState == APP_SELECT ? L("seçim ekranı", "mode select") : (currentMode == MODE_CARBOT ? "CARBOT" : "ARMBOT"));
  s += String(L(" | CARBOT bağlantı: ", " | CARBOT link: ")) + (linkAlive(lastCarRxMs, now) ? L("var", "yes") : L("yok", "no"));
  s += String(L(" | ARMBOT bağlantı: ", " | ARMBOT link: ")) + (linkAlive(lastArmRxMs, now) ? L("var", "yes") : L("yok", "no"));
  iotbot.serialWrite(s);
  bool distanceKnown = lastDistanceMs != 0 && (now - lastDistanceMs) < DISTANCE_TIMEOUT_MS;
  iotbot.serialWrite(String(L("CARBOT: mesafe ", "CARBOT: distance ")) + (distanceKnown ? String(carbotDistance) + " cm" : String("---")) +
                     L(" | far ", " | lights ") + (ledState ? L("açık", "on") : L("kapalı", "off")) +
                     (ledAutoMode ? L(" (otomatik)", " (auto)") : ""));
  iotbot.serialWrite(String(L("ARMBOT: gövde ", "ARMBOT: base ")) + armRotAngle + L(" omuz ", " shoulder ") + armShoulderAngle +
                     L(" dirsek ", " elbow ") + armElbowAngle + L(" kıskaç ", " gripper ") + armGripAngle);
}

void handleCommand(const String &cmd)
{
  if (cmd == "yardim" || cmd == "help" || cmd == "?")
  {
    printHelp();
  }
  else if (cmd == "durum" || cmd == "status")
  {
    printStatus();
  }
  else if (cmd == "dil" || cmd == "lang" || cmd == "language")
  {
    turkish = !turkish;
    iotbot.serialWrite(L("Dil: Türkçe", "Language: English"));
    // Ekrani yeni dille yeniden ciz / redraw the screen in the new language
    if (appState == APP_SELECT)
      showModeSelectScreen();
    else if (currentMode == MODE_CARBOT)
      showCarbotScreen();
    else
      showArmbotScreen();
    lastLcdMs = 0;
    printHelp();
  }
  else
  {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

void setup()
{
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.serialWrite(L("IOTBOT kablosuz kumanda başlıyor...", "IOTBOT wireless controller starting..."));

  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, L("Kalibrasyon?", "Calibrate?"));
  iotbot.lcdWriteCR(0, 1, L("B3=Evet, ENC=Geç", "B3=Yes, ENC=Skip"));

  bool doCalibration = false;
  unsigned long startMillis = millis();
  bool b3WasPressed = iotbot.button3Read();
  bool encWasPressed = (digitalRead(ENCODER_BUTTON_PIN) == LOW);

  while (millis() - startMillis < 5000)
  {
    bool b3PressedNow = iotbot.button3Read();
    bool encPressedNow = (digitalRead(ENCODER_BUTTON_PIN) == LOW);

    // Require an actual press transition, protecting against stuck states
    if (b3PressedNow && !b3WasPressed)
    {
      doCalibration = true;
      break;
    }
    if (encPressedNow && !encWasPressed)
    {
      break;
    }

    b3WasPressed = b3PressedNow;
    encWasPressed = encPressedNow;

    int timeLeft = 5 - ((millis() - startMillis) / 1000);
    // Geri sayim 3. satirda (eskiden 2. satirdaki yazinin ustune biniyordu)
    // / Countdown on row 3 (it used to overwrite the text on row 2)
    iotbot.lcdWriteFixedTxt(0, 2, (String(L("Süre: ", "Time: ")) + timeLeft + L(" sn", " s")).c_str(), 12);
    delay(50);
  }

  iotbot.lcdClear();
  if (doCalibration)
  {
    iotbot.lcdWriteCR(0, 0, L("Joystick'i bırakın", "Release joystick"));
    iotbot.lcdWriteCR(0, 1, L("Sonra B3'e basın", "Then press B3"));
    delay(500); // debounce B3 from previous selection
    while (!iotbot.button3Read())
    {
      delay(50);
    }
  }
  else
  {
    iotbot.lcdWriteCR(0, 0, L("Kalibre ediliyor...", "Calibrating..."));
    iotbot.lcdWriteCR(0, 1, L("Dokunmayın!", "Do not touch!"));
    delay(1000);
  }
  // Joystick orta noktalari. ESP-NOW HENUZ baslamadi, bu yuzden X (ADC2) de
  // simdi guvenle okunabilir. / Joystick centers. ESP-NOW has NOT started yet,
  // so X (ADC2) can still be read reliably here.
  long xSum = 0, ySum = 0;
  for (int i = 0; i < 50; i++)
  {
    xSum += iotbot.joystickXRead();
    ySum += iotbot.joystickYRead();
    delay(10);
  }
  joyXCenter = xSum / 50;
  joyYCenter = ySum / 50;
  if (abs(joyXCenter - 2048) > 1000)
    joyXCenter = 2048;
  if (abs(joyYCenter - 2048) > 1000)
    joyYCenter = 2048;
  joyXCache = joyXCenter;
  iotbot.serialWrite(String(L("Joystick orta noktası X/Y: ", "Calibrated center X/Y: ")) + String(joyXCenter) + ", " + String(joyYCenter));

  armData.deviceType = TYPE_ARM_CMD;
  carData.deviceType = TYPE_CAR_CMD;

  iotbot.initESPNow();
  iotbot.setWiFiChannel(1);
  iotbot.startListening();
  if (!iotbot.addBroadcastPeer(1))
    iotbot.serialWrite(L("Yayın (broadcast) eşi eklenemedi!", "Failed to add the broadcast peer!"));

  showModeSelectScreen();
  bootMs0 = millis();
  lastEncoderVal = iotbot.encoderRead();
  iotbot.serialWrite(L("Kurulum tamam. Encoder ile mod seçin.", "Setup complete. Pick a mode with the encoder."));
  printHelp();
}

// Gelen paketler: CARBOT telemetrisi ve ARMBOT sinyali.
// / Incoming packets: CARBOT telemetry and ARMBOT heartbeat.
void handleIncoming(unsigned long now)
{
  if (!iotbot.newData)
    return;
  CodlaiESPNowMessage msg = iotbot.receivedData;
  iotbot.newData = false;

  bool carTelemetry = (msg.deviceType == TYPE_CAR_TELEMETRY);
  // Eski CARBOT alicilari telemetriyi tip 2 ile gonderir (action=0, hiz=0);
  // gercek komutlarda action hep 1-3'tur. / Older CARBOT receivers send
  // telemetry as type 2 (action=0, speed=0); real commands always have action 1-3.
  if (msg.deviceType == TYPE_CAR_CMD && msg.action == 0 && msg.axis2 == 0)
    carTelemetry = true;

  if (carTelemetry)
  {
    lastCarRxMs = now;
    if (msg.axis3 >= 0)
    {
      carbotDistance = msg.axis3;
      lastDistanceMs = now;
    }
  }
  else if (msg.deviceType == TYPE_ARM_HEARTBEAT)
  {
    lastArmRxMs = now;
  }
}

void loop()
{
  unsigned long now = millis();
  bool encPressed = (digitalRead(ENCODER_BUTTON_PIN) == LOW);
  bool encClick = (now - bootMs0) > ENC_BOOT_GUARD_MS && encPressed && !encBtnPrev && (now - encLastMs) > ENC_DEBOUNCE_MS;
  if (encClick)
    encLastMs = now;
  encBtnPrev = encPressed;

  handleIncoming(now);

  // Seri komutlar: veri yoksa hemen doner / serial commands: returns at once when there is no data
  String cmd;
  if (readCommand(cmd))
    handleCommand(cmd);

  // --- Mode Selection (encoder turn) ---
  if (appState == APP_SELECT)
  {
    int enc = iotbot.encoderRead();
    int diff = enc - lastEncoderVal;
    if (diff != 0)
    {
      lastEncoderVal = enc;
      int prevSel = modeSelIndex;
      modeSelIndex = (diff > 0) ? 1 : 0;
      if (modeSelIndex != prevSel)
        updateModeSelectLine();
    }
    if (encClick)
    {
      appState = APP_RUN;
      enterMode((modeSelIndex == 0) ? MODE_CARBOT : MODE_ARMBOT);
    }
    return;
  }

  // --- Running Mode ---
  if (encClick)
    enterMode((currentMode == MODE_CARBOT) ? MODE_ARMBOT : MODE_CARBOT);

  // --- ARMBOT Logic ---
  if (currentMode == MODE_ARMBOT)
  {
    // Govde donusu: encoder (her loop'ta, adim kacirmamak icin)
    // / Base rotation: encoder (every loop, so no counts are missed)
    int currEnc = iotbot.encoderRead();
    int encDiff = currEnc - lastEncoderVal;
    lastEncoderVal = currEnc;
    if (encDiff != 0)
      armRotAngle = constrain(armRotAngle + encDiff * ENCODER_DEG_PER_COUNT, 0, 180);

    // Joystick X'i her loop'ta dene (ADC2 bos oldugu anlari yakalamak icin)
    // / Try joystick X every loop (to catch the moments ADC2 is free)
    tryReadJoystickX(now);

    // Govde (joystick X) ve omuz (joystick Y): SABIT hizda, loop hizindan bagimsiz
    // / Base (joystick X) and shoulder (joystick Y): at a FIXED rate, independent of loop speed
    if (now - lastArmTickMs >= ARM_TICK_MS)
    {
      lastArmTickMs = now;
      // X okumasi tazeyse kullan; degilse X ortada sayilir (kol kendi kendine donmez)
      // / Use X only when fresh; otherwise X counts as centered (the arm never turns by itself)
      if (joystickXFresh(now))
      {
        int dx = joyXCache - joyXCenter;
        if (abs(dx) > JOY_DEADZONE_ARM)
        {
          int step = min((abs(dx) / JOY_STEP_DIV) + 1, JOY_STEP_MAX);
          // Kablolu kumandayla ayni yon: sola itince aci artar / same direction as the wired controller: left increases the angle
          armRotAngle = constrain(armRotAngle + (dx < 0 ? step : -step), 0, 180);
        }
      }
      int dy = iotbot.joystickYRead() - joyYCenter;
      if (abs(dy) > JOY_DEADZONE_ARM)
      {
        int step = min((abs(dy) / JOY_STEP_DIV) + 1, JOY_STEP_MAX);
        armShoulderAngle = constrain(armShoulderAngle + (dy < 0 ? -step : step), 0, 180);
      }
    }

    // Dirsek: potansiyometre / Elbow: potentiometer
    armElbowAngle = constrain(map(iotbot.potentiometerRead(), 0, 4095, 0, 180), 0, 180);

    // Kiskac: B3 ac/kapa / Gripper: B3 open/close
    bool b3Now = iotbot.button3Read();
    if (b3Now && !b3Prev && (now - b3LastChange) > B3_DEBOUNCE_MS)
    {
      const int GRIP_OPEN = 20;
      const int GRIP_CLOSE = 120;
      armGripAngle = (armGripAngle < (GRIP_OPEN + GRIP_CLOSE) / 2) ? GRIP_CLOSE : GRIP_OPEN;
      b3LastChange = now;
    }
    b3Prev = b3Now;

    armData.axis1 = armRotAngle;
    armData.axis2 = armShoulderAngle;
    armData.axis3 = armElbowAngle;
    armData.gripper = armGripAngle;
    armData.action = 0;

    if (now - lastSendMs >= SEND_INTERVAL_MS)
    {
      iotbot.sendESPNow(broadcastAddress, (uint8_t *)&armData, sizeof(armData));
      lastSendMs = now;
    }

    if (now - lastLcdMs >= LCD_INTERVAL_MS)
    {
      lastLcdMs = now;
      iotbot.lcdWriteFixed(4, 1, armRotAngle, 3);
      iotbot.lcdWriteFixed(12, 1, armShoulderAngle, 3);
      iotbot.lcdWriteFixed(4, 2, armElbowAngle, 3);
      iotbot.lcdWriteFixed(12, 2, armGripAngle, 3);
      // "*" = ARMBOT'tan sinyal geliyor / heartbeat received, "-" = yok / none
      iotbot.lcdWriteFixedTxt(18, 0, linkAlive(lastArmRxMs, now) ? "*" : "-", 2);
      // "X+" = joystick X okunabiliyor / readable, "X-" = ADC2 mesgul / busy
      iotbot.lcdWriteFixedTxt(15, 0, joystickXFresh(now) ? "X+" : "X-", 2);
    }
    // ARMBOT kullanilirken CARBOT magaza modunda / CARBOT in store mode while ARMBOT is used
    sendStoreModeToIdleRobot(now);
    return;
  }

  // --- CARBOT Logic ---
  int dy = iotbot.joystickYRead() - joyYCenter;

  // Direksiyon: potansiyometre (joystick X ESP-NOW ile okunamiyor)
  // / Steering: potentiometer (joystick X cannot be read with ESP-NOW on)
  int steerAngle = constrain(map(iotbot.potentiometerRead(), 0, 4095, 135, 45), 45, 135);
  carData.axis1 = steerAngle;

  // Mesafe bilgisi eskidiyse (far acikken alici olcum yapamaz) bilinmiyor say.
  // / Treat a stale distance as unknown (the receiver cannot measure while the lights are on).
  bool distanceKnown = lastDistanceMs != 0 && (now - lastDistanceMs) < DISTANCE_TIMEOUT_MS;
  bool tooClose = distanceKnown && carbotDistance < 10;

  // Hiz: 0-180 (90 dur, <80 geri, >100 ileri) / Speed: 0-180 (90 stop, <80 back, >100 forward)
  int speed = 90;
  if (dy < -DEADZONE)
    speed = map(dy, -DEADZONE, -joyYCenter, 80, 0);
  else if (dy > DEADZONE && !tooClose)
    speed = map(dy, DEADZONE, 4095 - joyYCenter, 100, 180);
  speed = constrain(speed, 0, 180);
  carData.axis2 = speed;
  bool forwardBlocked = (dy > DEADZONE && tooClose);

  // LED - Joystick & B3 Button Handling
  const int LDR_THRESHOLD = 1200;
  bool autoShouldOn = (iotbot.ldrRead() < LDR_THRESHOLD);

  bool joyBtnRaw = !iotbot.joystickButtonRead(); // Basiliyken LOW / LOW while pressed
  bool b3Now = iotbot.button3Read();

  static bool comboHandled = false;
  static unsigned long comboTimer = 0;
  bool ledChanged = false;

  // Joystick butonu + B3 birlikte: far otomatik/elle / together: lights auto/manual
  if (joyBtnRaw && b3Now)
  {
    if (!comboHandled && (now - comboTimer > JOY_DEBOUNCE_MS))
    {
      ledAutoMode = !ledAutoMode;
      iotbot.buzzerPlay(1500, 100);
      comboHandled = true;
      ledChanged = true;
    }
  }
  else
  {
    comboTimer = now;
    if (!joyBtnRaw && !b3Now)
      comboHandled = false;
  }

  // Tek joystick basisi: far ac/kapa (birakinca) / single press: lights toggle (on release)
  switch (joyBtnState)
  {
  case 0:
    if (joyBtnRaw)
    {
      joyBtnState = 1;
      joyBtnTimer = now;
    }
    break;
  case 1:
    if (now - joyBtnTimer > JOY_DEBOUNCE_MS)
      joyBtnState = joyBtnRaw ? 2 : 0;
    break;
  case 2:
    if (!joyBtnRaw)
    {
      if (!comboHandled)
      {
        ledState = !ledState;
        ledChanged = true;
      }
      joyBtnState = 3;
      joyBtnTimer = now;
    }
    break;
  case 3:
    if (joyBtnRaw)
      joyBtnTimer = now;
    else if (now - joyBtnTimer > JOY_DEBOUNCE_MS)
      joyBtnState = 0;
    break;
  }

  if (ledAutoMode && ledState != autoShouldOn)
  {
    ledState = autoShouldOn;
    ledChanged = true;
  }
  if (ledChanged)
    drawLedField();

  if (b3Now && !comboHandled)
    carData.action = 1; // Korna / horn
  else
    carData.action = ledState ? 2 : 3;

  if (now - lastSendMs >= SEND_INTERVAL_MS)
  {
    iotbot.sendESPNow(broadcastAddress, (uint8_t *)&carData, sizeof(carData));
    lastSendMs = now;
  }

  // CARBOT surulurken ARMBOT magaza modunda / ARMBOT in store mode while CARBOT is driven
  sendStoreModeToIdleRobot(now);

  if (now - lastLcdMs >= LCD_INTERVAL_MS)
  {
    lastLcdMs = now;
    iotbot.lcdWriteFixed(17, 0, steerAngle, 3);
    // "*" = CARBOT'tan telemetri geliyor / telemetry received, "-" = yok / none
    iotbot.lcdWriteFixedTxt(14, 0, linkAlive(lastCarRxMs, now) ? "*" : "-", 2);

    static bool blinkOn = false;
    char buf[17];
    if (forwardBlocked)
    {
      // Engel 10 cm'den yakin: ileri kilitli, uyari yanip soner
      // / Obstacle closer than 10 cm: forward locked, warning blinks
      blinkOn = !blinkOn;
      if (blinkOn)
      {
        snprintf(buf, sizeof(buf), "%s", L("   !! DUR !!    ", "   !! STOP !!   "));
        iotbot.buzzerPlay(2500, 40);
      }
      else
        snprintf(buf, sizeof(buf), L("Mesafe: %3d cm  ", "Dist:   %3d cm  "), carbotDistance);
    }
    else if (!distanceKnown)
      snprintf(buf, sizeof(buf), "%s", L("Mesafe: ---     ", "Dist:   ---     "));
    else if (carbotDistance > 300)
      snprintf(buf, sizeof(buf), "%s", L("Mesafe: Serbest ", "Dist:   clear   "));
    else
      snprintf(buf, sizeof(buf), L("Mesafe: %3d cm  ", "Dist:   %3d cm  "), carbotDistance);
    iotbot.lcdWriteFixedTxt(0, 2, buf, 16);
  }
}
