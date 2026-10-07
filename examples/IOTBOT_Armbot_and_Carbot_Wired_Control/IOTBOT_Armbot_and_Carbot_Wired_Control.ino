/*
 * IOTBOT Armbot and Carbot Wired Control
 *
 * TR: IOTBOT, CARBOT'u ya da ARMBOT'u KABLOYLA dogrudan surer. Iki robot da ayni
 * P1-P5 soketlerini (IO25, 26, 27, 32, 33) kullanir: ayni anda TEK robot takilir,
 * mod degistirirken kabloyu da degistirin. Kablosuz surum icin:
 * IOTBOT_Armbot_and_Carbot_Wireless_Control.ino
 * EN: The IOTBOT drives CARBOT or ARMBOT directly over a CABLE. Both robots use
 * the same P1-P5 sockets (IO25, 26, 27, 32, 33): plug in ONE robot at a time and
 * swap the cable when you switch modes. Wireless version:
 * IOTBOT_Armbot_and_Carbot_Wireless_Control.ino
 *
 * Kontroller / Controls:
 *   Mod / mode: Joystick sol/sag + encoder'a bas; calisirken encoder'a bas = mod degistir
 *   CARBOT: Joystick X = direksiyon / steering, Joystick Y = hiz / speed,
 *           B1 = korna (nota potansiyometreden) / horn (note from the pot),
 *           B2 = far otomatik/elle / lights auto/manual, B3 = far ac/kapa (elle) / lights toggle (manual)
 *   ARMBOT: Joystick X = govde / base, Joystick Y = omuz / shoulder, Pot = dirsek / elbow,
 *           Encoder = kiskac / gripper, B3 = kiskac ac/kapa / gripper open/close,
 *           B1 = korna / horn, B2 = nota / note, Joystick butonu = magaza (demo) modu / store (demo) mode
 *
 * Seri port (115200 baud), Türkçe veya İngilizce / Serial port, Turkish or English:
 *   yardim / help   -> komut listesi / command list
 *   durum / status  -> mod, far, kol açıları / mode, lights, arm angles
 *   dil / lang      -> ekran ve seri port dilini değiştir / switch screen and serial language
 *   (Mağaza modu çalışırken seri komutlar beklemede kalır / serial commands wait
 *   while store mode is running.)
 * Ekranda / on screen: "(O)" / "(A)" = far otomatik (LDR) / lights automatic (LDR),
 * NT = korna notası / horn note.
 */

#include <IOTBOT.h>
#include <CARBOT.h>
#include <ARMBOT.h>

IOTBOT iotbot;
CARBOT carBot;
ARMBOT armbot;

// Tip tanimlari EN USTTE olmali: Arduino (ve PlatformIO) fonksiyon prototiplerini
// dosyadaki ilk fonksiyonun onune ekler; bu tipler daha asagida tanimlanirsa
// "'B12State' does not name a type" hatasi alinir. / Type definitions must be AT
// THE TOP: Arduino (and PlatformIO) inserts function prototypes before the first
// function in the file; if these types are defined further down you get
// "'B12State' does not name a type".
struct B12State { bool b1; bool b2; int raw; };
enum Mode
{
  MODE_CARBOT = 0,
  MODE_ARMBOT = 1
};
// App state: startup mode selection vs running
enum AppState
{
  APP_SELECT = 0,
  APP_RUN = 1
};

// Dil seçimi: true = Türkçe, false = English. Seri porttan "dil" / "lang" ile de değişir.
// Language: true = Turkish, false = English. Can also be changed with "dil" / "lang".
// (Tiplerden SONRA olmali, bkz. yukaridaki not / must come AFTER the types, see the note above)
bool turkish = true;
const char *L(const char *tr, const char *en) { return turkish ? tr : en; }

static bool espNowStarted = false;         // ESP-NOW geç başlatma (ADC2 okuma için)

// ADC2 (GPIO15, GPIO4) Wi-Fi çakışmasını hafifletmek için cache + retry mekanizması
static int lastGoodJoyX = 2048; // Joystick X varsayılan orta
static int lastGoodB12Raw = 4095; // B1/B2 analog hat belirteci
static unsigned long lastJoyXReadMs = 0;
static unsigned long lastB12ReadMs = 0;
static const unsigned long JOY_X_PERIOD_MS = 30;  // okuma sıklığını düşür (çarpışmayı azalt)
static const unsigned long B12_PERIOD_MS   = 45;  // buton analog okuma sıklığı

inline int safeReadADC2(int pin, int &cache, int attempts = 6){
  // Birkaç deneme ile düzgün (0..4095 arası ve 0/4095 değil) değer yakalamaya çalış
  for(int i=0;i<attempts;i++){
    int v = iotbot.analogReadPin(pin);
    if(v > 20 && v < 4075){ // makul aralık
      cache = v;
      return cache;
    }
    delayMicroseconds(150); // Wi-Fi ile yarışmayı azalt küçük bekleme
  }
  return cache; // başarısız -> son iyi değer
}

inline int readJoystickXFiltered(){
  unsigned long now = millis();
  if(now - lastJoyXReadMs >= JOY_X_PERIOD_MS){
    safeReadADC2(JOYSTICK_X_PIN, lastGoodJoyX);
    lastJoyXReadMs = now;
  }
  return lastGoodJoyX;
}

inline B12State readB12Filtered(){
  unsigned long now = millis();
  if(now - lastB12ReadMs >= B12_PERIOD_MS){
    safeReadADC2(B1_AND_B2_BUTTON_PIN, lastGoodB12Raw);
    lastB12ReadMs = now;
  }
  B12State s; s.raw = lastGoodB12Raw; s.b1 = (s.raw > 3500); s.b2 = (s.raw > 1500 && s.raw < 3000); return s;
}

// Fixed joystick center assumption (later we can auto-calibrate)
static const int JOY_CENTER = 2048;
static const int DEADZONE = 300; // Deadzone for joystick axes

inline int joyDelta(int raw) { return raw - JOY_CENTER; }

// LED toggle via B3 button
static bool ledState = true;           // default ON (auto may override)
static bool ledAutoMode = true;        // AUTO by default (global for screen updates)
static bool b3Prev = false;            // previous reading
static unsigned long b3LastChange = 0; // debounce timer
static const unsigned long B3_DEBOUNCE_MS = 50;

// Horn and extra buttons (Joystick button, B1, B2)
static bool b1Prev = false;
static bool b2Prev = false;
static unsigned long lastBeepMs = 0;
static const unsigned long BEEP_DEBOUNCE_MS = 180; // avoid tone spam

// Mode switching via encoder button (CARBOT / ARMBOT)
static bool encBtnPrev = false;
static unsigned long encLastMs = 0;
static const unsigned long ENC_DEBOUNCE_MS = 150;
static Mode currentMode = MODE_CARBOT;
static unsigned long bootMs0 = 0;
static const unsigned long ENC_BOOT_GUARD_MS = 800; // ignore encoder for first 0.8s
static AppState appState = APP_SELECT;
static int modeSelIndex = 0; // 0=CARBOT, 1=ARMBOT
// Blink state for AUTO warning when pressing B3 in AUTO mode
static bool aBlinkActive = false;
static bool aBlinkPhaseOn = true;
static int aBlinkCycles = 0; // counts ON states reached
static unsigned long aBlinkNextMs = 0;

// ARMBOT state & buzzer
static bool armbotInited = false;
static int armRotAngle = 90;     // Axis1
static int armShoulderAngle = 90;// Axis2
static int armElbowAngle = 50;   // Axis3
static int armGripAngle = 60;    // Gripper
static int armNoteIdx = 0;       // ARMBOT note selection (via B2)
static bool armHornActive = false;
// Store (mağaza) modu: ARMBOT sırasında joystick butonuyla aç/kapa
static bool storeModeActive = false;
static bool storeJoyPrev = false;
static unsigned long storeJoyLastMs = 0;
static const unsigned long STORE_JOY_DEBOUNCE_MS = 300;

// Debounced joystick button check (returns true on a new press)
inline bool joystickPressedDebounce(bool &prev, unsigned long &lastMs, unsigned long db=STORE_JOY_DEBOUNCE_MS){
  // joystickButtonRead() ham pin seviyesini verir: BASILIYKEN LOW (false).
  // Eskiden ters okunuyordu; ARMBOT moduna girer girmez magaza modu kendiliginden
  // basliyor, cikis da birakinca oluyordu. / joystickButtonRead() returns the
  // raw pin level: LOW (false) while PRESSED. It used to be read inverted, so
  // store mode started by itself on entering ARMBOT mode and exited on release.
  bool now = !iotbot.joystickButtonRead();
  unsigned long t = millis();
  if(now && !prev && (t - lastMs) > db){ prev = true; lastMs = t; return true; }
  if(!now) prev = false;
  return false;
}

// Placeholder for store/demo movements — implement your movements here
// Forward declaration: showArmbotScreen is defined later in this file but
// storeModeActions may call it, so declare it here for the compiler.
void showArmbotScreen();
void ensureArmbotInit(); // Ekrandaki aci alanlarini doldurur / fills the angle fields on screen

void storeModeActions(){
  const int SLOW_SPEED = 8; // larger -> slower movement (ms per step)
  // Save current positions to restore later
  int savedRot = armRotAngle;
  int savedElb = armElbowAngle;
  int savedGrip = armGripAngle;

  while(storeModeActive){
    // Allow exit if joystick pressed
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){
      storeModeActive = false; break;
    }

    // 1) Elbow: move between 15 and 120 degrees 3 times, finish at 60
    for(int cycle=0; cycle<3 && storeModeActive; cycle++){
      armbot.axis3Motion(15, SLOW_SPEED);
      armElbowAngle = 15;
      delay(300);
      if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

      armbot.axis3Motion(120, SLOW_SPEED);
      armElbowAngle = 120;
      delay(400);
      if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

      // return to low for full back-and-forth
      armbot.axis3Motion(15, SLOW_SPEED);
      armElbowAngle = 15;
      delay(300);
      if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }
    }
    if(!storeModeActive) break;
    // finish elbow at 60
    armbot.axis3Motion(60, SLOW_SPEED);
    armElbowAngle = 60;
    delay(300);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

    // 2) Rotation sequence: 0 -> 180 -> 90 -> 180
    armbot.axis1Motion(0, SLOW_SPEED);
    armRotAngle = 0;
    delay(400);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

    armbot.axis1Motion(180, SLOW_SPEED);
    armRotAngle = 180;
    delay(500);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

    armbot.axis1Motion(90, SLOW_SPEED);
    armRotAngle = 90;
    delay(400);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

    armbot.axis1Motion(180, SLOW_SPEED);
    armRotAngle = 180;
    delay(400);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

    // 3) Elbow to 20
    armbot.axis3Motion(20, SLOW_SPEED);
    armElbowAngle = 20;
    delay(300);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

    // 4) Close gripper
    armbot.gripperMotion(120, SLOW_SPEED);
    armGripAngle = 120;
    delay(350);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

    // 5) Elbow to 70
    armbot.axis3Motion(70, SLOW_SPEED);
    armElbowAngle = 70;
    delay(350);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

    // 6) Rotation to 0
    armbot.axis1Motion(0, SLOW_SPEED);
    armRotAngle = 0;
    delay(400);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

    // 7) Elbow to 120 and open gripper
    armbot.axis3Motion(120, SLOW_SPEED);
    armElbowAngle = 120;
    delay(400);
    armbot.gripperMotion(20, SLOW_SPEED);
    armGripAngle = 20;
    delay(400);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }

    // 8) Return to saved positions
    armbot.axis3Motion(savedElb, SLOW_SPEED);
    armElbowAngle = savedElb;
    armbot.axis1Motion(savedRot, SLOW_SPEED);
    armRotAngle = savedRot;
    armbot.gripperMotion(savedGrip, SLOW_SPEED);
    armGripAngle = savedGrip;
    delay(600);
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){ storeModeActive=false; break; }
    // loop continues while storeModeActive
  }

  // When exiting store mode restore ARMBOT screen
  // (aci alanlari da doldurulur; eskiden hareket edene kadar bos kaliyordu
  // / the angle fields are filled too; they used to stay blank until a move)
  showArmbotScreen();
  ensureArmbotInit();
  iotbot.serialWrite(L(">> Mağaza modu kapalı", ">> Store mode off"));
}
// ARMBOT dynamic calibration
static bool armCalibrated = false;
static int joyXMin=4095, joyXMax=0, joyXCenter=2048;
static int joyYMin=4095, joyYMax=0, joyYCenter=2048;
static int joyDeadZoneX=150, joyDeadZoneY=150; // will refine after calibration
// Encoder incremental gripper
static long encLastCount = 0; // store previous encoder count for delta steps
static const int GRIP_STEP = 2; // small step per encoder tick
static const int ENCODER_DEG_PER_TICK = 10; // 1 tick -> 10° ; hız 2x
static const int JOY_STEP_MAX = 5; // max incremental step per loop
static const int JOY_STEP_DIV = 300; // scale factor from deflection to step

// IOTBOT onboard buzzer
void playModeChime(Mode m)
{
  if (m == MODE_CARBOT)
  {
    // ascending: CARBOT
    iotbot.buzzerPlay(880, 90);
    iotbot.buzzerPlay(1200, 110);
  }
  else
  {
    // descending: ARMBOT
    iotbot.buzzerPlay(1200, 90);
    iotbot.buzzerPlay(880, 110);
  }
}

// Utility: fixed-width integer write (pads trailing spaces to clear leftovers)
void lcdWriteFixed(int col, int row, int value, int width)
{
  char buf[8];
  snprintf(buf, sizeof(buf), "%-*d", width, value); // left align, pad spaces
  iotbot.lcdWriteCR(col, row, buf);
}
// Utility: fixed text padded to width
void lcdWriteFixedTxt(int col, int row, const char *txt, int width)
{
  char buf[24];
  snprintf(buf, sizeof(buf), "%-*s", width, txt);
  iotbot.lcdWriteCR(col, row, buf);
}

// Note mapping (potentiometer) discrete musical notes for display & horn
struct NoteMap
{
  const char *name;
  int freq;
};
static const NoteMap NOTES[] = {
    {"C4", 262}, {"D4", 294}, {"E4", 330}, {"F4", 349}, {"G4", 392}, {"A4", 440}, {"B4", 494}, {"C5", 523}};

int mapPotToNoteIndex(int pot)
{
  int seg = 4096 / (int)(sizeof(NOTES) / sizeof(NOTES[0]));
  int idx = pot / seg;
  if (idx < 0)
    idx = 0;
  int maxIdx = (int)(sizeof(NOTES) / sizeof(NOTES[0])) - 1;
  if (idx > maxIdx)
    idx = maxIdx;
  return idx;
}

static int lastPotNoteIdx = -1; // for display refresh control
// Horn hold state
static bool hornActive = false;
static int hornLastNoteIdx = -1;

// ARMBOT high-loudness note table (kHz-range for passive buzzer resonance)
struct ArmNote { const char* name; int freq; };
static const ArmNote ARMBOT_NOTES[] = {
  {"1.6", 1600}, {"1.8", 1800}, {"2.0", 2000}, {"2.2", 2200}, {"2.4", 2400}, {"2.6", 2600}
};
inline int mapPotToArmNoteIndex(int pot){
  int seg = 4096 / (int)(sizeof(ARMBOT_NOTES)/sizeof(ARMBOT_NOTES[0]));
  int idx = pot / seg; if(idx < 0) idx = 0; int maxIdx = (int)(sizeof(ARMBOT_NOTES)/sizeof(ARMBOT_NOTES[0]))-1; if(idx>maxIdx) idx=maxIdx; return idx;
}

// Far isaretinin yeri ve metni: "(O)" otomatik / "(A)" auto
// Light auto marker position and text
static const int LED_AUTO_COL = 11;
const char *ledAutoMark() { return L("(O)", "(A)"); }

// Satir 1, sutun 0-13: "FAR:AÇIK   (O)" (iotbot.lcdWriteFixedTxt Turkce harfleri dogru gosterir)
// Row 1, cols 0-13: light state + auto marker (iotbot.lcdWriteFixedTxt shows Turkish letters correctly)
void drawLedField()
{
  iotbot.lcdWriteCR(0, 1, L("FAR:", "LED:"));
  iotbot.lcdWriteFixedTxt(4, 1, ledState ? L("AÇIK", "ON") : L("KAPALI", "OFF"), 7);
  iotbot.lcdWriteFixedTxt(LED_AUTO_COL, 1, ledAutoMode ? ledAutoMark() : "", 3);
}

void showCarbotScreen()
{
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, L("MOD:CARBOT   AÇI:", "MODE:CARBOT  ANG:")); // Aci sutun 17 / angle at col 17
  // Satir 1 / row 1: "FAR:AÇIK   (O) NT:C4"
  drawLedField();
  iotbot.lcdWriteCR(14, 1, "NT:");    // Korna notasi / horn note, cols 14..16
  lcdWriteFixedTxt(17, 1, "---", 3);  // placeholder; will be replaced immediately
  iotbot.lcdWriteFixedTxt(2, 2, L("X:Direksiyon Y:Hız", "X:Steer  Y:Drive"), 18);
  iotbot.lcdWriteFixedTxt(0, 3, L("B1:Korna  ENC:MOD", "B1:Horn  ENC:MODE"), 20);
}

void showArmbotScreen()
{
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, L("MOD:ARMBOT  NT:", "MODE:ARMBOT NT:"));
  lcdWriteFixedTxt(16, 0, "---", 3); // note placeholder
  // GÖV=gövde OMZ=omuz DRS=dirsek KSK=kıskaç / ROT=base SHO=shoulder ELB=elbow GRP=gripper
  // Line1: ROT and SHO
  iotbot.lcdWriteCR(0, 1, L("GÖV:", "ROT:")); lcdWriteFixedTxt(4, 1, "", 3);
  iotbot.lcdWriteCR(8, 1, L("OMZ:", "SHO:"));  lcdWriteFixedTxt(12,1, "", 3);
  // Line2: ELB and GRP
  iotbot.lcdWriteCR(0, 2, L("DRS:", "ELB:")); lcdWriteFixedTxt(4, 2, "", 3);
  iotbot.lcdWriteCR(8, 2, L("KSK:", "GRP:")); lcdWriteFixedTxt(12,2, "", 3);
  // Line3: hints
  iotbot.lcdWriteFixedTxt(0, 3, L("B1:Korna B2:Nota", "B1:Horn  B2:Note"), 20);
}

void updateModeSelectLine()
{
  if (modeSelIndex == 0)
  {
    lcdWriteFixedTxt(0, 2, " [CARBOT]   ARMBOT", 20);
  }
  else
  {
    lcdWriteFixedTxt(0, 2, "  CARBOT   [ARMBOT]", 20);
  }
}

void showModeSelectScreen()
{
  iotbot.lcdClear();
  iotbot.lcdWriteCR(1, 0, L("Lütfen Mod Seçin", "Please Select Mode"));
  updateModeSelectLine();
  iotbot.lcdWriteFixedTxt(0, 3, L(" Joy:Seç  Enc:Başla", " Joy:Pick Enc:Start"), 20);
}

// Populate NOTE field from potentiometer (used when entering CARBOT)
// (Merged) initNoteFromPot already defined earlier; remove duplicate.

// ARMBOT ve CARBOT ESP32'de AYNI 5 pini kullanir (IO25, 26, 27, 32, 33 = P1-P5),
// yani ayni sokete takilirlar. Bir robotun servo/PWM baglantilari pinleri tutar;
// eskiden ARMBOT'a gecip geri donunce CARBOT motorlari ve direksiyonu (pinler
// hala kol servolarina bagli oldugu icin) bir daha calismiyordu. Her mod
// degisiminde birini birakip digerini YENIDEN bagliyoruz.
// / ARMBOT and CARBOT use the SAME 5 pins on the ESP32 (IO25, 26, 27, 32, 33 =
// P1-P5), so they plug into the same socket. A robot's servo/PWM links hold the
// pins; after switching to ARMBOT and back, CARBOT's motors and steering used to
// stay dead (the pins were still routed to the arm servos). On every mode
// switch we release one robot and RE-attach the other.
void switchHardwareToArmbot()
{
  carBot.stop();
  carBot.controlLED(false);
  carBot.end(); // Direksiyon servosunu birak / release the steering servo
  iotbot.buzzerStop();
  iotbot.relayWrite(false);
  armbot.begin(); // Kol servolarini pinlere yeniden bagla / re-attach the arm servos
  armbotInited = true;
}

void switchHardwareToCarbot()
{
  armbot.buzzerStop();
  if (armbotInited)
    armbot.end(); // Kol servolarini birak / release the arm servos
  iotbot.relayWrite(false);
  carBot.begin(); // Motor PWM ve direksiyonu yeniden bagla / re-attach motor PWM and steering
  carBot.controlLED(ledState);
}

// Fill the ARMBOT screen fields (hardware is attached by switchHardwareToArmbot)
void ensureArmbotInit()
{
  // Initial field values on ARMBOT screen
  lcdWriteFixed(4, 1, armRotAngle, 3);
  lcdWriteFixed(12, 1, armShoulderAngle, 3);
  lcdWriteFixed(4, 2, armElbowAngle, 3);
  lcdWriteFixed(12, 2, armGripAngle, 3);
  // Initialize ARMBOT note selection from current pot (start state)
  int pot0 = iotbot.potentiometerRead();
  armNoteIdx = mapPotToArmNoteIndex(pot0);
  lcdWriteFixedTxt(16, 0, ARMBOT_NOTES[armNoteIdx].name, 3);
  // Init encoder baseline to avoid jump on first tick
  encLastCount = iotbot.encoderRead();
}

// Populate NOTE field from potentiometer (used whenever entering CARBOT mode)
void initNoteFromPot()
{
  int pot0 = iotbot.potentiometerRead();
  int n0 = mapPotToNoteIndex(pot0);
  lcdWriteFixedTxt(17, 1, NOTES[n0].name, 3);
  lastPotNoteIdx = n0;
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
  iotbot.serialWrite(L("---- KABLOLU KUMANDA - Komutlar ----", "---- WIRED CONTROLLER - Commands ----"));
  iotbot.serialWrite(L("  yardim : bu liste", "  help   : this list"));
  iotbot.serialWrite(L("  durum  : mod, far, kol açıları", "  status : mode, lights, arm angles"));
  iotbot.serialWrite(L("  dil    : English'e geç", "  lang   : switch to Turkish"));
  iotbot.serialWrite(L("  Mod: joystick sol/sağ + encoder'a bas; çalışırken encoder = mod değiştir",
                       "  Mode: joystick left/right + press the encoder; while running encoder = switch mode"));
  iotbot.serialWrite(L("  CARBOT: Joy X=direksiyon, Joy Y=hız, B1=korna, B2=far oto/elle, B3=far aç/kapat",
                       "  CARBOT: Joy X=steering, Joy Y=speed, B1=horn, B2=lights auto/manual, B3=lights on/off"));
  iotbot.serialWrite(L("  ARMBOT: Joy X=gövde, Joy Y=omuz, Pot=dirsek, Encoder/B3=kıskaç, B1=korna, B2=nota, Joy butonu=mağaza modu",
                       "  ARMBOT: Joy X=base, Joy Y=shoulder, Pot=elbow, Encoder/B3=gripper, B1=horn, B2=note, Joy button=store mode"));
  iotbot.serialWrite(L("  Robotu P1-P5'e takın; mod değiştirirken kabloyu da değiştirin.",
                       "  Plug the robot into P1-P5; swap the cable when you switch modes."));
}

void printStatus()
{
  String s = String(L("Mod: ", "Mode: ")) +
             (appState == APP_SELECT ? L("seçim ekranı", "mode select") : (currentMode == MODE_CARBOT ? "CARBOT" : "ARMBOT"));
  s += String(L(" | far ", " | lights ")) + (ledState ? L("açık", "on") : L("kapalı", "off")) +
       (ledAutoMode ? L(" (otomatik)", " (auto)") : "");
  iotbot.serialWrite(s);
  iotbot.serialWrite(String(L("ARMBOT: gövde ", "ARMBOT: base ")) + armRotAngle + L(" omuz ", " shoulder ") + armShoulderAngle +
                     L(" dirsek ", " elbow ") + armElbowAngle + L(" kıskaç ", " gripper ") + armGripAngle);
}

// Ekrani yeni dille yeniden ciz / redraw the screen in the new language
void redrawScreen()
{
  if (appState == APP_SELECT)
  {
    showModeSelectScreen();
  }
  else if (currentMode == MODE_CARBOT)
  {
    showCarbotScreen();
    initNoteFromPot();
  }
  else
  {
    showArmbotScreen();
    lcdWriteFixed(4, 1, armRotAngle, 3);
    lcdWriteFixed(12, 1, armShoulderAngle, 3);
    lcdWriteFixed(4, 2, armElbowAngle, 3);
    lcdWriteFixed(12, 2, armGripAngle, 3);
    lcdWriteFixedTxt(16, 0, ARMBOT_NOTES[armNoteIdx].name, 3);
  }
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
    redrawScreen();
    printHelp();
  }
  else
  {
    iotbot.serialWrite(String(L("Bilinmeyen komut: ", "Unknown command: ")) + cmd + L("  (yardim yazın)", "  (type help)"));
  }
}

// Mod degisince seri porta yaz / print the mode on the serial port when it changes
void printMode()
{
  iotbot.serialWrite(currentMode == MODE_CARBOT ? L(">> Mod: CARBOT (CARBOT kablosunu P1-P5'e takın)", ">> Mode: CARBOT (plug the CARBOT cable into P1-P5)")
                                                : L(">> Mod: ARMBOT (ARMBOT kablosunu P1-P5'e takın)", ">> Mode: ARMBOT (plug the ARMBOT cable into P1-P5)"));
}

void setup()
{
  carBot.begin();          // CARBOT init (servo + motors)
  iotbot.begin();          // IOTBOT init (LCD + inputs)
  iotbot.serialStart(115200); // Seri port (yardim/durum/dil) / serial port (help/status/lang)
  carBot.controlLED(true); // ensure LED ON by default

  showModeSelectScreen();
  bootMs0 = millis();
  iotbot.serialWrite(L("IOTBOT kablolu kumanda hazır. Joystick ile mod seçip encoder'a basın.",
                       "IOTBOT wired controller ready. Pick a mode with the joystick and press the encoder."));
  printHelp();
}

void loop()
{
  // Handle encoder button (active LOW) with boot guard
  bool encPressed = (digitalRead(ENCODER_BUTTON_PIN) == LOW);
  unsigned long now = millis();

  // Seri komutlar: veri yoksa hemen doner / serial commands: returns at once when there is no data
  String cmd;
  if (readCommand(cmd))
    handleCommand(cmd);

  if (appState == APP_SELECT)
  {
    // Joystick left/right selects mode
    int xRawSel = iotbot.joystickXRead();
    int dxSel = joyDelta(xRawSel);
    int prevSel = modeSelIndex;
    if (dxSel < -DEADZONE)
      modeSelIndex = 0;
    else if (dxSel > DEADZONE)
      modeSelIndex = 1;
    if (modeSelIndex != prevSel)
    {
      updateModeSelectLine();
    }
    // Confirm with encoder
    if ((now - bootMs0) > ENC_BOOT_GUARD_MS && encPressed && !encBtnPrev && (now - encLastMs) > ENC_DEBOUNCE_MS)
    {
      currentMode = (modeSelIndex == 0) ? MODE_CARBOT : MODE_ARMBOT;
      if (currentMode == MODE_CARBOT)
      {
        showCarbotScreen();
        initNoteFromPot();
      }
      else
      {
        switchHardwareToArmbot();
        showArmbotScreen();
        ensureArmbotInit();
        // Reset calibration flag to trigger fresh calibration
        armCalibrated = false;
      }
      playModeChime(currentMode);
      printMode();
      appState = APP_RUN;
      encLastMs = now;
    }
    encBtnPrev = encPressed;
    return; // stay here until mode is selected
  }
  else
  {
    // Running: toggle modes with encoder press
    if ((now - bootMs0) > ENC_BOOT_GUARD_MS && encPressed && !encBtnPrev && (now - encLastMs) > ENC_DEBOUNCE_MS)
    {
      currentMode = (currentMode == MODE_CARBOT) ? MODE_ARMBOT : MODE_CARBOT;
      if (currentMode == MODE_CARBOT)
      {
        switchHardwareToCarbot();
        showCarbotScreen();
        initNoteFromPot();
      }
      else
      {
        switchHardwareToArmbot();
        showArmbotScreen();
        ensureArmbotInit();
        armCalibrated = false;
      }
      playModeChime(currentMode);
      printMode();
      encLastMs = now;
    }
    encBtnPrev = encPressed;
  }

  if (currentMode == MODE_ARMBOT)
  {
    // Toggle store mode with joystick button (only in ARMBOT)
    if(joystickPressedDebounce(storeJoyPrev, storeJoyLastMs)){
      storeModeActive = !storeModeActive;
      if(storeModeActive){
        // Show minimal STORE MODE screen
        iotbot.lcdClear();
        iotbot.lcdWriteCR(0,0,L("MOD:ARMBOT", "MODE:ARMBOT"));
        iotbot.lcdWriteCR(0,1,L("   MAĞAZA MODU", "   STORE MODE"));
        iotbot.lcdWriteCR(0,3,L("Joy butonu: çıkış", "Joy button: exit"));
        iotbot.serialWrite(L(">> Mağaza modu açık (joystick butonu ile çıkış)", ">> Store mode on (exit with the joystick button)"));
      } else {
        // Restore normal ARMBOT screen
        showArmbotScreen();
        ensureArmbotInit();
      }
    }
    if(storeModeActive){
      // While store mode active, run user-defined demo actions and skip normal controls
      storeModeActions();
      return; // do not run the regular ARMBOT control path
    }
    // One-time quick calibration on entering ARMBOT
    if(!armCalibrated){
      long sumX=0,sumY=0; joyXMin=4095; joyXMax=0; joyYMin=4095; joyYMax=0;
      for(int i=0;i<40;i++){
        int x=iotbot.joystickXRead(); int y=iotbot.joystickYRead();
        sumX+=x; sumY+=y;
        if(x<joyXMin) joyXMin=x; if(x>joyXMax) joyXMax=x;
        if(y<joyYMin) joyYMin=y; if(y>joyYMax) joyYMax=y;
        delay(3);
      }
      joyXCenter = (int)(sumX/40);
      joyYCenter = (int)(sumY/40);
      int devX = max(joyXMax-joyXCenter, joyXCenter-joyXMin);
      int devY = max(joyYMax-joyYCenter, joyYCenter-joyYMin);
      joyDeadZoneX = constrain(devX + 60, 80, 300);
      joyDeadZoneY = constrain(devY + 60, 80, 300);
      armCalibrated = true;
    }

    // ARMBOT controls
    int xRaw = iotbot.joystickXRead();
    int yRaw = iotbot.joystickYRead();
    int potRaw = iotbot.potentiometerRead();

    int dx = xRaw - joyXCenter;
    int dy = yRaw - joyYCenter;

    // Compute target angles 0..180
    int rotTarget = armRotAngle;
    if (abs(dx) > joyDeadZoneX)
    {
      // incremental adjustment instead of absolute mapping
      int step = (abs(dx) / JOY_STEP_DIV) + 1;
      if(step > JOY_STEP_MAX) step = JOY_STEP_MAX;
      // invert direction per feedback: sola (dx<0) art, sağa (dx>0) azalsın (veya tam tersi ihtiyaca göre)
      rotTarget += (dx < 0 ? +step : -step);
      if (rotTarget < 0) rotTarget = 0;
      if (rotTarget > 180) rotTarget = 180;
    }

    int shoTarget = armShoulderAngle;
    if (abs(dy) > joyDeadZoneY)
    {
      int step = (abs(dy) / JOY_STEP_DIV) + 1;
      if(step > JOY_STEP_MAX) step = JOY_STEP_MAX;
      shoTarget += (dy < 0 ? -step : step); // forward (positive dy) increases angle
      if (shoTarget < 0) shoTarget = 0;
      if (shoTarget > 180) shoTarget = 180;
    }

    int elbTarget = (int)((long)potRaw * 180L / 4095L);
    if (elbTarget < 0)
      elbTarget = 0;
    if (elbTarget > 180)
      elbTarget = 180;

    // Apply motions if changed (blocking library - use small speed)
    const int ARM_SPEED = 1; // ms per step
    if (rotTarget != armRotAngle)
    {
      armbot.axis1Motion(rotTarget, ARM_SPEED);
      armRotAngle = rotTarget;
      lcdWriteFixed(4, 1, armRotAngle, 3);
    }
    if (shoTarget != armShoulderAngle)
    {
      armbot.axis2Motion(shoTarget, ARM_SPEED);
      armShoulderAngle = shoTarget;
      lcdWriteFixed(12, 1, armShoulderAngle, 3);
    }
    if (elbTarget != armElbowAngle)
    {
      armbot.axis3Motion(elbTarget, ARM_SPEED);
      armElbowAngle = elbTarget;
      lcdWriteFixed(4, 2, armElbowAngle, 3);
    }

    // B3 button toggles gripper fully open/close (replacing encoder)
    bool b3NowArm = iotbot.button3Read();
    static bool b3PrevArm = false;
    if(b3NowArm && !b3PrevArm && (now - b3LastChange) > B3_DEBOUNCE_MS){
      const int GRIP_OPEN_ANGLE = 20;
      const int GRIP_CLOSE_ANGLE = 120;
      int target = (armGripAngle < (GRIP_OPEN_ANGLE + GRIP_CLOSE_ANGLE)/2) ? GRIP_CLOSE_ANGLE : GRIP_OPEN_ANGLE;
      armbot.gripperMotion(target, ARM_SPEED);
      armGripAngle = target;
      lcdWriteFixed(12, 2, armGripAngle, 3);
      b3LastChange = now;
    }
    b3PrevArm = b3NowArm;

    // Encoder incremental gripper adjust (small steps per tick), only in ARMBOT
  long encNow = iotbot.encoderRead();
  long rawTicks = encNow - encLastCount; // quadrature count delta
  long delta = rawTicks * ENCODER_DEG_PER_TICK; // convert ticks directly to degrees
    if(delta != 0){
  int newGrip = armGripAngle + (delta>0 ? ENCODER_DEG_PER_TICK : -ENCODER_DEG_PER_TICK);
      if(newGrip < 15) newGrip = 15; if(newGrip > 120) newGrip = 120; // cap at [15,120]
      if(newGrip != armGripAngle){
        armbot.gripperMotion(newGrip, ARM_SPEED);
        armGripAngle = newGrip;
        lcdWriteFixed(12,2, armGripAngle,3);
      }
      encLastCount = encNow;
    }

    // B2 cycles loud ARMBOT note index
    bool b2NowArm = iotbot.button2Read();
    if (b2NowArm && !b2Prev && (now - lastBeepMs) > BEEP_DEBOUNCE_MS)
    {
      armNoteIdx = (armNoteIdx + 1) % (int)(sizeof(ARMBOT_NOTES) / sizeof(ARMBOT_NOTES[0]));
      lcdWriteFixedTxt(16, 0, ARMBOT_NOTES[armNoteIdx].name, 3);
      // short confirmation beep
      armbot.buzzerPlay(ARMBOT_NOTES[armNoteIdx].freq, 90);
      lastBeepMs = now;
    }
    b2Prev = b2NowArm;

    // B1 horn on ARMBOT buzzer, continuous while held; live note updates
    bool b1NowArm = iotbot.button1Read();
    if (b1NowArm)
    {
      if (!armHornActive)
      {
        armHornActive = true;
      }
      armbot.buzzerStart(ARMBOT_NOTES[armNoteIdx].freq);
    }
    else if (armHornActive)
    {
      armHornActive = false;
      armbot.buzzerStop();
    }
    b1Prev = b1NowArm;

    return; // done with ARMBOT branch
  }

  int xRaw = iotbot.joystickXRead();
  int yRaw = iotbot.joystickYRead();
  int ldrRaw = iotbot.ldrRead();
  int potRaw = iotbot.potentiometerRead();

  int dx = joyDelta(xRaw);
  int dy = joyDelta(yRaw);

  // Steering (45 - 135 range)
  int angle = 90;
  if (abs(dx) > DEADZONE)
  {
    float norm = (float)dx / 2048.0f; // yaklaşık -1..1
    angle = 90 + (int)(norm * 45.0f);
    if (angle < 45)
      angle = 45;
    if (angle > 135)
      angle = 135;
  }
  // Send steering command before starting motors so servo pulse is applied
  // prior to any motor current surge which may cause supply sag on some setups.
  carBot.steer(angle);

  // Angle display fixed-width (3 chars) at col 16 row 0
  lcdWriteFixed(17, 0, angle, 3);

  // Forward / Backward (inverted per user request) - degisken hiz (PWM):
  // deadzone'dan tam sapmaya (2048) kadar 50-255 araligina eslenir.
  // Forward / Backward (inverted per user request) - variable (PWM) speed:
  // mapped from the deadzone edge to full deflection (2048) into 50-255.
  if (dy < -DEADZONE)
  {
    int speed = constrain(map(-dy, DEADZONE, 2048, 50, 255), 0, 255);
    carBot.moveBackward(speed);
  }
  else if (dy > DEADZONE)
  {
    int speed = constrain(map(dy, DEADZONE, 2048, 50, 255), 0, 255);
    carBot.moveForward(speed);
  }
  else
  {
    carBot.stop();
  }

  // LED Auto/Manual mode with B2, Auto uses LDR, Manual uses B3 toggle
  // ledAutoMode is global (declared above) for screen refresh helpers
  const int LDR_THRESHOLD = 1200; // adjust by ambient
  bool autoShouldOn = (ldrRaw < LDR_THRESHOLD);

  // B2 toggles auto/manual
  bool b2Now = iotbot.button2Read();
  if (b2Now && !b2Prev && (now - lastBeepMs) > BEEP_DEBOUNCE_MS)
  {
    ledAutoMode = !ledAutoMode;
    if (ledAutoMode)
    {
      iotbot.buzzerPlay(900, 70);
      iotbot.buzzerPlay(1200, 80);
    }
    else
    {
      iotbot.buzzerPlay(700, 70);
      iotbot.buzzerPlay(500, 80);
    }
    lastBeepMs = now;
    // Update auto marker next to LED state
    drawLedField();
  }
  b2Prev = b2Now;

  // Manual toggle on B3
  bool b3Now = iotbot.button3Read();
  if (!ledAutoMode && b3Now && !b3Prev && (now - b3LastChange) > B3_DEBOUNCE_MS)
  {
    ledState = !ledState;
    carBot.controlLED(ledState);
    if (ledState)
    {
      iotbot.buzzerPlay(1200, 80);
      iotbot.buzzerPlay(1500, 90);
    }
    else
    {
      iotbot.buzzerPlay(600, 80);
      iotbot.buzzerPlay(450, 90);
    }
    // LED durumu ve otomatik isareti / LED state and auto marker
    drawLedField();
    b3LastChange = now;
  }
  else if (ledAutoMode && b3Now && !b3Prev && (now - b3LastChange) > B3_DEBOUNCE_MS)
  {
    // In AUTO mode, pressing B3 triggers (A) blink 3 times (non-blocking)
    aBlinkActive = true;
    aBlinkPhaseOn = false; // start from off to make first toggle turn it on
    aBlinkCycles = 0;
    aBlinkNextMs = now; // trigger immediate toggle
    b3LastChange = now;
  }
  b3Prev = b3Now;

  // Handle (A) blink state machine
  if (aBlinkActive && now >= aBlinkNextMs)
  {
    aBlinkPhaseOn = !aBlinkPhaseOn;
    iotbot.lcdWriteFixedTxt(LED_AUTO_COL, 1, aBlinkPhaseOn ? ledAutoMark() : "", 3);
    aBlinkNextMs = now + 160; // blink period
    if (aBlinkPhaseOn)
    {
      aBlinkCycles++;
      if (aBlinkCycles >= 3)
      {
        aBlinkActive = false;
        // ensure it ends ON to show AUTO
        iotbot.lcdWriteFixedTxt(LED_AUTO_COL, 1, ledAutoMark(), 3);
      }
    }
  }

  // Auto mode applies LED based on LDR with feedback on change
  static int lastAutoApplied = -1;
  if (ledAutoMode)
  {
    int newState = autoShouldOn ? 1 : 0;
    // Elle moddan otomatige donuldugunde far LDR'ye uymuyorsa da uygula (eskiden
    // LDR degismedikce far elle birakildigi gibi kaliyordu).
    // / Also apply when returning from manual to auto and the light does not match
    // the LDR (it used to stay as set manually until the LDR changed).
    if (newState != lastAutoApplied || ledState != (newState == 1))
    {
      ledState = (newState == 1);
      carBot.controlLED(ledState);
      if (ledState)
      {
        iotbot.buzzerPlay(1000, 70);
      }
      else
      {
        iotbot.buzzerPlay(450, 70);
      }
      // LED durumu ve otomatik isareti / LED state and auto marker
      drawLedField();
      lastAutoApplied = newState;
    }
  }

  // Pot note determination + display refresh if changed
  int noteIdx = mapPotToNoteIndex(potRaw);
  if (noteIdx != lastPotNoteIdx)
  {
    lcdWriteFixedTxt(17, 1, NOTES[noteIdx].name, 3); // update note at new position
    lastPotNoteIdx = noteIdx;
  }

  // B1 horn: play while held; update frequency live with potentiometer
  bool b1Now = iotbot.button1Read();
  if (b1Now)
  {
    if (!hornActive)
    {
      hornActive = true;
      hornLastNoteIdx = -1; // force immediate set
    }
    if (noteIdx != hornLastNoteIdx)
    {
      iotbot.buzzerStart(NOTES[noteIdx].freq); // start or update
      hornLastNoteIdx = noteIdx;
    }
  }
  else
  {
    if (hornActive)
    {
      hornActive = false;
      iotbot.buzzerStop(); // stop
    }
  }
  b1Prev = b1Now;

  // Turn signal relay blink when steering past threshold
  const int STEER_LEFT_THRESHOLD = 70;   // angle below means left turn region
  const int STEER_RIGHT_THRESHOLD = 110; // angle above means right turn region
  static unsigned long relayLastMs = 0;
  static bool relayState = false;
  if (angle < STEER_LEFT_THRESHOLD || angle > STEER_RIGHT_THRESHOLD)
  {
    if (now - relayLastMs > 300)
    { // blink period
      relayState = !relayState;
      iotbot.relayWrite(relayState);
      relayLastMs = now;
    }
  }
  else
  {
    // steering centered: ensure relay off
    if (relayState)
    {
      relayState = false;
      iotbot.relayWrite(false);
    }
  }
}