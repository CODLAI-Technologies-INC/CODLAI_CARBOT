/*
 * IOTBOT Armbot and Carbot Wireless Control (ESP-NOW Master)
 */

#define USE_ESPNOW
#define USE_WIFI
#include <IOTBOT.h>

IOTBOT iotbot;

// Broadcast Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Data Structures
CodlaiESPNowMessage armData;
CodlaiESPNowMessage carData;

// Variables for Logic
static int joyXCenter = 2048;
static int joyYCenter = 2048;
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
static AppState appState = APP_SELECT;

// Buttons & Debounce
static bool encBtnPrev = false;
static unsigned long encLastMs = 0;
static const unsigned long ENC_DEBOUNCE_MS = 150;
static unsigned long bootMs0 = 0;
static const unsigned long ENC_BOOT_GUARD_MS = 800;

// Joystick Button RTOS State for LED control
static uint8_t joyBtnState = 0; // 0=IDLE, 1=DEBOUNCE_PRESS, 2=PRESSED, 3=WAIT_RELEASE
static unsigned long joyBtnTimer = 0;
static const unsigned long JOY_LONG_PRESS_MS = 2000;
static const unsigned long JOY_DEBOUNCE_MS = 30;

// Armbot State
static int armRotAngle = 90;      // Axis1
static int armShoulderAngle = 90; // Axis2
static int armElbowAngle = 50;    // Axis3
static int armGripAngle = 60;     // Gripper
static int lastEncoderVal = 0;
static bool armCalibrated = false;
static int joyXMin = 4095, joyXMax = 0;
static int joyYMin = 4095, joyYMax = 0;
static int joyDeadZoneX = 150, joyDeadZoneY = 150;
static const int JOY_STEP_MAX = 5;
static const int JOY_STEP_DIV = 300;

// Carbot State
static bool ledState = true;
static bool ledAutoMode = true;
static bool b3Prev = false;
static unsigned long b3LastChange = 0;
static const unsigned long B3_DEBOUNCE_MS = 50;
static unsigned long lastBeepMs = 0;
static const unsigned long BEEP_DEBOUNCE_MS = 180;

// Sending Timer
static unsigned long lastSendMs = 0;
static const unsigned long SEND_INTERVAL_MS = 50;

void showCarbotScreen()
{
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "CARBOT (WIFI)");
  iotbot.lcdWriteCR(0, 1, "LED:");
  iotbot.lcdWriteFixedTxt(4, 1, ledState ? "ON" : "OFF", 3);
  iotbot.lcdWriteCR(7, 1, ledAutoMode ? "(A)" : "   ");
  iotbot.lcdWriteFixedTxt(12, 1, iotbot.lastSendStatus ? "CON:*  " : "CON:*/ ", 7);

  iotbot.lcdWriteFixedTxt(0, 2, "Mesafe: --- cm  ", 16);

  iotbot.lcdWriteCR(0, 3, "JBtn:LED B3:Horn");
}

void showArmbotScreen()
{
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "ARMBOT (WIFI)");
  iotbot.lcdWriteCR(0, 1, "ROT:");
  iotbot.lcdWriteFixedTxt(4, 1, "", 3);
  iotbot.lcdWriteCR(8, 1, "SHO:");
  iotbot.lcdWriteFixedTxt(12, 1, "", 3);
  iotbot.lcdWriteFixedTxt(18, 0, iotbot.lastSendStatus ? "*" : "*/", 2);

  iotbot.lcdWriteCR(0, 2, "ELB:");
  iotbot.lcdWriteFixedTxt(4, 2, "", 3);
  iotbot.lcdWriteCR(8, 2, "GRP:");
  iotbot.lcdWriteFixedTxt(12, 2, "", 3);
  iotbot.lcdWriteCR(0, 3, "B1/B2 devredisi");
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
  iotbot.lcdWriteCR(1, 0, "Select Wireless Mode");
  updateModeSelectLine();
  iotbot.lcdWriteFixedTxt(0, 3, "   Encoder: Start", 20);
}

void setup()
{
  iotbot.begin();
  iotbot.serialStart(115200);
  iotbot.serialWrite("IOTBOT Wireless Control System Starting...");

  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "Kalibrasyon?");
  iotbot.lcdWriteCR(0, 1, "B3=Evet, ENC=Gec");

  bool doCalibration = false;
  unsigned long startMillis = millis();
  bool b3WasPressed = iotbot.button3Read();
  bool encWasPressed = (digitalRead(ENCODER_BUTTON_PIN) == LOW);

  while(millis() - startMillis < 5000){
    bool b3PressedNow = iotbot.button3Read();
    bool encPressedNow = (digitalRead(ENCODER_BUTTON_PIN) == LOW);

    // Require an actual press transition, protecting against stuck states
    if (b3PressedNow && !b3WasPressed) {
      doCalibration = true;
      break;
    }
    if (encPressedNow && !encWasPressed) {
      break;
    }

    b3WasPressed = b3PressedNow;
    encWasPressed = encPressedNow;

    int timeLeft = 5 - ((millis() - startMillis) / 1000);
    iotbot.lcdWriteFixedTxt(14, 1, (String(timeLeft) + "s ").c_str(), 4);
    delay(50);
  }

  if (doCalibration) {
    iotbot.lcdClear();
    iotbot.lcdWriteCR(0, 0, "Joystick Birak");
    iotbot.lcdWriteCR(0, 1, "Sonra B3'e Bas");
    delay(500); // debounce B3 from previous selection
    while (!iotbot.button3Read()) { delay(50); }
    long xSum = 0, ySum = 0;
    for(int i=0; i<50; i++){
       xSum += iotbot.joystickXRead();
       ySum += iotbot.joystickYRead();
       delay(10);
    }
    joyXCenter = xSum / 50;
    joyYCenter = ySum / 50;

    Serial.print("Joy X: "); Serial.print(joyXCenter);
    Serial.print(" Joy Y: "); Serial.println(joyYCenter);
  } else {
    iotbot.lcdClear();
    iotbot.lcdWriteCR(0, 0, "Calibrating...");
    iotbot.lcdWriteCR(0, 1, "Do not touch!");
    delay(1000);
    long xSum = 0, ySum = 0;
    for(int i=0; i<50; i++){
       xSum += iotbot.joystickXRead();
       ySum += iotbot.joystickYRead();
       delay(10);
    }
    joyXCenter = xSum / 50;
    joyYCenter = ySum / 50;
  }

  // Sanity check: Handle X and Y independently
  if (abs(joyXCenter - 2048) > 1000)
  {
    joyXCenter = 2048;
  }

  if (abs(joyYCenter - 2048) > 1000)
  {
    joyYCenter = 2048;
  }

  iotbot.serialWrite("Calibrated Center: " + String(joyXCenter) + ", " + String(joyYCenter));

  showModeSelectScreen();
  bootMs0 = millis();
  lastEncoderVal = iotbot.encoderRead();

  armData.deviceType = 1;
  carData.deviceType = 2;

  iotbot.initESPNow();
  iotbot.setWiFiChannel(1);
  iotbot.startListening(); 

  if (!iotbot.addBroadcastPeer(1))
  {
    iotbot.serialWrite("Failed to add peer");
    return;
  }

  iotbot.serialWrite("Setup Complete.");
}

void loop()
{
  unsigned long now = millis();
  bool encPressed = (digitalRead(ENCODER_BUTTON_PIN) == LOW);

  static int carbotDistance = 999;
  static bool distanceWarningActive = false;
  static unsigned long lastDistanceBlinkMs = 0;
  static bool distanceBlinkState = false;

  if (iotbot.newData) {
    iotbot.newData = false;
    if (appState == APP_RUN && currentMode == MODE_CARBOT) {
      if (iotbot.receivedData.deviceType == 2) {
         carbotDistance = iotbot.receivedData.axis3;
         if (!distanceWarningActive) {
           char buf[17];
           if (carbotDistance > 300) {
             snprintf(buf, sizeof(buf), "Mesafe: Serbest ");
           } else {
             snprintf(buf, sizeof(buf), "Mesafe: %3d cm  ", carbotDistance);
           }
           iotbot.lcdWriteFixedTxt(0, 2, buf, 16);
         }
      }
    }
  }

  // --- Mode Selection ---
  if (appState == APP_SELECT)
  {
    int xRawSel = iotbot.joystickXRead();
    int dxSel = xRawSel - joyXCenter;
    int prevSel = modeSelIndex;
    if (dxSel < -DEADZONE)
      modeSelIndex = 0;
    else if (dxSel > DEADZONE)
      modeSelIndex = 1;

    if (modeSelIndex != prevSel)
      updateModeSelectLine();

    if ((now - bootMs0) > ENC_BOOT_GUARD_MS && encPressed && !encBtnPrev && (now - encLastMs) > ENC_DEBOUNCE_MS)
    {
      currentMode = (modeSelIndex == 0) ? MODE_CARBOT : MODE_ARMBOT;
      if (currentMode == MODE_CARBOT)
        showCarbotScreen();
      else
        showArmbotScreen();
      appState = APP_RUN;
      encLastMs = now;
    }
    encBtnPrev = encPressed;
    return;
  }

  // --- Running Mode ---

  // Switch Mode
  if ((now - bootMs0) > ENC_BOOT_GUARD_MS && encPressed && !encBtnPrev && (now - encLastMs) > ENC_DEBOUNCE_MS)
  {
    currentMode = (currentMode == MODE_CARBOT) ? MODE_ARMBOT : MODE_CARBOT;
    if (currentMode == MODE_CARBOT)
      showCarbotScreen();
    else
    {
      showArmbotScreen();
      armCalibrated = false;
    }
    encLastMs = now;
  }
  encBtnPrev = encPressed;

  // --- ARMBOT Logic ---
  if (currentMode == MODE_ARMBOT)
  {
    if (!armCalibrated)
    {
      joyDeadZoneX = 300;
      joyDeadZoneY = 300;
      armCalibrated = true;
    }

    int xRaw = iotbot.joystickXRead();
    int yRaw = iotbot.joystickYRead();
    int potRaw = iotbot.potentiometerRead();

    if (xRaw == 0)
      xRaw = 2048;

    int dx = xRaw - joyXCenter;
    int dy = yRaw - joyYCenter;

    // Axis 1 (Rotation)
    if (abs(dx) > joyDeadZoneX)
    {
      int step = (abs(dx) / JOY_STEP_DIV) + 1;
      if (step > JOY_STEP_MAX)
        step = JOY_STEP_MAX;
      armRotAngle += (dx < 0 ? +step : -step);
      armRotAngle = constrain(armRotAngle, 0, 180);
    }

    // Axis 2 (Shoulder)
    if (abs(dy) > joyDeadZoneY)
    {
      int step = (abs(dy) / JOY_STEP_DIV) + 1;
      if (step > JOY_STEP_MAX)
        step = JOY_STEP_MAX;
      armShoulderAngle += (dy < 0 ? -step : step);
      armShoulderAngle = constrain(armShoulderAngle, 0, 180);
    }

    // Axis 3 (Elbow)
    armElbowAngle = map(potRaw, 0, 4095, 0, 180);
    armElbowAngle = constrain(armElbowAngle, 0, 180);

    // Gripper
    int currEnc = iotbot.encoderRead();
    int encDiff = currEnc - lastEncoderVal;
    lastEncoderVal = currEnc;
    if (encDiff != 0)
    {
      armGripAngle += encDiff * 5;
      armGripAngle = constrain(armGripAngle, 10, 120);
    }

    bool b3Now = iotbot.button3Read();
    if (b3Now && !b3Prev && (now - b3LastChange) > B3_DEBOUNCE_MS)
    {
      const int GRIP_OPEN = 20;
      const int GRIP_CLOSE = 120;
      armGripAngle = (armGripAngle < (GRIP_OPEN + GRIP_CLOSE) / 2) ? GRIP_CLOSE : GRIP_OPEN;
      b3LastChange = now;
    }
    b3Prev = b3Now;

    iotbot.lcdWriteFixed(4, 1, armRotAngle, 3);
    iotbot.lcdWriteFixed(12, 1, armShoulderAngle, 3);
    iotbot.lcdWriteFixed(4, 2, armElbowAngle, 3);
    iotbot.lcdWriteFixed(12, 2, armGripAngle, 3);

    armData.axis1 = armRotAngle;
    armData.axis2 = armShoulderAngle;
    armData.axis3 = armElbowAngle;
    armData.gripper = armGripAngle;
    armData.action = 0; // Actions disabled in armbot for now without B1/B2
    
    if (now - lastSendMs >= SEND_INTERVAL_MS)
    {
      iotbot.sendESPNow(broadcastAddress, (uint8_t *)&armData, sizeof(armData));
      lastSendMs = now;
      iotbot.lcdWriteFixedTxt(18, 0, iotbot.lastSendStatus ? "*" : "*/", 2);
    }
  }
  // --- CARBOT Logic ---
  else
  {
    int yRaw = iotbot.joystickYRead();
    int ldrRaw = iotbot.ldrRead();
    int dy = yRaw - joyYCenter;

    // Steering: Use Potentiometer instead of Joystick X
    int potRaw = iotbot.potentiometerRead();
    int steerAngle = map(potRaw, 0, 4095, 135, 45); // Reversed
    steerAngle = constrain(steerAngle, 45, 135);
    carData.axis1 = steerAngle;
    iotbot.lcdWriteFixed(17, 0, steerAngle, 3);

    // Speed
    // Carbot expects 0-180 range: <80 Backward, 80-100 Stop, >100 Forward
    int speed = 90; // Default Stop
    if (dy < -DEADZONE)
    {
      // Backward (0 to 80)
      speed = map(dy, -DEADZONE, -joyYCenter, 80, 0);
      distanceWarningActive = false; // Clear warning when moving back or stopping
    }
    else if (dy > DEADZONE)
    {
      // Forward (100 to 180)
      if (carbotDistance < 10) {
         speed = 90; // Stop! Block forward movement.
         distanceWarningActive = true;
         
         // Blink logic & Sound for Warning
         if (now - lastDistanceBlinkMs > 250) {
            distanceBlinkState = !distanceBlinkState;
            if (distanceBlinkState) {
               iotbot.lcdWriteFixedTxt(0, 2, "   !! DUR !!    ", 16);
               iotbot.buzzerPlay(2500, 100);
            } else {
               char buf[17];
               snprintf(buf, sizeof(buf), "Mesafe: %3d cm  ", carbotDistance);
               iotbot.lcdWriteFixedTxt(0, 2, buf, 16);
            }
            lastDistanceBlinkMs = now;
         }
      } else {
         speed = map(dy, DEADZONE, 4095 - joyYCenter, 100, 180);
         distanceWarningActive = false;
      }
    }
    else 
    {
      distanceWarningActive = false;
    }
    
    speed = constrain(speed, 0, 180);
    carData.axis2 = speed;

    // LED - Joystick & B3 Button Handling
    const int LDR_THRESHOLD = 1200;
    bool autoShouldOn = (ldrRaw < LDR_THRESHOLD);

    // joystickButtonRead() is usually LOW when pressed for INPUT_PULLUP
    bool joyBtnRaw = !iotbot.joystickButtonRead(); // true when physically pressed down
    bool b3Now = iotbot.button3Read();
    
    static bool comboHandled = false;
    static unsigned long comboTimer = 0;
    
    // Check simultaneous press for Auto Mode (Combination)
    if (joyBtnRaw && b3Now) {
        if (!comboHandled && (now - comboTimer > JOY_DEBOUNCE_MS)) {
            // Both pressed and debounced
            ledAutoMode = !ledAutoMode;
            iotbot.lcdWriteCR(7, 1, ledAutoMode ? "(A)" : "   ");
            iotbot.buzzerPlay(1500, 100);
            comboHandled = true;
        }
    } else {
        comboTimer = now; // Reset debounce if not both pressed
        if (!joyBtnRaw && !b3Now) {
            comboHandled = false; // Reset combo lock only when BOTH are released
        }
    }
    
    // Single JoyBtn press for LED (Debounced, triggers on release)
    switch(joyBtnState) {
       case 0: // IDLE
          if (joyBtnRaw) {
             joyBtnState = 1;
             joyBtnTimer = now;
          }
          break;
       case 1: // DEBOUNCE_PRESS
          if (now - joyBtnTimer > JOY_DEBOUNCE_MS) {
             if (joyBtnRaw) {
                joyBtnState = 2; // confirmed press
             } else {
                joyBtnState = 0; // glitch
             }
          }
          break;
       case 2: // PRESSED
          if (!joyBtnRaw) {
             // Released - toggle LED ONLY if we didn't just use it for a combo
             if (!comboHandled) {
                ledState = !ledState;
                iotbot.lcdWriteFixedTxt(4, 1, ledState ? "ON " : "OFF", 3);
             }
             joyBtnState = 3; // WAIT_RELEASE
             joyBtnTimer = now;
          }
          break;
       case 3: // WAIT_RELEASE
          if (!joyBtnRaw) {
             if (now - joyBtnTimer > JOY_DEBOUNCE_MS) {
                joyBtnState = 0; // back to IDLE
             }
          } else {
             joyBtnTimer = now; // reset timer if bouncing
          }
          break;
    }

    if (ledAutoMode)
    {
      ledState = autoShouldOn;
      iotbot.lcdWriteFixedTxt(4, 1, ledState ? "ON " : "OFF", 3);
    }

    carData.action = 0;
    // B3 acts as horn ONLY if it's not being used as a combo key
    if (b3Now && !comboHandled)  
      carData.action = 1;
    else if (ledState)
      carData.action = 2; // LED ON
    else
      carData.action = 3; // LED OFF

    if (now - lastSendMs >= SEND_INTERVAL_MS)
    {
      iotbot.sendESPNow(broadcastAddress, (uint8_t *)&carData, sizeof(carData));
      lastSendMs = now;
      iotbot.lcdWriteFixedTxt(12, 1, iotbot.lastSendStatus ? "CON:*  " : "CON:*/ ", 7);
    }
  }
}
