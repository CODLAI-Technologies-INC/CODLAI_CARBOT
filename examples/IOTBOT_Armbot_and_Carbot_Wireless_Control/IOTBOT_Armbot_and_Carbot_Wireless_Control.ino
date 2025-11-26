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

// Connection Status
// volatile bool lastSendStatus = false; // Now handled by library

// Callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
// {
//   lastSendStatus = (status == ESP_NOW_SEND_SUCCESS);
// }

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
static bool b2Prev = false;
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
  iotbot.lcdWriteCR(12, 1, iotbot.lastSendStatus ? "CON:OK " : "CON:ERR");

  iotbot.lcdWriteFixedTxt(2, 2, "X:Steer  Y:Drive", 16);

  iotbot.lcdWriteCR(0, 3, "B1:Horn  ENC:MODE");
}

void showArmbotScreen()
{
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "ARMBOT (WIFI)");
  iotbot.lcdWriteCR(0, 1, "ROT:");
  iotbot.lcdWriteFixedTxt(4, 1, "", 3);
  iotbot.lcdWriteCR(8, 1, "SHO:");
  iotbot.lcdWriteFixedTxt(12, 1, "", 3);
  iotbot.lcdWriteCR(19, 0, iotbot.lastSendStatus ? "*" : "!");

  iotbot.lcdWriteCR(0, 2, "ELB:");
  iotbot.lcdWriteFixedTxt(4, 2, "", 3);
  iotbot.lcdWriteCR(8, 2, "GRP:");
  iotbot.lcdWriteFixedTxt(12, 2, "", 3);
  iotbot.lcdWriteCR(0, 3, "B1:Horn  B2:Note");
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

  iotbot.initESPNow();
  iotbot.setWiFiChannel(1);
  // esp_now_register_send_cb(OnDataSent); // Handled by library now

  if (!iotbot.addBroadcastPeer(1))
  {
    iotbot.serialWrite("Failed to add peer");
    return;
  }

  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "Calibrating...");
  iotbot.lcdWriteCR(0, 1, "Do not touch!");
  delay(2000); // Wait for power to stabilize

  iotbot.calibrateJoystick(joyXCenter, joyYCenter, 50); // Take 50 samples

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

  iotbot.serialWrite("Setup Complete.");
}

void loop()
{
  unsigned long now = millis();
  bool encPressed = (digitalRead(ENCODER_BUTTON_PIN) == LOW);

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
      // Re-calibrate specifically for Armbot if needed, or just use global center
      // The original code had a specific calibration here. Let's keep it simple and use global center for now
      // or re-run calibration.
      // Actually, let's just use the global center to avoid delay.
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
    armData.action = 0;
    if (iotbot.button1Read())
      armData.action = 1;
    else if (iotbot.button2Read())
      armData.action = 2;

    if (now - lastSendMs >= SEND_INTERVAL_MS)
    {
      iotbot.sendESPNow(broadcastAddress, (uint8_t *)&armData, sizeof(armData));
      lastSendMs = now;
      iotbot.lcdWriteCR(19, 0, iotbot.lastSendStatus ? "*" : "!");
    }
  }
  // --- CARBOT Logic ---
  else
  {
    int xRaw = iotbot.joystickXRead();
    int yRaw = iotbot.joystickYRead();
    int ldrRaw = iotbot.ldrRead();
    int dx = xRaw - joyXCenter;
    int dy = yRaw - joyYCenter;

    // Steering
    int angle = 90;
    if (abs(dx) > DEADZONE)
    {
      float range = (dx > 0) ? (4095.0f - joyXCenter) : (float)joyXCenter;
      float norm = (float)dx / range;
      angle = 90 + (int)(norm * 45.0f);
      angle = constrain(angle, 45, 135);
    }
    carData.axis1 = angle;
    iotbot.lcdWriteFixed(17, 0, angle, 3);

    // Speed
    // Carbot expects 0-180 range: <80 Backward, 80-100 Stop, >100 Forward
    int speed = 90; // Default Stop
    if (dy < -DEADZONE)
    {
      // Backward (0 to 80)
      speed = map(dy, -DEADZONE, -joyYCenter, 80, 0);
    }
    else if (dy > DEADZONE)
    {
      // Forward (100 to 180)
      speed = map(dy, DEADZONE, 4095 - joyYCenter, 100, 180);
    }
    speed = constrain(speed, 0, 180);
    carData.axis2 = speed;

    // LED
    const int LDR_THRESHOLD = 1200;
    bool autoShouldOn = (ldrRaw < LDR_THRESHOLD);

    bool b2Now = iotbot.button2Read();
    if (b2Now && !b2Prev && (now - lastBeepMs) > BEEP_DEBOUNCE_MS)
    {
      ledAutoMode = !ledAutoMode;
      iotbot.lcdWriteCR(7, 1, ledAutoMode ? "(A)" : "   ");
      lastBeepMs = now;
    }
    b2Prev = b2Now;

    bool b3Now = iotbot.button3Read();
    if (!ledAutoMode && b3Now && !b3Prev && (now - b3LastChange) > B3_DEBOUNCE_MS)
    {
      ledState = !ledState;
      iotbot.lcdWriteFixedTxt(4, 1, ledState ? "ON" : "OFF", 3);
      b3LastChange = now;
    }
    b3Prev = b3Now;

    if (ledAutoMode)
    {
      ledState = autoShouldOn;
      iotbot.lcdWriteFixedTxt(4, 1, ledState ? "ON" : "OFF", 3);
    }

    carData.action = 0;
    if (iotbot.button1Read())
      carData.action = 1;
    else if (ledState)
      carData.action = 2;
    else
      carData.action = 3;

    if (now - lastSendMs >= SEND_INTERVAL_MS)
    {
      iotbot.sendESPNow(broadcastAddress, (uint8_t *)&carData, sizeof(carData));
      lastSendMs = now;
      iotbot.lcdWriteCR(12, 1, iotbot.lastSendStatus ? "CON:OK " : "CON:ERR");
    }
  }
}