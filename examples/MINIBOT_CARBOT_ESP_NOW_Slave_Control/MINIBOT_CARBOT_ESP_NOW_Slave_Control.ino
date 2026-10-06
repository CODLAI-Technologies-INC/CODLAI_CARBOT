/*
 * CARBOT ESP-NOW Slave Control / CARBOT ESP-NOW Slave Kontrolü
 *
 * This example receives commands from an IOTBOT (Master) via ESP-NOW to control CARBOT.
 * Bu örnek, IOTBOT'tan (Master) ESP-NOW üzerinden gelen komutları alarak CARBOT'u kontrol eder.
 * Master: IOTBOT_Armbot_and_Carbot_Wireless_Control.ino
 *
 * Komut / command (deviceType 2):
 *   axis1  = direksiyon / steering (45-135)
 *   axis2  = hiz / speed 0-180: 90 dur / stop, <80 geri / backward, >100 ileri / forward
 *            (90'dan uzaklastikca PWM hizi artar / PWM speed grows away from 90)
 *   action = 1 korna / horn, 2 far acik / lights on, 3 far kapali / lights off,
 *            10 MAGAZA MODU / STORE MODE (kumanda su an ARMBOT'u kontrol ediyor /
 *            the controller is using ARMBOT right now)
 * Telemetri / telemetry (deviceType 3, bu karttan / from this board):
 *   axis3  = mesafe cm / distance cm, -1 = olculemedi / not measured (lights on)
 *
 * MAGAZA MODU / STORE MODE: Varsayilan olarak arac YERINDE gosteri yapar (far,
 * direksiyon, kisa korna) - kol kullanilirken masadan kendi kendine surulmesin
 * diye. Ileri-geri surerek gosteri icin STORE_MODE_DRIVE = true yapin.
 * / By default the car does an IN-PLACE show (lights, steering, short horn) so
 * it never drives off the table on its own while the arm is used. Set
 * STORE_MODE_DRIVE = true for a show that drives back and forth.
 *
 * GUVENLIK / SAFETY: 500 ms boyunca komut gelmezse (kumanda kapandi, menzil
 * disi, ARMBOT moduna gecildi) arac DURUR. / If no command arrives for 500 ms
 * (controller off, out of range, switched to ARMBOT mode) the car STOPS.
 */

#define USE_ESPNOW
#define USE_WIFI
#define USE_SERVO
#include <MINIBOT.h>
#include <CARBOT.h>

MINIBOT minibot;
CARBOT carbot;

static const uint8_t TYPE_CAR_CMD = 2;
static const uint8_t TYPE_CAR_TELEMETRY = 3;
static const unsigned long COMMAND_TIMEOUT_MS = 500; // Bu surede komut yoksa dur / stop if no command for this long
static const int MIN_PWM = 50;                        // Motorun kalkabildigi en dusuk PWM / lowest PWM that still moves the motor
static const uint8_t ACTION_STORE_MODE = 10;
static const bool STORE_MODE_DRIVE = false;           // true = gosteride yavasca ileri-geri sur / true = drive slowly back and forth in the show
static const int STORE_DRIVE_PWM = 110;               // Gosteri surus hizi / show driving speed
static const unsigned long STORE_STEP_MS = 450;       // Gosteri adim suresi / show step duration

static uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  carbot.begin();

  minibot.serialWrite("Initializing ESP-NOW Slave (Carbot)...");

  minibot.initESPNow();
  minibot.setWiFiChannel(1); // Master ile aynı kanalda olmalı / Must be on same channel as Master

  // Telemetri geri gonderebilmek icin yayin peer'i / broadcast peer to send telemetry back
#if defined(ESP8266)
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
#elif defined(ESP32)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
#endif

  minibot.startListening();
  minibot.serialWrite("Ready to receive commands! / Komutları almaya hazır!");

  // Startup Sound
  carbot.buzzerPlay(2000, 100);
  delay(100);
  carbot.buzzerPlay(3000, 100);
}

// Korna ve far, ultrasonik sensorle ayni pinleri paylasir: bunlar kullanilirken
// mesafe olculemez. / Horn and lights share pins with the ultrasonic sensor:
// no distance can be measured while they are in use.
static unsigned long lastDistSendMs = 0;
static unsigned long lastCommandMs = 0;
static bool stoppedByTimeout = true;
static bool actionRequested = false;
static bool storeMode = false;
static uint8_t storeStep = 0;
static unsigned long storeStepMs = 0;

// Magaza modu gosterisinin bir adimi (bloklamaz) / one step of the store-mode show (non-blocking)
void runStoreStep() {
  switch (storeStep % 8) {
    case 0: carbot.steer(90);  carbot.controlLED(true);  break;
    case 1: carbot.steer(60);  carbot.controlLED(false); break;
    case 2: carbot.steer(120); carbot.controlLED(true);  break;
    case 3: carbot.steer(90);  carbot.controlLED(false);
            carbot.buzzerPlay(1800, 40); break;
    case 4: carbot.controlLED(true);
            if (STORE_MODE_DRIVE) carbot.moveForward(STORE_DRIVE_PWM); break;
    case 5: carbot.stop(); carbot.controlLED(false); break;
    case 6: carbot.controlLED(true);
            if (STORE_MODE_DRIVE) carbot.moveBackward(STORE_DRIVE_PWM); break;
    default: carbot.stop(); carbot.controlLED(false); break;
  }
  storeStep++;
}

// 0-180 hiz komutunu harekete cevir / turn the 0-180 speed command into motion
void applySpeed(int speedVal) {
  if (speedVal > 100) {
    carbot.moveForward(constrain(map(speedVal, 100, 180, MIN_PWM, 255), MIN_PWM, 255));
  } else if (speedVal < 80) {
    carbot.moveBackward(constrain(map(speedVal, 80, 0, MIN_PWM, 255), MIN_PWM, 255));
  } else {
    carbot.stop();
  }
}

void loop() {
  unsigned long now = millis();

  if (minibot.newData)
  {
    CodlaiESPNowMessage msg = minibot.receivedData;
    minibot.newData = false;

    // Sadece GERCEK komutlar: tip 2 ve action 1-3. Baska araclarin (eski
    // surum) telemetrisi de tip 2'dir ama action=0'dir - onu komut sanip
    // "tam hiz geri" gitmeyelim. / Only REAL commands: type 2 with action 1-3.
    // Other (older) cars' telemetry is also type 2 but has action=0 - don't
    // mistake it for a "full speed backward" command.
    if (msg.deviceType == TYPE_CAR_CMD && msg.action == ACTION_STORE_MODE)
    {
      // Magaza modu komutu da "kumanda burada" demektir: zaman asimini tazeler
      // / A store-mode command also means "controller is here": refreshes the timeout
      lastCommandMs = now;
      stoppedByTimeout = false;
      actionRequested = true; // Far/korna pinleri kullanimda, mesafe olculmez / LED/horn pins busy, no distance
      if (!storeMode) {
        storeMode = true;
        storeStep = 0;
        storeStepMs = 0;
        carbot.stop();
        if (carbot.isUltrasonicActive()) carbot.disableUltrasonic();
      }
    }
    else if (msg.deviceType == TYPE_CAR_CMD && msg.action >= 1 && msg.action <= 3)
    {
      lastCommandMs = now;
      stoppedByTimeout = false;
      storeMode = false; // Normal komut: gosteriyi birak / normal command: leave the show

      carbot.steer(constrain(msg.axis1, 45, 135));
      applySpeed(msg.axis2);

      actionRequested = false;
      if (msg.action == 1)
      {
        actionRequested = true;
        if (carbot.isUltrasonicActive()) carbot.disableUltrasonic();
        carbot.buzzerPlay(1000, 50); // Horn
      }
      else if (msg.action == 2)
      {
        actionRequested = true;
        if (carbot.isUltrasonicActive()) carbot.disableUltrasonic();
        carbot.controlLED(true); // Lights On
      }
      else // action == 3
      {
        // Ultrasonik modda LED zaten sonuk; gereksiz ac-kapa yapmayalim.
        // / In ultrasonic mode the LED is already off; avoid needless toggling.
        if (!carbot.isUltrasonicActive()) carbot.controlLED(false);
      }
    }
  }

  // Komut kesildiyse dur / stop when commands stop arriving
  // (magaza komutlari 500 ms'de bir gelir; payi biraz fazla tut / store commands
  // come every 500 ms, so allow a little more time in store mode)
  unsigned long timeoutMs = storeMode ? (COMMAND_TIMEOUT_MS * 3) : COMMAND_TIMEOUT_MS;
  if (!stoppedByTimeout && (now - lastCommandMs) > timeoutMs)
  {
    carbot.stop();
    carbot.steer(90);
    stoppedByTimeout = true;
    storeMode = false;
    actionRequested = false;
  }

  // Magaza modu gosterisi / store-mode show
  if (storeMode && (storeStepMs == 0 || now - storeStepMs >= STORE_STEP_MS)) {
    storeStepMs = now;
    runStoreStep();
  }

  // Telemetri ~300 ms'de bir: far/korna yoksa mesafe, varsa -1 (bilinmiyor).
  // Her durumda gonderilir, kumanda bagliligi buradan anlar.
  // / Telemetry every ~300 ms: distance when no lights/horn, otherwise -1
  // (unknown). Always sent, so the controller can tell the link is alive.
  if (now - lastDistSendMs >= 300) {
    lastDistSendMs = now;
    int distanceCm = -1;
    if (!actionRequested) {
      if (!carbot.isUltrasonicActive()) carbot.enableUltrasonic();
      float dist = carbot.readUltrasonicCM();
      if (dist >= 0) distanceCm = (int)dist;
    }
    CodlaiESPNowMessage msg = {};
    msg.deviceType = TYPE_CAR_TELEMETRY;
    msg.axis3 = distanceCm;
    esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
  }
}
