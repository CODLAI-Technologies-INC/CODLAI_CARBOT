/*
 * CARBOT ESP-NOW Slave Control / CARBOT ESP-NOW Slave Kontrolü
 * 
 * This example receives commands from an IOTBOT (Master) via ESP-NOW to control CARBOT.
 * Bu örnek, IOTBOT'tan (Master) ESP-NOW üzerinden gelen komutları alarak CARBOT'u kontrol eder.
 */

#define USE_ESPNOW
#define USE_WIFI
#define USE_SERVO
#include <MINIBOT.h>
#include <CARBOT.h>

MINIBOT minibot;
CARBOT carbot;

void setup() {
  minibot.begin();
  minibot.serialStart(115200);
  carbot.begin();
  
  minibot.serialWrite("Initializing ESP-NOW Slave (Carbot)...");
  
  minibot.initESPNow();
  minibot.setWiFiChannel(1); // Master ile aynı kanalda olmalı / Must be on same channel as Master
  
  // Add broadcast peer to send data back
#if defined(ESP8266)
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
#elif defined(ESP32)
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
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

// Actions (Horn, LED) share pins with Ultrasonic.
// When horn or LED is needed, the ultrasonic is disabled automatically.
// Bu yüzden master tarafindan korna vs isteniyorsa mesafe olcmeyi bekletelim.
static unsigned long lastDistSendMs = 0;
bool actionRequested = false;

void loop() {
  unsigned long now = millis();

  if (minibot.newData)
  {
    minibot.newData = false;

    if (minibot.receivedData.deviceType != 2) // 2 = Carbot
      return; 

    carbot.steer(minibot.receivedData.axis1);

    // axis2: isaretli hiz degeri (-255..255) - negatif geri, pozitif ileri,
    // buyukluk PWM hizi (bkz. IOTBOT_Armbot_and_Carbot_Wireless_Control.ino'daki
    // ayni kural). / axis2: signed speed value (-255..255) - negative is
    // backward, positive is forward, magnitude is the PWM speed (same
    // convention as IOTBOT_Armbot_and_Carbot_Wireless_Control.ino).
    int speedVal = minibot.receivedData.axis2;
    if (speedVal > 10) {
      carbot.moveForward(speedVal);
    } else if (speedVal < -10) {
      carbot.moveBackward(-speedVal);
    } else {
      carbot.stop();
    }

    actionRequested = false;

    // Actions
    if (minibot.receivedData.action == 1)
    {
      actionRequested = true;
      // Korna basiliyken mesafe sensorunu kapali tut
      if (carbot.isUltrasonicActive()) carbot.disableUltrasonic();
      carbot.buzzerPlay(1000, 50); // Horn
    }
    else if (minibot.receivedData.action == 2)
    {
      actionRequested = true;
      if (carbot.isUltrasonicActive()) carbot.disableUltrasonic();
      carbot.controlLED(true); // Lights On
    }
    else if (minibot.receivedData.action == 3)
    {
      // Işıkları Kapat (Lights Off)
      // Ultrasonik modda LED zaten sönük durumdadır.
      // Bu yüzden gereksiz yere ultrasoniği "kapat-aç" yapmamak için sadece ultrasonik aktif değilse farı kapat.
      if (!carbot.isUltrasonicActive()) {
        carbot.controlLED(false); // Lights Off
      }
    }
  }

  // Handle Ultrasonic Data Sending (Send every ~200ms)
  if (!actionRequested && (now - lastDistSendMs >= 300)) {
    if (!carbot.isUltrasonicActive()) {
      carbot.enableUltrasonic();
    }
    float dist = carbot.readUltrasonicCM();
    if (dist >= 0) {
      CodlaiESPNowMessage msg;
      msg.deviceType = 2; 
      msg.axis1 = 0;
      msg.axis2 = 0;
      msg.axis3 = (int)dist; // Use axis3 for distance
      msg.gripper = 0;
      msg.action = 0;

      uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#if defined(ESP8266)
      esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
#elif defined(ESP32)
      esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
#endif
    }
    lastDistSendMs = now;
  }
}
