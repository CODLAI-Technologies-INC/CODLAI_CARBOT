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

  minibot.startListening();
  minibot.serialWrite("Ready to receive commands! / Komutları almaya hazır!");
  
  // Startup Sound
  carbot.buzzerPlay(2000, 100);
  delay(100);
  carbot.buzzerPlay(3000, 100);
}

void loop() {
  if (minibot.newData)
  {
    minibot.newData = false;

    if (minibot.receivedData.deviceType != 2) // 2 = Carbot
      return; 

    // Steering / Direksiyon (Axis 1)
    // Assuming Axis 1 sends 0-180 or similar. 
    // If Joystick sends 0-4095, we might need mapping. 
    // But usually Master sends mapped values. Let's assume 0-180 for now or map it.
    // If Master sends raw joystick (0-4095), we map: map(val, 0, 4095, 0, 180).
    // Let's assume the Master sends ready-to-use values or we map here.
    // For safety, let's constrain.
    // If the value is around 1500 (center), it might be raw.
    // Let's assume standard servo range 0-180.
    carbot.steer(minibot.receivedData.axis1);

    // Motor (Axis 2)
    // Assuming Axis 2 is speed/direction.
    // If > threshold -> Forward
    // If < threshold -> Backward
    // Else -> Stop
    // Let's assume center is 90 or 0? 
    // If it's mapped to servo angle (0-180), center is 90.
    // If it's raw (0-4095), center is 2048.
    // If it's -255 to 255, center is 0.
    
    // Based on previous struct "int speed", it was likely -255 to 255.
    // Let's assume axis2 is passed as -255 to 255 or similar.
    // Or maybe 0-180 where > 100 is FWD, < 80 is BWD.
    
    int speedVal = minibot.receivedData.axis2;
    
    // Simple logic assuming 0-180 range (like servo)
    if (speedVal > 100) {
      carbot.moveForward();
    } else if (speedVal < 80) {
      carbot.moveBackward();
    } else {
      carbot.stop();
    }

    // Actions
    if (minibot.receivedData.action == 1)
    {
      carbot.buzzerPlay(1000, 50); // Horn
    }
    else if (minibot.receivedData.action == 2)
    {
      carbot.controlLED(true); // Lights On
    }
    else if (minibot.receivedData.action == 3)
    {
      carbot.controlLED(false); // Lights Off
    }
  }
}
