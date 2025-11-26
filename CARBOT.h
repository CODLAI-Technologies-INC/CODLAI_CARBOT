/*
 * CODLAI CARBOT Library
 * 
 * Structure Information:
 * This is a lightweight library designed for motor and servo control.
 * It does not require a configuration file as it does not include heavy dependencies.
 * 
 * How to Add New Features:
 * Simply add new function declarations in CARBOT.h and implementations in CARBOT.cpp.
 * If adding heavy dependencies (like WiFi), consider implementing a Config file structure similar to IOTBOT.
 */

#ifndef CARBOT_H
#define CARBOT_H

#include "Arduino.h"

#if defined(USE_ESPNOW)
#if defined(ESP32)
  #include <esp_now.h>
  #include <esp_wifi.h>
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <espnow.h>
  #include <ESP8266WiFi.h>
#endif
#endif

// Include the appropriate library based on the platform
#if defined(ESP32)
#include <ESP32Servo.h>
#elif defined(ESP8266)
#include <Servo.h>
#else
#include <Servo.h>
#endif

// Structure to receive data via ESP-NOW
#ifndef CODLAI_ESPNOW_MESSAGE_DEFINED
#define CODLAI_ESPNOW_MESSAGE_DEFINED
typedef struct {
  uint8_t deviceType; // 1 = Armbot, 2 = Carbot
  int axis1;
  int axis2;
  int axis3;
  int gripper;
  uint8_t action; // 0=None, 1=Horn, 2=Note
} CodlaiESPNowMessage;
#endif

class CARBOT
{
public:
  CARBOT();                                     // Constructor / Yapıcı
  void begin();                                 // Initialize the car bot / Araç botunu başlat
  void end();                                   // Stop the car bot and detach servos / Araç botunu durdur ve servoları ayır
  void moveForward();                           // Move the car forward / Aracı ileri hareket ettir
  void moveBackward();                          // Move the car backward / Aracı geri hareket ettir
  void stop();                                  // Stop the car / Aracı durdur
  void steer(int angle);                        // Steer the car (0-180 degrees) / Direksiyonu verilen açıya çevir
  void controlLED(bool state);                  // Control the car's LED headlights / Farları kontrol et
  void buzzerPlay(int frequency, int duration); // Play a sound with the buzzer / Buzzer çal
  void istiklalMarsiCal();                      // Play the National Anthem melody / İstiklal Marşı'nı çal

  /*********************************** Serial Port ***********************************
   */
  void serialStart(int baundrate);
  void serialWrite(const char *message);
  void serialWrite(String message);
  void serialWrite(long value);
  void serialWrite(int value);
  void serialWrite(float value);
  void serialWrite(bool value);

  /*********************************** ESP-NOW ***********************************
   */
#if defined(USE_ESPNOW)
  void initESPNow();
  void setWiFiChannel(int channel);
  void sendESPNow(const uint8_t *macAddr, const uint8_t *data, int len);
  void registerOnRecv(esp_now_recv_cb_t cb);

  // ESP-NOW Data Handling
  CodlaiESPNowMessage receivedData;
  volatile bool newData = false;
  static CARBOT* _instance;

  void startListening() {
      _instance = this;
      #if defined(ESP32)
      registerOnRecv([](const uint8_t *mac, const uint8_t *incomingData, int len) {
      #elif defined(ESP8266)
      registerOnRecv([](uint8_t *mac, uint8_t *incomingData, uint8_t len) {
      #endif
          if (_instance && len == sizeof(CodlaiESPNowMessage)) {
              memcpy(&_instance->receivedData, incomingData, sizeof(CodlaiESPNowMessage));
              _instance->newData = true;
          }
      });
  }
#endif

private:
  Servo _steeringServo; // Servo object for steering / Direksiyon servo motor nesnesi
  int currentAngle = 0;

  // Pins for motor, servo, buzzer, and LED / Motor, servo, buzzer ve LED pinleri
  int _steeringPin;
  int _motorPin1;
  int _motorPin2;
  int _buzzerPin;
  int _ledPin;

  void configurePins(); // Configure pins based on the platform / Platforma göre pinleri ayarla
};

/*********************************** IMPLEMENTATION ***********************************/

// Constructor / Yapıcı
inline CARBOT::CARBOT()
{
#if defined(ESP32)
  _steeringPin = 32;
  _motorPin1 = 27;
  _motorPin2 = 33;
  _buzzerPin = 25;
  _ledPin = 26;
#elif defined(ESP8266)
  _steeringPin = 13;
  _motorPin1 = 12;
  _motorPin2 = 14;
  _buzzerPin = 5;
  _ledPin = 4;
#endif
}

// Initialize the car bot / Araç botunu başlat
inline void CARBOT::begin()
{
  configurePins();
#if defined(ESP32)
  _steeringServo.attach(_steeringPin, 500, 2500);
  // **ESP32 için 1000-2000 µs kullan**
#elif defined(ESP8266)
  _steeringServo.attach(_steeringPin, 500, 2500);
#else
  if (!servo.attach(pin)) // **ESP32 için 1000-2000 µs kullan**
#endif

  _steeringServo.write(90); // Set steering to the initial position / Direksiyonu başlangıç pozisyonuna ayarla
}

// Stop the car bot and detach servos / Araç botunu durdur ve servoları ayır
inline void CARBOT::end()
{
  stop(); // Stop motors
  controlLED(false); // Turn off LED
  _steeringServo.detach();
}

// Configure pins based on the platform / Platforma göre pinleri ayarla
inline void CARBOT::configurePins()
{
  pinMode(_motorPin1, OUTPUT);
  pinMode(_motorPin2, OUTPUT);
  pinMode(_buzzerPin, OUTPUT);
  pinMode(_ledPin, OUTPUT);
}

// Move the car forward / Aracı ileri hareket ettir
inline void CARBOT::moveForward()
{
  digitalWrite(_motorPin1, HIGH);
  digitalWrite(_motorPin2, LOW);
}

// Move the car backward / Aracı geri hareket ettir
inline void CARBOT::moveBackward()
{
  digitalWrite(_motorPin1, LOW);
  digitalWrite(_motorPin2, HIGH);
}

// Stop the car / Aracı durdur
inline void CARBOT::stop()
{
  digitalWrite(_motorPin1, LOW);
  digitalWrite(_motorPin2, LOW);
}

// Steer the car (0-180 degrees) / Direksiyonu verilen açıya çevir
inline void CARBOT::steer(int angle)
{
  _steeringServo.write(constrain(angle, 0, 180));
}

// Control the car's LED headlights / Farları kontrol et
inline void CARBOT::controlLED(bool state)
{
  digitalWrite(_ledPin, state ? LOW : HIGH); // LED is active LOW / LED aktif LOW
}

// Play a sound with the buzzer / Buzzer çal
inline void CARBOT::buzzerPlay(int frequency, int duration)
{
#if defined(ESP32)
  analogWrite(_buzzerPin, frequency);
  delay(duration);
  analogWrite(_buzzerPin, 0);
#elif defined(ESP8266)
  tone(_buzzerPin, frequency, duration);
  delay(duration);
  noTone(_buzzerPin);
#endif
}

// Play the National Anthem / İstiklal Marşı'nı çal
inline void CARBOT::istiklalMarsiCal()
{
#if defined(ESP32)
  // Adjusted tones for analogWrite (mapped to appropriate PWM values)
  buzzerPlay(100, 400); // C4
  delay(400);
  buzzerPlay(130, 400); // E4
  delay(400);
  buzzerPlay(160, 400); // G4
  delay(400);
  buzzerPlay(145, 400); // F4
  delay(400);
  buzzerPlay(130, 600); // E4
  delay(600);
  buzzerPlay(145, 400); // F4
  delay(400);
  buzzerPlay(130, 400); // E4
  delay(400);
  buzzerPlay(115, 400); // D4
  delay(400);
  buzzerPlay(100, 600); // C4
  delay(600);
  buzzerPlay(130, 400); // E4
  delay(400);
  buzzerPlay(145, 400); // F4
  delay(400);
  buzzerPlay(160, 400); // G4
  delay(400);
  buzzerPlay(130, 600); // E4
  delay(600);
  buzzerPlay(115, 400); // D4
  delay(400);
  buzzerPlay(100, 400); // C4
  delay(400);
  buzzerPlay(115, 600); // D4
  delay(600);

#elif defined(ESP8266)
  buzzerPlay(262, 400);
  delay(400);
  buzzerPlay(330, 400);
  delay(400);
  buzzerPlay(392, 400);
  delay(400);
  buzzerPlay(349, 400);
  delay(400);
  buzzerPlay(330, 600);
  delay(600);
  buzzerPlay(349, 400);
  delay(400);
  buzzerPlay(330, 400);
  delay(400);
  buzzerPlay(294, 400);
  delay(400);
  buzzerPlay(262, 600);
  delay(600);
  buzzerPlay(330, 400);
  delay(400);
  buzzerPlay(349, 400);
  delay(400);
  buzzerPlay(392, 400);
  delay(400);
  buzzerPlay(330, 600);
  delay(600);
  buzzerPlay(294, 400);
  delay(400);
  buzzerPlay(262, 400);
  delay(400);
  buzzerPlay(294, 600);
  delay(600);
#endif
}

/*********************************** Serial Port ***********************************
 */
inline void CARBOT::serialStart(int baudrate)
{
  Serial.begin(baudrate);
}

inline void CARBOT::serialWrite(const char *message)
{
  Serial.println(message);
}

inline void CARBOT::serialWrite(String message)
{
  Serial.println(message.c_str());
}

inline void CARBOT::serialWrite(long value)
{
  Serial.println(String(value).c_str());
}

inline void CARBOT::serialWrite(int value)
{
  Serial.println(String(value).c_str());
}

inline void CARBOT::serialWrite(float value)
{
  Serial.println(String(value).c_str());
}

inline void CARBOT::serialWrite(bool value)
{
  Serial.println(value ? "true" : "false");
}

/*********************************** ESP-NOW IMPLEMENTATION ***********************************/
#if defined(USE_ESPNOW)

inline void CARBOT::initESPNow()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != 0)
  {
    return;
  }
}

inline void CARBOT::setWiFiChannel(int channel)
{
#if defined(ESP32)
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
#elif defined(ESP8266)
    wifi_set_channel(channel);
#endif
}

inline void CARBOT::sendESPNow(const uint8_t *macAddr, const uint8_t *data, int len)
{
#if defined(ESP32)
  if (!esp_now_is_peer_exist(macAddr))
  {
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, macAddr, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      return;
    }
  }
  esp_now_send(macAddr, data, len);
#elif defined(ESP8266)
  if (!esp_now_is_peer_exist(const_cast<uint8_t*>(macAddr)))
  {
    if (esp_now_add_peer(const_cast<uint8_t*>(macAddr), ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != 0)
    {
      return;
    }
  }
  esp_now_send(const_cast<uint8_t*>(macAddr), const_cast<uint8_t*>(data), len);
#endif
}

inline void CARBOT::registerOnRecv(esp_now_recv_cb_t cb)
{
  esp_now_register_recv_cb(cb);
}

// Initialize static member
inline CARBOT* CARBOT::_instance = nullptr;
#endif

#endif
