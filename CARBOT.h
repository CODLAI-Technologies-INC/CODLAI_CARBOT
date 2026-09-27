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
  void moveForward(int speed = 255);            // Move the car forward at a given PWM speed (0-255, default = full speed) / Aracı verilen PWM hızında (0-255, varsayılan tam hız) ileri hareket ettir
  void moveBackward(int speed = 255);           // Move the car backward at a given PWM speed (0-255, default = full speed) / Aracı verilen PWM hızında (0-255, varsayılan tam hız) geri hareket ettir
  void stop();                                  // Stop the car / Aracı durdur
  void steer(int angle);                        // Steer the car (0-180 degrees) / Direksiyonu verilen açıya çevir
  void servoTestPose(bool highPose);            // Servo test pose / Servo test pozu
  void dcMotorTestForward();                    // DC motor test forward / DC motor test ileri
  void hornLedTest(bool active);                // Horn+LED test state / Korna+LED test durumu
  void standardModeForward();                   // Standard forward mode / Standart ileri mod
  void storeModeStep(uint8_t step);             // Execute one store mode step / Store mod adımı çalıştır
  void controlLED(bool state);                  // Control the car's LED headlights / Farları kontrol et
  void buzzerPlay(int frequency, int duration); // Play a sound with the buzzer / Buzzer çal
  void enableUltrasonic(int echoPin = -1, int trigPin = -1); // Use ultrasonic sensor (shared pins auto disable LED/buzzer) / Ultrasonik sensörü kullan (paylaşılan pinlerde LED/buzzer otomatik kapanır)
  void disableUltrasonic();                     // Restore LED/buzzer control after ultrasonic mode / Ultrasonik moddan sonra LED/buzzer kontrolünü geri getir
  float readUltrasonicCM(unsigned long timeout = 30000); // Auto-enable if needed, return cm or -1 / Gerekirse otomatik açar, cm veya -1 döner
  bool isUltrasonicActive() const;               // Check if ultrasonic mode is currently enabled / Ultrasonik modun açık olup olmadığını kontrol et
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
  int _ultrasonicEchoPin = -1; // Echo pin shared with the LED / LED ile paylaşılan Echo pini
  int _ultrasonicTrigPin = -1; // Trig pin shared with the buzzer / Buzzer ile paylaşılan Trig pini
  bool _ultrasonicActive = false; // Tracks when ultrasonic mode reuses LED/buzzer pins / Ultrasonik mod LED/buzzer pinlerini yeniden kullandığında takip edilir
  static const int _buzzerLedcChannel = 14;
  static const int _motor1LedcChannel = 12; // ESP32: motorPin1 icin ayrilmis PWM kanali / dedicated PWM channel for motorPin1
  static const int _motor2LedcChannel = 13; // ESP32: motorPin2 icin ayrilmis PWM kanali / dedicated PWM channel for motorPin2

  void configurePins(); // Configure pins based on the platform / Platforma göre pinleri ayarla
  bool ultrasonicUsesSharedPins() const; // Check if ultrasonic pins overlap LED/buzzer pins / Ultrasonik pinler LED/buzzer ile cakisiyor mu
  void warnSharedPins(const char *messageEn, const char *messageTr); // Print bilingual warning / Iki dilli uyari yaz
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
  _ultrasonicEchoPin = _ledPin;
  _ultrasonicTrigPin = _buzzerPin;
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

#if defined(ESP32)
  pinMode(_buzzerPin, OUTPUT);
  ledcSetup(_buzzerLedcChannel, 2000, 8);
  ledcAttachPin(_buzzerPin, _buzzerLedcChannel);

  // Degisken hiz (PWM) icin motor pinlerini LEDC kanallarina bagla / attach
  // the motor pins to LEDC channels for variable-speed (PWM) control.
  ledcSetup(_motor1LedcChannel, 20000, 8); // 20kHz: motor uguldamasini onlemek icin duyulabilir aralik disinda / above the audible range to avoid motor whine
  ledcAttachPin(_motorPin1, _motor1LedcChannel);
  ledcSetup(_motor2LedcChannel, 20000, 8);
  ledcAttachPin(_motorPin2, _motor2LedcChannel);
#elif defined(ESP8266)
  // ESP8266'nin analogWrite() varsayilan araligi 0-1023'tur; 0-255 (standart
  // Arduino PWM) skalasiyla calisabilmek icin araligi burada sabitliyoruz.
  // ESP8266's analogWrite() defaults to a 0-1023 range; fix it here so we
  // can work with the standard Arduino 0-255 PWM scale.
  analogWriteRange(255);
#endif
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
  if (!_ultrasonicActive || !ultrasonicUsesSharedPins())
  {
    pinMode(_buzzerPin, OUTPUT);
    pinMode(_ledPin, OUTPUT);
  }
}

// Move the car forward at a given PWM speed (0-255) / Aracı verilen PWM hızında ileri hareket ettir
inline void CARBOT::moveForward(int speed)
{
  speed = constrain(speed, 0, 255);
#if defined(ESP32)
  ledcWrite(_motor2LedcChannel, 0);
  ledcWrite(_motor1LedcChannel, speed);
#elif defined(ESP8266)
  analogWrite(_motorPin2, 0);
  analogWrite(_motorPin1, speed);
#endif
}

// Move the car backward at a given PWM speed (0-255) / Aracı verilen PWM hızında geri hareket ettir
inline void CARBOT::moveBackward(int speed)
{
  speed = constrain(speed, 0, 255);
#if defined(ESP32)
  ledcWrite(_motor1LedcChannel, 0);
  ledcWrite(_motor2LedcChannel, speed);
#elif defined(ESP8266)
  analogWrite(_motorPin1, 0);
  analogWrite(_motorPin2, speed);
#endif
}

// Stop the car / Aracı durdur
inline void CARBOT::stop()
{
#if defined(ESP32)
  ledcWrite(_motor1LedcChannel, 0);
  ledcWrite(_motor2LedcChannel, 0);
#elif defined(ESP8266)
  analogWrite(_motorPin1, 0);
  analogWrite(_motorPin2, 0);
#endif
}

// Steer the car (0-180 degrees) / Direksiyonu verilen açıya çevir
inline void CARBOT::steer(int angle)
{
  _steeringServo.write(constrain(angle, 0, 180));
}

inline void CARBOT::servoTestPose(bool highPose)
{
  steer(highPose ? 180 : 0);
}

inline void CARBOT::dcMotorTestForward()
{
  moveForward();
}

inline void CARBOT::hornLedTest(bool active)
{
  if (active) {
    buzzerPlay(1000, 80);
    controlLED(true);
  } else {
    controlLED(false);
  }
}

inline void CARBOT::standardModeForward()
{
  moveForward();
}

inline void CARBOT::storeModeStep(uint8_t step)
{
  switch (step % 4) {
    case 0:
      moveForward();
      break;
    case 1:
      steer(45);
      break;
    case 2:
      moveBackward();
      break;
    default:
      stop();
      break;
  }
}

// Control the car's LED headlights / Farları kontrol et
inline void CARBOT::controlLED(bool state)
{
  if (_ultrasonicActive && ultrasonicUsesSharedPins()) {
    warnSharedPins(
      "Warning: LED requested; ultrasonic disabled due to shared pins.",
      "Uyari: LED istendi; paylasilan pinler nedeniyle ultrasonik devre disi."
    );
    disableUltrasonic();
  }
  digitalWrite(_ledPin, state ? LOW : HIGH); // LED is active LOW / LED aktif LOW
}

// Play a sound with the buzzer / Buzzer çal
inline void CARBOT::buzzerPlay(int frequency, int duration)
{
  if (_ultrasonicActive && ultrasonicUsesSharedPins()) {
    warnSharedPins(
      "Warning: Buzzer requested; ultrasonic disabled due to shared pins.",
      "Uyari: Buzzer istendi; paylasilan pinler nedeniyle ultrasonik devre disi."
    );
    disableUltrasonic();
  }
#if defined(ESP32)
  ledcSetup(_buzzerLedcChannel, frequency, 8);
  ledcWriteTone(_buzzerLedcChannel, frequency);
  delay(duration);
  ledcWriteTone(_buzzerLedcChannel, 0);
#elif defined(ESP8266)
  tone(_buzzerPin, frequency, duration);
  delay(duration);
  noTone(_buzzerPin);
#endif
}

inline void CARBOT::enableUltrasonic(int echoPin, int trigPin)
{
  if (echoPin < 0) {
    echoPin = _ledPin;
  }
  if (trigPin < 0) {
    trigPin = _buzzerPin;
  }
  _ultrasonicEchoPin = echoPin;
  _ultrasonicTrigPin = trigPin;
  if (!_ultrasonicActive && ultrasonicUsesSharedPins()) {
    warnSharedPins(
      "Warning: Ultrasonic enabled; LED/Buzzer disabled due to shared pins.",
      "Uyari: Ultrasonik acildi; paylasilan pinler nedeniyle LED/Buzzer devre disi."
    );
  }
  pinMode(_ultrasonicEchoPin, INPUT);
  pinMode(_ultrasonicTrigPin, OUTPUT);
  digitalWrite(_ultrasonicTrigPin, LOW);
  _ultrasonicActive = true;
}

inline void CARBOT::disableUltrasonic()
{
  _ultrasonicActive = false;
  configurePins();
}

inline bool CARBOT::isUltrasonicActive() const
{
  return _ultrasonicActive;
}

inline float CARBOT::readUltrasonicCM(unsigned long timeout)
{
  if (!_ultrasonicActive) {
    enableUltrasonic(_ultrasonicEchoPin, _ultrasonicTrigPin);
  }
  if (_ultrasonicEchoPin < 0 || _ultrasonicTrigPin < 0) {
    return -1;
  }
  digitalWrite(_ultrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_ultrasonicTrigPin, LOW);
  unsigned long duration = pulseIn(_ultrasonicEchoPin, HIGH, timeout);
  if (duration == 0) {
    return -1;
  }
  return duration / 58.0f;
}

inline bool CARBOT::ultrasonicUsesSharedPins() const
{
  return (_ultrasonicEchoPin == _ledPin || _ultrasonicEchoPin == _buzzerPin ||
          _ultrasonicTrigPin == _ledPin || _ultrasonicTrigPin == _buzzerPin);
}

inline void CARBOT::warnSharedPins(const char *messageEn, const char *messageTr)
{
  Serial.println(messageEn);
  Serial.println(messageTr);
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
