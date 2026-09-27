# Changelog

# CODLAI ERA (New Models)

## [Unreleased]

## [1.1.1] - 2026-09-27
### Changed
- `CodlaiESPNowMessage` yapisina IOTBOT/MINIBOT/ROLEBOT 1.7.0/1.5.0/1.5.0'daki basit ESP-NOW mesajlasma ozelligiyle (`espNowSendText`/`espNowSendNumber`) UYUMLU KALMASI icin `char text[32]` ve `float value` alanlari eklendi. CARBOT'un kendi ESP-NOW kullanimini (ARM/CAR kontrolu) ETKILEMEZ - sadece ayni sketch icinde IOTBOT/MINIBOT/ROLEBOT ile birlikte kullanildiginda yapi boyutunun (sizeof) tutarli kalmasini saglar. Yeni bir CARBOT fonksiyonu eklenmedi.

## [1.1.0] - 2026-09-27
### Added
- Yeni ornek: `CARBOT_Guard_Mode_Example.ino` - arac yerinde durup on ultrasonik sensorle etrafini gozetler; bir seyin yaklastigini algilarsa farlari yakip sonduru, korna calar ve kisa bir sure geri kacar (bekci arac modu).
- **Degisken hiz (PWM) destegi**: `moveForward(int speed = 255)` ve `moveBackward(int speed = 255)` artik bir PWM hiz degeri (0-255) kabul ediyor - onceden motor pinlerine sadece `digitalWrite(HIGH/LOW)` yapiliyordu, yani araç sadece tam hizda ileri/geri gidebiliyordu. ESP32'de motor pinleri LEDC kanallarina (20kHz, motor uguldamasini onlemek icin), ESP8266'da `analogWrite()`'a (0-255 araligina sabitlenmis) baglandi. Varsayilan deger 255 (tam hiz) oldugu icin eski `moveForward()`/`moveBackward()` cagrilari davranis degistirmeden calismaya devam ediyor - geriye donuk uyumlu. Mobil uygulama/kumanda tarafinda gercek analog gaz/joystick kontrolu icin gerekliydi.

### Fixed
- `MINIBOT_CARBOT_ESP_NOW_Slave_Control.ino`, gelen `axis2` degerini (>100 ileri, <80 geri seklinde) sabit esiklerle 3 duruma indirgeyip sadece tam hizda calistiriyordu; bu deger aslinda isaretli bir hiz (-255..255, bkz. `IOTBOT_Armbot_and_Carbot_Wireless_Control.ino`) olarak hesaplaniyordu ama kullanilmiyordu. Artik gercek PWM hizini `moveForward(speed)`/`moveBackward(speed)`'e iletiyor.
- `IOTBOT_Armbot_and_Carbot_Wired_Control.ino`'daki joystick ile CARBOT surme mantigi da ayni sekilde artik sabit tam hiz yerine joystick sapmasina orantili degisken hiz gonderiyor.

## [1.0.4] - 2026-02-16
### Added
- Automatic ultrasonic/LED/buzzer conflict management with bilingual warnings.
- `readUltrasonicCM` now auto-enables ultrasonic mode when needed.

### Changed
- Ultrasonic mode only disables LED/buzzer when shared pins are used.

## [1.0.3] - 2025-03-09
### Fixed
- PlatformIO yeniden yayını için sürüm numarası artırıldı.

## [1.0.0] - 2025-03-04
### Added
- **Rebranding**: Transitioned from CODROB to CODLAI.
- Standardized library structure.
- Added `serialStart` and `serialWrite` wrappers.
- Updated examples to use library wrappers.
- Initial Release for PlatformIO and Arduino IDE.

---

# CODROB ERA (Legacy Models)

## [1.2.3] - 2025-03-04
### Added
- Added Arduino IDE Suport

## [1.0.3] - 2025-01-23
### Fixed
- Resolved buzzer and servo signal conflicts for ESP32 and ESP8266.
- Improved LED control for consistent behavior.

### Added
- Conditional dependencies for ESP32 and ESP8266 for better compatibility.
- Error logging during servo initialization.

### Changed
- Optimized servo initialization for better reliability.

## [1.0.2] - 2025-01-20
### Fixed
- Corrected LED control logic.
- Improved motor speed handling for smoother transitions.

## [1.0.1] - 2025-01-10
### Added
- Support for backward and forward motor movements.
- Enhanced buzzer tone functionality.

## [1.0.0] - 2025-01-05
### Added
- Initial release with basic car control functionality.

