# CODLAI CARBOT Library Reference / Kütüphane Referansı

## Introduction / Giriş
**EN:** The CARBOT library controls a 2-wheel drive car with steering servo. It includes motor control, steering, LED headlights, buzzer, and ESP-NOW communication features.
**TR:** CARBOT kütüphanesi, direksiyon servolu 2 tekerlekten çekişli bir aracı kontrol eder. Motor kontrolü, direksiyon, LED farlar, buzzer ve ESP-NOW haberleşme özelliklerini içerir.

## Functions / Fonksiyonlar

### begin
**EN:** Initializes the library, configures motor/LED/buzzer pins, and attaches the steering servo.
**TR:** Kütüphaneyi başlatır, motor/LED/buzzer pinlerini ayarlar ve direksiyon servosunu bağlar.
**Syntax:** `void begin()`

### end
**EN:** Stops the car, turns off LEDs, and detaches the steering servo.
**TR:** Aracı durdurur, LED'leri kapatır ve direksiyon servosunu ayırır.
**Syntax:** `void end()`

### moveForward
**EN:** Moves the car forward.
**TR:** Aracı ileri hareket ettirir.
**Syntax:** `void moveForward()`

### moveBackward
**EN:** Moves the car backward.
**TR:** Aracı geri hareket ettirir.
**Syntax:** `void moveBackward()`

### stop
**EN:** Stops the car motors.
**TR:** Araç motorlarını durdurur.
**Syntax:** `void stop()`

### steer
**EN:** Sets the steering servo angle (0-180). 90 is usually center.
**TR:** Direksiyon servo açısını ayarlar (0-180). 90 genellikle merkezdir.
**Syntax:** `void steer(int angle)`

### controlLED
**EN:** Turns the headlights on or off.
**TR:** Farları açar veya kapatır.
**Syntax:** `void controlLED(bool state)`
*   `state`: `true` (ON) or `false` (OFF)

### buzzerPlay
**EN:** Plays a tone at a specific frequency for a specific duration.
**TR:** Belirli bir frekansta ve sürede bir ton çalar.
**Syntax:** `void buzzerPlay(int frequency, int duration)`

### istiklalMarsiCal
**EN:** Plays the Turkish National Anthem melody.
**TR:** İstiklal Marşı melodisini çalar.
**Syntax:** `void istiklalMarsiCal()`

### initESPNow
**EN:** Initializes the ESP-NOW protocol for wireless communication.
**TR:** Kablosuz haberleşme için ESP-NOW protokolünü başlatır.
**Syntax:** `void initESPNow()`

### setWiFiChannel
**EN:** Sets the WiFi channel for ESP-NOW communication.
**TR:** ESP-NOW haberleşmesi için WiFi kanalını ayarlar.
**Syntax:** `void setWiFiChannel(int channel)`

### sendESPNow
**EN:** Sends data to a specific MAC address via ESP-NOW.
**TR:** ESP-NOW üzerinden belirli bir MAC adresine veri gönderir.
**Syntax:** `void sendESPNow(const uint8_t *macAddr, const uint8_t *data, int len)`

### registerOnRecv
**EN:** Registers a callback function to handle received ESP-NOW data.
**TR:** Alınan ESP-NOW verilerini işlemek için bir geri çağırma (callback) fonksiyonu kaydeder.
**Syntax:** `void registerOnRecv(esp_now_recv_cb_t cb)`

### serialStart
**EN:** Starts serial communication at the specified baud rate.
**TR:** Seri haberleşmeyi belirtilen baud hızında başlatır.
**Syntax:** `void serialStart(int baudrate)`

### serialWrite
**EN:** Writes data to the serial port (Overloaded for String, int, float, bool).
**TR:** Seri porta veri yazar (String, int, float, bool için aşırı yüklenmiştir).
**Syntax:** `void serialWrite(data)`
