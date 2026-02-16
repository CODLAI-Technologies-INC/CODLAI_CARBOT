# CODLAI_CARBOT Library Documentation / Kütüphane Dokümantasyonu

**EN:** The `CARBOT` library controls a 2-wheeled vehicle with steering (Ackermann-like).
**TR:** `CARBOT` kütüphanesi, 2 tekerlekli ve direksiyonlu (Ackermann benzeri) bir aracı kontrol eder.

### Basic Control / Temel Kontrol
*   `void begin()`
    *   **EN:** Initializes the motors and the steering servo.
    *   **TR:** Motorları ve direksiyon servosunu başlatır.
*   `void end()`
    *   **EN:** Stops the motors and detaches the servo.
    *   **TR:** Motorları durdurur ve servoyu ayırır.
*   `void moveForward()`
    *   **EN:** Drives the vehicle forward.
    *   **TR:** Aracı ileri sürer.
*   `void moveBackward()`
    *   **EN:** Drives the vehicle backward.
    *   **TR:** Aracı geri sürer.
*   `void stop()`
    *   **EN:** Stops the motors.
    *   **TR:** Motorları durdurur.
*   `void steer(int angle)`
    *   **EN:** Sets the steering angle (0-180 degrees).
    *   **TR:** Direksiyon açısını ayarlar (0-180 derece).
*   `void controlLED(bool state)`
    *   **EN:** Turns the headlights on/off.
    *   **TR:** Farları açar/kapatır.

### Ultrasonic Sensor / Ultrasonik Sensor
*   `void enableUltrasonic(int echoPin = -1, int trigPin = -1)`
    *   **EN:** Enables ultrasonic mode. Shared pins auto-disable LED/buzzer and print warnings.
    *   **TR:** Ultrasonik modu açar. Paylasilan pinlerde LED/buzzer otomatik kapanir ve uyari basilir.
*   `void disableUltrasonic()`
    *   **EN:** Disables ultrasonic mode and restores LED/buzzer control.
    *   **TR:** Ultrasonik modu kapatir ve LED/buzzer kontrolunu geri getirir.
*   `float readUltrasonicCM(unsigned long timeout = 30000)`
    *   **EN:** Auto-enables ultrasonic if needed and returns distance in cm, or -1 on timeout.
    *   **TR:** Gerekirse otomatik açar ve cm dondurur, zaman asiminda -1 verir.
*   `bool isUltrasonicActive() const`
    *   **EN:** Reports whether ultrasonic mode is active.
    *   **TR:** Ultrasonik modun aktif olup olmadigini bildirir.

### Sound and Music / Ses ve Müzik
*   `void buzzerPlay(int frequency, int duration)`
    *   **EN:** Plays a tone.
    *   **TR:** Ton çalar.
*   `void istiklalMarsiCal()`
    *   **EN:** Plays the Turkish National Anthem (İstiklal Marşı).
    *   **TR:** İstiklal Marşı'nı çalar.

### Communication / İletişim
*   `void serialStart(int baudrate)`
    *   **EN:** Starts the serial port.
    *   **TR:** Seri portu başlatır.
*   `void serialWrite(...)`
    *   **EN:** Writes data to the serial port.
    *   **TR:** Seri porta veri yazar.
*   **ESP-NOW**:
    *   `void initESPNow()`
        *   **EN:** Initializes ESP-NOW.
        *   **TR:** ESP-NOW başlatır.
    *   `void setWiFiChannel(int channel)`
        *   **EN:** Sets the channel.
        *   **TR:** Kanal ayarlar.
    *   `void sendESPNow(...)`
        *   **EN:** Sends data.
        *   **TR:** Veri gönderir.
    *   `void registerOnRecv(...)`
        *   **EN:** Registers the data reception function.
        *   **TR:** Veri alma fonksiyonunu kaydeder.
    *   `void startListening()`
        *   **EN:** Starts listening for data.
        *   **TR:** Veri dinlemeyi başlatır.
