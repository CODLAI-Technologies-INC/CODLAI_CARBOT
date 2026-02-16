# CARBOT Library User Guide  
This library is specially designed and produced by the CODLAI developer team to control the CARBOT product.  

![CARBOT Image](<images/1.jpg>)  

## Dependencies & Versions / Bağımlılıklar ve Sürümler

### Platform Versions
- **ESP8266**: 2.5.0 - 3.0.2
- **ESP32**: 1.0.6 - 2.0.14

### Library Dependencies
- **ESP32Servo**: ^1.1.0 (Only for ESP32 / Sadece ESP32 için)
- **Servo**: Built-in (For ESP8266 / ESP8266 için dahili)

---

## Ultrasonic Sensor (Shared Pins)
The ultrasonic sensor shares the LED and buzzer pins by default:
- **ESP8266**: echo=GPIO4 (LED), trig=GPIO5 (buzzer)
- **ESP32**: echo=GPIO26 (LED), trig=GPIO25 (buzzer)

Behavior:
- `readUltrasonicCM()` automatically enables ultrasonic mode if it is off.
- If ultrasonic is active on shared pins and you call LED/buzzer, ultrasonic is disabled automatically.
- If ultrasonic is enabled on shared pins, LED/buzzer are disabled automatically.
- Every conflict prints a bilingual warning on Serial (EN + TR).

---

## Using with Arduino IDE  

### Installation  

1. Open the Arduino IDE.  
2. Go to **"Sketch" -> "Include Library" -> "Manage Libraries..."** from the menu bar.  
3. Type **"CARBOT"** in the search box.  
4. Find the CARBOT library and click the **"Install"** button to complete the installation.  

---

# CARBOT Kütüphanesi Kullanım Kılavuzu  
Bu kütüphane CODLAI geliştirici ekibi tarafından CARBOT ürününü kontrol etmek için özel olarak tasarlanmış ve üretilmiştir.  

![CARBOT Görsel](<images/1.jpg>)  

---

## Arduino IDE ile Kullanım  

### Kurulum  

1. Arduino IDE'yi açın.  
2. Menü çubuğundan **"Sketch" -> "Include Library" -> "Manage Libraries..."** seçeneğine gidin.  
3. Arama kutusuna **"CARBOT"** yazın.  
4. CARBOT kütüphanesini bulun ve **"Install"** düğmesine tıklayarak kurulumu tamamlayın.  

---

## Ultrasonik Sensör (Paylaşılan Pinler)
Varsayılan olarak ultrasonik sensör LED ve buzzer pinlerini paylaşır:
- **ESP8266**: echo=GPIO4 (LED), trig=GPIO5 (buzzer)
- **ESP32**: echo=GPIO26 (LED), trig=GPIO25 (buzzer)

Davranış:
- `readUltrasonicCM()` çağrıldığında ultrasonik mod kapalıysa otomatik açılır.
- Ultrasonik mod (paylaşılan pinlerde) açıkken LED/buzzer çağrılırsa ultrasonik otomatik kapanır.
- Ultrasonik açıldığında LED/buzzer otomatik devre dışı kalır.
- Her çakışmada seri porta İngilizce ve Türkçe uyarı basılır.

---

# Library Structure & Contributing / Kütüphane Yapısı ve Katkıda Bulunma
This is a lightweight library.
- **Structure:** Simple class-based structure without complex configuration.
- **Extension:** Add new functions directly to the class. If adding heavy modules, consider refactoring to a modular design.

Bu hafif bir kütüphanedir.
- **Yapı:** Karmaşık yapılandırma gerektirmeyen basit sınıf tabanlı yapı.
- **Genişletme:** Yeni fonksiyonları doğrudan sınıfa ekleyin. Ağır modüller ekleyecekseniz, modüler bir tasarıma geçmeyi düşünün.

# Katkıda Bulunma / Contributing  
Katkıda bulunmak isterseniz, lütfen GitHub deposuna **Pull Request** gönderin.  
If you'd like to contribute, please send a **Pull Request** to the GitHub repository.  

---

# Lisans / License  
Bu kütüphane **2024 Yılında Samed KAYA** tarafından lisanslanmıştır. Detaylar için LICENSE dosyasına bakınız.  
This library is licensed by **Samed KAYA in 2024**. See the LICENSE file for details.  

---

## Additional Visuals / Ek Görseller  

![CARBOT Front](<images/2.jpg>)  
![CARBOT Angle](<images/3.jpg>)  
![CARBOT Diagram](<images/4.jpg>)  
![CARBOT Vector](<images/5.jpg>)  
