# Changelog

# CODLAI ERA (New Models)

## [Unreleased]

## [1.2.0] - 2026-10-07
### Added
- **Ornekler bastan yazildi (7 ornek):** hepsi ayni kurala uyuyor - en ustte `bool turkish` ile TR/EN secimi, calisirken seri porttan `dil`/`lang` ile degisim, iki dilli ve bloklamayan seri komutlar (`yardim`/`help`). Bir seyi suren ornekler OTOMATIK gosteriyle baslar, buton ile MANUEL moda gecilir.
- Ornekler `Klasor/Klasor.ino` yapisina tasindi: Arduino IDE *Dosya > Ornekler* menusunde hepsi gorunur. `library.json` "examples" alani glob kullaniyor.
- `examples/examples.json`: her ornegin yolu, karti, gereken moduller/ayarlar, TR/EN ozeti ve seri komutlari (editor.codlai.com "Kutuphane Ornekleri" ekrani icin; `scripts/generate_examples_json.py` ile uretilir).

### Fixed
- ESP32: ultrasonik kullanildiktan sonra buzzer bir daha calmiyordu (pinMode LEDC baglantisini koparir); her calista yeniden baglaniyor.
- Ultrasonik <-> LED/buzzer paylasilan pin uyarisi her cagrida seri portu dolduruyordu; her yon icin acilista bir kez yaziliyor.
- Desteklenmeyen platformda net `#error` mesaji.

## [1.1.3] - 2026-09-29
### Fixed
- CodlaiESPNowMessage aciklamasi: deviceType 22-29 editor.codlai.com ozel/eslesmeli mesajlasma bloklarina rezerve edildi (22 ozel metin, 23 ozel sayi, 24 eslesme teklifi, 25 eslesme kabulu; 26-29 bos). Kutuphane davranisi degismedi - `espNowAvailable()` hala yalniz 20/21'i gorur.
- **KRITIK - kablosuz CARBOT hiz uyumsuzlugu:** 1.1.1'deki degisken hiz guncellemesi `MINIBOT_CARBOT_ESP_NOW_Slave_Control.ino`'yu isaretli hiz (-255..255) bekleyecek sekilde degistirmis, ama kumanda (`IOTBOT_Armbot_and_Carbot_Wireless_Control.ino`) hep 0-180 (90 = dur) gonderiyordu. Sonuc: kumanda "dur" dediginde arac ILERI gidiyor, geri komutu da yavas ileri oluyordu. Alici yeniden 0-180 kuralini kullaniyor (90 dur, <80 geri, >100 ileri) ve 90'dan uzaklastikca PWM hizi 50-255 arasinda artiyor; boylece kartlara yuklu eski kumandalarla da calisiyor.
- Kablosuz CARBOT guvenligi: 500 ms komut gelmezse (kumanda kapandi, menzil disi, ARMBOT moduna gecildi) arac duruyor; kumanda ARMBOT'a gecerken araca ayrica dur komutu gonderiyor. Alici sadece gercek komutlari (tip 2, action 1-3) isliyor - baska bir aracin telemetrisini "tam hiz geri" sanmiyor.
- CARBOT telemetrisi ayri tipte (deviceType 3) gonderiliyor ve far/korna acikken bile 300 ms'de bir yollaniyor (mesafe -1 = bilinmiyor). Kumanda eskimis mesafeyi artik "bilinmiyor" sayiyor: eskiden far acikken son olcum 10 cm altindaysa ileri hareket kalici olarak kilitleniyordu.
- Kablosuz kumanda: joystick X (GPIO15, ADC2) ESP-NOW acikken okunamadigi icin ARMBOT govde donusu hic calismiyor ve mod secim ekraninda ARMBOT secilemiyordu. Govde donusu encoder'a, mod secimi encoder donusune tasindi (kiskac B3 ile ac/kapa). Kol hareket hizi artik loop hizindan bagimsiz (20 ms adim). Baglanti gostergesi artik robottan gelen paketlere bakiyor (yayin paketlerinde gonderim onayi olmadigi icin eskisi hep "bagli" gosteriyordu).
- Kablolu kumanda (`IOTBOT_Armbot_and_Carbot_Wired_Control.ino`): ARMBOT'a gecip CARBOT'a donunce motorlar ve direksiyon calismiyordu (iki robot ayni pinleri kullanir ve pinler kol servolarinda kaliyordu); her mod degisiminde bir robot birakilip digeri yeniden baglaniyor. Magaza (demo) modu joystick butonunu ters okuyup ARMBOT moduna girer girmez kendiliginden basliyordu. CARBOT'taki kopya ARMBOT'taki guncel (degisken hizli) kopyayla esitlendi.
- Depoda eski bir PlatformIO kurulum kaydi (`.piopm`, surum 1.2.3) izleniyordu ve GitHub'a da gidiyordu; kutuphaneyi GitHub'dan ya da yerel klasorden (symlink) kuran projelerde bagimlilik agaci yanlis surum gosteriyordu. Dosya kaldirildi ve `.gitignore`'a eklendi. (Duvar projesi oturumunun bulgusu.)
- `IOTBOT_Armbot_and_Carbot_Wired_Control.ino` derlenmiyordu ("'B12State' does not name a type"): `B12State`, `Mode`, `AppState` tipleri ilk fonksiyondan sonra tanimliydi, Arduino ise fonksiyon prototiplerini ilk fonksiyonun onune ekler. Tip tanimlari dosyanin basina tasindi.
- `IOTBOT_Armbot_and_Carbot_Wireless_Control.ino` derlenmiyordu ("stray 'ï' in program"): dosyanin basinda UTF-8 BOM vardi; Arduino basina `#include <Arduino.h>` ekleyince BOM satir ortasinda kaliyordu. BOM kaldirildi.

### Added
- Kablosuz kontrol: **otomatik magaza modu** - kumanda bir robotu kontrol ederken digerine 500 ms'de bir magaza modu komutu (tip 1/2, `action = 10`) gonderiyor. CARBOT alicisi varsayilan olarak YERINDE gosteri yapiyor (far, direksiyon, kisa korna; masadan kendi kendine surulmesin diye), `STORE_MODE_DRIVE = true` ile yavasca ileri-geri de surebiliyor. Normal komut gelince gosteri hemen bitiyor.
- Kablosuz kumanda: ARMBOT govdesi joystick X ile de donuyor. X (GPIO15, ADC2) WiFi/ESP-NOW calisirken cogu zaman mesgul oldugu icin `adc2_get_raw()` ile okunuyor; sadece basarili ve taze (200 ms) okumalar kullaniliyor, ekranda `X+`/`X-` gosteriliyor, encoder yedek olarak calismaya devam ediyor.

## [1.1.2] - 2026-09-27
### Changed
- ESP-NOW alicisi (`startListening()`) artik eski (kutuphanenin onceki surumlerinde daha kucuk olan) `CodlaiESPNowMessage` boyutundaki paketleri de kabul ediyor - bkz. CODLAI_IOTBOT 1.7.1'deki ayni degisiklik.

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

