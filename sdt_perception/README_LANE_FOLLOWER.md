# Lane Follower Node

Bu node, kamera görüntüsünü kullanarak tek şerit takibi yapar ve köşe algıladığında tank dönüşü gerçekleştirir.

## Özellikler

- **Tek Şerit Takibi**: Kamera görüntüsünden şerit çizgilerini tespit eder
- **Köşe Algılama**: Dönüş noktalarında köşe tespiti yapar
- **Tank Dönüşü**: Köşe algılandığında 90 derece tank dönüşü yapar
- **PID Kontrol**: Şerit takibi için PID kontrolcü kullanır
- **Parametrik Yapı**: Tüm parametreler konfigürasyon dosyasından ayarlanabilir

## Kullanım

### Node'u Çalıştırma

```bash
# Launch dosyası ile çalıştırma
ros2 launch sdt_perception lane_follower.launch.py

# Veya direkt node olarak çalıştırma
ros2 run sdt_perception lane_follower_node
```

### Gerekli Topic'ler

**Subscribed Topics:**
- `/camera/camera/color/image_raw` (sensor_msgs/Image): Kamera görüntüsü

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist): Robot kontrol komutları
- `/lane_debug/image` (sensor_msgs/Image): Debug görüntüsü (şerit tespiti, köşe algılama, tank dönüşü durumu)

## Konfigürasyon

Tüm parametreler `config/lane_follower.yaml` dosyasından ayarlanabilir:

### Hız Parametreleri
- `max_linear_speed`: Maksimum ileri hız (varsayılan: 1.0)
- `max_angular_speed`: Maksimum açısal hız (varsayılan: 1.0)
- `min_angular_speed`: Minimum açısal hız (varsayılan: -1.0)

### Şerit Takip Parametreleri
- `turn_threshold`: Köşe algılama eşiği (piksel)
- `turn_duration`: Tank dönüşü süresi (saniye)

### PID Kontrol Parametreleri
- `kp`: Oransal kazanç
- `ki`: İntegral kazanç
- `kd`: Türevsel kazanç

### Görüntü İşleme Parametreleri
- `canny_low/high`: Canny kenar tespiti eşikleri
- `hough_threshold`: Hough çizgi tespiti eşiği
- `min_line_length`: Minimum çizgi uzunluğu
- `max_line_gap`: Maksimum çizgi boşluğu

### Köşe Algılama Parametreleri
- `corner_hough_threshold`: Köşe tespiti için Hough eşiği
- `corner_min_line_length`: Köşe tespiti için minimum çizgi uzunluğu
- `corner_max_line_gap`: Köşe tespiti için maksimum çizgi boşluğu
- `min_vertical_lines`: Köşe algılama için minimum dikey çizgi sayısı

## Algoritma

### Şerit Tespiti
1. Görüntüyü gri tonlamaya çevir
2. Gaussian blur uygula
3. Canny kenar tespiti yap
4. Alt yarıyı ROI olarak al
5. Hough çizgi tespiti ile şerit çizgilerini bul
6. Şerit merkezini hesapla

### Köşe Algılama
1. Görüntüyü gri tonlamaya çevir
2. Canny kenar tespiti yap
3. Alt yarıyı ROI olarak al
4. Hough çizgi tespiti yap
5. Dikey çizgileri say
6. Belirli sayıda dikey çizgi varsa köşe olarak algıla

### Şerit Takibi
1. Şerit merkezi ile görüntü merkezi arasındaki hatayı hesapla
2. PID kontrolcü ile açısal hız hesapla
3. Hız sınırlarını uygula
4. `/cmd_vel` topic'ine yayınla

### Tank Dönüşü
1. Köşe algılandığında dönüş yönünü belirle
2. İleri hızı sıfırla
3. Maksimum açısal hızla dönüş yap
4. Belirlenen süre sonunda dönüşü tamamla

## Debug Görüntüsü

Node `/lane_debug/image` topic'ine debug görüntüsü yayınlar. Bu görüntüde:

- **Yeşil çizgi**: Görüntü merkezi
- **Kırmızı çizgi**: Tespit edilen şerit merkezi
- **Sarı çizgiler**: Tespit edilen şerit çizgileri
- **Mavi dikdörtgen**: ROI (Region of Interest) bölgesi
- **Metin bilgileri**: Hata mesafesi, köşe algılama durumu, tank dönüşü durumu

Debug görüntüsünü görüntülemek için:
```bash
ros2 run rqt_image_view rqt_image_view
```

## Hata Ayıklama

Node çalışırken aşağıdaki log mesajlarını görebilirsiniz:

- `Şerit takibi - Hata: X.XX, Angular: X.XXX`: Normal şerit takibi
- `Köşe algılandı! Dönüş yönü: Sol/Sağ`: Köşe algılama
- `Tank dönüşü - Süre: X.Xs`: Tank dönüşü devam ediyor
- `Tank dönüşü tamamlandı`: Dönüş tamamlandı
- `Şerit bulunamadı - Robot durduruldu`: Şerit tespit edilemedi

## Notlar

- Node, şerit bulunamadığında robotu durdurur
- Tank dönüşü sırasında ileri hareket yapılmaz
- PID parametreleri robotun dinamiklerine göre ayarlanmalıdır
- Köşe algılama parametreleri ortam koşullarına göre optimize edilmelidir
