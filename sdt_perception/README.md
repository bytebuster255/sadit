# SDT Perception Paketi

Bu paket, robot için şerit tespiti ve şerit takibi özelliklerini sağlar.

## Özellikler

### Lane Detector Node
- Basit şerit tespiti
- PID kontrol ile şerit takibi
- Debug görüntüsü yayınlama

### Lane Follower Node
- Gelişmiş şerit tespiti
- Köşe algılama
- Tank dönüşü (90 derece)
- PID kontrol ile şerit takibi
- Debug görüntüsü yayınlama

## Kurulum

### Gereksinimler
```bash
pip3 install opencv-python numpy
```

### Build
```bash
cd /home/gunes/sadit_ws
colcon build --packages-select sdt_perception
source install/setup.bash
```

## Kullanım

### Lane Detector Node
```bash
# Launch dosyası ile çalıştırma
ros2 launch sdt_perception lane_detector.launch.py

# Veya direkt node olarak çalıştırma
ros2 run sdt_perception lane_detector_node
```

### Lane Follower Node
```bash
# Launch dosyası ile çalıştırma
ros2 launch sdt_perception lane_follower.launch.py

# Veya direkt node olarak çalıştırma
ros2 run sdt_perception lane_follower_node
```

## Topic'ler

### Subscribed Topics
- `/camera/image_raw` (sensor_msgs/Image): Kamera görüntüsü

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Robot kontrol komutları
- `/lane_debug/image` (sensor_msgs/Image): Debug görüntüsü

## Konfigürasyon

Tüm parametreler konfigürasyon dosyalarından ayarlanabilir:

- `config/lane_detector.yaml`: Lane Detector parametreleri
- `config/lane_follower.yaml`: Lane Follower parametreleri

### Önemli Parametreler

#### Hız Parametreleri
- `max_linear_speed`: Maksimum ileri hız
- `max_angular_speed`: Maksimum açısal hız
- `min_angular_speed`: Minimum açısal hız

#### PID Kontrol Parametreleri
- `kp`: Oransal kazanç
- `ki`: İntegral kazanç
- `kd`: Türevsel kazanç

#### Görüntü İşleme Parametreleri
- `canny_low/high`: Canny kenar tespiti eşikleri
- `hough_threshold`: Hough çizgi tespiti eşiği
- `min_line_length`: Minimum çizgi uzunluğu
- `max_line_gap`: Maksimum çizgi boşluğu

## Debug

Debug görüntüsünü görüntülemek için:
```bash
ros2 run rqt_image_view rqt_image_view
```

Debug görüntüsünde:
- **Mavi dikdörtgen**: ROI (Region of Interest) bölgesi
- **Yeşil çizgi**: Görüntü merkezi
- **Kırmızı çizgi**: Tespit edilen şerit merkezi
- **Sarı çizgiler**: Tespit edilen şerit çizgileri
- **Metin bilgileri**: Hata mesafesi, köşe algılama durumu

## Algoritma

### Şerit Tespiti
1. Görüntüyü gri tonlamaya çevir
2. Gaussian blur uygula
3. Canny kenar tespiti yap
4. Alt yarıyı ROI olarak al
5. Hough çizgi tespiti ile şerit çizgilerini bul
6. Sol ve sağ şeritleri ayır
7. Şerit merkezini hesapla

### Şerit Takibi
1. Şerit merkezi ile görüntü merkezi arasındaki hatayı hesapla
2. PID kontrolcü ile açısal hız hesapla
3. Hız sınırlarını uygula
4. `/cmd_vel` topic'ine yayınla

### Köşe Algılama (Lane Follower)
1. Dikey çizgileri tespit et
2. Belirli sayıda dikey çizgi varsa köşe olarak algıla
3. Tank dönüşü başlat

### Tank Dönüşü (Lane Follower)
1. İleri hızı sıfırla
2. Maksimum açısal hızla dönüş yap
3. Belirlenen süre sonunda dönüşü tamamla

## Notlar

- Robot diff-drive ile hareket ediyor
- Tank dönüşü sırasında ileri hareket yapılmaz
- Şerit bulunamadığında robot durdurulur
- PID parametreleri robotun dinamiklerine göre ayarlanmalıdır
