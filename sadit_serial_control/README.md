# SADIT Serial Control Package

Bu paket, SADIT robotunun Arduino ile seri iletişim kurarak 4 tekerlekli diferansiyel sürüş kontrolünü sağlar.

## Özellikler

- 4 tekerlekli diferansiyel sürüş kontrolü
- ROS2 Twist mesajlarını Arduino PWM komutlarına çevirme
- Gerçek zamanlı tekerlek durumu takibi
- Batarya voltajı izleme
- Joint states yayınlama
- Yapılandırılabilir parametreler

## Kurulum

```bash
# Workspace'de paketi derle
colcon build --packages-select sadit_serial_control

# Environment'ı source et
source install/setup.bash
```

## Kullanım

### Launch Dosyası ile Çalıştırma

```bash
# Varsayılan parametrelerle
ros2 launch sadit_serial_control sadit_control.launch.py

# Özel seri port ile
ros2 launch sadit_serial_control sadit_control.launch.py serial_port:=/dev/ttyUSB1

# Özel parametrelerle
ros2 launch sadit_serial_control sadit_control.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200 \
    max_linear_speed:=255 \
    wheel_separation:=0.5 \
    wheel_radius:=0.048
```

### Manuel Çalıştırma

```bash
ros2 run sadit_serial_control sadit_serial_controller
```

## Parametreler

| Parametre | Varsayılan | Açıklama |
|-----------|------------|----------|
| `serial_port` | `/dev/ttyUSB0` | Arduino bağlantı portu |
| `baud_rate` | `115200` | Seri iletişim hızı |
| `max_linear_speed` | `255` | Maksimum linear hız (PWM) |
| `max_angular_speed` | `30.0` | Maksimum angular hız (derece) |
| `linear_scale_factor` | `127.5` | Linear hız ölçekleme faktörü |
| `angular_scale_factor` | `30.0` | Angular hız ölçekleme faktörü |
| `wheel_separation` | `0.5` | Tekerlekler arası mesafe (m) |
| `wheel_radius` | `0.048` | Tekerlek yarıçapı (m) |
| `enable_4wd` | `true` | 4 tekerlek sürüş aktif |

## Topic'ler

### Subscribers
- `/cmd_vel` (geometry_msgs/Twist): Robot hareket komutları

### Publishers
- `/arduino_status` (std_msgs/String): Arduino durum mesajları
- `/joint_states` (sensor_msgs/JointState): Tekerlek joint durumları
- `/current_speed` (std_msgs/Float32): Mevcut hız
- `/battery_voltage` (std_msgs/Float32): Batarya voltajı

## Arduino Komut Formatı

### Gönderilen Komutlar
```
MOVE:motor1_pwm,motor2_pwm,motor3_pwm,motor4_pwm
```

Motor sıralaması:
- motor1: Sol ön tekerlek
- motor2: Sol arka tekerlek  
- motor3: Sağ ön tekerlek
- motor4: Sağ arka tekerlek

Örnek:
```
MOVE:150,150,150,150    # Düz ileri
MOVE:-100,-100,100,100  # Sola dönüş
MOVE:100,100,-100,-100  # Sağa dönüş
MOVE:0,0,0,0           # Dur
```

### Alınan Mesajlar
```
STATUS:motor1=150,motor2=150,motor3=150,motor4=150,battery=12.5
CMD_ACK:MOVE
CMD_ACK:STOP
ERROR:Invalid command
```

## Arduino Kodu Gereksinimleri

Arduino kodunuz aşağıdaki özelliklere sahip olmalıdır:

1. **4 Motor Kontrolü**: Sol ön, sol arka, sağ ön, sağ arka tekerlekler
2. **PWM Kontrolü**: Her motor için 0-255 arası PWM kontrolü (negatif değerler geri yön için)
3. **Seri İletişim**: 115200 baud rate ile seri iletişim
4. **Durum Raporlama**: STATUS mesajları ile tekerlek hızları ve batarya voltajı
5. **Komut Doğrulama**: CMD_ACK mesajları ile komut onayı
6. **Motor Pin Yapısı**: 
   - Motor 1 (Sol ön): LPWM1=51, RPWM1=50, LEN1=53, REN1=52
   - Motor 2 (Sol arka): LPWM2=47, RPWM2=46, LEN2=49, REN2=48
   - Motor 3 (Sağ ön): LPWM3=43, RPWM3=42, LEN3=45, REN3=44
   - Motor 4 (Sağ arka): LPWM4=39, RPWM4=38, LEN4=41, REN4=40

### Hazır Arduino Kodu

Bu paket ile birlikte `arduino/sadit_arduino_serial.ino` dosyası gelir. Bu dosyayı Arduino IDE'de açıp Arduino'nuzu yükleyebilirsiniz.

## Test

### Teleop ile Test
```bash
# Yeni terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Başka terminal
ros2 topic echo /arduino_status
ros2 topic echo /joint_states
```

### Manuel Test
```bash
# İleri git
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Dön
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

# Dur
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Sorun Giderme

### Seri Port Bağlantı Sorunu
```bash
# Mevcut seri portları listele
ls /dev/ttyUSB*
ls /dev/ttyACM*

# Port izinlerini kontrol et
sudo chmod 666 /dev/ttyUSB0
```

### Arduino Yanıt Vermiyor
1. Arduino kodunun yüklü olduğunu kontrol edin
2. Seri port bağlantısını kontrol edin
3. Baud rate ayarlarını kontrol edin
4. Arduino Serial Monitor ile test edin

## Lisans

Apache-2.0
