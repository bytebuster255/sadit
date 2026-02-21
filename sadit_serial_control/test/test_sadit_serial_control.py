#!/usr/bin/env python3

import serial
import time
import sys

class SaditMotorTester:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """
        SADIT Robot motor test sınıfı
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        
        # Motor tanımlamaları
        self.motors = {
            'motor1': 'Sağ Arka Motor',
            'motor2': 'Sol Arka Motor', 
            'motor3': 'Sağ Ön Motor',
            'motor4': 'Sol Ön Motor'
        }
        
        # Test parametreleri
        self.test_speed = 140  # Test hızı (0-140 arası)
        self.test_duration = 2  # Test süresi (saniye)
        
    def connect(self):
        """Seri bağlantıyı kur"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(2)  # Arduino reset için bekle
            print(f"✅ {self.port} portuna bağlandı")
            return True
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def disconnect(self):
        """Seri bağlantıyı kapat"""
        if self.serial_conn:
            self.serial_conn.close()
            print("🔌 Bağlantı kapatıldı")
    
    def send_command(self, command):
        """Komut gönder ve yanıt al"""
        if not self.serial_conn:
            print("❌ Bağlantı yok!")
            return None
            
        try:
            self.serial_conn.write(f"{command}\n".encode())
            time.sleep(0.1)
            
            # Yanıt oku
            response = ""
            while self.serial_conn.in_waiting:
                response += self.serial_conn.readline().decode().strip()
            
            return response
        except Exception as e:
            print(f"❌ Komut gönderme hatası: {e}")
            return None
    
    def stop_all_motors(self):
        """Tüm motorları durdur"""
        print("🛑 Tüm motorlar durduruluyor...")
        self.send_command("STOP")
        time.sleep(0.5)
    
    def test_single_motor(self, motor_num, speed=None, direction='forward'):
        """
        Tek motoru test et
        motor_num: 1-4 arası motor numarası
        speed: hız değeri (None ise test_speed kullanır)
        direction: 'forward' veya 'backward'
        """
        if motor_num < 1 or motor_num > 4:
            print("❌ Geçersiz motor numarası! 1-4 arası olmalı.")
            return
        
        if speed is None:
            speed = self.test_speed
        
        # Motor hızlarını hazırla
        speeds = [0, 0, 0, 0]
        motor_index = motor_num - 1
        
        if direction == 'forward':
            speeds[motor_index] = speed
        elif direction == 'backward':
            speeds[motor_index] = -speed
        else:
            print("❌ Geçersiz yön! 'forward' veya 'backward' olmalı.")
            return
        
        motor_name = self.motors[f'motor{motor_num}']
        direction_text = "ileri" if direction == 'forward' else "geri"
        
        print(f"🔄 {motor_name} {direction_text} yönde test ediliyor...")
        print(f"   Hız: {abs(speed)}, Süre: {self.test_duration} saniye")
        
        # Komutu gönder
        command = f"MOVE:{speeds[0]},{speeds[1]},{speeds[2]},{speeds[3]}"
        response = self.send_command(command)
        
        if response:
            print(f"   Arduino yanıtı: {response}")
        
        # Test süresi kadar bekle
        time.sleep(self.test_duration)
        
        # Motoru durdur
        self.stop_all_motors()
        print(f"✅ {motor_name} testi tamamlandı\n")
    
    def test_all_motors_forward(self):
        """Tüm motorları ileri yönde test et"""
        print("🚀 Tüm motorlar ileri yönde test ediliyor...")
        command = f"MOVE:{self.test_speed},{self.test_speed},{self.test_speed},{self.test_speed}"
        response = self.send_command(command)
        
        if response:
            print(f"Arduino yanıtı: {response}")
        
        time.sleep(self.test_duration)
        self.stop_all_motors()
        print("✅ Tüm motorlar testi tamamlandı\n")
    
    def test_all_motors_backward(self):
        """Tüm motorları geri yönde test et"""
        print("🔙 Tüm motorlar geri yönde test ediliyor...")
        command = f"MOVE:{-self.test_speed},{-self.test_speed},{-self.test_speed},{-self.test_speed}"
        response = self.send_command(command)
        
        if response:
            print(f"Arduino yanıtı: {response}")
        
        time.sleep(self.test_duration)
        self.stop_all_motors()
        print("✅ Tüm motorlar testi tamamlandı\n")
    
    def test_turn_right(self):
        """Sağa dönüş testi"""
        print("🔄 Sağa dönüş testi...")
        command = f"MOVE:{self.test_speed},{-self.test_speed},{self.test_speed},{-self.test_speed}"
        response = self.send_command(command)
        
        if response:
            print(f"Arduino yanıtı: {response}")
        
        time.sleep(self.test_duration)
        self.stop_all_motors()
        print("✅ Sağa dönüş testi tamamlandı\n")
    
    def test_turn_left(self):
        """Sola dönüş testi"""
        print("🔄 Sola dönüş testi...")
        command = f"MOVE:{-self.test_speed},{self.test_speed},{-self.test_speed},{self.test_speed}"
        response = self.send_command(command)
        
        if response:
            print(f"Arduino yanıtı: {response}")
        
        time.sleep(self.test_duration)
        self.stop_all_motors()
        print("✅ Sola dönüş testi tamamlandı\n")
    
    def interactive_test(self):
        """İnteraktif test menüsü"""
        while True:
            print("\n" + "="*50)
            print("🤖 SADIT Robot Motor Test Menüsü")
            print("="*50)
            print("1. Motor 1 (Sağ Arka) - İleri")
            print("2. Motor 2 (Sol Arka) - İleri")
            print("3. Motor 3 (Sağ Ön) - İleri")
            print("4. Motor 4 (Sol Ön) - İleri")
            print("5. Motor 1 (Sağ Arka) - Geri")
            print("6. Motor 2 (Sol Arka) - Geri")
            print("7. Motor 3 (Sağ Ön) - Geri")
            print("8. Motor 4 (Sol Ön) - Geri")
            print("9. Tüm Motorlar - İleri")
            print("10. Tüm Motorlar - Geri")
            print("11. Sağa Dönüş")
            print("12. Sola Dönüş")
            print("13. Tüm Motorları Durdur")
            print("14. Test Parametrelerini Değiştir")
            print("0. Çıkış")
            print("="*50)
            
            choice = input("Seçiminizi yapın (0-14): ").strip()
            
            if choice == '0':
                break
            elif choice == '1':
                self.test_single_motor(1, direction='forward')
            elif choice == '2':
                self.test_single_motor(2, direction='forward')
            elif choice == '3':
                self.test_single_motor(3, direction='forward')
            elif choice == '4':
                self.test_single_motor(4, direction='forward')
            elif choice == '5':
                self.test_single_motor(1, direction='backward')
            elif choice == '6':
                self.test_single_motor(2, direction='backward')
            elif choice == '7':
                self.test_single_motor(3, direction='backward')
            elif choice == '8':
                self.test_single_motor(4, direction='backward')
            elif choice == '9':
                self.test_all_motors_forward()
            elif choice == '10':
                self.test_all_motors_backward()
            elif choice == '11':
                self.test_turn_right()
            elif choice == '12':
                self.test_turn_left()
            elif choice == '13':
                self.stop_all_motors()
            elif choice == '14':
                self.change_test_parameters()
            else:
                print("❌ Geçersiz seçim!")
    
    def change_test_parameters(self):
        """Test parametrelerini değiştir"""
        print("\n🔧 Test Parametreleri:")
        print(f"Mevcut hız: {self.test_speed}")
        print(f"Mevcut süre: {self.test_duration} saniye")
        
        try:
            new_speed = input(f"Yeni hız ({self.test_speed}): ").strip()
            if new_speed:
                self.test_speed = int(new_speed)
                self.test_speed = max(0, min(140, self.test_speed))  # 0-140 arası sınırla
            
            new_duration = input(f"Yeni süre ({self.test_duration}): ").strip()
            if new_duration:
                self.test_duration = float(new_duration)
                self.test_duration = max(0.5, min(10, self.test_duration))  # 0.5-10 arası sınırla
                
            print(f"✅ Yeni parametreler: Hız={self.test_speed}, Süre={self.test_duration}")
        except ValueError:
            print("❌ Geçersiz değer!")
    
    def run_quick_test(self):
        """Hızlı test - tüm motorları sırayla test et"""
        print("⚡ Hızlı test başlıyor...")
        
        # Tüm motorları ileri yönde test et
        for i in range(1, 5):
            self.test_single_motor(i, direction='forward')
            time.sleep(1)
        
        # Tüm motorları geri yönde test et
        for i in range(1, 5):
            self.test_single_motor(i, direction='backward')
            time.sleep(1)
        
        print("✅ Hızlı test tamamlandı!")

def main():
    """Ana fonksiyon"""
    print("🤖 SADIT Robot Motor Test Programı")
    print("="*40)
    
    # Port seçimi
    default_port = '/dev/ttyUSB0'
    port = input(f"Port adresi ({default_port}): ").strip()
    if not port:
        port = default_port
    
    # Test nesnesini oluştur
    tester = SaditMotorTester(port=port)
    
    # Bağlantıyı kur
    if not tester.connect():
        print("❌ Bağlantı kurulamadı! Program sonlandırılıyor.")
        return
    
    try:
        # Test modunu seç
        print("\nTest modunu seçin:")
        print("1. İnteraktif test (menü)")
        print("2. Hızlı test (otomatik)")
        
        mode = input("Seçiminiz (1-2): ").strip()
        
        if mode == '1':
            tester.interactive_test()
        elif mode == '2':
            tester.run_quick_test()
        else:
            print("❌ Geçersiz seçim!")
            
    except KeyboardInterrupt:
        print("\n\n⚠️ Program kullanıcı tarafından durduruldu.")
    except Exception as e:
        print(f"\n❌ Beklenmeyen hata: {e}")
    finally:
        # Güvenlik için motorları durdur
        tester.stop_all_motors()
        tester.disconnect()
        print("👋 Program sonlandırıldı.")

if __name__ == "__main__":
    main()


