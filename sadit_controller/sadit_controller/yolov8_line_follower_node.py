import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os

class YoloV8LineFollowerNode(Node):
    def __init__(self):
        super().__init__('yolov8_line_follower_node')
        self.get_logger().info('YOLOv8 Çizgi Takip Düğümü Başlatılıyor...')

        # Model yükleme işlemini ayrı bir fonksiyonda yapıyoruz
        self.model = self.load_model()
        if self.model is None:
            self.get_logger().error('Model yüklenemedi, düğüm kapatılıyor.')
            return

        # CvBridge nesnesi
        self.bridge = CvBridge()
        
        # Kamera görüntüsü için abone oluştur
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data, 
        )

        # Robotun hız komutlarını yayınlamak için yayıncı oluştur
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # İşlenmiş görüntüyü RViz'de görmek için yayıncı oluştur
        self.processed_image_publisher = self.create_publisher(Image, 'processed_image', 10)

        self.get_logger().info('Düğüm başlatıldı ve kamera verileri bekleniyor.')

    def load_model(self):
        try:
            # Model dosyasının tam yolunu bul
            package_dir = get_package_share_directory('sadit_controller')
            model_path = os.path.join(package_dir, 'models', 'best.pt')

            # Dosyanın varlığını kontrol et
            if not os.path.exists(model_path):
                self.get_logger().error(f"Model dosyası bulunamadı: {model_path}")
                return None

            model = YOLO(model_path)
            self.get_logger().info('YOLOv8 modeli başarıyla yüklendi.')
            return model
        except Exception as e:
            self.get_logger().error(f"Model yüklenirken bir hata oluştu: {e}")
            return None

    def image_callback(self, msg):
        # Eğer model yüklenememişse, işlem yapma
        if self.model is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Hatası: {e}")
            return

        # Görüntüyü YOLOv8 için 640x640 olarak yeniden boyutlandır
        img = cv2.resize(cv_image, (640, 640))

        # Model ile tahmin yap
        results = self.model(img, verbose=False)

        twist_msg = Twist()
        line_found = False

        # Tespit sonuçlarını işle
        for r in results:
            for box in r.boxes:
                class_name = self.model.names[int(box.cls[0])]
                
                # Eğer tespit edilen nesne 'line' ise devam et
                if class_name == 'line':
                    line_found = True
                    xmin, ymin, xmax, ymax = box.xyxy[0].tolist()

                    # Tespit kutusunun merkezini bul
                    line_center_x = int((xmin + xmax) / 2)
                    line_center_y = int((ymin + ymax) / 2)
                    
                    # Merkezi görselleştir
                    cv2.circle(img, (line_center_x, line_center_y), 5, (0, 255, 0), -1)
                    
                    # Robotun orta noktasından sapmayı hesapla
                    error = line_center_x - (640 // 2)

                    # Orantılı kontrol (P-Controller) ile robot hızını ayarla
                    twist_msg.linear.x = 0.2
                    twist_msg.angular.z = -float(error) / 200.0

                    self.get_logger().info(f"Çizgi algılandı. Hata: {error}. Dönüş: {twist_msg.angular.z:.2f}")

        # Eğer çizgi bulunamazsa robotu durdur
        if not line_found:
            self.get_logger().info("Çizgi bulunamadı. Robot durduruldu.")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        # Hareket komutunu yayınla
        self.cmd_vel_publisher.publish(twist_msg)

        # İşlenmiş görüntüyü RViz için yayınla
        processed_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.processed_image_publisher.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)
    yolov8_line_follower_node = YoloV8LineFollowerNode()
    try:
        rclpy.spin(yolov8_line_follower_node)
    except KeyboardInterrupt:
        pass
    finally:
        yolov8_line_follower_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()