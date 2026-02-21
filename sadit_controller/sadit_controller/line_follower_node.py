import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.get_logger().info('Line Follower Node Başlatıldı')

        # Görüntü mesajlarını dönüştürmek için CvBridge nesnesi oluşturun
        self.bridge = CvBridge()
        
        # Kamera görüntüsünü dinlemek için abone (subscriber) oluşturun
        # Topic adının kameranızın yayınladığı isimle eşleştiğinden emin olun
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # İşlenmiş görüntüyü RViz'de göstermek için yayıncı (publisher) oluşturun
        self.processed_image_publisher = self.create_publisher(Image, 'processed_image', 10)

    def image_callback(self, msg):
        try:
            # ROS Image mesajını OpenCV formatına (bgr8) dönüştürün
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Hatası: {e}")
            return

        # Görüntü boyutlandırma ve ROI (İlgi Bölgesi) seçimi
        frame = cv2.resize(cv_image, (640, 480))
        height, width, _ = frame.shape
        # Çizgiyi aramak için çerçevenin alt kısmını kullanın
        roi = frame[int(height * 0.7):height, 0:width]

        # ROI'yi HSV renk uzayına dönüştürün
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Siyah çizgi için HSV eşikleme değerlerini belirleyin
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([179, 255, 60])
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # Gürültüyü azaltmak ve boşlukları doldurmak için morfolojik işlemler uygulayın
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Çizginin merkezini (centroid) bulmak için momentleri hesaplayın
        M = cv2.moments(mask)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Bulunan merkezi ana kare üzerinde kırmızı bir daire ile işaretleyin
            # 'cy' ROI'ye göre olduğu için, 'int(height * 0.7)' ekleyerek tam çerçevedeki doğru konumunu bulun
            cv2.circle(frame, (cx, int(height * 0.7) + cy), 5, (0, 0, 255), -1)
            
            # Hata değerini (merkezin orta hattan sapması) hesaplayın
            error = cx - width // 2
            
            # Hata değerine göre yönü belirleyin
            if error < -40:
                direction = "Sola Don"
            elif error > 40:
                direction = "Saga Don"
            else:
                direction = "Düz Git"

            # Yön bilgisini ana kareye yazın
            cv2.putText(frame, direction, (50, 50),
                                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            self.get_logger().info(f"Yön: {direction}")
            
        else:
            # Çizgi bulunamazsa uyarı yazısı gösterin
            direction = "Cizgi Kayip!"
            cv2.putText(frame, direction, (50, 50),
                                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            self.get_logger().info(direction)
        
        # Son olarak, işlenmiş görüntüyü ROS 2 mesajına dönüştürün ve yayınlayın
        processed_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.processed_image_publisher.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    line_follower_node = LineFollowerNode()
    rclpy.spin(line_follower_node)
    line_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()