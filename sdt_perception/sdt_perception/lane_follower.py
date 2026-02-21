#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, 'lane_debug', 10)
        self.offset_pub = self.create_publisher(Float32, 'lane_offset', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', 
            self.image_callback, 
            10
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.0
        self.min_angular_speed = -1.0
        
        # PID Controller parameters - daha yumuşak kontrol
        self.kp = 0.5
        self.ki = 0.001 
        self.kd = 0.0005
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Lane detection parameters
        self.roi_height = 0.3  # Region of interest height (30% of image - very close to camera)
        self.roi_width = 1.0   # Region of interest width (100% of image - full width)
        
        self.get_logger().info('Lane Follower node başlatıldı')
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image for lane detection
            lane_center, debug_image, lane_detected = self.detect_lane(cv_image)
            
            if lane_detected:
                # Calculate control commands
                linear_speed, angular_speed = self.calculate_control(lane_center, cv_image.shape[1])
                
                # Publish cmd_vel
                self.publish_cmd_vel(linear_speed, angular_speed)
                
                # Publish lane offset
                image_center = cv_image.shape[1] // 2
                offset = (lane_center - image_center) / image_center
                self.offset_pub.publish(Float32(data=offset))
            else:
                # No lane detected - stop the vehicle
                self.publish_cmd_vel(0.0, 0.0)
                self.offset_pub.publish(Float32(data=0.0))
                self.get_logger().warn('Şerit algılanamadı - araç durduruldu')
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
    
    def detect_lane(self, image):
        """Siyah düz şerit algılama fonksiyonu"""
        height, width = image.shape[:2]
        
        # Region of interest - sadece görüntünün alt kısmı
        roi_y1 = int(height * 0.6)  # Görüntünün alt %40'ı
        roi_y2 = height
        roi_x1 = 0
        roi_x2 = width
        
        roi = image[roi_y1:roi_y2, roi_x1:roi_x2]
        
        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Siyah şerit için threshold - çok düşük değer
        _, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create debug image
        debug_image = image.copy()
        cv2.rectangle(debug_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 0), 2)
        cv2.line(debug_image, (width//2, roi_y1), (width//2, roi_y2), (255, 0, 0), 2)
        
        # Add threshold image to debug
        thresh_small = cv2.resize(thresh, (200, 150))
        thresh_color = cv2.cvtColor(thresh_small, cv2.COLOR_GRAY2BGR)
        debug_image[10:160, width-210:width-10] = thresh_color
        cv2.putText(debug_image, 'Threshold', (width-200, 170), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if contours:
            # Şerit benzeri konturları filtrele
            valid_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 200:  # Minimum alan
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Aspect ratio kontrolü - şerit uzun olmalı
                    aspect_ratio = w / h if h > 0 else 0
                    
                    # Şerit benzeri şekil kontrolü
                    if aspect_ratio > 2.0 and aspect_ratio < 15.0:  # Uzun ve ince
                        # Solidity kontrolü - şerit dolu olmalı
                        hull = cv2.convexHull(contour)
                        hull_area = cv2.contourArea(hull)
                        solidity = float(area) / hull_area if hull_area > 0 else 0
                        
                        if solidity > 0.7:  # %70'den fazla dolu
                            # Extent kontrolü - şerit dikdörtgen benzeri olmalı
                            rect_area = w * h
                            extent = float(area) / rect_area if rect_area > 0 else 0
                            
                            if extent > 0.5:  # %50'den fazla dikdörtgen benzeri
                                valid_contours.append(contour)
            
            if valid_contours:
                # En büyük geçerli konturu seç
                largest_contour = max(valid_contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                # Kontur merkezini hesapla
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Orijinal görüntü koordinatlarına çevir
                    lane_center_x = cx + roi_x1
                    
                    # Debug görüntüsüne çiz
                    cv2.circle(debug_image, (lane_center_x, roi_y1 + cy), 10, (0, 0, 255), -1)
                    cv2.line(debug_image, (lane_center_x, roi_y1), (lane_center_x, roi_y2), (0, 255, 0), 2)
                    cv2.drawContours(debug_image, [largest_contour], -1, (255, 255, 0), 2, offset=(roi_x1, roi_y1))
                    
                    # Bilgi ekle
                    cv2.putText(debug_image, f'Lane Center: {lane_center_x}', (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(debug_image, f'Image Center: {width//2}', (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(debug_image, f'Lane Detected: YES', (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(debug_image, f'Area: {area:.0f}', (10, 120), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(debug_image, f'Total Contours: {len(contours)}', (10, 150), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(debug_image, f'Valid Contours: {len(valid_contours)}', (10, 180), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(debug_image, f'Error: {lane_center_x - width//2}', (10, 210), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    return lane_center_x, debug_image, True
        
        # Şerit algılanamadı
        cv2.putText(debug_image, f'Lane Center: {width//2}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_image, f'Image Center: {width//2}', (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_image, f'Lane Detected: NO', (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(debug_image, f'Total Contours: {len(contours)}', (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_image, f'Valid Contours: 0', (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return width // 2, debug_image, False
    
    def calculate_control(self, lane_center, image_width):
        """PID kontrolcü ile hız hesaplama"""
        image_center = image_width // 2
        error = lane_center - image_center
        
        # Normalize error to -1 to 1 range
        normalized_error = error / (image_width / 2)
        
        # PID control
        self.integral += normalized_error
        derivative = normalized_error - self.prev_error
        
        # Anti-windup for integral
        self.integral = np.clip(self.integral, -1, 1)
        
        # Calculate angular speed
        angular_speed = self.kp * normalized_error + self.ki * self.integral + self.kd * derivative
        
        # Normalize angular speed
        angular_speed = np.clip(angular_speed, self.min_angular_speed, self.max_angular_speed)
        
        # Calculate linear speed - daha yumuşak geçiş
        # Küçük dönüşlerde tam hız, büyük dönüşlerde yavaşla
        turn_factor = abs(angular_speed) / self.max_angular_speed
        linear_speed = self.max_linear_speed * (1 - turn_factor * 0.7)  # Maksimum %70 yavaşlama
        linear_speed = max(0.2, linear_speed)  # Minimum hız 0.2 m/s
        
        self.prev_error = normalized_error
        
        return linear_speed, angular_speed
    
    def publish_cmd_vel(self, linear_speed, angular_speed):
        """cmd_vel mesajını yayınla"""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = angular_speed
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info(f'Linear: {linear_speed:.2f}, Angular: {angular_speed:.2f}')

def main(args=None):
    rclpy.init(args=args)
    lane_follower = LaneFollower()
    
    try:
        rclpy.spin(lane_follower)
    except KeyboardInterrupt:
        pass
    finally:
        lane_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
