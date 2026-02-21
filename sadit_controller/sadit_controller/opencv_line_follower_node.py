# This node processes camera images to follow a line and publishes both
# movement commands and a processed image for visualization.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from simple_pid import PID

class OpenCVLineFollowerNode(Node):
    """
    A ROS 2 node that combines line-following logic with a processed image output.
    It subscribes to an image topic, detects a black line, publishes a Twist message
    for robot control, and publishes the annotated image for visualization.
    """
    def __init__(self):
        super().__init__('opencv_line_follower_node')
        self.get_logger().info('OpenCV Line Follower Node Started.')

        # Initialize CvBridge for converting ROS Image messages to OpenCV images
        self.bridge = CvBridge()
        
        # Declare parameters for control and visualization
        self.declare_parameter('setpoint', 320)
        self.declare_parameter('forward_speed', 0.1)
        self.declare_parameter('max_angular_speed', 1.0)
        self.setpoint = self.get_parameter('setpoint').get_parameter_value().integer_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        # Initialize the PID controller
        self.pid = PID(0.5, 0.2, 0.01, setpoint=self.setpoint, sample_time=0.01, output_limits=(-80, 80))

        # Create a subscriber for the raw camera image
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Subscribing to /camera/camera/color/image_raw')

        # Create a publisher to send Twist commands to the robot
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Publishing to /cmd_vel')

        # Create a publisher for the processed image
        self.processed_image_publisher = self.create_publisher(Image, 'processed_image', 10)
        self.get_logger().info('Publishing to /processed_image')

    def image_callback(self, msg):
        """
        Callback function that processes the image, calculates a Twist command,
        and publishes both the command and the annotated image.
        """
        try:
            # Convert ROS Image message to OpenCV BGR format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Resize and select the Region of Interest (ROI)
        frame = cv2.resize(cv_image, (640, 480))
        height, width, _ = frame.shape
        # Use the bottom part of the frame to look for the line
        roi = frame[int(height * 0.7):height, 0:width]

        # Convert ROI to HSV and apply a mask to find the black line
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([179, 255, 60])
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # Apply morphological operations to clean up the mask
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Calculate moments to find the centroid of the line
        M = cv2.moments(mask)
        
        cx = width // 2  # Default to center if no line is found
        direction = "Line Lost!"

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Draw the centroid on the original frame
            cv2.circle(frame, (cx, int(height * 0.7) + cy), 5, (0, 0, 255), -1)
            
            # Calculate the error (deviation from the center)
            error = cx - width // 2
            
            # Get the PID control output
            control_output = self.pid(cx)

            # Determine direction based on error
            if error < -40:
                direction = "Turn Left"
            elif error > 40:
                direction = "Turn Right"
            else:
                direction = "Go Straight"

            # Create and publish the Twist message
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = float(self.forward_speed)
            # Map PID output to a realistic angular velocity range
            cmd_vel_msg.angular.z = float(control_output / 80.0) * self.max_angular_speed
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            
            self.get_logger().info(f"Published cmd_vel: linear.x={cmd_vel_msg.linear.x:.2f}, angular.z={cmd_vel_msg.angular.z:.2f}")
            
        else:
            # If no line is found, publish a zero Twist command to stop the robot
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Add text to the annotated frame
        cv2.putText(frame, direction, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        self.get_logger().info(f"Direction: {direction}")
        
        # Convert the annotated OpenCV image to a ROS Image message and publish it
        processed_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.processed_image_publisher.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVLineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
