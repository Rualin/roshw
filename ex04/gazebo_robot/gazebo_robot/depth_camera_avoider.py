import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
import numpy as np

class DepthStopper(Node):
    def __init__(self):
        super().__init__('depth_camera_avoider')

        qos_policy = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            Image,
            '/depth_camera',
            self.listener_callback,
            qos_policy)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        
        self.stop_distance = 1.0
        self.get_logger().info('Smart Depth Stopper запущен!')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            return

        height, width = cv_image.shape
        
        h_center = int(height / 2)
        w_center = int(width / 2)
        
        crop_img = cv_image[h_center-10:h_center+10, w_center-50:w_center+50]

        crop_img = np.nan_to_num(crop_img, posinf=10.0, neginf=10.0, nan=10.0)
        
        crop_img[crop_img == 0.0] = 10.0

        if crop_img.size > 0:
            min_dist = np.min(crop_img)
        else:
            min_dist = 10.0

        twist = Twist()

        if min_dist < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warning(f'СТОП! Вижу что-то в {min_dist:.2f}м')
        else:
            twist.linear.x = 0.5
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DepthStopper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()