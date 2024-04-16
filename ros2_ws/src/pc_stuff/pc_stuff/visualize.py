import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('visualize')
        self.subscription = self.create_subscription(CompressedImage, '/oak_d_pro/depth/compressed', self.image_callback, 10)
        self.br = CvBridge()

    def image_callback(self, msg):
        cv_image = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
