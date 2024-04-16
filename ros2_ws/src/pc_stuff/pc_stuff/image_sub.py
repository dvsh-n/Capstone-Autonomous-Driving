import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageSub(Node):
    def __init__(self):
        super().__init__('image_sub')
        self.sub_NN_out= self.create_subscription(CompressedImage, '/oak_d_pro/spatial_NN_out/compressed', self.NN_out_callback, 10)
        self.sub_depth = self.create_subscription(CompressedImage, '/oak_d_pro/depth/compressed', self.depth_callback, 10)
        self.br = CvBridge()

    def depth_callback(self, msg):
        depth_img = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Depth", depth_img)
        cv2.waitKey(1)

    def NN_out_callback(self, msg):
        NN_out_img = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Mobilenet", NN_out_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSub()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
