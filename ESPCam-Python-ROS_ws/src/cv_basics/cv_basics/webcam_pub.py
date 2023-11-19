
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import requests

class ImagePublisher(Node):

  def __init__(self):
    super().__init__('image_publisher')
    # Parameters for camera
    self.declare_parameter("ESPCam_URL", "http://192.168.0.102")
    self.URL = str(self.get_parameter("ESPCam_URL").value)
    self.declare_parameter("ESPCam_res", 7)
    self.res = int(self.get_parameter("ESPCam_res").value)
    self.declare_parameter("ESPCam_led", 255)
    self.led = int(self.get_parameter("ESPCam_led").value)
    self.declare_parameter("ESPCam_qlt", 4)
    self.qlt = int(self.get_parameter("ESPCam_qlt").value)
    self.declare_parameter("ESPCam_num", 1)
    self.num = int(self.get_parameter("ESPCam_num").value)

    # Publisher
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)

    # Timer
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Camera Settings
    self.set_resolution()  
    self.set_quality()
    self.set_flash()
    self.set_awb()

    # CV Settings
    self.cap = cv2.VideoCapture(self.URL + ":81/stream")
    self.br = CvBridge()
  
  # Timer_cb
  def timer_callback(self):
    ret, frame = self.cap.read()   
    if ret == True:
        self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

    self.get_logger().info('Publishing video frame')

  # Resuolutions: 10: UXGA(1600x1200) 9: SXGA(1280x1024) 8: XGA(1024x768) 7: SVGA(800x600) 6: VGA(640x480) 5: CIF(400x296) 4: QVGA(320x240) 3: HQVGA(240x176) 0: QQVGA(160x120)
  def set_resolution(self):
      requests.get(self.URL + "/control?var=framesize&val={}".format(self.res))
  # Quality: 4: Best, 63: Worst 
  def set_quality(self):
    requests.get(self.URL + "/control?var=quality&val={}".format(self.qlt))
  # Auto White Balance: 1: True, 0: False
  def set_awb(self, awb: int=1):
    requests.get(self.URL + "/control?var=awb&val={}".format(awb))
  # LED Flash: 0: Off, 255: Brightest
  def set_flash(self):
      requests.get(self.URL + "/control?var=led_intensity&val={}".format(self.led))
  
def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()