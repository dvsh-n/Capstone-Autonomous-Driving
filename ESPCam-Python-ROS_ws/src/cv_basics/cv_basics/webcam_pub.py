
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
    self.declare_parameter("ESPCam_URL", "http://192.168.0.102")
    self.URL = str(self.get_parameter("ESPCam_URL").value)
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)

    self.set_resolution(10)  
    self.set_quality(4) 
    self.set_flash(255)
    self.set_awb(1)
    self.cap = cv2.VideoCapture(self.URL + ":81/stream")
    self.br = CvBridge()
   
  def timer_callback(self):
    ret, frame = self.cap.read()   
    if ret == True:
        self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

    self.get_logger().info('Publishing video frame')

  '''Resuolutions: 10: UXGA(1600x1200) 9: SXGA(1280x1024) 8: XGA(1024x768) 7: SVGA(800x600) 6: VGA(640x480) 5: CIF(400x296) 4: QVGA(320x240) 3: HQVGA(240x176) 0: QQVGA(160x120)'''
  def set_resolution(self,  index: int=7):
      requests.get(self.URL + "/control?var=framesize&val={}".format(index))

  def set_quality(self, value: int=4):
    requests.get(self.URL + "/control?var=quality&val={}".format(value))
  
  def set_awb(self, awb: int=1):
    requests.get(self.URL + "/control?var=awb&val={}".format(awb))

  def set_flash(self, intensity: int=255):
      requests.get(self.URL + "/control?var=led_intensity&val={}".format(intensity))
  
def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()