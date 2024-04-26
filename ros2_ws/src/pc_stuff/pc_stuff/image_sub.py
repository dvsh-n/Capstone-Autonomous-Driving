import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import requests
import threading  

class ImageSub(Node):
    def __init__(self):
        super().__init__('image_sub')
        self.sub_NN_out= self.create_subscription(CompressedImage, '/oak_d_pro/spatial_NN_out/compressed', self.NN_out_callback, 10)
        self.sub_depth = self.create_subscription(CompressedImage, '/oak_d_pro/depth/compressed', self.depth_callback, 10)
        self.br = CvBridge()

        self.declare_parameter("ESPCam_URL", "http://192.168.0.102")
        self.URL = str(self.get_parameter("ESPCam_URL").value)
        self.declare_parameter("ESPCam_res", 8)
        self.res = int(self.get_parameter("ESPCam_res").value)
        self.declare_parameter("ESPCam_led", 0)
        self.led = int(self.get_parameter("ESPCam_led").value)
        self.declare_parameter("ESPCam_qlt", 4)
        self.qlt = int(self.get_parameter("ESPCam_qlt").value)
        self.declare_parameter("ESPCam_num", 1)
        self.num = int(self.get_parameter("ESPCam_num").value)

        self.cap = cv2.VideoCapture(self.URL + ":81/stream")

        self.espcam = threading.Thread(target=self.espcam_cb)
        self.espcam.start()

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

    def espcam_cb(self): 
        while True:
            if self.cap.isOpened():
                ret, frame = self.cap.read()

                frame = cv2.resize(frame, (640, 400))

                cv2.imshow("frame", frame)

                key = cv2.waitKey(1)

                if key == 'q':
                    break

        cv2.destroyAllWindows()
        self.cap.release()

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
