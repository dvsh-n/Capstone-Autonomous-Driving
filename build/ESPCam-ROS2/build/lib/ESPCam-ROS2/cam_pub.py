import rclpy
from rclpy.node import Node
import cv2
import requests

URL = "http://192.168.0.102"
# AWB = True

cap = cv2.VideoCapture(URL + ":81/stream")

# '''Resuolutions: 10: UXGA(1600x1200) 9: SXGA(1280x1024) 8: XGA(1024x768) 7: SVGA(800x600) 6: VGA(640x480) 5: CIF(400x296) 4: QVGA(320x240) 3: HQVGA(240x176) 0: QQVGA(160x120)'''
# def set_resolution(url: str, index: int=1):
#     requests.get(url + "/control?var=framesize&val={}".format(index))

# '''4-63'''
# def set_quality(url: str, value: int=4):
#     requests.get(url + "/control?var=quality&val={}".format(value))


# def set_awb(url: str, awb: int=1):
#     requests.get(url + "/control?var=awb&val={}".format(awb))

# def set_flash(url: str, intensity: int=255):
#     requests.get(url + "/control?var=led_intensity&val={}".format(intensity))

# ## Camera Steup
# set_resolution(URL, 7)
# set_quality(URL, 4)
# set_awb(URL, 1)
# set_flash(URL, 255)

class ESPCam_Pub(Node): 
    def __init__(self):
        super().__init__("ESPCam Publisher")


def main(args=None):
    rclpy.init(args=args)
    node = ESPCam_Pub() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
