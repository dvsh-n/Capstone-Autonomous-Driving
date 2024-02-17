import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml

class cam_info_pub(Node):

    def __init__(self):
        super().__init__('camera_info_pub')
        self.publisher = self.create_publisher(CameraInfo, 'oak_d_pro/camera_info', 10)
        self.timer = self.create_timer(1.0, self.publish_camera_info)
        self.camera_info = self.load_camera_info('/home/ubuntu/Capstone-Autonomous-Driving/ros2_ws/src/oak_d_pro/config/rgbcam_config.yaml')

    def load_camera_info(self, filename):
        with open(filename, 'r') as file:
            camera_info_dict = yaml.safe_load(file)
        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = camera_info_dict['frame_id']
        camera_info_msg.height = camera_info_dict['image_height']
        camera_info_msg.width = camera_info_dict['image_width']
        camera_info_msg.distortion_model = camera_info_dict['distortion_model']
        camera_info_msg.d = camera_info_dict['d']
        camera_info_msg.k = camera_info_dict['k']
        camera_info_msg.r = camera_info_dict['r']
        camera_info_msg.p = camera_info_dict['p']
        camera_info_msg.binning_x = camera_info_dict.get('binning_x', 0)
        camera_info_msg.binning_y = camera_info_dict.get('binning_y', 0)
        camera_info_msg.roi.x_offset = camera_info_dict['roi']['x_offset']
        camera_info_msg.roi.y_offset = camera_info_dict['roi']['y_offset']
        camera_info_msg.roi.height = camera_info_dict['roi']['height']
        camera_info_msg.roi.width = camera_info_dict['roi']['width']
        camera_info_msg.roi.do_rectify = camera_info_dict['roi']['do_rectify']
        return camera_info_msg

    def publish_camera_info(self):
        self.camera_info.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = cam_info_pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
