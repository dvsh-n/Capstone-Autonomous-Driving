#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from datetime import timedelta
import cv2 # OpenCV library
import depthai as dai
import numpy as np

class oak_d_pro(Node):

    def __init__(self):
        super().__init__('oak_d_pro')
        self.pub_depth = self.create_publisher(Image, 'oak_d_pro/depth', 10)
        self.pub_color = self.create_publisher(Image, 'oak_d_pro/color', 10)
        self.pub_quat = self.create_publisher(Quaternion, 'oak_d_pro/imu/quaternion', 10)
        self.pipeline, self.depth  = self.create_pipeline()
        self.cv_bridge = CvBridge()

    def create_pipeline(self):
        # Closer-in minimum depth, disparity range is doubled (from 95 to 190):
        extended_disparity = False
        # Better accuracy for longer distance, fractional disparity 32-levels:
        subpixel = False
        # Better handling for occlusions:
        lr_check = True

        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        depth = pipeline.create(dai.node.StereoDepth)
        color = pipeline.create(dai.node.ColorCamera)
        imu = pipeline.create(dai.node.IMU)
        sync = pipeline.create(dai.node.Sync)
        xoutImu = pipeline.create(dai.node.XLinkOut)
        xoutImu.setStreamName("imu")

        xoutGrp = pipeline.create(dai.node.XLinkOut)
        xoutGrp.setStreamName("xout")

        # Properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")

        color.setCamera("color")
        color.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 120)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        
        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY) # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7) # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        depth.setLeftRightCheck(lr_check)
        depth.setExtendedDisparity(extended_disparity)
        depth.setSubpixel(subpixel)

        sync.setSyncThreshold(timedelta(milliseconds=10))
        sync.setSyncAttempts(-1)

        # Linking
        monoLeft.out.link(depth.left)
        monoRight.out.link(depth.right)

        depth.disparity.link(sync.inputs["depth"])
        imu.out.link(sync.inputs["imu"])
        color.video.link(sync.inputs["video"])

        sync.out.link(xoutGrp.input)
    
        return pipeline, depth

    def run(self):
        # Connect to device and start pipeline
        device = dai.Device(self.pipeline)

        # Output queue will be used to get the disparity frames from the outputs defined above
        groupQueue = device.getOutputQueue("xout", maxSize=10, blocking=False)

        # IR LED Illumination and IR Dot Projector
        device.setIrLaserDotProjectorIntensity(1)
        device.setIrFloodLightIntensity(1)

        while rclpy.ok():
            groupMessage = groupQueue.get()
            inDisparity = groupMessage["depth"]
            imuMessage = groupMessage["imu"]
            inCamera = groupMessage["video"]

            frame_depth = inDisparity.getFrame()
            frame_depth = (frame_depth * (255 / self.depth.initialConfig.getMaxDisparity())).astype(np.uint8) # Normalization for better visualization
            frame_depth = cv2.applyColorMap(frame_depth, cv2.COLORMAP_JET)
            depth_msg = self.cv_bridge.cv2_to_imgmsg(frame_depth, "bgr8")
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "camera_link_optical"
            self.pub_depth.publish(depth_msg)

            frame_color = inCamera.getCvFrame()
            color_msg = self.cv_bridge.cv2_to_imgmsg(frame_color, "bgr8")
            color_msg.header.stamp = self.get_clock().now().to_msg()
            color_msg.header.frame_id = "camera_link_optical"
            self.pub_color.publish(color_msg)

            RotationVector = imuMessage.packets[-1].rotationVector
            quat_msg = Quaternion(x=RotationVector.i, y=RotationVector.j, z=RotationVector.k, w=RotationVector.real)
            self.pub_quat.publish(quat_msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = oak_d_pro()
    try:
        publisher.run()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()