#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Float64MultiArray
import cv2
import depthai as dai
import numpy as np

class Depth_Frame_Pub(Node):

    def __init__(self):
        super().init('depth_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'OAK-D-Pro/depth', 10)
        self.pipeline, self.depth  = self.create_pipeline()

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
        xout = pipeline.create(dai.node.XLinkOut)

        xout.setStreamName("disparity")

        # Properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")

        # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        depth.setLeftRightCheck(lr_check)
        depth.setExtendedDisparity(extended_disparity)
        depth.setSubpixel(subpixel)

        # Linking
        monoLeft.out.link(depth.left)
        monoRight.out.link(depth.right)
        depth.disparity.link(xout.input)
    
        return pipeline, depth

    def run(self):
        frame_msg = Float64MultiArray()
        # Connect to device and start pipeline
        device = dai.Device(self.pipeline)

        # Output queue will be used to get the disparity frames from the outputs defined above
        q = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

        # IR LED Illumination and IR Dot Projector
        device.setIrLaserDotProjectorIntensity(1)
        device.setIrFloodLightIntensity(1)

        while rclpy.ok():
            inDisparity = q.get()  # blocking call, will wait until a new data has arrived
            frame = inDisparity.getFrame()
            frame_msg.data = np.array(frame).flatten().astype(np.float64)
            self.publisher.publish(frame_msg)
            self.get_logger().info('Publishing Frame')
            
def main(args=None):
    rclpy.init(args=args)
    depth_publisher = Depth_Frame_Pub()
    try:
        depth_publisher.run()
    except KeyboardInterrupt:
        pass
    finally:
        depth_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
