#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from datetime import timedelta
import cv2 # OpenCV library
import depthai as dai
import numpy as np
import os
import time

class oak_d_pro(Node):

    def __init__(self):
        super().__init__('oak_d_pro')
        self.pub_depth = self.create_publisher(Image, 'oak_d_pro/depth', 10)
        self.pub_color = self.create_publisher(Image, 'oak_d_pro/spatial_NN_out', 10)
        self.pipeline, self.depth  = self.create_pipeline()
        self.labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
        self.nnBlobPath = os.getcwd() + r"\mobilenet-ssd_openvino_2021.4_6shave.blob"
        self.cv_bridge = CvBridge()

    def create_pipeline(self):
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        depth = pipeline.create(dai.node.StereoDepth)
        color = pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        sync = pipeline.create(dai.node.Sync)
    
        xoutGrp = pipeline.create(dai.node.XLinkOut)
        xoutGrp.setStreamName("xout")

        # Properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")

        color.setPreviewSize(300, 300)
        color.setCamera("color")
        color.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        color.setInterleaved(False)
        color.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        
        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY) # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7) # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        depth.setLeftRightCheck(True)
        depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        depth.setExtendedDisparity(False)
        depth.setSubpixel(True)
        depth.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

        spatialDetectionNetwork.setBlobPath(self.nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        sync.setSyncThreshold(timedelta(milliseconds=10))
        sync.setSyncAttempts(-1)

        # Linking
        monoLeft.out.link(depth.left)
        monoRight.out.link(depth.right)

        color.preview.link(spatialDetectionNetwork.input)
        depth.depth.link(spatialDetectionNetwork.inputDepth)
        
        depth.disparity.link(sync.inputs["depth"])
        color.video.link(sync.inputs["video"])
        spatialDetectionNetwork.out.link(sync.inputs["nn"])

        sync.out.link(xoutGrp.input)

        return pipeline, depth

    def run(self):
        # Connect to device and start pipeline
        pipeline, depth = self.create_pipeline()
        device = dai.Device(pipeline)

        # Output queue will be used to get the disparity frames from the outputs defined above
        groupQueue = device.getOutputQueue("xout", maxSize=10, blocking=False)

        # IR LED Illumination and IR Dot Projector
        device.setIrLaserDotProjectorIntensity(1)
        device.setIrFloodLightIntensity(1)

        startTime = time.monotonic()
        counter = 0
        fps = 0

        while rclpy.ok():
            groupMessage = groupQueue.get()
            inDepth = groupMessage["depth"]
            inColor = groupMessage["video"]
            inNN = groupMessage["nn"]

            counter+=1
            current_time = time.monotonic()
            if (current_time - startTime) > 1 :
                fps = counter / (current_time - startTime)
                counter = 0
                startTime = current_time

            frame_depth = inDepth.getFrame()
            frame_depth = (frame_depth * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8) # Normalization for better visualization
            depth_downscaled = frame_depth[::4]
            if np.all(depth_downscaled == 0):
                min_depth = 0  # Set a default minimum depth value when all elements are zero
            else:
                min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
            max_depth = np.percentile(depth_downscaled, 99)
            depthFrameColor = np.interp(frame_depth, (min_depth, max_depth), (0, 255)).astype(np.uint8)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)

            depth_msg = self.cv_bridge.cv2_to_imgmsg(depthFrameColor, "bgr8")
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "depth"
            self.pub_depth.publish(depth_msg)

            frame_color = inColor.getCvFrame()
            frame_color = cv2.resize(frame_color, (640, 400), interpolation=cv2.INTER_AREA)
            detections = inNN.detections

            # If the frame is available, draw bounding boxes on it and show the frame
            height = frame_color.shape[0]
            width  = frame_color.shape[1]
            for detection in detections:
                roiData = detection.boundingBoxMapping
                roi = roiData.roi
                roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                topLeft = roi.topLeft()
                bottomRight = roi.bottomRight()
                xmin = int(topLeft.x)
                ymin = int(topLeft.y)
                xmax = int(bottomRight.x)
                ymax = int(bottomRight.y)
                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), (255,255,255), 1)

                # Denormalize bounding box
                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)
                try:
                    label = self.labelMap[detection.label]
                except:
                    label = detection.label
                cv2.putText(frame_color, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame_color, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame_color, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame_color, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame_color, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

                cv2.rectangle(frame_color, (x1, y1), (x2, y2), (255, 0, 0), cv2.FONT_HERSHEY_SIMPLEX)

                cv2.putText(frame_color, "NN fps: {:.2f}".format(fps), (2, frame_color.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255,255,255))
                
            color_msg = self.cv_bridge.cv2_to_imgmsg(frame_color, "bgr8")
            color_msg.header.stamp = self.get_clock().now().to_msg()
            color_msg.header.frame_id = "color_frame"
            self.pub_color.publish(color_msg)

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