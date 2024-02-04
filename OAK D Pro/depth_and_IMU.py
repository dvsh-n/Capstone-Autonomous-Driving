#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
from datetime import timedelta

def create_pipeline():
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
    # color = pipeline.create(dai.node.ColorCamera)
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

    # color.setCamera("color")

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

    sync.out.link(xoutGrp.input)

    return pipeline, depth

# Connect to device and start pipeline

pipeline, depth = create_pipeline()
device = dai.Device(pipeline)

# Output queue will be used to get the disparity frames from the outputs defined above
groupQueue = device.getOutputQueue("xout", maxSize=10, blocking=False)

# IR LED Illumination and IR Dot Projector
device.setIrLaserDotProjectorIntensity(1)
device.setIrFloodLightIntensity(1)

while True:
    groupMessage = groupQueue.get()
    inDisparity = groupMessage["depth"]
    imuMessage = groupMessage["imu"]

    frame = inDisparity.getFrame()
    frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8) # Normalization for better visualization
    # cv2.imshow("disparity", frame) # B/W 
    frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET) # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
    cv2.imshow("disparity_color", frame)
    
    print("Device timestamp imu: " + str(imuMessage.getTimestampDevice()))
    latestRotationVector = imuMessage.packets[-1].rotationVector
    imuF = "{:.4f}"
    print(f"Quaternion: i: {imuF.format(latestRotationVector.i)} j: {imuF.format(latestRotationVector.j)} "
        f"k: {imuF.format(latestRotationVector.k)} real: {imuF.format(latestRotationVector.real)}")
    
    if cv2.waitKey(1) == ord('q'):
        break