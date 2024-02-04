import depthai as dai
import numpy as np

# Create a pipeline
pipeline = dai.Pipeline()

# Create a depth node
depth_node = pipeline.createDepth()
depth_node.setConfidenceThreshold(200)

# Create an XLink output
xout = pipeline.createXLinkOut()
xout.setStreamName("depth")
depth_node.depth.link(xout.input)

# Connect to the device and start the pipeline
with dai.Device(pipeline) as device:

    # Output queue will be used to get the depth frames from the output defined above
    q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    while True:
        in_depth = q_depth.tryGet()  # Retrieve a depth frame
        
        if in_depth is not None:
            # Get the depth frame as a NumPy array
            frame = in_depth.getFrame()
            # The depth frame is in fixed-point representation, converting it to meters
            depth_meters = frame.astype(np.float32) / 1000.0  # Assuming the depth is in millimeters
            print(depth_meters)
            # Use `depth_meters` NumPy array for further processing
            print("Depth frame in meters:", depth_meters)

            # Add your code here to process the depth data

            # Exit loop after processing one frame for demonstration
            break