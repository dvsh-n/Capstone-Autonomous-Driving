import depthai as dai

def test_oakd_pro_connection():
    # Create a pipeline
    pipeline = dai.Pipeline()

    # Try to connect to the OAK-D Pro device and start the pipeline
    try:
        # Connect to the device
        with dai.Device(pipeline) as device:
            # Print device information
            print("Successfully connected to OAK-D Pro!")
            print("Device Info: ", device.getDeviceInfo())

            # If the script reaches this point, the device is successfully connected
            return True
    except Exception as e:
        print(f"Failed to connect to OAK-D Pro: {e}")
        return False

if __name__ == "__main__":
    if test_oakd_pro_connection():
        print("Connection test passed.")
    else:
        print("Connection test failed. Please check your device and connections.")
