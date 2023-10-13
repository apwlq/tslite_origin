import pyrealsense2 as rs

# Configure depth stream
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the RealSense pipeline
pipeline = rs.pipeline()
pipeline.start(config)

try:
    while True:
        # Wait for a new frame
        frames = pipeline.wait_for_frames()

        # Get the depth frame
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        # Convert the depth frame to a numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Now you can work with the depth image data
        # For example, you can access the depth value at pixel (x, y)
        x, y = 320, 240  # Example pixel coordinates
        depth_value = depth_image[y, x]
        print(f"Depth value at ({x}, {y}): {depth_value} millimeters")

finally:
    pipeline.stop()
