#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

model = YOLO('yolov8s.pt')

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
  if s.get_info(rs.camera_info.name) == 'RGB Camera':
    found_rgb = True
    break
if not found_rgb:
  print("The demo requires Depth camera with Color sensor")
  exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)


# Set the depth scale
depth_scale = 0.0010000000474974513

# Create spatial and temporal filters for depth image
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()

try:
  while True:

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
      continue

    print(depth_frame.get_data())
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Apply filters to depth image
    depth_frame = spatial.process(depth_frame)
    depth_frame = temporal.process(depth_frame)
    depth_image = np.asanyarray(depth_frame.get_data())

    # Convert the depth image to meters
    depth_image = depth_image * depth_scale

    # Detect objects using YOLOv8
    results = model(color_image)

    # Process the results
    for result in results:
      boxes = result.boxes
      for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        confidence = box.conf[0].cpu().numpy()
        class_id = box.cls[0].cpu().numpy()

        if confidence < 0.5:
          continue  # Skip detections with low confidence

        # Calculate the distance to the object
        object_depth = np.median(depth_image[y1:y2, x1:x2])
        label = f"{object_depth:.2f}m"

        # Draw a rectangle around the object
        cv2.rectangle(color_image, (x1, y1), (x2, y2), (252, 119, 30), 2)

        # Draw the bounding box
        cv2.putText(color_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

        # Print the object's class and distance
        print(f"{model.names[int(class_id)]}: {object_depth:.2f}m")


    # Show the image
    cv2.imshow("Color Image", color_image)
    cv2.waitKey(1)






    # # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # depth_colormap_dim = depth_colormap.shape
    # color_colormap_dim = color_image.shape

    # # If depth and color resolutions are different, resize color image to match depth image for display
    # if depth_colormap_dim != color_colormap_dim:
    #   resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
    #   images = np.hstack((resized_color_image, depth_colormap))
    # else:
    #   images = np.hstack((color_image, depth_colormap))

    # Show images
    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', images)
    # cv2.waitKey(1)

finally:

  # Stop streaming
  pipeline.stop()