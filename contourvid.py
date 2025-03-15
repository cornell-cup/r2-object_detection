import cv2
from ultralytics import YOLO

# Load the YOLO segmentation model
# Note: Using yolov8s-seg.pt instead of yolov8s.pt for segmentation
model = YOLO('yolov8s-seg.pt')

# Run inference on video with segmentation
results = model.predict(
    source="demo.mov",
    conf=0.4,
    save=True,  # Save the results
    project="runs/segment",  # Changed from detect to segment
    name="demo_results",  # Name of the output folder
    mode='segment'  # Specify segmentation mode
)

print("Video segmentation complete! Check the runs/segment/demo_results folder for the output video.")

# Optional: If you want to process the segmentation masks in real-time
# for result in results:
#     masks = result.masks  # Get segmentation masks
#     boxes = result.boxes  # Get bounding boxes
#     classes = result.boxes.cls  # Get class IDs
#     conf = result.boxes.conf  # Get confidence scores