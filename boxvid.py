import cv2
from ultralytics import YOLO

# Load model
model = YOLO('yolov8s.pt')

# Run inference on video
results = model.predict(
    source="demo.mov",
    conf=0.4,
    save=True,  # Save the results
    project="runs/detect",  # Save results to runs/detect
    name="demo_results"  # Name of the output folder
)

print("Video processing complete! Check the runs/detect/demo_results folder for the output video.")