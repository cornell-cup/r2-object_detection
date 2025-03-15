import cv2
from ultralytics import YOLO
import os

def load_model(weights_path='yolov8s.pt'):
    """
    Load a YOLO model
    Args:
        weights_path: Path to weights (defaults to pretrained YOLOv8s)
    Returns:
        Loaded model
    """
    model = YOLO(weights_path)
    print(f"Loaded model from: {weights_path}")
    return model

def get_colors(cls_num):
    """Generate distinct colors for each class"""
    base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), 
                   (255, 255, 0), (255, 0, 255), (0, 255, 255),
                   (128, 0, 0), (0, 128, 0), (0, 0, 128),
                   (128, 128, 0), (128, 0, 128), (0, 128, 128)]
    return base_colors[cls_num % len(base_colors)]

def process_folder_images(model, folder_path, conf_threshold=0.4):
    """
    Process all images in a folder and return classifications
    Args:
        model: YOLO model
        folder_path: Path to folder containing images
        conf_threshold: Confidence threshold for detection
    Returns:
        Dictionary of filename: classifications
    """
    results_dict = {}
    valid_extensions = ('.jpg', '.jpeg', '.png', '.bmp')
    
    # Check if folder exists
    if not os.path.exists(folder_path):
        raise FileNotFoundError(f"Folder not found: {folder_path}")
    
    # Process each image in the folder
    for filename in os.listdir(folder_path):
        if filename.lower().endswith(valid_extensions):
            image_path = os.path.join(folder_path, filename)
            image = cv2.imread(image_path)
            
            if image is None:
                print(f"Could not read image: {filename}")
                continue
                
            # Run detection
            results = model(image, conf=conf_threshold)
            
            # Extract classifications
            classifications = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = result.names[cls]
                    classifications.append(f"{class_name} ({conf:.2f})")
            
            results_dict[filename] = classifications
            print(f"Processed {filename}: {classifications}")
    
    return results_dict

def run_detection(model, conf_threshold=0.4, folder_path=""):
    """
    Run detection either on real-time camera feed or on images in a folder
    Args:
        model: YOLO model
        conf_threshold: Confidence threshold for detection
        folder_path: Path to folder containing images (if empty, uses camera)
    """
    # If folder path is provided, process images in folder
    if folder_path:
        results = process_folder_images(model, folder_path, conf_threshold)
        print("\nClassification Results:")
        for filename, classifications in results.items():
            print(f"{filename}: {classifications}")
        return results
    
    # Otherwise, run real-time detection with camera
    video_cap = cv2.VideoCapture(0)
    
    try:
        while True:
            ret, frame = video_cap.read()
            if not ret:
                continue

            # Run detection with tracking
            results = model.track(frame, stream=True, conf=conf_threshold)

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Get class and confidence
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # Get class name
                    class_name = result.names[cls]
                    
                    # Draw box and label
                    color = get_colors(cls)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    label = f'{class_name} {conf:.2f}'
                    cv2.putText(frame, label, (x1, y1 - 10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            cv2.imshow('Object Detection', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        video_cap.release()
        cv2.destroyAllWindows()

def main():
    # Load the pretrained model
    model = load_model()
    
    # Example usage:
    # For camera detection:
    run_detection(model)
    
    # For folder processing:
    # run_detection(model, folder_path="path/to/your/images")

if __name__ == "__main__":
    main()