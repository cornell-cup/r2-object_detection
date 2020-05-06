import cv2
video_capture=cv2.VideoCapture(0)
if not video_capture.isOpened():
    raise Exception("could not open")
ret, frame =video_capture.read()
video_capture.release()
