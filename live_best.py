import cv2
from ultralytics import YOLO

# 1. Load your trained "Gold" weights
model_path = "best.pt"
model = YOLO(model_path)

# 2. Open the FLIR Boson (Usually index 0 or 2 for UVC cameras on Ubuntu)
camera_index = 1 
cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)

if not cap.isOpened():
    print(f"Error: Could not open thermal camera at /dev/video{camera_index}.")
    print("Try changing camera_index to 1 or 2 in file.")
    exit()

print("Starting live thermal inference.")

# 3. The Real-Time Inference Loop
while cap.isOpened():
    success, frame = cap.read()
    if success:
        # Run YOLO inference (stream=True keeps memory usage efficient)
        results = model(frame, stream=True)

        # Draw the bounding boxes on the frame
        for r in results:
            annotated_frame = r.plot()
            cv2.imshow("Husky Thermal Vision Test", annotated_frame)

        # Press 'q' to quit the window
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        print("Dropped frame from camera...")
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
