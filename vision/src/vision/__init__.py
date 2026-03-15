import cv2


rtsp_url = "rtsp://192.168.144.25:8554/main.264"

# Create a VideoCapture object
cap = cv2.VideoCapture(rtsp_url)

# Check if the stream opened successfully
if not cap.isOpened():
    print("Error: Could not open RTSP stream")
    exit()

while True:
    # Read a frame from the stream
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to read frame")
        break



# Release the video capture object and destroy all windows
cap.release()
