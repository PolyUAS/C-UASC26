import os
import time

import cv2
import torch
import ultralytics

print(ultralytics.checks())

print(f"CUDA is available from torch? : {torch.cuda.is_available()}")

print(cv2.__version__)

print("CUDA devices:", cv2.cuda.getCudaEnabledDeviceCount())
print("Build info:", cv2.getBuildInformation())


# TODO look into using the below:
#
#
# def gstreamer_pipeline(sensor_id=0, capture_width=1920, capture_height=1080, display_width=960, display_height=540, framerate=30, flip_method=0):
#     return (
#         "nvarguscamerasrc sensor-id=%d ! "
#         "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
#         "nvvidconv flip-method=%d ! "
#         "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
#         "videoconvert ! "
#         "video/x-raw, format=(string)BGR ! appsink"
#         % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
#     )
#
#
# cap = cv2.VideoCapture(gstreamer_pipeline(sensor_id=0), cv2.CAP_GSTREAMER)

RENDER_WINDOW = False

rtsp_url = "rtsp://192.168.144.25:8554/main.264"
cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)

if not cap.isOpened():
    print("Error: Could not open RTSP stream")
    exit()

# allow decoder to sync with keyframe
for _ in range(30):
    cap.read()
    time.sleep(0.02)

while True:
    ret, frame = cap.read()
    # print("got cap!")

    if not ret:
        print("Dropped frame")
        continue

    if RENDER_WINDOW:
        # show frame
        cv2.imshow("A8 Mini Stream", frame)

        # press q to quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


cap.release()


if RENDER_WINDOW:
    cv2.destroyAllWindows()
