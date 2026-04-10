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


rtsp_url = "rtsp://192.168.144.25:8554/main.264"

RENDER_WINDOW = False

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
