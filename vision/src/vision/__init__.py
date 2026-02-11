"""
Simple webcam capture with OpenCV
"""

import os
import sys
import time
from pathlib import Path
from time import sleep
from typing import Any

import capnp
import cv2
import zenoh
from numpy.typing import NDArray

# Load schema
SCHEMA_PATH = Path(__file__).parent / "detections.capnp"
schema: Any = capnp.load(str(SCHEMA_PATH))  # type: ignore


def create_detection_message(
    frame: NDArray, detections: list[dict] | None = None
) -> bytes:
    """
    Create a Cap'n Proto message from detection data

    Args:
        frame: OpenCV frame (numpy array)
        detections: List of detection objects (optional)

    Returns:
        Serialized Cap'n Proto message bytes
    """
    # Create a new DetectionFrame message
    msg = schema.Frame.new_message()

    # Set timestamp (nanoseconds since epoch)

    # Set frame metadata
    msg.width = frame.shape[1]
    msg.height = frame.shape[0]

    # If you have detections, add them
    if detections:
        msg.init("detections", len(detections))

        for i, det in enumerate(detections):
            msg.detections[i].x = det["x"]
            msg.detections[i].y = det["y"]
            msg.detections[i].width = det["width"]
            msg.detections[i].height = det["height"]
            msg.detections[i].confidence = det["confidence"]
            msg.detections[i].classId = det["class_id"]
            if "label" in det:
                msg.detections[i].label = det["label"]

    # Serialize to bytes
    print(msg)
    return msg.to_bytes()


def main():
    """Open webcam and display feed"""
    print("Opening webcam...")

    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    # Open the default camera (0)
    cap = cv2.VideoCapture(0)

    # cap = cv2.VideoCapture("rtsp://192.168.144.25:8554/main.264")

    pub = session.declare_publisher("vision/detections/targets")  # or whatever

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open webcam")
        sys.exit(1)

    sleep(0.5)  # wait for everything to open

    print("Webcam opened successfully!")
    # print("Press 'q' to quit")
    print("Press 's' to save a snapshot")

    snapshot_count = 0

    while True:
        try:
            # Capture frame-by-frame
            ret, frame = cap.read()

            if not ret:
                print("Error: Can't receive frame. Exiting...")
                break

            # Display the frame
            cv2.imshow("Webcam Feed", frame)

            print(create_detection_message(frame))

            # Wait for key press (1ms)

            key = cv2.waitKey(1) & 0xFF

            # Press 'q' to quit
            # if key == ord("q"):
            #     print("Quitting...")
            #     break

            # Press 's' to save snapshot
            if key == ord("s"):
                snapshot_count += 1
                filename = f"snapshot_{snapshot_count}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Saved {filename}")
        except KeyboardInterrupt:
            print("\nexiting vision...")
            break

    # Release everything
    cap.release()
    session.close()
    cv2.destroyAllWindows()
    print("Webcam closed")


if __name__ == "__main__":
    main()
