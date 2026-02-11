"""
Simple webcam capture with OpenCV
"""

import os
import sys
import time
from pathlib import Path
from time import sleep
from typing import Any

import cv2
import flatbuffers
import zenoh
from numpy import uint32
from numpy.typing import NDArray

import vision.Detection as Detection
import vision.Frame as Frame


def create_detection_message(
    frame: NDArray, detections: list[dict] | None = None
) -> bytes:
    """
    Create a FlatBuffers message from detection data

    Args:
        frame: OpenCV frame (numpy array)
        detections: List of detection dicts with keys: ctr_x, ctr_y, width, height, class_id, confidence, metadata

    Returns:
        Serialized FlatBuffers message bytes
    """
    builder = flatbuffers.Builder(1024)

    # Build detection objects if provided
    detection_offsets = []
    if detections:
        for det in detections:
            # Create strings first (they need to be created before the table)
            class_id_offset = builder.CreateString(det.get("class_id", "unknown"))
            metadata_offset = builder.CreateString(det.get("metadata", ""))

            # Create Detection
            Detection.Start(builder)
            Detection.AddCtrX(builder, det.get("ctr_x", 0))
            Detection.AddCtrY(builder, det.get("ctr_y", 0))
            Detection.AddWidth(builder, det.get("width", 0))
            Detection.AddHeight(builder, det.get("height", 0))
            Detection.AddClassId(builder, class_id_offset)
            Detection.AddMetadata(builder, metadata_offset)
            Detection.AddConfidence(builder, det.get("confidence", 0.0))
            detection_offsets.append(Detection.End(builder))

    # Create vector of detections
    Frame.StartDetectionsVector(builder, len(detection_offsets))
    for det_offset in reversed(detection_offsets):  # FlatBuffers builds in reverse
        builder.PrependUOffsetTRelative(det_offset)
    detections_vector = builder.EndVector()

    # Create frame metadata string
    frame_metadata_offset = builder.CreateString("")

    # Create Frame
    Frame.Start(builder)
    Frame.AddWidth(builder, frame.shape[1])
    Frame.AddHeight(builder, frame.shape[0])
    Frame.AddDetections(builder, detections_vector)
    Frame.AddMetadata(builder, frame_metadata_offset)
    frame_offset = Frame.End(builder)

    # Finish the buffer
    builder.Finish(frame_offset)

    # Return serialized bytes
    return bytes(builder.Output())


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

    pub = session.declare_publisher("vision/detections")  # or whatever

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
