"""
Simple webcam capture with OpenCV
"""

import sys

import cv2
import zenoh


def main():
    """Open webcam and display feed"""
    print("Opening webcam...")

    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    # Open the default camera (0)
    cap = cv2.VideoCapture(0)

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open webcam")
        sys.exit(1)

    print("Webcam opened successfully!")
    # print("Press 'q' to quit")
    print("Press 's' to save a snapshot")

    snapshot_count = 0

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Can't receive frame. Exiting...")
            break

        # Display the frame
        cv2.imshow("Webcam Feed", frame)

        # Wait for key press (1ms)
        try:
            key = cv2.waitKey(1) & 0xFF
        except KeyboardInterrupt:
            print("\nexiting vision...")
            sys.exit(0)
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

    # Release everything
    cap.release()
    cv2.destroyAllWindows()
    print("Webcam closed")


if __name__ == "__main__":
    main()
