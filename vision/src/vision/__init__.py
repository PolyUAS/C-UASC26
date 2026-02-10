"""
Simple webcam capture with OpenCV
"""

import sys

import cv2


def main():
    """Open webcam and display feed"""
    print("Opening webcam...")

    # Open the default camera (0)
    cap = cv2.VideoCapture(0)

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open webcam")
        sys.exit(1)

    print("Webcam opened successfully!")
    print("Press 'q' to quit")
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
        key = cv2.waitKey(1) & 0xFF

        # Press 'q' to quit
        if key == ord("q"):
            print("Quitting...")
            break

        # Press 's' to save snapshot
        elif key == ord("s"):
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
