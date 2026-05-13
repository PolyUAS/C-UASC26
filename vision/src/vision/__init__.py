import os
import queue
import threading
import time

import cv2


rtsp_url = "rtsp://192.168.144.25:8554/main.264"
RENDER_WINDOW = os.environ.get("VISION_RENDER", "1") == "1"

frame_queue = queue.Queue(maxsize=5)


def create_capture():
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        "rtsp_transport;udp|"
        "fflags;nobuffer|"
        "flags;low_delay|"
        "probesize;32|"
        "analyzeduration;0|"
        "stimeout;2000000"
    )

    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("Failed to open stream")
        return None

    # Drop initial buffered/corrupt frames.
    for _ in range(40):
        cap.read()

    return cap


def capture_thread():
    while True:
        cap = create_capture()

        if cap is None:
            time.sleep(3)
            continue

        print("Connected to stream")

        try:
            while True:
                ret, frame = cap.read()

                if not ret or frame is None:
                    print("Read failed, reconnecting...")
                    break

                if frame_queue.full():
                    frame_queue.get_nowait()

                frame_queue.put_nowait(frame)

        except Exception as exc:
            print(f"Capture error: {exc}")

        finally:
            cap.release()
            time.sleep(1.5)


def main():
    threading.Thread(target=capture_thread, daemon=True).start()

    try:
        while True:
            try:
                frame = frame_queue.get(timeout=0.15)

                if RENDER_WINDOW:
                    cv2.imshow("A8 Mini Stream", frame)

                    key = cv2.waitKey(1) & 0xFF

                    if key == ord("q"):
                        break

                    if key == ord("r"):
                        print("Manual reconnect requested")

            except queue.Empty:
                continue

    finally:
        if RENDER_WINDOW:
            cv2.destroyAllWindows()

        print("Exited cleanly")


if __name__ == "__main__":
    main()