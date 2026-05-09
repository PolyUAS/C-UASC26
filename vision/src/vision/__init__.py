import os
import queue
import sys
import threading
import time

import cv2

print(hasattr(cv2, "cuda"))
print(cv2.cuda.getCudaEnabledDeviceCount() if hasattr(cv2, "cuda") else "no cv2.cuda")

sys.exit(1)

rtsp_url = "rtsp://192.168.144.25:8554/main.264"
RENDER_WINDOW = False

# Recommended FFMPEG options for low-latency RTSP
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;udp|"  # try 'tcp' if udp has packet loss
    "fflags;nobuffer|"
    "flags;low_delay|"
    "probesize;32|"
    "analyzeduration;0|"
    "stimeout;2000000"  # 2 seconds
)


def create_capture():
    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("→ Failed to open RTSP stream")
        return None
    # Throw away initial frames until we get a clean keyframe
    for _ in range(40):
        cap.read()
    return cap


frame_queue = queue.Queue(maxsize=5)  # small queue → drops old frames


def capture_thread():
    while True:
        cap = create_capture()
        if cap is None:
            time.sleep(3)
            continue

        print("→ Connected to stream")
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("→ Read failed → reconnecting...")
                    break

                try:
                    if frame_queue.full():
                        frame_queue.get_nowait()  # drop oldest
                    frame_queue.put_nowait(frame)
                except queue.Full:
                    pass  # should not happen with get_nowait + put_nowait combo

        except Exception as e:
            print(f"Capture error: {e}")
        finally:
            cap.release()
            time.sleep(1.5)


# Start background capture
threading.Thread(target=capture_thread, daemon=True).start()

# ────────────────────────────────────────────────
# Main display loop
# ────────────────────────────────────────────────

try:
    while True:
        try:
            frame = frame_queue.get(timeout=0.15)
            if RENDER_WINDOW:
                cv2.imshow("A8 Mini Stream", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("r"):  # press 'r' to force reconnect
                    print("→ Manual reconnect requested")
                    # (the thread will handle it automatically anyway)

        except queue.Empty:
            # No new frame → just keep the last one or show nothing
            continue

finally:
    if RENDER_WINDOW:
        cv2.destroyAllWindows()
    print("Exited cleanly")
