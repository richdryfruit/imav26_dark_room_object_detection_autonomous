"""
Jetson Doll Counter — ROS 2 Flight Version
==========================================
Detects and counts dolls using YOLO + ByteTrack, reports the cumulative
count to the ROS 2 network, and drives an Arduino Uno + all-pin
TFT shield over USB serial for a local physical readout.
"""

import os
import time
import cv2
import numpy as np
from ultralytics import YOLO

# --- ROS 2 Imports ---
import rclpy
from std_msgs.msg import Int32

# ==========================================================================
# CONFIGURATION
# ==========================================================================

CAMERA_INDEX = 0           
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "db.engine")

CONFIDENCE_THRESHOLD = 0.55
MIN_FRAMES_TO_CONFIRM = 5
REID_MAX_DISTANCE_PX = 70
REID_MAX_TIME_GAP_SEC = 4.0

TRACKER_YAML_PATH = os.path.join(SCRIPT_DIR, "custom_bytetrack.yaml")
TRACKER_YAML_CONTENT = """
tracker_type: bytetrack
track_high_thresh: 0.5
track_low_thresh: 0.1
new_track_thresh: 0.6
track_buffer: 500
match_thresh: 0.8
fuse_score: True
"""

# --- Arduino Uno + all-pin TFT shield ---
ENABLE_ARDUINO_TFT = True
ARDUINO_TFT_PORT = "/dev/ttyACM0"   
ARDUINO_TFT_BAUD = 115200

# --- GUI window (auto-disables itself if no display is available) ---
_gui_requested = os.environ.get("JW_ENABLE_GUI", "1") == "1"
_display_available = bool(os.environ.get("DISPLAY"))
ENABLE_GUI_WINDOW = _gui_requested and _display_available

if _gui_requested and not _display_available:
    print("[INFO] No DISPLAY environment variable found — running headless. "
          "GUI window will not be opened (this is expected under systemd).")


# ==========================================================================
# DOLL COUNTER 
# ==========================================================================

class DollCounter:
    def __init__(self):
        self.total_count = 0
        self._hits_per_track = {}
        self._track_to_doll = {}
        self._dolls = {}
        self._next_doll_id = 1

    def update(self, detections, now=None):
        if now is None:
            now = time.time()

        active_track_ids = {d[0] for d in detections}
        display_info = []

        for track_id, box, conf in detections:
            centroid = self._centroid(box)

            if track_id in self._track_to_doll:
                doll_id = self._track_to_doll[track_id]
                self._dolls[doll_id]["centroid"] = centroid
                self._dolls[doll_id]["last_seen"] = now
                display_info.append((track_id, box, doll_id, True))
                continue

            self._hits_per_track[track_id] = self._hits_per_track.get(track_id, 0) + 1

            if self._hits_per_track[track_id] < MIN_FRAMES_TO_CONFIRM:
                display_info.append((track_id, box, None, False))
                continue

            matched_doll_id = self._find_reidentification_match(
                centroid, now, active_track_ids
            )

            if matched_doll_id is not None:
                doll_id = matched_doll_id
            else:
                doll_id = self._next_doll_id
                self._next_doll_id += 1
                self.total_count += 1

            self._track_to_doll[track_id] = doll_id
            self._dolls[doll_id] = {"centroid": centroid, "last_seen": now}
            display_info.append((track_id, box, doll_id, True))

        return display_info

    def _find_reidentification_match(self, centroid, now, active_track_ids):
        currently_active_doll_ids = {
            doll_id
            for tid, doll_id in self._track_to_doll.items()
            if tid in active_track_ids
        }

        best_match = None
        best_dist = REID_MAX_DISTANCE_PX

        for doll_id, info in self._dolls.items():
            if doll_id in currently_active_doll_ids:
                continue  
            if now - info["last_seen"] > REID_MAX_TIME_GAP_SEC:
                continue  
            dist = self._distance(centroid, info["centroid"])
            if dist < best_dist:
                best_dist = dist
                best_match = doll_id

        return best_match

    @staticmethod
    def _centroid(box):
        x1, y1, x2, y2 = box
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    @staticmethod
    def _distance(p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


# ==========================================================================
# SETUP
# ==========================================================================

def write_tracker_config():
    with open(TRACKER_YAML_PATH, "w") as f:
        f.write(TRACKER_YAML_CONTENT.strip())
    print(f"[INFO] Generated tracker config: {TRACKER_YAML_PATH}")

def init_arduino_tft():
    if not ENABLE_ARDUINO_TFT:
        return None
    try:
        from arduino_tft_display import ArduinoTftDisplay
        display = ArduinoTftDisplay(port=ARDUINO_TFT_PORT, baud=ARDUINO_TFT_BAUD)
        print("[INFO] Arduino TFT display initialized.")
        display.ser.write(b"F:jw_ros2_arduinocount.py\n")
        display.ser.write(b"S:YOLO Active\n")
        return display
    except Exception as e:
        print(f"[WARN] Arduino TFT not initialized ({e}). Continuing without it.")
        return None

def draw_detection(frame, box, doll_id, is_counted):
    x1, y1, x2, y2 = map(int, box)
    if is_counted:
        color = (0, 255, 0)        
        label = f"Doll #{doll_id}"
    else:
        color = (0, 255, 255)      
        label = "confirming..."

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        frame, label, (x1, max(0, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
    )

def draw_hud(frame, visible_count, total_count):
    hud_bg = frame.copy()
    cv2.rectangle(hud_bg, (0, 0), (FRAME_WIDTH, 50), (0, 0, 0), -1)
    cv2.addWeighted(hud_bg, 0.65, frame, 0.35, 0, frame)
    cv2.putText(
        frame, f"VISIBLE ON SCREEN: {visible_count}", (15, 22),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
    )
    cv2.putText(
        frame, f"TOTAL DOLLS COUNTED: {total_count}", (15, 45),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
    )


# ==========================================================================
# MAIN
# ==========================================================================

def main():
    global ENABLE_GUI_WINDOW

    write_tracker_config()

    print("[INFO] Loading TensorRT YOLO model...")
    model = YOLO(MODEL_PATH, task="detect")

    print("[INFO] Initializing Webcam...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)

    if not cap.isOpened():
        print("[ERROR] Could not open webcam. Check CAMERA_INDEX.")
        return

    # --- ROS 2 Node Init ---
    rclpy.init()
    ros_node = rclpy.create_node('doll_counter_node')
    count_pub = ros_node.create_publisher(Int32, '/doll_count', 10)
    last_sent_count = -1

    arduino_tft = init_arduino_tft()
    counter = DollCounter()

    if ENABLE_GUI_WINDOW:
        print("\nPress 'q' (in the video window) to quit.\n")
    else:
        print("\n[INFO] GUI window disabled/unavailable — running headless.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to grab frame from camera.")
            break

        results = model.track(
            frame, tracker=TRACKER_YAML_PATH, persist=True, verbose=False
        )[0]

        detections = []
        if results.boxes is not None and results.boxes.id is not None:
            boxes = results.boxes.xyxy.cpu().numpy()
            track_ids = results.boxes.id.cpu().numpy().astype(int)
            confidences = results.boxes.conf.cpu().numpy()

            for box, track_id, conf in zip(boxes, track_ids, confidences):
                if conf < CONFIDENCE_THRESHOLD:
                    continue  
                detections.append((int(track_id), box, float(conf)))

        display_info = counter.update(detections)

        # Draw to frame memory (useful if saving video later, or if GUI is on)
        for track_id, box, doll_id, is_counted in display_info:
            draw_detection(frame, box, doll_id, is_counted)

        visible_count = len(detections)
        total_count = counter.total_count

        draw_hud(frame, visible_count, total_count)

        # --- ROS 2 Publish ---
        if total_count != last_sent_count:
            try:
                msg = Int32()
                msg.data = total_count
                count_pub.publish(msg)
                print(f"[ROS 2] Published total count: {total_count}")
                last_sent_count = total_count
            except Exception as e:
                print(f"[WARN] ROS 2 publish failed ({e}).")

        # Process ROS 2 callbacks/events
        rclpy.spin_once(ros_node, timeout_sec=0.0)

        # --- Arduino TFT Write ---
        if arduino_tft is not None:
            try:
                arduino_tft.show_count(visible_count, total_count)
            except Exception as e:
                print(f"[WARN] Arduino TFT write failed ({e}); disabling it.")
                arduino_tft = None

        if ENABLE_GUI_WINDOW:
            cv2.imshow("Jetson Doll Tracker", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    # --- Cleanup ---
    cap.release()
    if ENABLE_GUI_WINDOW:
        cv2.destroyAllWindows()
    if arduino_tft is not None:
        arduino_tft.close()

    ros_node.destroy_node()
    rclpy.shutdown()

    print(f"\n[RESULT] Final cumulative doll count: {counter.total_count}")


if __name__ == "__main__":
    main()