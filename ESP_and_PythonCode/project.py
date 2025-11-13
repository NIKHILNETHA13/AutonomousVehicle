import cv2
import threading
import time
import requests
from ultralytics import YOLO

# === CONFIGURATION ===
CAM_URL = "http://192.168.137.209:81/stream"          # ESP32-CAM video stream
ESP32_CAR_URL = "http://192.168.137.79/set_speed?spd="  # ESP32 speed control endpoint
MODEL_PATH = "best.pt"                                # Fine-tuned YOLOv11 model

# === GLOBALS ===
frame = None
stopped = False
detected_speed = None
lock = threading.Lock()

# === LOAD YOLO MODEL ===
print("[INFO] Loading YOLOv11 Traffic Sign Model...")
model = YOLO(MODEL_PATH)
print("[INFO] Model loaded successfully.")

# === FRAME CAPTURE THREAD ===
def grab_frames():
    global frame, stopped
    cap = cv2.VideoCapture(CAM_URL)
    if not cap.isOpened():
        print("❌ Could not open ESP32-CAM stream.")
        stopped = True
        return

    print("✅ Stream started. Detecting traffic signs... Press Q to quit.")
    while not stopped:
        ret, f = cap.read()
        if not ret:
            continue
        with lock:
            frame = f
    cap.release()

# === YOLO DETECTION THREAD ===
def detect_signs():
    global detected_speed
    prev_speed = None
    last_send_time = 0
    prev_time = time.time()
    fps = 0

    while not stopped:
        if frame is None:
            time.sleep(0.05)
            continue

        with lock:
            f = frame.copy()

        # Resize for faster detection
        resized = cv2.resize(f, (416, 416))

        # Run YOLO prediction
        results = model.predict(resized, verbose=False, conf=0.6)
        detected_speed = None
        stop_detected = False

        for r in results:
            for box in r.boxes:
                cls = int(box.cls)
                label = model.names[cls].upper()

                # Draw bounding box
                (x1, y1, x2, y2) = map(int, box.xyxy[0])
                cv2.rectangle(resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(resized, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # === STOP SIGN LOGIC ===
                if "STOP" in label:
                    stop_detected = True
                    detected_speed = "0"  # PWM 0 = stop
                    break

                # === SPEED SIGN LOGIC ===
                if "SPEED" in label or "LIMIT" in label:
                    try:
                        detected_speed = label.split()[-1]  # extract number at end
                    except:
                        pass

            if stop_detected:
                break

        # === SEND TO ESP32 ===
        if detected_speed and (detected_speed != prev_speed or time.time() - last_send_time > 1.5):
            try:
                requests.get(f"{ESP32_CAR_URL}{detected_speed}", timeout=1)
                if stop_detected:
                    print(f"[🚦] STOP detected → Sending STOP (speed 0)")
                else:
                    print(f"[DETECTED] Speed Limit: {detected_speed}")
                prev_speed = detected_speed
                last_send_time = time.time()
            except requests.exceptions.RequestException:
                print("[WARN] Could not send data to ESP32.")

        # === Accurate FPS Calculation ===
        current_time = time.time()
        fps = 1 / (current_time - prev_time + 1e-5)
        prev_time = current_time

        # === Overlay Detected Speed and FPS ===
        display_text = f"Speed: {detected_speed if detected_speed else '--'}"
        cv2.putText(resized, display_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(resized, f"FPS: {fps:.1f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow("Traffic Sign Detection (YOLOv11)", resized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

# === START THREADS ===
grab_thread = threading.Thread(target=grab_frames, daemon=True)
detect_thread = threading.Thread(target=detect_signs, daemon=True)

grab_thread.start()
detect_thread.start()

# === MAIN LOOP ===
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    stopped = True
    grab_thread.join()
    detect_thread.join()
    print("\n🛑 Exiting cleanly.")
