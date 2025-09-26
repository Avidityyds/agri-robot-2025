import cv2, numpy as np
import serial
import time
import signal
import atexit
import sys

# ======= 調整參數 =======
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 9600
CAM_INDEX   = 0
FRAME_W, FRAME_H = 640, 480

Kp = 0.25
U_MAX = 120 # 最高修正值

# 左右輪各自的 base speed
BASE_L = 85
BASE_R = 85

ROWS = [40, 90, 140] # 與底部的距離（像素）
BAND_HEIGHT = 30
MIN_PIXELS = 200
LOST_HOLD = 12
PRINT_EVERY = 3
FAILSAFE_MODE = "straight" # 失線後直走
# =======================

def clamp(x, lo, hi): return max(lo, min(hi, x))

ser = None
def _safe_stop():
    global ser
    try:
        if ser and ser.is_open:
            ser.write(b"L:0 R:0\n")
            ser.flush()
            time.sleep(0.02)
    except Exception:
        pass

atexit.register(_safe_stop)
for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, lambda s, f: (_safe_stop(), sys.exit(0)))

def build_red_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low1, high1 = (0, 60, 50), (12, 255, 255)
    low2, high2 = (170, 60, 50), (180, 255, 255)
    m1 = cv2.inRange(hsv, np.array(low1), np.array(high1))
    m2 = cv2.inRange(hsv, np.array(low2), np.array(high2))
    mask = cv2.bitwise_or(m1, m2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8), iterations=1)
    mask[:FRAME_H//2, :] = 0   # 只用下半部，避開雜點
    return mask

def band_centroid_x(band):
    ys, xs = np.nonzero(band)
    if xs.size < MIN_PIXELS:
        return None, xs.size
    return int(xs.mean()), xs.size

def main():
    global ser
    # Serial
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2)
    time.sleep(2.0)

    # Camera
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        print("[ERR] 無法開啟相機"); return

    center_x = FRAME_W // 2
    n = len(ROWS)
    weights = np.linspace(1.0, 0.5, num=n); weights /= weights.sum()

    lost_counter = 0
    last_u = 0.0
    frame_idx = 0

    print("[INFO] 開始循跡。Ctrl+C 會強制停車。")
    while True:  
        ok, frame = cap.read()
        if not ok:
            print("[WARN] 相機讀取失敗"); break

        mask = build_red_mask(frame)

        errors, found, counts = [], 0, []
        for i, d in enumerate(ROWS):         
            y_mid = FRAME_H - d
            y1 = clamp(y_mid - BAND_HEIGHT//2, 0, FRAME_H-1)
            y2 = clamp(y_mid + BAND_HEIGHT//2, 0, FRAME_H-1)
            band = mask[y1:y2, :]
            cx, num = band_centroid_x(band)
            counts.append(num)
            if cx is not None:
                errors.append((center_x - cx, weights[i]))
                found += 1

        if errors:
            wsum = sum(w for _, w in errors)
            error = sum(e*w for e, w in errors) / (wsum if wsum>1e-6 else 1.0)
            u = clamp(Kp * error, -U_MAX, U_MAX)
            last_u = u
            lost_counter = 0
        else:
            lost_counter += 1
            if lost_counter <= LOST_HOLD:
                u = last_u * 0.6
            else:
                u = 0.0  # 直走或停車由下方 FAILSAFE_MODE 決定

        # 用左右各自的 base speed
        L = clamp(int(BASE_L + u), 0, 255)
        R = clamp(int(BASE_R - u), 0, 255)

        # 失線策略
        if lost_counter > LOST_HOLD:
            if FAILSAFE_MODE == "straight":
                L = clamp(int(BASE_L), 0, 255)
                R = clamp(int(BASE_R), 0, 255)
                u = 0.0
            else:  # "stop"
                L = R = 0

        ser.write(f"L:{L} R:{R}\n".encode())

        frame_idx += 1
        if frame_idx % PRINT_EVERY == 0:
            if found:
                print(f"[PWM] L={L:>3}  R={R:>3}  u={u:>5.1f}  found={found}/{n}  bases=({BASE_L},{BASE_R})  pixels={counts}")
            else:
                mode = "STRAIGHT" if FAILSAFE_MODE=='straight' else "STOP"
                print(f"[LOST] {lost_counter}/{LOST_HOLD} → L={L:>3} R={R:>3} u={u:>5.1f} mode={mode} bases=({BASE_L},{BASE_R})")

    _safe_stop()
    try: ser.close()
    except: pass

if __name__ == "__main__":
    main()
