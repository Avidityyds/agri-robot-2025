import cv2, numpy as np
import serial
import time
import signal
import atexit
import sys
import os

# ===== 調整參數 =====
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 9600
CAM_INDEX   = 0
FRAME_W, FRAME_H = 640, 480

Kp = 0.5
U_MAX = 120 # 最高修正值

# 左右輪各自的 base speed
BASE_L = 180
BASE_R = 170

ROWS = [40, 90, 140] # ROWS與底部的距離（像素）
BAND_HEIGHT = 30 # 每條ROWS的高度（像素）
MIN_PIXELS = 200 # 判定有線的最低像素數
LOST_HOLD = 12 # 幾幀後算失線
PRINT_EVERY = 3
FAILSAFE_MODE = "stop" # 失線後策略 "straight" 直走 or "stop" 停車
# ===================

# ===== 影像處理參數 =====
S_MIN = 40
V_MIN = 40
HSV_RED1 = ((0,   60, V_MIN),  (10, 255,255))   # 紅區1
HSV_RED2 = ((170, 60, V_MIN),  (179,255,255))   # 紅區2

# 綠草地（用於抑制）
HSV_GREEN = ((35,  30, 30),    (85, 255,255))

# YCrCb：Cr 高→越紅；Otsu + 一個保底最小值（避免暗處誤判）
CR_MIN_FLOOR = 150

# Lab：a* > 128 偏紅；越大越紅
A_STAR_MIN = 140

# 閉運算補線
KERNEL_CLOSE_W = 3
KERNEL_CLOSE_H = 3
# ======================

def clamp(x, lo, hi): return max(lo, min(hi, x))

# ====== 影像處理 ======
def red_mask_hsv(frame_bgr):
    """HSV：紅色雙區（排除過低 S/V），回傳 mask 及 hsv。"""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    valid = (s >= S_MIN) & (v >= V_MIN)

    m1 = cv2.inRange(hsv, HSV_RED1[0], HSV_RED1[1])
    m2 = cv2.inRange(hsv, HSV_RED2[0], HSV_RED2[1])
    mh = (m1 | m2).astype(bool)

    out = np.zeros_like(h, dtype=np.uint8)
    out[mh & valid] = 255
    return out, hsv

def green_mask_hsv(hsv):
    """HSV 綠色遮罩（草地），用來抑制背景誤檢。"""
    mg = cv2.inRange(hsv, HSV_GREEN[0], HSV_GREEN[1])
    return mg

def red_mask_cr(frame_bgr, cr_floor=CR_MIN_FLOOR):
    """YCrCb 的 Cr：Otsu 自動 + floor 保底，輸出紅區二值。"""
    ycrcb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YCrCb)
    _, cr, _ = cv2.split(ycrcb)
    thr, _mask = cv2.threshold(cr, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thr = max(int(thr), int(cr_floor))
    _, m = cv2.threshold(cr, thr, 255, cv2.THRESH_BINARY)
    return m

def red_mask_lab(frame_bgr):
    """Lab 的 a*：> 128 偏紅；閾值越高越嚴。"""
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2Lab)
    _, a, _ = cv2.split(lab)
    _, m = cv2.threshold(a, A_STAR_MIN, 255, cv2.THRESH_BINARY)
    return m

def thin_line_friendly_close(mask):
    """輕量閉運算補線"""
    k_h = cv2.getStructuringElement(cv2.MORPH_RECT, (KERNEL_CLOSE_W, 1))
    k_v = cv2.getStructuringElement(cv2.MORPH_RECT, (1, KERNEL_CLOSE_H))
    m = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_h)
    m = cv2.morphologyEx(m,    cv2.MORPH_CLOSE, k_v)
    return m

def build_red_mask(frame_bgr, suppress_green=True, cr_floor=CR_MIN_FLOOR):
    """
    以你前面的流程產生紅線遮罩，給 PID 控制使用。
    與原 PID 版一致，最後仍保留「只用下半部」以降低遠處雜點影響。
    """
    # HSV 紅
    m_hsv, hsv = red_mask_hsv(frame_bgr)
    # Cr（紅度）
    m_cr = red_mask_cr(frame_bgr, cr_floor=cr_floor)
    # Lab a*（偏紅）
    m_a  = red_mask_lab(frame_bgr)
    # 融合：HSV 紅 ∧ (Cr ∨ a*)
    mask = cv2.bitwise_and(m_hsv, cv2.bitwise_or(m_cr, m_a))

    # 抑制明顯綠草地範圍（可選）
    if suppress_green:
        mg = green_mask_hsv(hsv)
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(mg))

    # 以 S/V 有效像素加一道保險
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    sv_valid = ((s >= S_MIN) & (v >= V_MIN)).astype(np.uint8) * 255
    mask = cv2.bitwise_and(mask, sv_valid)

    # 細線友善閉運算
    mask = thin_line_friendly_close(mask)

    # 與原先 PID 行為一致：只用下半部，避開遠處雜點
    mask[:FRAME_H//2, :] = 0
    return mask
# =====================

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
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
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

        mask = build_red_mask(frame, suppress_green=True, cr_floor=CR_MIN_FLOOR)

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
