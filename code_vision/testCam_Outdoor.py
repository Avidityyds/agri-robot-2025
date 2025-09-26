import cv2
import numpy as np
import sys, os, time

CAM_INDEX = 0  
W, H = 1280, 720

# ====== 偵測參數 ======
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
# ====================

def open_camera(idx):
    # 依平台選擇相機後端
    if sys.platform.startswith("darwin"):   # macOS
        backends = [cv2.CAP_AVFOUNDATION, 0]
    elif os.name == "nt":                   # Windows
        backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, 0]
    else:                                   # Linux
        backends = [cv2.CAP_V4L2, 0]

    for b in backends:
        cap = cv2.VideoCapture(idx) if b == 0 else cv2.VideoCapture(idx, b)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
            if b == cv2.CAP_V4L2:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            return cap
        if cap: cap.release()
    return None

def red_mask_hsv(frame_bgr):
    """HSV：紅色雙區（排除過低 S/V），回傳 mask 及 hsv 供後續用。"""
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
    # OpenCV 的 threshold 回傳 (thr, mask)
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

def overlay_red(frame_bgr, mask):
    """把紅線高亮覆蓋在原圖上（紅色半透明）。"""
    red_vis = np.zeros_like(frame_bgr)
    red_vis[:, :, 2] = 255  # 紅通道
    overlay = np.where(mask[..., None] == 255,
                       cv2.addWeighted(frame_bgr, 0.3, red_vis, 0.7, 0),
                       frame_bgr)
    return overlay

def main():
    cap = open_camera(CAM_INDEX)
    if not cap:
        print("無法開啟攝影機。請換 CAM_INDEX=0/1，並確認 App 有相機權限。")
        return
    time.sleep(0.2)  # 暖機

    suppress_green = True       # 可按 'g' 切換
    cr_floor = CR_MIN_FLOOR     # 可用 '=' / '-' 微調
    print("提示：按 'g' 切換是否抑制綠草地；'=' / '-' 微調 Cr 最小值；'q' 離開。")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("讀取影像失敗"); break

        # HSV 紅
        m_hsv, hsv = red_mask_hsv(frame)

        # Cr（紅度）
        m_cr = red_mask_cr(frame, cr_floor=cr_floor)

        # Lab a*（偏紅）
        m_a  = red_mask_lab(frame)

        # 融合：HSV 紅 ∧ (Cr ∨ a*)
        mask = cv2.bitwise_and(m_hsv, cv2.bitwise_or(m_cr, m_a))

        # 抑制明顯綠草地範圍
        if suppress_green:
            mg = green_mask_hsv(hsv)
            mask = cv2.bitwise_and(mask, cv2.bitwise_not(mg))

        # 以 S/V 有效像素加一道保險
        s, v = hsv[:, :, 1], hsv[:, :, 2]
        sv_valid = ((s >= S_MIN) & (v >= V_MIN)).astype(np.uint8) * 255
        mask = cv2.bitwise_and(mask, sv_valid)

        # 閉運算
        mask = thin_line_friendly_close(mask)

        # 顯示：原圖 + 紅線高亮；以及二值遮罩
        overlay = overlay_red(frame, mask)
        cv2.imshow("Camera (Red line on grass)", overlay)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('g'):
            suppress_green = not suppress_green
            print(f"抑制綠草地：{suppress_green}")
        elif key == ord('=') or key == ord('+'):
            cr_floor = min(255, cr_floor + 1)
            print(f"Cr 最小值保底 -> {cr_floor}")
        elif key == ord('-') or key == ord('_'):
            cr_floor = max(0, cr_floor - 1)
            print(f"Cr 最小值保底 -> {cr_floor}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
