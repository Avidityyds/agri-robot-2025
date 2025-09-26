import cv2
import numpy as np
import sys, os, time

CAM_INDEX = 0 # 鏡頭
W, H = 1280, 720

# 紅色 HSV 兩段（可再調）
LOW1 = (0,   80, 80)
HIGH1= (10, 255,255)
LOW2 = (170, 80, 80)
HIGH2= (180,255,255)

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

def main():
    cap = open_camera(CAM_INDEX)
    if not cap:
        print("無法開啟攝影機。請換 CAM_INDEX=0/1，並確認 App 有相機權限。")
        return
    time.sleep(0.2)  # 暖機

    while True:
        ok, frame = cap.read()
        if not ok:
            print("讀取影像失敗"); break

        # 產生二值化遮罩（紅色）
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOW1, HIGH1) | cv2.inRange(hsv, LOW2, HIGH2)
        # 去雜訊（可視需要關掉或調整）
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5,5),np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))

        # 顯示兩個視窗
        cv2.imshow("Camera", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
