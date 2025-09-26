import cv2

# 假設最多有 5 支攝影機可以測（自己調整範圍）
CAM_INDEXES = range(5)
FRAME_W, FRAME_H = 640, 480

def open_cam(idx):
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    return cap

def main():
    caps = []
    for idx in CAM_INDEXES:
        cap = open_cam(idx)
        if cap.isOpened():
            print(f"[OK] Camera {idx} 可以使用")
            caps.append((idx, cap))
        else:
            print(f"[X]  Camera {idx} 無法開啟")

    if not caps:
        print("找不到任何可用攝影機")
        return

    win_names = [f"Camera {idx}" for idx,_ in caps]

    while True:
        for (idx, cap), win in zip(caps, win_names):
            ok, frame = cap.read()
            if not ok:
                print(f"[警告] Camera {idx} 讀取失敗")
                continue

            cv2.imshow(win, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    for _, cap in caps:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
