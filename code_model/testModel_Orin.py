import time
import cv2
from ultralytics import YOLO

# === 設定參數 ===
MODEL_PATH = "best.pt"     # YOLO 權重
CAM_INDEX  = 0             # /dev/videoX 對應的索引
FRAME_W    = 1280
FRAME_H    = 720
CONF_THRES = 0.3
IOU_THRES  = 0.25
DEVICE     = 0             # 0=GPU 第0張卡；若沒有GPU或想用CPU，改成 "cpu"
SHOW_FPS   = True

def open_camera(cam_index=0, w=1280, h=720):
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        raise SystemExit(f"無法開啟攝影機 {cam_index}")

    # 設定解析度與壓縮格式（MJPG 在許多 UVC 上能提升取流效率）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    # cap.set(cv2.CAP_PROP_FPS, 30)  # 需要時可嘗試設置

    return cap

def main():
    # 1) 載入模型
    model = YOLO(MODEL_PATH)

    # 2) 開相機
    cap = open_camera(CAM_INDEX, FRAME_W, FRAME_H)

    # 3) 連續偵測
    fps_hist = []
    win_name = "YOLO - Webcam (press q to quit)"
    while True:
        ok, frame = cap.read()
        if not ok:
            print("讀取影像失敗，結束。")
            break

        t0 = time.time()
        # 直接對 frame (numpy array, BGR) 推論
        results = model(
            frame,
            conf=CONF_THRES,
            iou=IOU_THRES,
            device=DEVICE,
            verbose=False
        )

        annotated = results[0].plot()  # 繪製 bbox/label

        # 4) 疊上 FPS
        if SHOW_FPS:
            fps = 1.0 / max(1e-6, time.time() - t0)
            fps_hist.append(fps)
            if len(fps_hist) > 30:
                fps_hist.pop(0)
            avg_fps = sum(fps_hist) / len(fps_hist)
            cv2.putText(
                annotated,
                f"FPS: {avg_fps:.1f}  (device: {DEVICE})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # 5) 顯示
        cv2.imshow(win_name, annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
