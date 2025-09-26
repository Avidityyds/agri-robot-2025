import time
import argparse
from collections import deque
import cv2
from ultralytics import YOLO

#參數
DEFAULTS = dict(
    model="/Users/jameschen/Desktop/機器人專題/室內影像辨識模型/best.pt",
    cam=0,
    w=1280, 
    h=720,
    conf=0.25,
    iou=0.45,
    device=None,   # None=自動；也可填 "cpu" / "cuda:0" / 整數GPU索引
    show_fps=True
)

def parse_args():
    p = argparse.ArgumentParser(description="YOLO webcam - local window only")
    p.add_argument("--model", default=DEFAULTS["model"])
    p.add_argument("--cam", type=int, default=DEFAULTS["cam"])
    p.add_argument("--w", type=int, default=DEFAULTS["w"])
    p.add_argument("--h", type=int, default=DEFAULTS["h"])
    p.add_argument("--conf", type=float, default=DEFAULTS["conf"])
    p.add_argument("--iou", type=float, default=DEFAULTS["iou"])
    p.add_argument("--device", default=DEFAULTS["device"])
    p.add_argument("--show-fps", action="store_true", default=DEFAULTS["show_fps"])
    return p.parse_args()

def pick_device(user_device):
    if user_device is None or str(user_device).lower() == "none":
        try:
            import torch
            return "cuda:0" if torch.cuda.is_available() else "cpu"
        except Exception:
            return "cpu"
    return user_device

def open_camera(cam_index=0, w=1280, h=720):
    # 在部分 Windows 裝置，用 CAP_DSHOW 比較穩；若失敗則退回預設
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(cam_index)

    if not cap.isOpened():
        raise SystemExit(f"無法開啟攝影機 {cam_index}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    # 用 MJPG 取流較順
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    # cap.set(cv2.CAP_PROP_FPS, 30)
    return cap

def main():
    args = parse_args()
    device = pick_device(args.device)
    print(f"[INFO] 使用裝置: {device}")

    # 1) 載入模型
    model = YOLO(args.model)

    # 2) 開相機
    cap = open_camera(args.cam, args.w, args.h)

    # 3) 連續偵測並顯示
    fps_hist = deque(maxlen=30)
    win_name = "YOLO - Webcam (press q or Esc to quit)"

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("讀取影像失敗，結束。")
                break

            t0 = time.time()
            results = model(
                frame,
                conf=args.conf,
                iou=args.iou,
                device=device,
                verbose=False
            )

            annotated = results[0].plot()

            if args.show_fps:
                fps = 1.0 / max(1e-6, time.time() - t0)
                fps_hist.append(fps)
                avg_fps = sum(fps_hist) / len(fps_hist)
                cv2.putText(
                    annotated,
                    f"FPS: {avg_fps:.1f}  (device: {device})",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow(win_name, annotated)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q') or k == 27:  # q 或 ESC 離開
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
