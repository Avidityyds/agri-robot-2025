# 全部移動都「寫死分段」(多段 PWM×秒數)；
# 只有：
#   1) 第一關停車用側鏡 YOLO 決定目標動物（第一個穩定）→ 亮 LED、記下重量
#   2) 中間到飼料機：到點直接 STRETCH（不帶重量），等 STRETCH* 回覆。
#   3) 最後一關：固定三段行進；每段結束先停車偵測第一關目標 → 若有看到就亮 LED 3s

import cv2
import time
import sys
import glob
import serial
from collections import deque
from ultralytics import YOLO

# ========= 可調參數 =========
MODEL_PATH  = "best.pt"
SERIAL_PORT = "/dev/ttyACM0"   # 仍會先嘗試這個，失敗就自動偵測
BAUDRATE    = 9600

LED_ON_SEC = 3.0                # 第一關定案後亮燈秒數（會自動關）
STOP_CHECK_SEC = 1.5            # 每段結束停下來偵測的時間（秒）

# 只開側視相機（偵測用）
CAM_SIDE_INDEX = 4
CAM_W, CAM_H   = 640, 480

# --- 寫死分段（(秒, 左PWM, 右PWM)） ---
# 起點 → 動物區：跑完後停下來辨識
PRE_TO_ANIMAL_SEGMENTS = [
    (1.0, 0, 0),       # Serial 休息
    (14.0, 80, 50),    # 14秒 80 50
]

# 動物區 → 飼料機：跑完後停下來，直接 STRETCH（不帶重量）
ANIMAL_TO_FEEDER_SEGMENTS = [
    (15.0, 95, 50),
]

# 飼料機 → 投餵區（最後一關）：不邊跑邊看；
# 跑完每一段就停車→偵測第一關目標→有看到就亮LED 3秒
FEEDER_TO_FEEDZONE_SEGMENTS = [
    (3.5, 95, 50),
    (11.5, 95, 50),
    (5.0,  95, 50),
    (4.0,  95, 50),
]

# 出場（可自行配置）
EXIT_SEGMENTS = [
    (30.0, 95, 50),
]

# 類別/LED/重量
YOLO_CLASS_ROLE = {"chicken":"CHICKEN", "cows":"COW", "pig":"PIG", "machine":"FEEDER"}
ALIASES = {"cow":"cows", "feeder":"machine"}
ANIMAL_ROLES = ("CHICKEN", "PIG", "COW")
ROLE_TO_LED    = {"CHICKEN":"G", "PIG":"Y", "COW":"R"}
ROLE_TO_WEIGHT = {"CHICKEN":70,  "PIG":155, "COW":205}

# 偵測門檻
DET_CONF_TH = 0.50
FINAL_SEE_MIN_WIDTH = 0   # 若想避免太遠的小框，可設 60~100
# ===========================

# ===== 工具 =====
def clamp(v, lo, hi): return max(lo, min(hi, v))

def normalize_classname(name: str):
    k = name.strip().lower()
    if k in ALIASES: k = ALIASES[k]
    return k

def bbox_center_w(b):
    x1,y1,x2,y2 = map(int, b)
    cx = (x1+x2)//2
    w  = (x2-x1)
    return cx, w

# ===== 串列 & 握手 =====
def autodetect_serial(baud=9600, prefer=None):
    candidates = []
    if prefer:
        candidates.append(prefer)
    candidates += ["/dev/ttyACM0","/dev/ttyACM1","/dev/ttyUSB0","/dev/ttyUSB1"]
    candidates += sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))

    tried = set()
    for p in candidates:
        if p in tried: continue
        tried.add(p)
        try:
            ser = serial.Serial(p, baudrate=baud, timeout=0.05)
            time.sleep(2.2)
            ser.reset_input_buffer()
            ser.write(b"PING\n")
            t0 = time.time()
            while time.time() - t0 < 1.2:
                line = ser.readline().decode("utf-8","ignore").strip()
                if line == "PONG" or line == "READY":
                    print(f"[SER] Connected {p} @ {baud}")
                    return ser
            ser.close()
        except:
            pass
    print("[ERR] 找不到可用的 Arduino 或無法握手（PONG/READY）")
    sys.exit(2)

class SerialClient:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(2.2)
            self.ser.reset_input_buffer()
        except Exception:
            self.ser = autodetect_serial(baud, prefer=port)

    def send(self, line):
        self.ser.write((line.strip()+"\n").encode("utf-8"))

    def read_line(self):
        try:
            s = self.ser.readline().decode("utf-8", errors="ignore").strip()
            return s if s else None
        except:
            return None

    def expect(self, prefix, timeout=20.0):
        t0 = time.time()
        while time.time()-t0 < timeout:
            s = self.read_line()
            if s:
                print(f"[ARD] {s}")  # 回覆都印出
                if s.startswith(prefix):
                    return s
        return None

    def read_and_print_nonblock(self):
        s = self.read_line()
        while s:
            print(f"[ARD] {s}")
            s = self.read_line()

    def close(self):
        try: self.ser.close()
        except: pass

def drive(sc, vL, vR): sc.send(f"L:{int(clamp(vL,-255,255))} R:{int(clamp(vR,-255,255))}")
def stop(sc): sc.send("STOP")

def led(sc, color, seconds=LED_ON_SEC):
    print(f"[STEP] LED {color} ON {seconds:.1f}s")
    sc.send(f"LED {color}")
    _ = sc.expect("LED OK", timeout=0.6)
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        sc.read_and_print_nonblock()
        time.sleep(0.02)
    sc.send("LED_STOP")
    _ = sc.expect("LED_STOP OK", timeout=0.6)
    print(f"[STEP] LED {color} OFF")

def stretch(sc):
    sc.send("STRETCH")

# 若之後還要出料，可保留；本版最後一關不再呼叫
def feed_servo(sc):
    sc.send("DOWN 180")

# ===== 推進段落（無即時偵測） =====
def run_segments(sc, segments, tag="segment", use_kick=False):
    for i, (sec, vL, vR) in enumerate(segments, 1):
        print(f"[{tag}] {i}/{len(segments)}: {sec:.2f}s  L={vL} R={vR}")
        drive(sc, vL, vR)
        t0 = time.time()
        while time.time()-t0 < sec:
            sc.read_and_print_nonblock()
            time.sleep(0.02)
    stop(sc); time.sleep(0.2); sc.read_and_print_nonblock()

# ===== 在停止狀態下做一次偵測：看到指定 target_role 就回 True =====
def check_seen_while_stopped(model, cam, target_role, dwell_sec=STOP_CHECK_SEC):
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < dwell_sec:
        ok, frm = cam.read()
        if not ok:
            time.sleep(0.01)
            continue
        r = model.predict(source=frm, conf=DET_CONF_TH, verbose=False)[0]
        for b, c, conf in zip(r.boxes.xyxy.cpu().numpy(),
                              r.boxes.cls.cpu().numpy(),
                              r.boxes.conf.cpu().numpy()):
            name = normalize_classname(r.names[int(c)])
            if name not in YOLO_CLASS_ROLE:
                continue
            role = YOLO_CLASS_ROLE[name]
            if role != target_role:
                continue
            if conf < DET_CONF_TH:
                continue
            _, bw = bbox_center_w(b)
            if FINAL_SEE_MIN_WIDTH > 0 and bw < FINAL_SEE_MIN_WIDTH:
                continue
            return True
    return False

# ===== 最後一關：每段跑完 → 停車 → 偵測 → 亮燈(若有看到) =====
def run_segments_stop_and_check(sc, segments, tag, model, cam, target_role):
    for i, (sec, vL, vR) in enumerate(segments, 1):
        print(f"[{tag}] RUN {i}/{len(segments)}: {sec:.2f}s  L={vL} R={vR}")
        # 跑這段
        drive(sc, vL, vR)
        t0 = time.time()
        while time.time()-t0 < sec:
            sc.read_and_print_nonblock()
            time.sleep(0.02)
        # 停車 → 偵測
        stop(sc); time.sleep(0.25); sc.read_and_print_nonblock()
        print(f"[{tag}] 停車偵測 {target_role}（{STOP_CHECK_SEC:.1f}s）...")
        seen = check_seen_while_stopped(model, cam, target_role, dwell_sec=STOP_CHECK_SEC)
        if seen:
            color = ROLE_TO_LED.get(target_role, "?")
            print(f"[{tag}] ✅ 看到 {target_role} → 亮 {color} 燈 {LED_ON_SEC:.0f}s")
            led(sc, color, seconds=LED_ON_SEC)
        else:
            print(f"[{tag}] ❌ 沒看到 {target_role}")
    # 段落全跑完再確保停住
    stop(sc); time.sleep(0.2); sc.read_and_print_nonblock()

# ===== 主流程 =====
def main():
    # 側視相機
    cam_side = cv2.VideoCapture(CAM_SIDE_INDEX)
    cam_side.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cam_side.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    if not cam_side.isOpened():
        print("[ERR] 開不了側視相機"); sys.exit(2)

    # YOLO
    model = YOLO(MODEL_PATH)

    # 串列（自動握手）
    sc = SerialClient(SERIAL_PORT, BAUDRATE)
    sc.send("PING")
    pong = sc.expect("PONG", timeout=1.2)
    print("[SER] PONG OK" if pong else "[SER] 沒收到 PONG（繼續測試）")

    # 1) 起點 → 動物區（寫死）
    run_segments(sc, PRE_TO_ANIMAL_SEGMENTS, tag="to_animal")

    # 2) 第一關：定案目標動物 → 亮燈 + 設定重量（僅記錄）
    def decide_first_animal(model, cam_side):
        print("[INFO] 停車偵測第一關動物 ...")
        stable = deque(maxlen=5)
        t0 = time.time()
        while True:
            ok, frm = cam_side.read()
            if not ok:
                if time.time()-t0 > 5.0:
                    print("[WARN] 影像讀取超時"); return None
                time.sleep(0.02); continue
            r = model.predict(source=frm, conf=DET_CONF_TH, verbose=False)[0]
            cand = []
            for b, c, conf in zip(r.boxes.xyxy.cpu().numpy(),
                                  r.boxes.cls.cpu().numpy(),
                                  r.boxes.conf.cpu().numpy()):
                name = normalize_classname(r.names[int(c)])
                if name in YOLO_CLASS_ROLE:
                    cand.append((YOLO_CLASS_ROLE[name], float(conf)))
            if cand:
                cand.sort(key=lambda x: x[1], reverse=True)
                stable.append(cand[0][0])
                print(f"[DBG] animal={cand[0][0]} conf={cand[0][1]:.2f} ({len(stable)}/5)")
                if len(stable)==5 and len(set(stable))==1:
                    return stable[-1]
            if time.time()-t0 > 5.0:
                return None

    target_role = decide_first_animal(model, cam_side) or "CHICKEN"
    target_grams = ROLE_TO_WEIGHT[target_role]
    led(sc, ROLE_TO_LED[target_role], seconds=LED_ON_SEC)
    print(f"[INFO] 目標={target_role} LED={ROLE_TO_LED[target_role]} 重量={target_grams}g（記錄）")

    # 3) 動物區 → 飼料機（寫死）
    run_segments(sc, ANIMAL_TO_FEEDER_SEGMENTS, tag="to_feeder")

    # 4) 到飼料機後：直接 STRETCH（不帶重量）
    print(f"[STEP] STRETCH（no weight）")
    stretch(sc)
    resp = sc.expect("STRETCH", timeout=15.0)  # 可能是 "STRETCH TEST OK" 或 "STRETCH_OK"
    print(f"[INFO] Arduino 回覆：{resp or '（逾時）'}")

    # 5) 餵食區：三段固定行進；每段結束停車→偵測→亮燈(若有看到)
    run_segments_stop_and_check(
        sc, FEEDER_TO_FEEDZONE_SEGMENTS, tag="feed_zone_stop_check",
        model=model, cam=cam_side, target_role=target_role
    )

    # 6) 出場（寫死）
    if EXIT_SEGMENTS:
        run_segments(sc, EXIT_SEGMENTS, tag="exit")

    # 清理
    sc.close()
    cam_side.release()
    cv2.destroyAllWindows()
    print("[DONE] 全流程完成。")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[EXIT] 手動中止")
