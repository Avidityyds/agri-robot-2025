"""
Orin 端互動式 Arduino 指令測試工具
- 支援：
  L:<vL> R:<vR> / STOP / LED <R|G|Y> / LED_STOP / PING / STRETCH / STRETCH <grams> / SCALE? / UP [deg] / DOWN [deg]
- 提供快捷：
  move vL vR      → 發送 "L:<vL> R:<vR>"
  f <pwm>         → 前進 (L=R=pwm)
  b <pwm>         → 後退 (L=R=-pwm)
  led r|g|y       → LED <R|G|Y>
  ledoff          → LED_STOP
  ping            → PING
  stretch [g]     → STRETCH 或 STRETCH <grams>
  scale? / weight → SCALE?
  up [deg]        → UP [deg]     （相對正轉，沒帶角度由 Arduino 預設 15°）
  down [deg]      → DOWN [deg]   （相對反轉，沒帶角度由 Arduino 預設 15°）
  send <rawline>  → 原樣送出（你自己打完整指令）
  help/?          → 顯示說明
  quit/exit       → 離開
"""

import sys
import time
import threading
import argparse
import glob
import shlex
import serial

DEFAULT_BAUD = 9600
READ_TIMEOUT = 0.05
EOL = "\n"

def list_candidate_ports():
    pats = ["/dev/ttyACM*", "/dev/ttyUSB*"]
    ports = []
    for p in pats:
        ports.extend(glob.glob(p))
    return sorted(ports)

def open_serial(port: str, baud: int):
    ser = serial.Serial(port=port, baudrate=baud, timeout=READ_TIMEOUT)
    time.sleep(2.0)  # Arduino reset 等待
    return ser

class SerialReader(threading.Thread):
    def __init__(self, ser: 'serial.Serial'):
        super().__init__(daemon=True)
        self.ser = ser
        self._stop = threading.Event()

    def run(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode("utf-8", errors="ignore").strip()
                    if s:
                        print(f"\033[92m<Arduino>\033[0m {s}")
            except Exception:
                break

    def stop(self):
        self._stop.set()

def send(ser, line: str):
    if not line.endswith("\n"):
        line += EOL
    ser.write(line.encode("utf-8"))
    print(f"\033[96m> {line.strip()}\033[0m")

def parse_and_send(ser, cmdline: str):
    if not cmdline.strip():
        return

    # 原樣傳：send <raw>
    if cmdline.startswith("send "):
        raw = cmdline[5:]
        send(ser, raw)
        return

    try:
        parts = shlex.split(cmdline)
    except ValueError:
        parts = cmdline.strip().split()

    if not parts:
        return

    c = parts[0].lower()

    # 說明 / 離開
    if c in ("help", "?"):
        print_help(); return
    if c in ("quit", "exit"):
        raise KeyboardInterrupt

    # 快捷移動
    if c == "move" and len(parts) == 3:
        vL = int(parts[1]); vR = int(parts[2])
        send(ser, f"L:{vL} R:{vR}"); return
    if c == "f" and len(parts) == 2:
        v = int(parts[1]); send(ser, f"L:{v} R:{v}"); return
    if c == "b" and len(parts) == 2:
        v = int(parts[1]); send(ser, f"L:{-v} R:{-v}"); return
    if c == "left" and len(parts) == 2:
        v = int(parts[1]); send(ser, f"L:{-v} R:{v}"); return
    if c == "right" and len(parts) == 2:
        v = int(parts[1]); send(ser, f"L:{v} R:{-v}"); return

    # LED
    if c == "led" and len(parts) == 2:
        color = parts[1].strip().upper()
        if color in ("R","G","Y"): send(ser, f"LED {color}")
        else: print("用法：led r|g|y")
        return
    if c in ("ledoff","led_stop","ledstop"):
        send(ser, "LED_STOP"); return

    # STOP / PING
    if c == "stop": send(ser, "STOP"); return
    if c == "ping": send(ser, "PING"); return

    # 伸縮
    if c == "stretch":
        if len(parts) == 2:
            grams = parts[1]; send(ser, f"STRETCH {grams}")
        else:
            send(ser, "STRETCH")
        return

    # 讀重量
    if c in ("scale?", "weight"):
        send(ser, "SCALE?"); return

    # 伺服：up / down
    if c == "up":
        if len(parts) == 2:
            send(ser, f"UP {parts[1]}")
        else:
            send(ser, "UP")  # 讓 Arduino 使用預設 15°
        return
    if c == "down":
        if len(parts) == 2:
            send(ser, f"DOWN {parts[1]}")
        else:
            send(ser, "DOWN")  # 預設 15°
        return

    # 原格式（萬用入口）
    if (cmdline.startswith("L:") and "R:" in cmdline) \
       or cmdline.startswith("LED ") \
       or cmdline in ("LED_STOP","STOP","PING","SCALE?","UP","DOWN","STRETCH") \
       or cmdline.startswith("STRETCH ") \
       or cmdline.startswith("UP ") \
       or cmdline.startswith("DOWN "):
        send(ser, cmdline); return

    print("看不懂這個指令；輸入 help 取得說明。")

def print_help():
    print("""
指令說明（左側是 CLI 快捷；右側是實際送去 Arduino 的內容）

  move vL vR        →  L:<vL> R:<vR>      （例：move 120 120）
  f <pwm>           →  L:<p>  R:<p>       （前進）
  b <pwm>           →  L:-<p> R:-<p>      （後退）
  stop              →  STOP
  led r|g|y         →  LED R|G|Y
  ledoff            →  LED_STOP
  ping              →  PING
  stretch           →  STRETCH            （伸5圈→收5圈：測試）
  stretch <grams>   →  STRETCH <grams>    （伸5圈→達標→收5圈）
  scale? / weight   →  SCALE?             （回覆 WEIGHT <g>）
  up [deg]          →  UP [deg]           （相對正轉；預設 15°）
  down [deg]        →  DOWN [deg]         （相對反轉；預設 15°）

  send <rawline>    →  原樣送出（直接打 Arduino 指令）
  help / ?          →  顯示這份說明
  quit / exit       →  離開
""")

def main():
    ap = argparse.ArgumentParser(description="Orin 端 Arduino 測試 CLI")
    ap.add_argument("--port", "-p", help="Serial 裝置路徑，例如 /dev/ttyACM0（若未指定會自動嘗試）")
    ap.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD, help=f"鮑率（預設 {DEFAULT_BAUD}）")
    args = ap.parse_args()

    port = args.port
    if not port:
        candidates = list_candidate_ports()
        if len(candidates) == 1:
            port = candidates[0]
            print(f"自動偵測到序列埠：{port}")
        elif len(candidates) > 1:
            print("找到多個序列埠，請用 --port 指定其一：")
            for c in candidates: print("  ", c)
            return
        else:
            print("找不到可用的序列埠，請用 --port 指定，例如： --port /dev/ttyACM0")
            return

    try:
        ser = open_serial(port, args.baud)
    except Exception as e:
        print(f"無法開啟序列埠 {port} @ {args.baud}: {e}")
        return

    print_help()
    print(f"\n已連線：{port} @ {args.baud} （Ctrl+C 離開）\n")

    reader = SerialReader(ser); reader.start()
    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            parse_and_send(ser, line)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        time.sleep(0.1)
        try:
            ser.close()
        except:
            pass
        print("\n已離開。")

if __name__ == "__main__":
    main()
