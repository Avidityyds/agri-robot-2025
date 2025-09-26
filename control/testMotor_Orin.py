import serial, time

PORT = "/dev/ttyACM0"  # orin 端先輸入 ls /dev/ttyACM* 確認有沒有 /dev/ttyACM0
BAUD = 9600

ser = serial.Serial(PORT, BAUD, timeout=0.5)
time.sleep(2)  # 等 Arduino reset

print("輸入左PWM 右PWM，例如: 120 80 代表左馬達PWM=120 右馬達PWM=80")
print("輸入 q 離開")

try:
    while True:
        cmd = input("> ").strip()
        if cmd.lower() == "q":
            break
        try:
            L, R = map(int, cmd.split())
        except:
            print("格式錯誤，請輸入: 左PWM 右PWM")
            continue
        s = f"L:{L} R:{R}\n"
        ser.write(s.encode())
        resp = ser.readline().decode().strip()
        if resp:
            print("[Arduino]", resp)
finally:
    ser.write(b"L:0 R:0\n")
    ser.close()