# Orin ⇄ Arduino PID 紅線循跡（室內）

## 概要
- Orin（Python/OpenCV）取像 → 以紅色線條估計偏差
- 以比例控制（Kp）計算兩輪速度差，送出 `L:<val> R:<val>` 給 Arduino
- 失線時可設定 `straight`（直走）或 `stop`（停車）

---

## 接線對照
同 [PWM 控制](pwm_control.md)。

---

## 主要參數

| 參數            | 預設值           | 用途與說明                          |
|-----------------|------------------|-------------------------------------|
| `BASE_L/R`      | 85 / 85          | 直線基準速度                        |
| `Kp`            | 0.25             | 比例增益（大→反應快，但易抖）       |
| `U_MAX`         | 120              | 修正量上限                          |
| `ROWS`          | [40, 90, 140]    | 底部往上三條掃描帶的位置（像素）    |
| `BAND_HEIGHT`   | 30               | 每條帶的厚度（像素）                |
| `MIN_PIXELS`    | 200              | 判定「有線」的最低像素數            |
| `LOST_HOLD`     | 12               | 最多容忍幾幀找不到線                |
| `FAILSAFE_MODE` | `"straight"`     | 失線策略：`"straight"` / `"stop"`   |

---

## 紅色檢測（HSV）
- 區間1：H 0–12，S ≥ 60，V ≥ 50  
- 區間2：H 170–180，S ≥ 60，V ≥ 50

---

## 診斷輸出範例

        [PWM] L=94 R=76 u=-9.1 found=3/3 pixels=[812,945,1010]
        [LOST] 5/12 → L=85 R=85 u=0.0 mode=STRAIGHT

---

## 程式碼
- Arduino：`testMotor_Arduino_outdoor.ino`  
- Orin：`testPID_Indoor.py`
