# Orin ⇄ Arduino PID 紅線循跡（室外）

## 概要
- 與室內 PID 流程相同，但增加：
  - 多色彩空間檢測（HSV + YCrCb + Lab）
  - 草地 Hue 抑制（35–85°）
  - 細線補線（閉運算）
- 預設失線策略：`stop`

---

## 接線對照
同 [PWM 控制](pwm_control.md)。

---

## 主要參數（控制）

| 參數            | 預設值         | 用途與說明                    |
|-----------------|----------------|-------------------------------|
| `BASE_L/R`      | 180 / 170      | 直線基準速度（室外可略大）    |
| `Kp`            | 0.5            | 比例增益                      |
| `U_MAX`         | 120            | 修正量上限                    |
| `ROWS`          | [40, 90, 140]  | 掃描帶位置                    |
| `BAND_HEIGHT`   | 30             | 掃描帶厚度                    |
| `MIN_PIXELS`    | 200            | 有效像素門檻                  |
| `LOST_HOLD`     | 12             | 容忍失線幀數                  |
| `FAILSAFE_MODE` | `"stop"`       | 失線策略                      |

---

## 影像處理參數（室外）
- HSV：`S ≥ 40`、`V ≥ 40`  
- HSV 紅區：0–10° 與 170–179°  
- HSV 綠區（草地抑制）：35–85°  
- YCrCb：`Cr ≥ 150`  
- Lab：`a* ≥ 140`  
- 閉運算 kernel：3×3（細線友善）

---

## 程式碼
- Arduino：`testMotor_Arduino_outdoor.ino`  
- Orin：`testPID_Outdoor.py`
