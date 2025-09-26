# Orin ⇄ Arduino 室內任務指令測試

## 概要
- **Orin (Python/pySerial)** 經 USB 串列傳送文字指令給 **Arduino**
- Arduino 解析後可控制：
  - 左右馬達（PWM 前/後/停）
  - LED 指示燈（R/G/Y）
  - 步進馬達（伸縮機構）
  - 重量感測器 HX711（即時重量）
  - 伺服馬達 SG90（上下角度）

---

## 接線表（部分）

### 前左馬達
| 訊號 | 腳位 |
|------|------|
| L_EN_FWD | 2 |
| L_EN_REV | 4 |
| L_PWM_FWD | 3 (PWM) |
| L_PWM_REV | 6 (PWM) |

### 前右馬達
| 訊號 | 腳位 |
|------|------|
| R_EN_FWD | 7 |
| R_EN_REV | 8 |
| R_PWM_FWD | 5 (PWM) |
| R_PWM_REV | 9 (PWM) |

### LED
| 顏色 | 腳位 |
|------|------|
| R | A5 |
| G | 12 |
| Y | 13 |

### HX711
| 訊號 | 腳位 |
|------|------|
| DOUT | A4 |
| SCK  | 11 |

---

## 程式碼
- Arduino：`testMission_Arduino_Indoor.ino`  
- Orin (Python)：`testMission_command.py`
